#!/usr/bin/env python3
"""Generate one mapping round's effective YAML from the production template.

Applies the operator's ``--set`` overrides, then injects the active vehicle's
body-filter parameters from its calibration.json.  This keeps
config/wuhu_truck29.yaml as a generic fallback while every vehicle carries its
own body mask in the calibration file.
"""
from __future__ import annotations

import json
import pathlib
import sys

import yaml

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from livo.body_filter import normalize, to_yaml_overrides  # noqa: E402


def write_effective_config(src: str, dst: str, calibration: str, sets: list[str]) -> list[str]:
    with open(src, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    params = doc["laser_mapping"]["ros__parameters"]
    logs = []
    for item in sets:
        key, _, raw = item.partition("=")
        node = params
        parts = key.split(".")
        for p in parts[:-1]:
            node = node.setdefault(p, {})
        old = node.get(parts[-1])
        try:
            val = yaml.safe_load(raw)
        except Exception:
            val = raw
        if isinstance(old, bool) and not isinstance(val, bool):
            val = str(raw).lower() in ("1", "true", "yes")
        elif isinstance(old, float) and isinstance(val, int):
            val = float(val)
        node[parts[-1]] = val
        logs.append(f"override {key}: {old} -> {val}")

    if calibration:
        try:
            calib = json.loads(pathlib.Path(calibration).read_text())
        except (OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"cannot read calibration {calibration}: {exc}") from exc
        filt = normalize(calib)
        if filt["enabled"]:
            ov = to_yaml_overrides(filt)
            ml = params.setdefault("multi_lidar", {})
            bf = ml.setdefault("body_filter", {})
            for k, v in ov["body_filter"].items():
                bf[k] = v
            pre = ml.setdefault("precheck", {})
            pre["body_box"] = ov["precheck"]["body_box"]
            logs.append(f"override multi_lidar.body_filter: from calibration {calibration}")
            logs.append(f"override multi_lidar.precheck.body_box: {ov['precheck']['body_box']}")
        else:
            logs.append("body_filter disabled in calibration; production yaml fallback retained")

    with open(dst, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True)
    return logs


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) < 3:
        raise SystemExit("usage: effective_config.py <src.yaml> <dst.yaml> <calibration.json|''> [key=value ...]")
    src, dst, calibration = argv[:3]
    logs = write_effective_config(src, dst, calibration or None, argv[3:])
    print("\n".join(logs))


if __name__ == "__main__":
    main()
