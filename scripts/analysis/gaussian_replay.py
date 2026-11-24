#!/usr/bin/env python3
"""Render a saved Gaussian map at recorded held-out cameras into a new results directory."""
import argparse
import json
from pathlib import Path
import shutil
import sys
import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
from livo.gaussian_global import replay_saved_map


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prepared", type=Path)
    parser.add_argument("model", type=Path, help="Map directory containing point_cloud.ply and map_info.json")
    parser.add_argument("output", type=Path, help="New results directory")
    parser.add_argument("--binary", type=Path, default=REPO / "build/gaussian/my_livo_gaussian")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    info = json.loads((args.model / "map_info.json").read_text())
    poses = args.prepared.parent / "camera_poses.json"
    if not poses.is_file():
        raise ValueError("Prepared input must include recorded camera_poses.json")
    recorded = {row["frame"]: row for row in json.loads((args.model / "camera_poses.json").read_text())}
    selected = json.loads(poses.read_text())
    for row in selected:
        previous = recorded.get(row["frame"])
        if (previous is None or any(previous[key] != row[key] for key in ("stamp", "scan_stamp", "validation")) or
                not np.allclose(previous["T_world_camera"], row["T_world_camera"], rtol=0, atol=1e-7)):
            raise ValueError(f"Prepared camera differs from the saved map: {row['frame']}")
    shutil.copyfile(poses, args.output / "camera_poses.json")
    info["metrics"] = replay_saved_map(args.prepared, args.model / "point_cloud.ply",
                                      args.output / "views", args.binary, info["background_rgb"])
    info.update(status="done", evaluated_model=str((args.model / "point_cloud.ply").resolve()),
                evaluation_input=str(args.prepared.resolve()), evaluation_only=True)
    (args.output / "map_info.json").write_text(json.dumps(info, indent=2))


if __name__ == "__main__":
    main()
