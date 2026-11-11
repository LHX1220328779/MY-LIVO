"""Operator-visible axle quality limits; identical units in Studio, CLI and reports."""
import math

LIMITS = {
    "max_translation_cm": {"label": "平移修正上限 (cm)", "default": 100.0, "step": 5},
    "max_rotation_deg": {"label": "旋转修正上限 (°)", "default": 6.0, "step": 0.5},
    "max_abs_time_ms": {"label": "最终时间偏移绝对值上限 (ms)", "default": 250.0, "step": 10},
    "max_residual_increase_mm": {"label": "残差中位数允许增加 (mm)", "default": 0.5, "step": 0.1},
    "warn_time_change_ms": {"label": "时间修正警告阈值 (ms)", "default": 80.0, "step": 5},
    "warn_split_rotation_deg": {"label": "两半旋转差警告阈值 (°)", "default": 0.2, "step": 0.05},
    "warn_split_translation_cm": {"label": "两半平移差警告阈值 (cm)", "default": 20.0, "step": 1},
    "warn_split_time_ms": {"label": "两半时间差警告阈值 (ms)", "default": 10.0, "step": 1},
}


def normalize_limits(values=None):
    values = {} if values is None else values
    if not isinstance(values, dict):
        raise ValueError("后轴标定门限必须是对象")
    unknown = set(values) - set(LIMITS)
    if unknown:
        raise ValueError(f"未知后轴标定门限：{', '.join(sorted(unknown))}")
    out = {}
    for key, spec in LIMITS.items():
        value = values.get(key, spec["default"])
        try:
            number = float(value)
        except (ValueError, TypeError):
            raise ValueError(f"{spec['label']} 必须是有限非负数") from None
        if isinstance(value, bool) or not math.isfinite(number) or number < 0:
            raise ValueError(f"{spec['label']} 必须是有限非负数")
        out[key] = number
    return out


def add_arguments(parser):
    for key, spec in LIMITS.items():
        parser.add_argument("--" + key.replace("_", "-"), type=float, default=spec["default"],
                            help=f"{spec['label']} (default: {spec['default']:g})")


def command_args(values):
    return [arg for key, value in normalize_limits(values).items()
            for arg in ("--" + key.replace("_", "-"), str(value))]
