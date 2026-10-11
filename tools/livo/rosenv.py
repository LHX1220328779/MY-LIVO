"""ROS 2 Humble process environment for MY-LIVO tools.

Every Python tool that touches rosbag2_py / rclpy must run under the Ubuntu system
interpreter (/usr/bin/python3) with the ROS 2 environment *fully* sourced. Historically the
tools only fixed PYTHONPATH, which works from an interactive shell where setup.bash was already
sourced, but silently fails when a service (for example the LIVO Studio web server) is started
from a plain `docker exec` shell: rosbag2_py then dies with
``ImportError: librosbag2_compression.so: cannot open shared object file`` because
LD_LIBRARY_PATH is missing. This module removes that class of failure:

    from livo import rosenv
    rosenv.ensure()          # re-exec this script under the sourced ROS environment if needed

    env = rosenv.environment()   # complete environment dict for subprocesses
"""
import os
import pathlib
import subprocess
import sys

ROS_DISTRO_SETUP = "/opt/ros/humble/setup.bash"
SYSTEM_PYTHON = "/usr/bin/python3"
REPO = pathlib.Path(__file__).resolve().parents[2]
LOCAL_SETUP = REPO / "install" / "fast_livo" / "share" / "fast_livo" / "local_setup.bash"
_BOOTSTRAP_FLAG = "LIVO_ROSENV_BOOTSTRAPPED"
_STRIP = ("CONDA_PREFIX", "CONDA_DEFAULT_ENV", "CONDA_SHLVL", "PYTHONHOME", "PYTHONPATH",
          "PROMPT_COMMAND", "BASH_ENV", "ENV")
_cache = None


def clean_base_environment(base=None):
    """Copy of `base` (default os.environ) without Conda/Python overrides."""
    env = dict(os.environ if base is None else base)
    for key in _STRIP:
        env.pop(key, None)
    env["PATH"] = "/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    env.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "SHM")
    profile = REPO / "config" / "fastdds_livo_shm.xml"
    if profile.is_file():
        env.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE", str(profile))
    env.setdefault("LC_ALL", "C.UTF-8")
    env.setdefault("LANG", "C.UTF-8")
    return env


def environment(base=None, refresh=False):
    """Environment dict with /opt/ros/humble (and the fast_livo overlay) sourced."""
    global _cache
    if _cache is not None and not refresh and base is None:
        return dict(_cache)
    if not os.path.exists(ROS_DISTRO_SETUP):
        raise RuntimeError(f"ROS 2 setup script not found: {ROS_DISTRO_SETUP} (run inside the calibration container)")
    script = f"set +u; source '{ROS_DISTRO_SETUP}'"
    if LOCAL_SETUP.exists():
        script += f"; source '{LOCAL_SETUP}'"
    script += "; env -0"
    out = subprocess.run(["/bin/bash", "--noprofile", "--norc", "-c", script], env=clean_base_environment(base),
                         capture_output=True, check=True).stdout
    env = {}
    for item in out.split(b"\0"):
        if not item:
            continue
        key, _, value = item.decode("utf-8", "replace").partition("=")
        env[key] = value
    env[_BOOTSTRAP_FLAG] = "1"
    if base is None:
        _cache = dict(env)
    return env


def is_sourced(env=None):
    env = os.environ if env is None else env
    return ("/opt/ros/humble" in env.get("AMENT_PREFIX_PATH", "")
            and "/opt/ros/humble/lib" in env.get("LD_LIBRARY_PATH", ""))


def ensure():
    """Re-exec the running script under /usr/bin/python3 with ROS sourced (idempotent)."""
    if sys.executable == SYSTEM_PYTHON and is_sourced():
        return
    if os.environ.get(_BOOTSTRAP_FLAG):
        raise RuntimeError("ROS environment bootstrap failed: still not sourced after re-exec "
                           f"(executable {sys.executable})")
    env = environment()
    os.execve(SYSTEM_PYTHON, [SYSTEM_PYTHON, *sys.argv], env)


def status():
    """Diagnostics for the Studio system page."""
    info = {"ros_setup": ROS_DISTRO_SETUP, "ros_setup_exists": os.path.exists(ROS_DISTRO_SETUP),
            "overlay_setup": str(LOCAL_SETUP), "overlay_exists": LOCAL_SETUP.exists(),
            "system_python": SYSTEM_PYTHON, "python_ok": os.path.exists(SYSTEM_PYTHON)}
    try:
        env = environment()
        info["ok"] = True
        info["ros_distro"] = env.get("ROS_DISTRO")
        info["rmw"] = env.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp (default)")
        info["transports"] = env.get("FASTDDS_BUILTIN_TRANSPORTS")
    except Exception as exc:  # noqa: BLE001
        info["ok"] = False
        info["error"] = str(exc)
    return info
