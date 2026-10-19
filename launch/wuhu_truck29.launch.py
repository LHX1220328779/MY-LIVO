from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def launch_rviz(context, config, current_cloud):
    # Only the camera display needs a derived config. Keep the installed
    # default usable for standalone RViz/LIO, and runtime files under results.
    if LaunchConfiguration("use_camera").perform(context).lower() == "true":
        with open(config, encoding="utf-8") as stream:
            settings = yaml.safe_load(stream)
        for display in settings["Visualization Manager"]["Displays"]:
            if display.get("Class") == "rviz_default_plugins/PointCloud2":
                display["Color Transformer"] = "RGB8"
                display["Channel Name"] = "rgb"
        output = os.path.realpath(LaunchConfiguration("output_directory").perform(context))
        results = os.path.realpath("/home/project/MY-LIVO/results")
        if os.path.commonpath([output, results]) != results:
            raise ValueError("RViz runtime configuration must stay under results/")
        os.makedirs(output, exist_ok=True)
        config = os.path.join(output, "camera_display.rviz")
        with open(config, "w", encoding="utf-8") as stream:
            yaml.safe_dump(settings, stream, sort_keys=False)
    return [Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", config],
        remappings=[("/rtk/cloud_current", current_cloud)], output="screen",
    )]


def _fastdds_profile():
    for path in (os.path.join(get_package_share_directory("fast_livo"), "config", "fastdds_livo_shm.xml"),
                 os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "config", "fastdds_livo_shm.xml")):
        if os.path.isfile(path):
            return os.path.realpath(path)
    return ""


def hevc_decoders():
    """Decoder preference of the camera stream. hevc_cuvid (NVDEC) is tried first only when the CUDA driver
    answers: with the GPU in a fault state (kernel Xid 31/154 "reboot required", cuInit -> CUDA_ERROR_UNKNOWN)
    the republish node segfaults inside the cuvid decoder instead of falling back, and a LIVO round then ends
    with zero frames. Without a working driver the stream is decoded in software (1080p HEVC at 30 fps fits
    on a few cores)."""
    import ctypes
    try:
        if ctypes.CDLL("libcuda.so.1").cuInit(0) == 0:
            return "hevc_cuvid,hevc_qsv,hevc_v4l2m2m,hevc"
        print("[wuhu_truck29.launch] CUDA driver unavailable (cuInit failed): camera stream decoded in software")
    except OSError:
        print("[wuhu_truck29.launch] libcuda.so.1 not found: camera stream decoded in software")
    return "hevc"


def generate_launch_description():
    share = get_package_share_directory("fast_livo")
    installed_config = os.path.join(share, "config", "wuhu_truck29.yaml")
    source_config = "/home/project/MY-LIVO/config/wuhu_truck29.yaml"
    default_config = (
        source_config if os.path.isfile(source_config) else installed_config
    )
    rviz_config = os.path.join(share, "rviz_cfg", "wuhu_truck29.rviz")
    default_output_directory = os.environ.get(
        "FAST_LIVO_OUTPUT_DIRECTORY",
        "/home/project/MY-LIVO/results",
    )
    config_file = LaunchConfiguration("config_file")
    use_camera = LaunchConfiguration("use_camera")
    use_rviz = LaunchConfiguration("use_rviz")
    use_rtk = LaunchConfiguration("use_rtk")
    # "" = follow use_rtk. --no-rtk rounds pass use_rtk:=false (native RViz layout, honest
    # display) together with rtk_backend:=true (exporter backend stays alive, observations
    # starved via rtk.observations_enabled=false in the config overrides).
    rtk_backend = LaunchConfiguration("rtk_backend")
    rear_axle_to_imu = LaunchConfiguration("rear_axle_to_imu")
    rviz_current_cloud = PythonExpression([
        "'/rtk/cloud_current' if '", use_rtk,
        "'.lower() == 'true' else ('/cloud_registered' if '", use_camera,
        "'.lower() == 'true' else '/cloud_current_frame')",
    ])

    decoder = Node(
        package="image_transport",
        executable="republish",
        name="midrange_camera_decoder",
        condition=IfCondition(use_camera),
        arguments=["ffmpeg", "raw"],
        parameters=[{
            "in.ffmpeg.decoders.hevc": hevc_decoders(),
        }],
        # In Humble the executable creates the suffixed input endpoint before
        # base-name remapping, so remap the concrete transport topic.
        remappings=[
            ("in/ffmpeg", "/midrange_camera/ffmpeg"),
            ("out", "/midrange_camera/image_raw"),
        ],
        output="screen",
    )

    def launch_mapping(context):
        overrides = {
            "common.img_en": ParameterValue(use_camera, value_type=bool),
            "rtk.enabled": ParameterValue(PythonExpression([
                "'", use_rtk, "'.lower() == 'true' or '", rtk_backend, "'.lower() == 'true'",
            ]), value_type=bool),
            "rtk.save_map": ParameterValue(LaunchConfiguration("save_map"), value_type=bool),
            "output.directory": LaunchConfiguration("output_directory"),
            "reference_frame_conversion.enabled": ParameterValue(
                rear_axle_to_imu, value_type=bool
            ),
            "use_sim_time": True,
        }
        # Saved-map product caps (metres, 0 = keep every return). Empty = keep the yaml value, so the
        # production default stays in config/wuhu_truck29.yaml and the console can override per run.
        for arg, key in (("save_map_max_range", "rtk.save_map_max_range"),
                         ("save_rgb_max_range", "rtk.save_rgb_max_range")):
            raw = LaunchConfiguration(arg).perform(context).strip()
            if raw:
                overrides[key] = float(raw)
        return [Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laser_mapping",
            output="screen",
            sigterm_timeout=LaunchConfiguration("finalize_timeout_sec"),
            parameters=[config_file, overrides],
        )]

    mapping = OpaqueFunction(function=launch_mapping)

    rviz = OpaqueFunction(
        function=launch_rviz,
        args=[rviz_config, rviz_current_cloud],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        # Raw 1920x1080 camera frames are too large for the local UDP path at
        # 30 Hz and can starve the IMU stream. Keep the high-bandwidth local
        # traffic on FastDDS shared memory, while preserving an explicit
        # override for deployments that need another transport.
        SetEnvironmentVariable(name="FASTDDS_BUILTIN_TRANSPORTS",
                               value=os.environ.get("FASTDDS_BUILTIN_TRANSPORTS") or "SHM"),
        # Large SHM segment + short heartbeat (config/fastdds_livo_shm.xml): the builtin 512 KB
        # segment fragments clouds/images and delays repairs by seconds on long rounds.
        SetEnvironmentVariable(name="FASTRTPS_DEFAULT_PROFILES_FILE",
                               value=os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE") or _fastdds_profile()),
        DeclareLaunchArgument("use_camera", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_rtk", default_value="true"),
        DeclareLaunchArgument("rtk_backend", default_value=""),
        DeclareLaunchArgument("save_map", default_value="false"),
        DeclareLaunchArgument("finalize_timeout_sec", default_value="600.0"),
        DeclareLaunchArgument("output_directory", default_value=default_output_directory),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("rear_axle_to_imu", default_value="true"),
        DeclareLaunchArgument("save_map_max_range", default_value="",
                              description="saved geometry map cap [m], 0 = all, empty = yaml"),
        DeclareLaunchArgument("save_rgb_max_range", default_value="",
                              description="saved colour map cap [m], 0 = all, -1 = same as geometry, empty = yaml"),
        SetEnvironmentVariable(name="ROS_LOG_DIR", value=os.environ.get(
            "ROS_LOG_DIR") or "/home/project/MY-LIVO/results/roslog"),
        decoder,
        mapping,
        rviz,
    ])
