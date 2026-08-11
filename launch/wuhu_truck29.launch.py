from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    share = get_package_share_directory("fast_livo")
    installed_config = os.path.join(share, "config", "wuhu_truck29.yaml")
    source_config = "/home/project/MY-LIVO1.0/config/wuhu_truck29.yaml"
    default_config = (
        source_config if os.path.isfile(source_config) else installed_config
    )
    rviz_config = os.path.join(share, "rviz_cfg", "wuhu_truck29.rviz")
    config_file = LaunchConfiguration("config_file")
    use_camera = LaunchConfiguration("use_camera")
    use_rviz = LaunchConfiguration("use_rviz")
    rear_axle_to_imu = LaunchConfiguration("rear_axle_to_imu")

    decoder = Node(
        package="image_transport",
        executable="republish",
        name="midrange_camera_decoder",
        condition=IfCondition(use_camera),
        arguments=["ffmpeg", "raw"],
        parameters=[{
            "in.ffmpeg.decoders.hevc":
                "hevc_cuvid,hevc_qsv,hevc_v4l2m2m,hevc",
        }],
        # In Humble the executable creates the suffixed input endpoint before
        # base-name remapping, so remap the concrete transport topic.
        remappings=[
            ("in/ffmpeg", "/midrange_camera/ffmpeg"),
            ("out", "/midrange_camera/image_raw"),
        ],
        output="screen",
    )

    mapping = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laser_mapping",
        output="screen",
        parameters=[
            config_file,
            {
                "common.img_en": ParameterValue(use_camera, value_type=bool),
                "reference_frame_conversion.enabled": ParameterValue(
                    rear_axle_to_imu, value_type=bool
                ),
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("rear_axle_to_imu", default_value="true"),
        decoder,
        mapping,
        rviz,
    ])
