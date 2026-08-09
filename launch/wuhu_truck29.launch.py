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
    config = os.path.join(share, "config", "wuhu_truck29.yaml")
    rviz_config = os.path.join(share, "rviz_cfg", "wuhu_truck29.rviz")
    use_camera = LaunchConfiguration("use_camera")
    use_rviz = LaunchConfiguration("use_rviz")

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
            config,
            {"common.img_en": ParameterValue(use_camera, value_type=bool)},
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
        decoder,
        mapping,
        rviz,
    ])
