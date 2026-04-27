from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_topic = LaunchConfiguration("camera_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    camera_fov = LaunchConfiguration("camera_fov")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_topic", default_value="/oak/rgb/image_raw"),
            DeclareLaunchArgument("depth_topic", default_value="/oak/stereo/depth"),
            DeclareLaunchArgument("point_cloud_topic", default_value="/oak/stereo/depth/points"),
            DeclareLaunchArgument("image_width", default_value="640"),
            DeclareLaunchArgument("image_height", default_value="400"),
            DeclareLaunchArgument("camera_fov", default_value="1.255"),
            Node(
                package="target_controller_detect",
                executable="color_detector_node",
                name="color_detector_node",
                output="screen",
                parameters=[
                    {
                        "camera_topic": camera_topic,
                        "detection_topic": "/target/detection2d",
                    }
                ],
            ),
            Node(
                package="target_controller_detect",
                executable="target_fusion_node",
                name="target_fusion_node",
                output="screen",
                parameters=[
                    {
                        "detection_topic": "/target/detection2d",
                        "point_cloud_topic": point_cloud_topic,
                        "state_topic": "/target/state",
                        "image_width": image_width,
                        "image_height": image_height,
                        "camera_fov": camera_fov,
                        "detection_hold_sec": 0.6,
                    }
                ],
            ),
            Node(
                package="target_controller_detect",
                executable="target_controller_node",
                name="target_controller_node",
                output="screen",
                parameters=[
                    {
                        "state_topic": "/target/state",
                        "cmd_topic": "/cmd_vel",
                    }
                ],
            ),
            Node(
                package="target_controller_detect",
                executable="target_debug_viewer_node",
                name="target_debug_viewer_node",
                output="screen",
                parameters=[
                    {
                        "camera_topic": camera_topic,
                        "depth_topic": depth_topic,
                        "detection_topic": "/target/detection2d",
                        "state_topic": "/target/state",
                    }
                ],
            ),
        ]
    )
