from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rgb_shm_name = LaunchConfiguration("rgb_shm_name")
    depth_shm_name = LaunchConfiguration("depth_shm_name")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    camera_fov = LaunchConfiguration("camera_fov")
    debug = LaunchConfiguration("debug")

    use_nn_detector = LaunchConfiguration("use_nn_detector")
    detector_model_path = LaunchConfiguration("detector_model_path")
    detector_backend = LaunchConfiguration("detector_backend")
    detector_target = LaunchConfiguration("detector_target")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rgb_shm_name", default_value="oak_rgb"),
            DeclareLaunchArgument("depth_shm_name", default_value="oak_depth"),
            DeclareLaunchArgument("image_width", default_value="960"),
            DeclareLaunchArgument("image_height", default_value="540"),
            DeclareLaunchArgument("camera_fov", default_value="1.255"),
            DeclareLaunchArgument("debug", default_value="false"),
            DeclareLaunchArgument("use_nn_detector", default_value="false"),
            DeclareLaunchArgument(
                "detector_model_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("target_controller_detect"), "models", "yolov8n.onnx"]
                ),
            ),
            DeclareLaunchArgument("detector_backend", default_value="opencv"),
            DeclareLaunchArgument("detector_target", default_value="cpu"),
            Node(
                package="target_controller_detect",
                executable="shm_color_detector_node",
                name="shm_color_detector_node",
                output="screen",
                condition=UnlessCondition(use_nn_detector),
                parameters=[
                    {
                        "shm_name": rgb_shm_name,
                        "detection_topic": "/target/detection2d",
                        "process_fps": 15.0,
                        "show_windows": debug,
                    }
                ],
            ),
            Node(
                package="target_controller_detect",
                executable="nn_person_detector_node",
                name="nn_person_detector_node",
                output="screen",
                condition=IfCondition(use_nn_detector),
                parameters=[
                    {
                        "camera_topic": "/unused_ros_image_topic",
                        "detection_topic": "/target/detection2d",
                        "model_path": detector_model_path,
                        "input_width": 640,
                        "input_height": 640,
                        "conf_threshold": 0.40,
                        "nms_threshold": 0.45,
                        "dnn_backend": detector_backend,
                        "dnn_target": detector_target,
                        "show_windows": debug,
                    }
                ],
            ),
            Node(
                package="target_controller_detect",
                executable="shm_depth_fusion_node",
                name="shm_depth_fusion_node",
                output="screen",
                parameters=[
                    {
                        "shm_name": depth_shm_name,
                        "detection_topic": "/target/detection2d",
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
                condition=IfCondition("false"),
                parameters=[
                    {
                        "detection_topic": "/target/detection2d",
                        "state_topic": "/target/state",
                    }
                ],
            ),
        ]
    )
