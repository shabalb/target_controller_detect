# Схема `target_controller_detect` с передачей RGB и depth через shared memory

```mermaid
%%{init: {"flowchart": {"defaultRenderer": "elk", "curve": "step"}} }%%
flowchart TD
    OAK["OAK camera<br/>RGB video + stereo depth"]

    SENDER["oak_gst_sender_publish_boost.py<br/><b>class OakGstSender</b><br/>build_depthai_pipeline()<br/>_video_loop()<br/>_depth_loop()"]

    RGB_SHM["POSIX shared memory<br/><b>oak_rgb</b><br/>ShmImageRing<br/>encoding=nv12<br/>header + slots"]
    DEPTH_SHM["POSIX shared memory<br/><b>oak_depth</b><br/>ShmImageRing<br/>encoding=16UC1<br/>depth in mm<br/>header + slots"]

    DETECTOR["shm_color_detector_node<br/><b>class ShmColorDetectorNode</b><br/>onTimer()<br/>ensureReader()<br/>detectRedTarget()"]
    FUSION["shm_depth_fusion_node<br/><b>class ShmDepthFusionNode</b><br/>onDetection()<br/>onTimer()<br/>extractDepthValues()"]
    CONTROLLER["target_controller_node<br/><b>class TargetControllerNode</b><br/>onState()<br/>decideMode()<br/>computeCommand()<br/>computeCommandTrack()"]

    ROS_BASE["ROS robot base / simulator"]
    TRACK_BASE["Track controller"]
    OPER["Operator / mode manager"]

    OAK -->|"DepthAI frames"| SENDER
    SENDER -->|"write RGB frames<br/>ShmImageHeader + ShmImageView<br/>name: oak_rgb"| RGB_SHM
    SENDER -->|"write depth frames<br/>ShmImageHeader + ShmImageView<br/>name: oak_depth"| DEPTH_SHM

    RGB_SHM -->|"read latest RGB frame<br/>ShmImageRingReader<br/>nv12 -> BGR"| DETECTOR
    DETECTOR -->|"/target/detection2d<br/>target_controller_detect::msg::Detection2D<br/>found, bbox, center, cell, score"| FUSION

    DEPTH_SHM -->|"read latest depth frame<br/>ShmImageRingReader<br/>16UC1"| FUSION
    FUSION -->|"/target/state<br/>target_controller_detect::msg::TargetState<br/>valid, lost, distance, angle, rel_x, rel_y"| CONTROLLER

    CONTROLLER -->|"/cmd_vel<br/>geometry_msgs::msg::Twist<br/>linear.x, angular.z"| ROS_BASE
    CONTROLLER -->|"/target/persecuit<br/>target_controller_detect::msg::CommandTwist<br/>left, right, light, forced"| TRACK_BASE

    OPER -->|"/target/follow<br/>std_msgs::msg::String<br/>switch control mode to follow"| DETECTOR
    OPER -->|"/target/follow<br/>std_msgs::msg::String<br/>switch control mode to follow"| FUSION
```

## Последовательность обработки

```mermaid
sequenceDiagram
    participant OAK as OAK camera
    participant Sender as oak_gst_sender_publish_boost.py<br/>OakGstSender
    participant RGB as SHM oak_rgb<br/>ShmImageRing
    participant Depth as SHM oak_depth<br/>ShmImageRing
    participant Detector as shm_color_detector_node<br/>ShmColorDetectorNode
    participant Fusion as shm_depth_fusion_node<br/>ShmDepthFusionNode
    participant Controller as target_controller_node<br/>TargetControllerNode
    participant Base as Robot / track base

    OAK->>Sender: RGB NV12 frames + depth 16UC1 frames
    Sender->>RGB: _video_loop(): write(frame_data)
    Sender->>Depth: _depth_loop(): write(depth_msg.getData())

    RGB->>Detector: ShmImageRingReader::latest()
    Detector->>Detector: onTimer()
    Detector->>Detector: nv12 -> BGR
    Detector->>Detector: detectRedTarget()
    Detector->>Fusion: /target/detection2d<br/>Detection2D

    Depth->>Fusion: ShmImageRingReader::latest()
    Fusion->>Fusion: onDetection()
    Fusion->>Fusion: onTimer()
    Fusion->>Fusion: extractDepthValues()
    Fusion->>Fusion: median depth + angle
    Fusion->>Controller: /target/state<br/>TargetState

    Controller->>Controller: onState()
    Controller->>Controller: decideMode()
    Controller->>Controller: computeCommand(), computeCommandTrack()
    Controller->>Base: /cmd_vel Twist<br/>/target/persecuit CommandTwist
```

## Что изменилось относительно ROS Image pipeline

- RGB-кадр читается `shm_color_detector_node` из сегмента общей памяти `oak_rgb`, а не из топика `/oak/rgb/image_raw`.
- Depth читается `shm_depth_fusion_node` из сегмента `oak_depth`, а не из `/oak/stereo/depth` или `PointCloud2`.
- Между детектором, fusion и контроллером остаются обычные ROS2-сообщения: `Detection2D`, `TargetState`, `Twist`, `CommandTwist`.
- Обработка включается только в режиме follow: обе SHM-ноды слушают `/target/follow` типа `std_msgs::msg::String`.

