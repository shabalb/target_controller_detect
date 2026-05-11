# Структурная схема `oak_gst_sender_publish_test.py`

## Общая структура

```mermaid
flowchart TD
    MAIN["main()"]
    ARGS["argparse\nhost, port, topics, image size/fps,\n--no-gst, --no-image, --no-depth"]
    SENDER["OakGstSender(...)"]
    START["sender.start()"]

    MAIN --> ARGS
    ARGS --> SENDER
    SENDER --> START

    START --> ROS_INIT["rclpy.init()\ncreate_node()\ncreate ROS publishers"]
    START --> GST_INIT["Gst.init()\nbuild_gst_pipeline()\nGst.parse_launch()"]
    START --> DAI_BUILD["build_depthai_pipeline()"]
    DAI_BUILD --> DAI_START["DepthAI pipeline.start()"]
    DAI_START --> THREADS["start worker threads"]
    THREADS --> SUPERVISOR["Supervisor loop\nspin_once()\npoll GStreamer bus\nprint status"]
```

## DepthAI pipeline

```mermaid
flowchart LR
    CAM_A["CAM_A\nRGB camera"]
    CAM_B["CAM_B\nleft mono"]
    CAM_C["CAM_C\nright mono"]

    CAM_A -->|"NV12 1920x1080 @ CAM_FPS"| VIDEO_Q["video_queue\nmaxSize=2, non-blocking"]
    CAM_A -->|"NV12/BGR/RGB\nimage_width x image_height @ image_fps"| IMAGE_Q["image_queue\nmaxSize=2, non-blocking"]

    CAM_B -->|"GRAY8 640x400 @ DEPTH_FPS"| STEREO["StereoDepth\nHIGH_DENSITY/FAST_DENSITY\nLR-check\nalign to CAM_A\noutput 640x400"]
    CAM_C -->|"GRAY8 640x400 @ DEPTH_FPS"| STEREO
    STEREO -->|"16UC1 depth, mm"| DEPTH_Q["depth_queue\nmaxSize=2, non-blocking"]
```

## Рабочие потоки

```mermaid
flowchart TD
    VIDEO_THREAD["_video_loop()\noak-gst-video"]
    IMAGE_THREAD["_image_loop()\noak-ros-image"]
    DEPTH_THREAD["_depth_loop()\noak-ros-depth"]
    SUPERVISOR["start() supervisor loop"]

    VIDEO_Q["video_queue.tryGet()"] --> VIDEO_THREAD
    IMAGE_Q["image_queue.tryGet()"] --> IMAGE_THREAD
    DEPTH_Q["depth_queue.tryGet()"] --> DEPTH_THREAD

    VIDEO_THREAD -->|"bytes(frame.getData())"| GST_BUF["Gst.Buffer\npts, duration"]
    GST_BUF -->|"appsrc.push-buffer"| GST_PIPE["GStreamer pipeline"]
    GST_PIPE --> ENC["mpph264enc\nRockchip HW H.264"]
    ENC --> RTP["h264parse\nrtph264pay"]
    RTP --> UDP["udpsink\nhost:port"]

    IMAGE_THREAD --> BUILD_IMG["build_image_msg()"]
    BUILD_IMG -->|"NV12 source"| NV12_BGR["nv12_to_bgr_bytes()\ncv2.cvtColor if available\nnumpy fallback"]
    BUILD_IMG --> ROS_RGB["ROS Image publisher\n/oak/rgb/image_raw\nencoding=bgr8/rgb8"]

    DEPTH_THREAD --> BUILD_DEPTH["build_depth_msg()"]
    BUILD_DEPTH --> ROS_DEPTH["ROS Image publisher\n/oak/stereo/depth\nencoding=16UC1"]

    SUPERVISOR --> SPIN["rclpy.spin_once(timeout=0)"]
    SUPERVISOR --> BUS["_poll_gst_bus()\nERROR/WARNING/EOS"]
    SUPERVISOR --> STATUS["_print_periodic_status()\ngst/rgb/depth fps"]
```

## GStreamer pipeline

```mermaid
flowchart LR
    APP["appsrc source\nNV12 raw\nis-live=true\nblock=false\nmax-buffers=2\nleaky downstream"]
    ENC["mpph264enc\nbps=4 Mbps\nbps-max=6 Mbps\nCBR\ngop=CAM_FPS"]
    PARSE["h264parse\nconfig-interval=1"]
    PAY["rtph264pay\nmtu=1400\npt=96"]
    UDP["udpsink\nsync=false\nasync=false"]

    APP --> ENC --> PARSE --> PAY --> UDP
```

## Жизненный цикл

```mermaid
sequenceDiagram
    participant M as main()
    participant S as OakGstSender
    participant D as DepthAI
    participant G as GStreamer
    participant R as ROS2
    participant T as Worker threads

    M->>S: create sender from CLI args
    M->>S: start()
    S->>R: init node and publishers
    S->>G: create and start pipeline
    S->>D: build pipeline and start()
    S->>T: start video/image/depth loops
    loop while running
        S->>R: spin_once()
        S->>G: poll bus
        S->>S: print status every 2 s
    end
    M-->>S: SIGINT/SIGTERM sets running=false
    S->>T: join threads
    S->>G: end-of-stream, set NULL
    S->>D: stop()
    S->>R: destroy node, shutdown()
```

## Назначение основных функций

- `build_depthai_pipeline()` создает камеры, выходные очереди RGB/video/depth и узел `StereoDepth`.
- `build_gst_pipeline()` собирает строку GStreamer для аппаратного H.264 и RTP/UDP отправки.
- `_video_loop()` обслуживает самый критичный путь: `video_queue -> appsrc -> RTP/UDP`.
- `_image_loop()` публикует цветной ROS `sensor_msgs/Image`; при NV12 конвертирует в BGR.
- `_depth_loop()` публикует depth ROS `sensor_msgs/Image` в формате `16UC1`.
- `_poll_gst_bus()` ловит ошибки, предупреждения и EOS из GStreamer.
- `stop()` останавливает потоки, GStreamer, DepthAI и ROS.

## Где возможны задержки

- `build_image_msg()` копирует кадр и может делать NV12->BGR конвертацию.
- `build_depth_msg()` копирует depth buffer перед публикацией.
- `publish()` сериализует большие `sensor_msgs/Image`.
- `StereoDepth` и дополнительные `requestOutput()` могут снижать пропускную способность внутри OAK/DepthAI даже при низкой CPU-нагрузке на одноплатнике.
