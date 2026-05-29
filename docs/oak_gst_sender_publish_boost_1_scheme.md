# Структурная схема `oak_gst_sender_publish_boost_1.py`

## Общая структура

```mermaid
%%{init: {"flowchart": {"defaultRenderer": "elk", "curve": "step"}} }%%
flowchart TD
    MAIN["main()"]
    ARGS["argparse<br/>host, port<br/>--no-depth<br/>--depth-fps, --depth-preset<br/>--no-shm<br/>--shm-prefix, --shm-slots<br/>SHM size/fps args"]
    SENDER["OakGstSender(...)"]
    START["sender.start()"]

    MAIN --> ARGS
    ARGS --> SENDER
    SENDER --> START

    START --> GST_INIT["Gst.init()<br/>build_gst_pipeline()<br/>Gst.parse_launch()"]
    START --> SHM_INIT["create ShmImageRing<br/>oak_rgb: bgr8<br/>oak_depth: 16UC1"]
    START --> DAI_BUILD["build_depthai_pipeline()"]
    DAI_BUILD --> DAI_START["DepthAI pipeline.start()"]
    DAI_START --> THREADS["start worker threads<br/>_video_loop()<br/>_depth_loop()"]
    THREADS --> SUPERVISOR["Supervisor loop<br/>_poll_gst_bus()<br/>_print_periodic_status()"]
```

## DepthAI pipeline

```mermaid
%%{init: {"flowchart": {"defaultRenderer": "elk", "curve": "step"}} }%%
flowchart TD
    CAM_A["CAM_A RGB camera<br/>sensorFps=60"]
    CAM_B["CAM_B left mono<br/>depth_fps"]
    CAM_C["CAM_C right mono<br/>depth_fps"]

    VIDEO_OUT["requestOutput()<br/>1920x1080 NV12"]
    VIDEO_Q["video_queue<br/>maxSize=2<br/>blocking=false"]

    LEFT_OUT["requestOutput()<br/>640x400 GRAY8"]
    RIGHT_OUT["requestOutput()<br/>640x400 GRAY8"]
    STEREO["StereoDepth<br/>HIGH_DENSITY / FAST_DENSITY<br/>LeftRightCheck=true<br/>align to CAM_A<br/>output 640x400"]
    DEPTH_Q["depth_queue<br/>16UC1 depth, mm<br/>maxSize=2<br/>blocking=false"]

    CAM_A --> VIDEO_OUT
    VIDEO_OUT --> VIDEO_Q

    CAM_B --> LEFT_OUT
    CAM_C --> RIGHT_OUT
    LEFT_OUT --> STEREO
    RIGHT_OUT --> STEREO
    STEREO --> DEPTH_Q
```

## Рабочие потоки и передача данных

```mermaid
%%{init: {"flowchart": {"defaultRenderer": "elk", "curve": "step"}} }%%
flowchart TD
    VIDEO_Q["video_queue.tryGet()<br/>NV12 1920x1080"]
    DEPTH_Q["depth_queue.tryGet()<br/>16UC1 640x400"]

    VIDEO_THREAD["_video_loop()<br/>thread: oak-gst-video"]
    DEPTH_THREAD["_depth_loop()<br/>thread: oak-ros-depth"]

    GST_BUF["Gst.Buffer<br/>pts, duration"]
    GST_PIPE["GStreamer RTP pipeline"]
    SHM_RGB_PREP["_maybe_write_rgb_shm()<br/>fps limit: shm_video_fps"]
    NV12_BGR["cv2.cvtColor()<br/>NV12 -> BGR"]
    RGB_RESIZE["cv2.resize()<br/>960x540 by default"]
    RGB_SHM["POSIX shared memory<br/>oak_rgb<br/>ShmImageRing<br/>encoding=bgr8<br/>slots=4 by default"]

    SHM_DEPTH_PREP["_maybe_write_depth_shm()<br/>fps limit: shm_depth_fps"]
    DEPTH_RESIZE["cv2.resize()<br/>nearest<br/>640x400 by default"]
    DEPTH_SHM["POSIX shared memory<br/>oak_depth<br/>ShmImageRing<br/>encoding=16UC1<br/>depth in mm<br/>slots=4 by default"]

    VIDEO_Q --> VIDEO_THREAD
    VIDEO_THREAD -->|"raw NV12 bytes"| GST_BUF
    GST_BUF -->|"appsrc.push-buffer"| GST_PIPE
    VIDEO_THREAD --> SHM_RGB_PREP
    SHM_RGB_PREP --> NV12_BGR
    NV12_BGR --> RGB_RESIZE
    RGB_RESIZE -->|"write(bgr.tobytes())"| RGB_SHM

    DEPTH_Q --> DEPTH_THREAD
    DEPTH_THREAD --> SHM_DEPTH_PREP
    SHM_DEPTH_PREP --> DEPTH_RESIZE
    DEPTH_RESIZE -->|"write(depth.tobytes())"| DEPTH_SHM
```

## GStreamer pipeline

```mermaid
%%{init: {"flowchart": {"defaultRenderer": "elk", "curve": "step"}} }%%
flowchart LR
    APP["appsrc source<br/>NV12 raw<br/>1920x1080@60<br/>is-live=true<br/>block=true"]
    ENC["mpph264enc<br/>bps=8 Mbps<br/>bps-max=10 Mbps<br/>CBR<br/>gop=60<br/>profile=high"]
    PARSE["h264parse<br/>config-interval=1"]
    PAY["rtph264pay<br/>mtu=1400<br/>pt=96"]
    UDP["udpsink<br/>host:port<br/>sync=false<br/>async=false"]

    APP --> ENC --> PARSE --> PAY --> UDP
```

## Жизненный цикл

```mermaid
sequenceDiagram
    participant M as main()
    participant S as OakGstSender
    participant G as GStreamer
    participant D as DepthAI
    participant RGB as SHM oak_rgb
    participant Depth as SHM oak_depth
    participant T as Worker threads

    M->>S: create sender from CLI args
    M->>S: start()
    S->>G: Gst.init(), parse_launch(), PLAYING
    S->>RGB: create ShmImageRing(bgr8)
    S->>Depth: create ShmImageRing(16UC1)
    S->>D: build_depthai_pipeline(), start()
    S->>T: start _video_loop() and _depth_loop()
    loop while running
        T->>G: push NV12 frame to appsrc
        T->>RGB: write resized BGR frame
        T->>Depth: write resized depth frame
        S->>G: poll bus
        S->>S: print status every 2 s
    end
    M-->>S: SIGINT/SIGTERM sets running=false
    S->>T: join threads
    S->>G: end-of-stream, set NULL
    S->>D: stop()
    S->>RGB: close/unlink
    S->>Depth: close/unlink
```

## Основные отличия варианта `_boost_1`

- ROS2-публикация `/oak/rgb/image_raw` и `/oak/stereo/depth` удалена из sender.
- RGB в GStreamer остается исходным `NV12 1920x1080@60`, а RGB в shared memory пишется отдельно как `bgr8`, по умолчанию `960x540@15`.
- Depth пишется в shared memory как `16UC1`, по умолчанию `640x400@15`.
- Два SHM-сегмента создаются по префиксу: `oak_rgb` и `oak_depth`.
- Контроль частоты SHM вынесен в `_maybe_write_rgb_shm()` и `_maybe_write_depth_shm()`.

