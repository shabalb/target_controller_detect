Place `yolov8n.onnx` or `yolov8n.rknn` in this directory.

Expected default ONNX launch path:
`<install_prefix>/share/target_controller_detect/models/yolov8n.onnx`

At runtime you can override with:
`detector_model_path:=/absolute/path/to/yolov8n.onnx`

For Rockchip NPU on RK3588/Radxa 5B, build the RKNN node and run with:
`detector_executable:=nn_person_detector_node_rknn detector_model_path:=/absolute/path/to/yolov8n.rknn`
