cmake_minimum_required(VERSION 3.8)
project(target_controller_detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_COLOR_DETECTOR "Build HSV color detector node" ON)
option(BUILD_NN_DETECTOR "Build YOLO ONNX detector node" ON)
option(BUILD_RKNN_DETECTOR "Build YOLO RKNN detector node for Rockchip NPU" OFF)
set(ONNXRUNTIME_ROOT "/home/s/.local/lib/python3.12/site-packages" CACHE PATH "Path to ONNX Runtime installation")
set(RKNN_ROOT "" CACHE PATH "Path to RKNN Runtime installation")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(Boost REQUIRED)

if(BUILD_NN_DETECTOR)
  find_path(ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    HINTS
      ${ONNXRUNTIME_ROOT}/include
      ${ONNXRUNTIME_ROOT}/include/onnxruntime
      /usr/local/include
      /usr/include
    PATH_SUFFIXES onnxruntime
                  onnxruntime/core/session
  )
  find_library(ONNXRUNTIME_LIBRARY
    NAMES onnxruntime libonnxruntime.so.1.26.0 libonnxruntime.so
    HINTS
      ${ONNXRUNTIME_ROOT}/lib
      ${ONNXRUNTIME_ROOT}/lib64
      ${ONNXRUNTIME_ROOT}/capi
      /usr/local/lib
      /usr/local/lib64
      /usr/lib
      /usr/lib64
  )
  if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
    message(FATAL_ERROR
      "ONNX Runtime C/C++ development files were not found. The Python wheel is not enough by itself: "
      "it may provide libonnxruntime.so under onnxruntime/capi, but it does not provide "
      "onnxruntime_cxx_api.h. Install the ONNX Runtime C/C++ release and build with "
      "-DONNXRUNTIME_ROOT=/path/to/onnxruntime, or set ONNXRUNTIME_INCLUDE_DIR and "
      "ONNXRUNTIME_LIBRARY explicitly.")
  endif()
endif()

if(BUILD_RKNN_DETECTOR)
  find_path(RKNN_INCLUDE_DIR
    NAMES rknn_api.h
    HINTS
      ${RKNN_ROOT}/include
      ${RKNN_ROOT}/runtime/Linux/librknn_api/include
      /usr/include
      /usr/local/include
  )
  find_library(RKNN_LIBRARY
    NAMES rknnrt librknnrt.so
    HINTS
      ${RKNN_ROOT}/lib
      ${RKNN_ROOT}/lib64
      ${RKNN_ROOT}/runtime/Linux/librknn_api/aarch64
      /usr/lib
      /usr/local/lib
      /usr/lib/aarch64-linux-gnu
  )
  if(NOT RKNN_INCLUDE_DIR OR NOT RKNN_LIBRARY)
    message(FATAL_ERROR
      "RKNN Runtime files were not found. Install Rockchip RKNN Runtime on the Radxa, "
      "or build with -DRKNN_ROOT=/path/to/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api.")
  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(msg_files
  "msg/Detection2D.msg"
  "msg/TargetState.msg"
  "msg/CommandTwist.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

add_executable(motion_validator src/motion_validator.cpp)
add_executable(target_fusion_node src/target_fusion_node.cpp)
add_executable(target_controller_node src/target_controller_node.cpp)
add_executable(target_debug_viewer_node src/target_debug_viewer_node.cpp)
add_executable(shm_image_info src/shm_image_info.cpp)
add_executable(shm_color_detector_node src/shm_color_detector_node.cpp)
add_executable(shm_depth_fusion_node src/shm_depth_fusion_node.cpp)

if(BUILD_COLOR_DETECTOR)
  add_executable(color_detector_node src/color_detector_node.cpp)
endif()

if(BUILD_NN_DETECTOR)
  add_executable(nn_person_detector_node src/nn_person_detector_node.cpp)
  target_include_directories(nn_person_detector_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
endif()

if(BUILD_RKNN_DETECTOR)
  add_executable(nn_person_detector_node_rknn src/nn_person_detector_node_rknn.cpp)
  target_include_directories(nn_person_detector_node_rknn PRIVATE ${RKNN_INCLUDE_DIR})
endif()

ament_target_dependencies(motion_validator rclcpp sensor_msgs geometry_msgs)
ament_target_dependencies(target_fusion_node rclcpp sensor_msgs std_msgs)
ament_target_dependencies(target_controller_node rclcpp geometry_msgs std_msgs)
ament_target_dependencies(target_debug_viewer_node rclcpp sensor_msgs cv_bridge std_msgs)
ament_target_dependencies(shm_color_detector_node rclcpp std_msgs)
ament_target_dependencies(shm_depth_fusion_node rclcpp std_msgs)

if(BUILD_COLOR_DETECTOR)
  ament_target_dependencies(color_detector_node rclcpp sensor_msgs cv_bridge std_msgs)
endif()

if(BUILD_NN_DETECTOR)
  ament_target_dependencies(nn_person_detector_node rclcpp sensor_msgs cv_bridge std_msgs)
endif()

if(BUILD_RKNN_DETECTOR)
  ament_target_dependencies(nn_person_detector_node_rknn rclcpp sensor_msgs cv_bridge std_msgs)
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(include)

target_link_libraries(target_debug_viewer_node ${OpenCV_LIBS})
target_link_libraries(shm_image_info ${Boost_LIBRARIES})
target_link_libraries(shm_color_detector_node ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_link_libraries(shm_depth_fusion_node ${OpenCV_LIBS} ${Boost_LIBRARIES})

if(BUILD_COLOR_DETECTOR)
  target_link_libraries(color_detector_node ${OpenCV_LIBS})
endif()

if(BUILD_NN_DETECTOR)
  target_link_libraries(nn_person_detector_node ${OpenCV_LIBS} ${Boost_LIBRARIES} ${ONNXRUNTIME_LIBRARY})
endif()

if(BUILD_RKNN_DETECTOR)
  target_link_libraries(nn_person_detector_node_rknn ${OpenCV_LIBS} ${Boost_LIBRARIES} ${RKNN_LIBRARY})
endif()

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(target_fusion_node "${cpp_typesupport_target}")
target_link_libraries(target_controller_node "${cpp_typesupport_target}")
target_link_libraries(target_debug_viewer_node "${cpp_typesupport_target}")
target_link_libraries(shm_color_detector_node "${cpp_typesupport_target}")
target_link_libraries(shm_depth_fusion_node "${cpp_typesupport_target}")

if(BUILD_COLOR_DETECTOR)
  target_link_libraries(color_detector_node "${cpp_typesupport_target}")
endif()

if(BUILD_NN_DETECTOR)
  target_link_libraries(nn_person_detector_node "${cpp_typesupport_target}")
endif()

if(BUILD_RKNN_DETECTOR)
  target_link_libraries(nn_person_detector_node_rknn "${cpp_typesupport_target}")
endif()

install(TARGETS motion_validator
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_COLOR_DETECTOR)
  install(TARGETS color_detector_node
    DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_NN_DETECTOR)
  install(TARGETS nn_person_detector_node
    DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_RKNN_DETECTOR)
  install(TARGETS nn_person_detector_node_rknn
    DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS target_fusion_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS target_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS target_debug_viewer_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS shm_image_info
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS shm_color_detector_node shm_depth_fusion_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_export_dependencies(rosidl_default_runtime)


ament_package()
