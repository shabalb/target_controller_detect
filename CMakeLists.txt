cmake_minimum_required(VERSION 3.8)
project(target_controller_detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

add_executable(move_target src/move_target.cpp)
add_executable(drive_robot src/drive_robot.cpp)
add_executable(image_viewer src/image_viewer.cpp)
add_executable(image_viewer_3d src/image_viewer_3d.cpp)
add_executable(motion_validator src/motion_validator.cpp)
add_executable(color_detector_node src/color_detector_node.cpp)
add_executable(target_fusion_node src/target_fusion_node.cpp)
add_executable(target_controller_node src/target_controller_node.cpp)
add_executable(target_debug_viewer_node src/target_debug_viewer_node.cpp)
ament_target_dependencies(move_target rclcpp )
ament_target_dependencies(drive_robot rclcpp geometry_msgs)
ament_target_dependencies(image_viewer rclcpp sensor_msgs cv_bridge message_filters geometry_msgs)
ament_target_dependencies(motion_validator rclcpp sensor_msgs geometry_msgs)
ament_target_dependencies(image_viewer_3d rclcpp sensor_msgs cv_bridge message_filters geometry_msgs)
ament_target_dependencies(color_detector_node rclcpp sensor_msgs cv_bridge std_msgs)
ament_target_dependencies(target_fusion_node rclcpp sensor_msgs std_msgs)
ament_target_dependencies(target_controller_node rclcpp geometry_msgs std_msgs)
ament_target_dependencies(target_debug_viewer_node rclcpp sensor_msgs cv_bridge std_msgs)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

# --- Gazebo (gz) зависимости ---
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

target_link_libraries(move_target
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

target_link_libraries(image_viewer ${OpenCV_LIBS})
target_link_libraries(image_viewer_3d ${OpenCV_LIBS})
target_link_libraries(color_detector_node ${OpenCV_LIBS})
target_link_libraries(target_debug_viewer_node ${OpenCV_LIBS})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(color_detector_node "${cpp_typesupport_target}")
target_link_libraries(target_fusion_node "${cpp_typesupport_target}")
target_link_libraries(target_controller_node "${cpp_typesupport_target}")
target_link_libraries(target_debug_viewer_node "${cpp_typesupport_target}")

install(TARGETS move_target 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS drive_robot 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS image_viewer
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS image_viewer_3d
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS motion_validator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS color_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS target_fusion_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS target_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS target_debug_viewer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_export_dependencies(rosidl_default_runtime)


ament_package()
