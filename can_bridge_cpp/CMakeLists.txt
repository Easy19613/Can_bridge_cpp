cmake_minimum_required(VERSION 3.8)
project(can_bridge_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(ros2_socketcan_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetDouble.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

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

add_executable(dm4340_can_sender src/dm4340_can_sender.cpp)
add_executable(dm4340_can_receiver src/dm4340_can_receiver.cpp)
add_executable(joy_control src/joy_control.cpp)
add_executable(ros2_param src/ros2_param.cpp)

ament_target_dependencies(
  dm4340_can_sender
  "rclcpp"
  "ros2_socketcan"
  "ros2_socketcan_msgs"
  "Eigen3"
  "sensor_msgs"
  "geometry_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)
ament_target_dependencies(
  dm4340_can_receiver
  "rclcpp"
  "ros2_socketcan"
  "ros2_socketcan_msgs"
  "Eigen3"
  "sensor_msgs"
  "geometry_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)
ament_target_dependencies(
  joy_control
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
)
ament_target_dependencies(
  ros2_param
  "rclcpp"
  "geometry_msgs"
  "rosidl_typesupport_cpp"
  "rosidl_default_runtime"  # 新增
)

# 安装目标可执行文件
install(TARGETS
dm4340_can_sender
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
dm4340_can_receiver
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
joy_control
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
ros2_param
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  ros2_param
  DESTINATION lib/${PROJECT_NAME}
)

# 添加头文件包含路径
target_include_directories(dm4340_can_sender PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_include_directories(dm4340_can_receiver PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_include_directories(joy_control PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_include_directories(ros2_param PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

# 链接依赖库
target_link_libraries(dm4340_can_sender
  ${rclcpp_LIBRARIES}
)
target_link_libraries(dm4340_can_receiver
  ${rclcpp_LIBRARIES}
)
target_link_libraries(joy_control
  ${rclcpp_LIBRARIES}
)

target_link_libraries(ros2_param
  ${rclcpp_LIBRARIES}
  ${PROJECT_NAME}__rosidl_generator_cpp  # 链接生成的接口库
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 导出依赖和头文件路径
ament_export_dependencies(rclcpp)
ament_export_include_directories(include)



ament_package()