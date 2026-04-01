cmake_minimum_required(VERSION 3.8)
project(ur5_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(master_ros2 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)

add_executable(ur5_control_node src/main.cpp)
ament_target_dependencies(ur5_control_node rclcpp sensor_msgs std_msgs master_ros2)

add_executable(pub_node src/pub.cpp)
ament_target_dependencies(pub_node rclcpp std_msgs master_ros2)

add_executable(sub_node src/sub.cpp)
ament_target_dependencies(sub_node rclcpp std_msgs master_ros2)

# --- CORRECT ORDER FOR MOTION TASK 1 ---
# 1. First create the executable
add_executable(motion_task1 src/motion_task1.cpp src/ik_controller.cpp)

# 2. Then add the include directories
target_include_directories(motion_task1 PUBLIC include)

# 3. Then add the dependencies
ament_target_dependencies(motion_task1 rclcpp sensor_msgs kdl_parser orocos_kdl urdf)
# ---------------------------------------

install(TARGETS ur5_control_node pub_node sub_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  motion_task1
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()



