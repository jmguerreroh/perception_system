cmake_minimum_required(VERSION 3.8)
project(perception_system)
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(perception_system_interfaces REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED QUIET)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(dependencies
  ament_cmake
  rclcpp
  rclcpp_components
  yolo_msgs
  visualization_msgs
  lifecycle_msgs
  rclcpp_lifecycle
  rclcpp_cascade_lifecycle
  vision_msgs
  # image_transport
  cv_bridge
  tf2_ros
  geometry_msgs
  # ament_index_cpp
  # image_transport
  # sensor_msgs
  # std_msgs
  OpenCV
  PCL
  pcl_conversions
  image_geometry
  # depth_image_proc
  Eigen3
  message_filters
  tf2
  tf2_eigen
  geometry_msgs
  tf2_geometry_msgs
  perception_system_interfaces
)

# OpenCV and PCL node
include_directories(include)
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/PeopleDetectionNode.cpp
  src/${PROJECT_NAME}/ObjectsDetectionNode.cpp
  src/${PROJECT_NAME}/DebugNode.cpp
  src/${PROJECT_NAME}/CollisionServer.cpp
  src/${PROJECT_NAME}/PerceptionListener.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

rclcpp_components_register_nodes(${PROJECT_NAME} "perception_system::PeopleDetectionNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "perception_system::ObjectsDetectionNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "perception_system::DebugNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "perception_system::CollisionServer")
rclcpp_components_register_nodes(${PROJECT_NAME} "perception_system::PerceptionListener")

add_executable(dt_people src/people_detection_main.cpp)
ament_target_dependencies(dt_people ${dependencies})
target_link_libraries(dt_people ${PROJECT_NAME})

add_executable(dt_objects src/objects_detection_main.cpp)
ament_target_dependencies(dt_objects ${dependencies})
target_link_libraries(dt_objects ${PROJECT_NAME})

add_executable(dt_debug src/debug_main.cpp)
ament_target_dependencies(dt_debug ${dependencies})
target_link_libraries(dt_debug ${PROJECT_NAME})

add_executable(collision_server src/collision_server_main.cpp)
ament_target_dependencies(collision_server ${dependencies})
target_link_libraries(collision_server ${PROJECT_NAME})

add_executable(perception_listener src/test_listener_main.cpp)
ament_target_dependencies(perception_listener ${dependencies})
target_link_libraries(perception_listener ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  dt_people
  dt_objects
  dt_debug
  collision_server
  perception_listener
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/)
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()