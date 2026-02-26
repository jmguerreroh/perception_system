cmake_minimum_required(VERSION 3.8)
project(perception_system_interfaces)

find_package(ament_cmake REQUIRED)
# find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rosidl_default_generators
  sensor_msgs
  std_msgs
  geometry_msgs
  yolo_msgs
)

set(srv_interfaces
    srv/ExtractNPlanes.srv
    srv/IsolatePCClasses.srv
    srv/IsolatePCBackground.srv
    srv/RemoveDepthClasses.srv
    srv/IsolateDepthClasses.srv
)

set(msg_interfaces
  msg/Detection.msg
  msg/DetectionArray.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${srv_interfaces}
    ${msg_interfaces}
    DEPENDENCIES ${dependencies}
)

ament_export_dependencies(rosidl_default_runtime sensor_msgs std_msgs geometry_msgs yolo_msgs)
ament_export_include_directories(include)
ament_package()
