cmake_minimum_required(VERSION 3.8)
project(warehouse_robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Order.msg"
  "msg/OrderList.msg"
  "msg/OrderUpdate.msg"
  "msg/RobotStatus.msg"
  "msg/ZoneState.msg"
  "msg/TrafficState.msg"
  "msg/TrafficCommand.msg"
  "msg/WarehouseEvent.msg"
  "msg/EntityPoses.msg"
  "msg/MetricsSnapshot.msg"
  "srv/AssignOrder.srv"
  "srv/RequestResources.srv"
  "srv/ReleaseResources.srv"
  "srv/GenerateOrders.srv"
  "srv/SubmitOrderBatch.srv"
  "srv/SetRate.srv"
  "srv/SimCommand.srv"
  "action/PickPlace.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
