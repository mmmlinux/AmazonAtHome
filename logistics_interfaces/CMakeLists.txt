cmake_minimum_required(VERSION 3.8)
project(logistics_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/TaskStatus.msg"
  "msg/SlotStatus.msg"
  "msg/WarehouseState.msg"
  "msg/ChargerStatus.msg"
  "msg/ChargerState.msg"
  "srv/SubmitTask.srv"
  "srv/SetSlotOccupancy.srv"
  "srv/AcquireWaypoint.srv"
  "srv/ReleaseWaypoint.srv"
  "srv/ChargerControl.srv"
  "action/LogisticsTask.action"
  "action/MoveToWaypoint.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
