cmake_minimum_required(VERSION 3.10.2)
project(pegasus_msgs)

# Default to C++17 and compiler flags to give all warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()
if(MSVC)
  add_compile_options(-WX -W2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install the header in the include folder, necessary for the actions to work in Windows properly
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/
)

# Services, messages and actions to generate
rosidl_generate_interfaces(${PROJECT_NAME}
  # Actions
  "action/VehicleTakeOff.action"
  "action/VehicleLand.action"
  "action/VehicleArm.action"
  # Services
  "srv/KillSwitch.srv"
  "srv/Arm.srv"
  "srv/Land.srv"
  "srv/Reboot.srv"
  "srv/ThrustCurve.srv"
  # Paths services/messages
  "srv/paths/Speed.msg"
  "srv/paths/AddWaypoint.srv"
  "srv/paths/AddArc.srv"
  "srv/paths/AddLine.srv"
  "srv/paths/AddCircle.srv"
  "srv/paths/AddLemniscate.srv"
  "srv/paths/ResetPath.srv"
  # Messages
  "msg/Status.msg"
  "msg/Health.msg"
  "msg/BodyVelocityControl.msg"
  "msg/AttitudeThrustControl.msg"
  "msg/AttitudeRateThrustControl.msg"
  "msg/ActuatorControl.msg"
  "msg/PositionControl.msg"
  "msg/InertialAccelerationControl.msg"
  "msg/RPY.msg"
  "msg/State.msg"
  "msg/Path.msg"
  "msg/PidStatistic.msg"
  "msg/PidStatistics.msg"
  # Depedencies for generating the messages, services and actions
  DEPENDENCIES 
  builtin_interfaces 
  std_msgs
  geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()