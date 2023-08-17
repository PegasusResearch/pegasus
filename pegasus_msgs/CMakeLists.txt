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
  
  # Driver Services
  "srv/driver/KillSwitch.srv"
  "srv/driver/Arm.srv"
  "srv/driver/Land.srv"
  "srv/driver/Offboard.srv"
  "srv/driver/PositionHold.srv"
  "srv/driver/Waypoint.srv"

  # Autopilot services
  "srv/autopilot/SetMode.srv"
  
  # Paths services/messages
  "srv/paths/Speed.msg"
  "srv/paths/AddWaypoint.srv"
  "srv/paths/AddArc.srv"
  "srv/paths/AddLine.srv"
  "srv/paths/AddCircle.srv"
  "srv/paths/AddLemniscate.srv"
  "srv/paths/ResetPath.srv"
  
  # Controllers services/messages
  "srv/controllers/StartMission.srv"
  "srv/controllers/LandMission.srv"

  # Messages
  "msg/autopilot/AutopilotStatus.msg"
  "msg/driver/Battery.msg"
  "msg/driver/ControlAttitude.msg"
  "msg/driver/ControlPosition.msg"
  "msg/driver/Health.msg"
  "msg/driver/RcStatus.msg"
  "msg/driver/RPY.msg"
  "msg/driver/SensorBarometer.msg"
  "msg/driver/SensorGps.msg"
  "msg/driver/SensorGpsInfo.msg"
  "msg/driver/Status.msg"
  
  # High level control
  "msg/paths/Path.msg"
  "msg/pid/PidStatistic.msg"
  "msg/pid/PidStatistics.msg"
  
  # Depedencies for generating the messages, services and actions
  DEPENDENCIES 
  builtin_interfaces 
  std_msgs
  geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()