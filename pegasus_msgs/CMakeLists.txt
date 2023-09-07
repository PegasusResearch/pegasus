##################################################################################
#
#   Copyright (C) 2023 Marcelo Jacinto. All rights reserved.
#   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name Pegasus nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#################################################################################
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
  "srv/driver/Takeoff.srv"

  # Autopilot services
  "srv/autopilot/SetMode.srv"
  
  # Paths services/messages
  "srv/trajectories/Speed.msg"
  "srv/trajectories/AddArc.srv"
  "srv/trajectories/AddLine.srv"
  "srv/trajectories/AddCircle.srv"
  "srv/trajectories/AddLemniscate.srv"
  "srv/trajectories/ResetPath.srv"
  
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
  "msg/driver/ThrustCurve.msg"
  "msg/driver/VehicleConstants.msg"
  
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