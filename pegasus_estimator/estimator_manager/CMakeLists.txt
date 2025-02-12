##################################################################################
#   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
#   Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions 
# are met:
#
# 1. Redistributions of source code must retain the above copyright 
# notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright 
# notice, this list of conditions and the following disclaimer in 
# the documentation and/or other materials provided with the distribution.
# 3. All advertising materials mentioning features or use of this 
# software must display the following acknowledgement: This product 
# includes software developed by Project Pegasus.
# 4. Neither the name of the copyright holder nor the names of its 
# contributors may be used to endorse or promote products derived 
# from this software without specific prior written permission.
#
# Additional Restrictions:
# 4. The Software shall be used for non-commercial purposes only. 
# This includes, but is not limited to, academic research, personal 
# projects, and non-profit organizations. Any commercial use of the 
# Software is strictly prohibited without prior written permission 
# from the copyright holders.
# 5. The Software shall not be used, directly or indirectly, for 
# military purposes, including but not limited to the development 
# of weapons, military simulations, or any other military applications. 
# Any military use of the Software is strictly prohibited without 
# prior written permission from the copyright holders.
# 6. The Software may be utilized for academic research purposes, 
# with the condition that proper acknowledgment is given in all 
# corresponding publications.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##################################################################################
cmake_minimum_required(VERSION 3.8)
project(estimator_manager)

# Default to C++20 and compiler flags to give all warnings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-sign-compare -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pegasus_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  "pluginlib"
  nav_msgs
  pegasus_msgs
)

# Define the executable
add_executable(${PROJECT_NAME} 
  # The main ROS2 node implementation
  src/estimator_manager.cpp
  src/main.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

add_definitions(${EIGEN3_DEFINITIONS})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Specify where to install the library
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

# Specify where to install the launch and configuration files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_package()
