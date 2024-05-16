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
cmake_minimum_required(VERSION 3.10.2)
project(pegasus_utils)

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
find_package(Eigen3 REQUIRED)

# Create the package as a header-only library
add_library(${PROJECT_NAME} INTERFACE)

# Define the include directories here
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
)

# TODO - Add necessary linking libraries here if needed
#target_link_libraries(${PROJECT_NAME} INTERFACE OpenSSL::SSL)

install(TARGETS ${PROJECT_NAME}
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Inform ament compile system that this ROS2 package depends on Eigen3
ament_export_dependencies(
  Eigen3
)

# Specify the export targets for the header only libraries
ament_export_targets("export_${PROJECT_NAME}")
ament_package()