cmake_minimum_required(VERSION 3.10.2)
project(autopilot_common)

# Default to C++17 and compiler flags to give all warnings
set(CMAKE_CXX_STANDARD 20)
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