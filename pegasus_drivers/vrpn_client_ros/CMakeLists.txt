cmake_minimum_required(VERSION 3.5)
project(vrpn_client_ros)

# Default to C++17 and setup the compiler flags for linux and macOS
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# If we are running on a Mac, then just make sure that the VRPN library is installed via homebrew
if(APPLE)

  if(EXISTS "/usr/local/Cellar/vrpn")
    # 1. Create the VRPN libraries manually
    add_library(libvrpn STATIC IMPORTED)
    set_target_properties(libvrpn PROPERTIES IMPORTED_LOCATION /usr/local/Cellar/vrpn/07.35/lib/libvrpn.a)
    set_target_properties(libvrpn PROPERTIES INTERFACE_INCLUDE_DIRECTORIES /usr/local/Cellar/vrpn/07.35/include)

    add_library(libquat STATIC IMPORTED)
    set_target_properties(libquat PROPERTIES IMPORTED_LOCATION /usr/local/Cellar/vrpn/07.35/lib/libquat.a)
    set_target_properties(libquat PROPERTIES INTERFACE_INCLUDE_DIRECTORIES /usr/local/Cellar/vrpn/07.35/include)
  endif()

# If we are running on a linux PC, then check if the VRPN library is installed on the /usr
# directory. If not, then just install it in the "install/"" directory of ros2 colcon workspace
elseif(UNIX AND NOT APPLE)

  # Check if we already have the VRPN library installed in /usr directory
  if(EXISTS "/usr/lib/libvrpn.a" AND EXISTS "/usr/lib/libquat.a")

    # 1. Create the VRPN libraries manually
    add_library(libvrpn STATIC IMPORTED)
    set_target_properties(libvrpn PROPERTIES IMPORTED_LOCATION /usr/lib/libvrpn.a)
    set_target_properties(libvrpn PROPERTIES INTERFACE_INCLUDE_DIRECTORIES /usr/include)

    add_library(libquat STATIC IMPORTED)
    set_target_properties(libquat PROPERTIES IMPORTED_LOCATION /usr/lib/libquat.a)
    set_target_properties(libquat PROPERTIES INTERFACE_INCLUDE_DIRECTORIES /usr/include)

  else()
    # If the library does not exist, then install vrpn locally
    set(VRPN_LOCAL_INSTALL TRUE)

    # 1. Download the VRPN library and install on the local ROS2 workspace to be linked against by the vrpn_ros_client
    include(ExternalProject)
    ExternalProject_Add(VRPN_LIBRARY
      PREFIX               vrpn_lib
      GIT_REPOSITORY       https://github.com/vrpn/vrpn.git
      GIT_TAG              v07.35
      UPDATE_DISCONNECTED  TRUE
      LOG_CONFIGURE ON
      LOG_INSTALL ON
      LOG_BUILD ON
      LOG_OUTPUT_ON_FAILURE ON
      SOURCE_DIR           ${CMAKE_BINARY_DIR}/vrpn/src
      BINARY_DIR           ${CMAKE_BINARY_DIR}/vrpn/build
      CONFIGURE_COMMAND    cmake -DCMAKE_VERBOSE_MAKEFILE=FALSE ${CMAKE_BINARY_DIR}/vrpn/src
      BUILD_COMMAND        make
      INSTALL_COMMAND      make install DESTDIR=${CMAKE_BINARY_DIR}/vrpn_lib
    )

    ExternalProject_Get_Property(VRPN_LIBRARY INSTALL_DIR)
    set(VRPN_INSTALL_DIR ${INSTALL_DIR})

    # Hack for cmake to not warn us about /include directory not existing yet
    # Check: https://stackoverflow.com/questions/45516209/cmake-how-to-use-interface-include-directories-with-externalproject
    file(MAKE_DIRECTORY ${VRPN_INSTALL_DIR}/usr/local/include)

    # 2. Create the VRPN libraries manually
    add_library(libvrpn STATIC IMPORTED GLOBAL)
    add_dependencies(libvrpn VRPN_LIBRARY)
    set_target_properties(libvrpn PROPERTIES IMPORTED_LOCATION ${VRPN_INSTALL_DIR}/usr/local/lib/libvrpn.a)
    set_target_properties(libvrpn PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${VRPN_INSTALL_DIR}/usr/local/include)

    add_library(libquat STATIC IMPORTED GLOBAL)
    add_dependencies(libquat VRPN_LIBRARY)
    set_target_properties(libquat PROPERTIES IMPORTED_LOCATION ${VRPN_INSTALL_DIR}/usr/local/lib/libquat.a)
    set_target_properties(libquat PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${VRPN_INSTALL_DIR}/usr/local/include)

  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Setup the local ROS2 package
include_directories(include ${CMAKE_BINARY_DIR}/vrpn_lib/usr/local/include)

# Compile the vrpn_client_node
add_executable(vrpn_client_node src/vrpn_client_node.cpp)
target_link_libraries(vrpn_client_node libvrpn libquat)
ament_target_dependencies(vrpn_client_node rclcpp std_msgs geometry_msgs)

# Install the launch files and configurations
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# Install the build targets
install(TARGETS vrpn_client_node DESTINATION lib/${PROJECT_NAME})

ament_package()
