find_package(catkin REQUIRED)

# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(using_markers_omega)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs)

# Declare our catkin package.
catkin_package()

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

link_directories(/home/olmo/jade_workspace/src/using_markers_omega/libraries)  
add_executable(basic_shapes_omega_exe basic_shapes_omega.cpp)
target_link_libraries(basic_shapes_omega_exe dhd usb-1.0)
target_link_libraries(basic_shapes_omega_exe ${catkin_LIBRARIES})








