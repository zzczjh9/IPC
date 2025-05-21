cmake_minimum_required(VERSION 3.0.2)
project(balance_mpc)

# Use C++11
add_compile_options(-std=c++11)

# Find catkin and required ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Generate messages if any
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    sensor_msgs 
    geometry_msgs
  DEPENDS 
    EIGEN3
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Declare the library
add_library(${PROJECT_NAME}
  src/balance_controller.cpp
  src/motor_driver.cpp
)

# Link the library
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  wiringPi
)

# Declare the executable
add_executable(balance_node src/balance_node.cpp)

# Link the executable
target_link_libraries(balance_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  wiringPi
) 