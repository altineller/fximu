cmake_minimum_required(VERSION 2.8.3)
project(fximu)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  rosserial_client
  rosserial_tivac
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
