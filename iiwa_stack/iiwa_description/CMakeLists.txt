cmake_minimum_required(VERSION 2.8.3)
project(iiwa_description)

find_package(catkin
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iiwa_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)