cmake_minimum_required(VERSION 2.8.3)
project(lolo_msgs)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CaptainStatus.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

