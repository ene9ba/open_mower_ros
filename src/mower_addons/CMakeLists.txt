cmake_minimum_required(VERSION 3.0.2)
project(mower_addons)




#
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
 

)



## Generate messages in the 'msg' folder
add_message_files(
   FILES
   raspi_inf.msg


 )

## Generate services in the 'srv' folder
#add_service_files(
#   FILES
#   AddTwoInts.srv
#   Service2.srv
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   #raspi_inf.msg
 ) 


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)




add_executable(mower_inf src/mower_machine_inf.cpp)
target_link_libraries(mower_inf ${catkin_LIBRARIES})
add_dependencies(mower_inf mower_addons_generate_messages_cpp)

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener mower_addons_generate_messages_cpp)

# Wifiscan build static library
add_library(MyLibWiFiScan src/wifi_scan.c )
# Wifiscan link above build lib to node
target_link_libraries(mower_inf MyLibWiFiScan )
# Wifiscan neccessary libs must be installed at system

# The library above depends on [libmnl](http://www.netfilter.org/projects/libmnl/) 
# for netlink nl80211 user space - kernel space communication.
target_link_libraries(MyLibWiFiScan mnl)
