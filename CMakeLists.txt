cmake_minimum_required(VERSION 2.8.3)
project(manipulation_markers)

find_package(catkin REQUIRED COMPONENTS 
    geometry_msgs
    interactive_markers 
    roscpp 
    roslib
    std_msgs
    tf
    visualization_msgs 
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
    FILES
    UpdateMarker.srv
    WalkStart.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    #LIBRARIES manipulation_marker_server
    CATKIN_DEPENDS 
    geometry_msgs
    message_runtime
    interactive_markers 
    roscpp 
    std_msgs
    tf
    visualization_msgs
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(manipulation_markers 
    src/contact_marker.cpp 
    src/manipulation_marker.cpp 
    src/manipulation_marker_server.cpp 
    src/masterarm.cpp
    src/polaris.cpp
    src/wheel.cpp 
)

target_link_libraries(manipulation_markers ${catkin_LIBRARIES})
add_dependencies(manipulation_markers ${PROJECT_NAME}_generate_messages_cpp)
