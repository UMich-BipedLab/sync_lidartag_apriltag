cmake_minimum_required(VERSION 2.8.3)
project(alignment_msgs)

find_package(
    catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    apriltag_msgs
    lidartag_msgs
)

add_message_files(
    FILES
    TagDetectionAlignment.msg
    TagDetectionAlignmentArray.msg
)

generate_messages(
    DEPENDENCIES std_msgs geometry_msgs sensor_msgs apriltag_msgs lidartag_msgs
)


catkin_package(
    CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

