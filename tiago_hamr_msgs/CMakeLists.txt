cmake_minimum_required(VERSION 2.8.3)
project(tiago_hamr_msgs)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs geometry_msgs message_generation std_msgs)

add_message_files(
        DIRECTORY msg
        FILES
        testmessage.msg)

add_service_files(
        DIRECTORY srv
        FILES
        testserver.srv)

#add_action_files(
#        FILES
#        test.action)

generate_messages(DEPENDENCIES geometry_msgs std_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs actionlib_msgs)