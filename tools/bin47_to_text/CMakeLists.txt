cmake_minimum_required(VERSION 2.8.12)
project(bin47_converter)

find_package(catkin REQUIRED COMPONENTS
        rospy)

catkin_package(
        CATKIN_DEPENDS rospy
)
