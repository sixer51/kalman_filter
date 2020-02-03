cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  geometry_msgs
  rospy
)