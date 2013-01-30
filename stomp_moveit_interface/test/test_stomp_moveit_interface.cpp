/*
 * test_stomp_moveit_interface.cpp
 *
 *  Created on: Jan 29, 2013
 *      Author: kalakris
 */

// simple linking test

#include <stomp_moveit_interface/stomp_planner.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_stomp_moveit_interface");
  stomp_moveit_interface::StompPlanner stomp_planner;
}
