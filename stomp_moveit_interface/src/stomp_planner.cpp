/*
 * stomp_planner.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: kalakris
 */

#include <stomp_moveit_interface/stomp_planner.h>

namespace stomp_moveit_interface
{

StompPlanner::StompPlanner()
{
}

StompPlanner::~StompPlanner()
{
}

void StompPlanner::init(const kinematic_model::KinematicModelConstPtr& model) { }

void StompPlanner::getPlanningAlgorithms(std::vector<std::string> &algs) const
{
  algs.clear();
  algs.push_back("STOMP");
  algs.push_back("CHOMP");
}

bool StompPlanner::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   moveit_msgs::MotionPlanResponse &res) const
{
  return true;
}

bool StompPlanner::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   moveit_msgs::MotionPlanDetailedResponse &res) const
{
  return true;
}

bool StompPlanner::canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
{
  return true;
}

void StompPlanner::terminate(void) const
{
}

} /* namespace stomp_moveit_interface */
