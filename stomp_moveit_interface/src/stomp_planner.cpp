/*
 * stomp_planner.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: kalakris
 */

#include <ros/ros.h>
#include <stomp/stomp.h>
#include <stomp_moveit_interface/stomp_optimization_task.h>
#include <stomp_moveit_interface/stomp_planner.h>
#include <moveit/collision_detection_fcl/collision_robot.h>
#include <moveit/collision_detection_fcl/collision_world.h>
#include <stomp/stomp_utils.h>

namespace stomp_moveit_interface
{

StompPlanner::StompPlanner():
    node_handle_("~")
{
}

StompPlanner::~StompPlanner()
{
}

void StompPlanner::init(const kinematic_model::KinematicModelConstPtr& model)
{
  kinematic_model_ = model;

  // read distance field params
  double size_x, size_y, size_z;
  bool use_signed_distance_field = true;
  double resolution, collision_tolerance, max_propagation_distance;
  double padding = 0.0;
  double scale = 1.0;

  STOMP_VERIFY(node_handle_.getParam("collision_space/size_x", size_x));
  STOMP_VERIFY(node_handle_.getParam("collision_space/size_y", size_y));
  STOMP_VERIFY(node_handle_.getParam("collision_space/size_z", size_z));
  STOMP_VERIFY(node_handle_.getParam("collision_space/resolution", resolution));
  STOMP_VERIFY(node_handle_.getParam("collision_space/collision_tolerance", collision_tolerance));
  STOMP_VERIFY(node_handle_.getParam("collision_space/max_propagation_distance", max_propagation_distance));

  // init collision checkers
  collision_robot_.reset(new collision_detection::CollisionRobotFCL(model));
  collision_world_.reset(new collision_detection::CollisionWorldFCL());
  std::map<std::string, std::vector<collision_detection::CollisionSphere> > link_body_decompositions;
  collision_robot_df_.reset(new collision_detection::CollisionRobotDistanceField(model,
                                                                                 link_body_decompositions,
                                                                                 size_x, size_y, size_z,
                                                                                 use_signed_distance_field,
                                                                                 resolution, collision_tolerance,
                                                                                 max_propagation_distance,
                                                                                 padding, scale));
  collision_world_df_.reset(new collision_detection::CollisionWorldDistanceField(size_x, size_y, size_z,
                                                                                 use_signed_distance_field,
                                                                                 resolution, collision_tolerance,
                                                                                 max_propagation_distance));
}

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
  ros::WallTime start_time = ros::WallTime::now();
  moveit_msgs::MotionPlanDetailedResponse detailed_res;
  bool success = solve(planning_scene, req, detailed_res);

  // construct the compact response from the detailed one
  res.trajectory_start = detailed_res.trajectory_start;
  res.group_name = detailed_res.group_name;
  res.trajectory = detailed_res.trajectory.back();
  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.planning_time = ros::Duration(wd.sec, wd.nsec);
  res.error_code = detailed_res.error_code;

  return success;
}

bool StompPlanner::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   moveit_msgs::MotionPlanDetailedResponse &res) const
{
  res.trajectory_start = req.start_state;
  res.group_name = req.group_name;

  boost::shared_ptr<StompOptimizationTask> stomp_task;
  boost::shared_ptr<stomp::STOMP> stomp_;

  int max_rollouts;
  STOMP_VERIFY(node_handle_.getParam("max_rollouts", max_rollouts));

  // first setup the task
  stomp_task.reset(new StompOptimizationTask(node_handle_, req.group_name,
                                             kinematic_model_,
                                             collision_robot_, collision_world_,
                                             collision_robot_df_, collision_world_df_));

  int num_threads=1;
  STOMP_VERIFY(stomp_task->initialize(num_threads, max_rollouts));

  // res.trajectory
  // res.description
  // res.processing_time
  // res.error_code

  return true;
}

bool StompPlanner::canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
{
  // check if planner is available
  if (req.planner_id != "STOMP" && req.planner_id != "CHOMP")
  {
    ROS_ERROR("STOMP: Planner %s not available.", req.planner_id.c_str());
    return false;
  }

  // check for single goal region
  if (req.goal_constraints.size() != 1)
  {
    ROS_ERROR("STOMP: Can only handle a single goal region.");
    return false;
  }

  // check that we have only joint constraints at the goal
  if (req.goal_constraints[0].position_constraints.size() > 0
      || req.goal_constraints[0].orientation_constraints.size() > 0
      || req.goal_constraints[0].visibility_constraints.size() > 0
      || req.goal_constraints[0].joint_constraints.size() == 0)
  {
    ROS_ERROR("STOMP: Can only handle joint space goals.");
    return false;
  }

  return true;
}

void StompPlanner::terminate(void) const
{
  ROS_ERROR("STOMP: terminate() not implemented yet!");
}

} /* namespace stomp_moveit_interface */
