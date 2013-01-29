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
#include <stomp/stomp_utils.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_fcl/collision_robot.h>
#include <moveit/collision_detection_fcl/collision_world.h>
#include <moveit/collision_distance_field/collision_world_distance_field.h>
#include <moveit/collision_distance_field/collision_robot_distance_field.h>

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
  STOMP_VERIFY(node_handle_.getParam("collision_space/size_x", df_size_x_));
  STOMP_VERIFY(node_handle_.getParam("collision_space/size_y", df_size_y_));
  STOMP_VERIFY(node_handle_.getParam("collision_space/size_z", df_size_z_));
  STOMP_VERIFY(node_handle_.getParam("collision_space/resolution", df_resolution_));
  STOMP_VERIFY(node_handle_.getParam("collision_space/collision_tolerance", df_collision_tolerance_));
  STOMP_VERIFY(node_handle_.getParam("collision_space/max_propagation_distance", df_max_propagation_distance_));
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
  ros::WallTime start_time = ros::WallTime::now();
  boost::shared_ptr<StompOptimizationTask> stomp_task;
  boost::shared_ptr<stomp::STOMP> stomp;

  int max_rollouts;
  STOMP_VERIFY(node_handle_.getParam("max_rollouts", max_rollouts));

  // prepare the collision checkers
  boost::shared_ptr<const collision_detection::CollisionRobot> collision_robot; /**< standard robot collision checker */
  boost::shared_ptr<const collision_detection::CollisionWorld> collision_world; /**< standard robot -> world collision checker */
  boost::shared_ptr<collision_detection::CollisionRobotDistanceField> collision_robot_df;    /**< distance field robot collision checker */
  boost::shared_ptr<collision_detection::CollisionWorldDistanceField> collision_world_df;    /**< distance field robot -> world collision checker */

  collision_robot = planning_scene->getCollisionRobot();
  collision_world = planning_scene->getCollisionWorld();
  std::map<std::string, std::vector<collision_detection::CollisionSphere> > link_body_decompositions;
  bool use_signed_distance_field = true;
  double padding = 0.0;
  double scale = 1.0;
  collision_robot_df.reset(new collision_detection::CollisionRobotDistanceField(kinematic_model_,
                                                                                 link_body_decompositions,
                                                                                 df_size_x_, df_size_y_, df_size_z_,
                                                                                 use_signed_distance_field,
                                                                                 df_resolution_, df_collision_tolerance_,
                                                                                 df_max_propagation_distance_,
                                                                                 padding, scale));
  collision_world_df.reset(new collision_detection::CollisionWorldDistanceField(df_size_x_, df_size_y_, df_size_z_,
                                                                                 use_signed_distance_field,
                                                                                 df_resolution_, df_collision_tolerance_,
                                                                                 df_max_propagation_distance_));
  copyObjects(collision_world, collision_world_df);


  // first setup the task
  stomp_task.reset(new StompOptimizationTask(node_handle_, req.group_name,
                                             kinematic_model_,
                                             collision_robot, collision_world,
                                             collision_robot_df, collision_world_df));

  int num_threads=1;
  STOMP_VERIFY(stomp_task->initialize(num_threads, max_rollouts));

  XmlRpc::XmlRpcValue features_xml;
  STOMP_VERIFY(node_handle_.getParam("features", features_xml));
  stomp_task->setFeaturesFromXml(features_xml);
  stomp_task->setControlCostWeight(0.00001);
  stomp_task->setMotionPlanRequest(planning_scene, req);

  stomp.reset(new stomp::STOMP());
  stomp->initialize(node_handle_, stomp_task);

  // TODO: don't hardcode these params
  bool success = stomp->runUntilValid(200, 10);

  std::vector<Eigen::VectorXd> best_params;
  double best_cost;
  stomp->getBestNoiselessParameters(best_params, best_cost);
  res.trajectory_start = req.start_state;
  res.group_name = req.group_name;
  res.trajectory.resize(1);
  stomp_task->parametersToJointTrajectory(best_params, res.trajectory[0].joint_trajectory);
  res.description.resize(1);
  res.description[0] = "STOMP";
  res.processing_time.resize(1);
  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.processing_time[0] = ros::Duration(wd.sec, wd.nsec);

  if (!success)
  {
    ROS_ERROR("STOMP: failed to find a collision-free plan");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return true;
  }

  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

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

void StompPlanner::copyObjects(const boost::shared_ptr<const collision_detection::CollisionWorld>& from_world,
                 const boost::shared_ptr<collision_detection::CollisionWorld>& to_world) const
{
  std::vector<std::string> object_ids = from_world->getObjectIds();
  for (size_t i=0; i<object_ids.size(); ++i)
  {
    collision_detection::CollisionWorld::ObjectConstPtr obj = from_world->getObject(object_ids[i]);
    to_world->addToObject(object_ids[i], obj->shapes_, obj->shape_poses_);
  }
}

} /* namespace stomp_moveit_interface */
