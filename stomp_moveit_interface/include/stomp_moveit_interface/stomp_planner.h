/*
 * stomp_planner.h
 *
 *  Created on: Jan 22, 2013
 *      Author: kalakris
 */

#ifndef STOMP_PLANNER_H_
#define STOMP_PLANNER_H_

#include <moveit/planning_interface/planning_interface.h>

namespace stomp_moveit_interface
{

class StompPlanner: public planning_interface::Planner
{
public:
  StompPlanner();
  virtual ~StompPlanner();

  virtual void init(const kinematic_model::KinematicModelConstPtr& model);

  /// Get a short string that identifies the planning interface
  virtual std::string getDescription(void) const { return "STOMP"; }

  /// Get the names of the known planning algorithms (values that can be filled as planner_id in the planning request)
  virtual void getPlanningAlgorithms(std::vector<std::string> &algs) const;

  /// Subclass must implement methods below
  virtual bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const moveit_msgs::MotionPlanRequest &req,
                     moveit_msgs::MotionPlanResponse &res) const;

  virtual bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const moveit_msgs::MotionPlanRequest &req,
                     moveit_msgs::MotionPlanDetailedResponse &res) const;

  /// Determine whether this plugin instance is able to represent this planning request
  virtual bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req)  const;

  /// Request termination, if a solve() function is currently computing plans
  virtual void terminate(void) const;

private:
  ros::NodeHandle node_handle_;

  //boost::shared_ptr<stomp::STOMP> stomp_;
  //std::map<std::string, boost::shared_ptr<StompOptimizationTask> > stomp_tasks_;
  kinematic_model::KinematicModelConstPtr kinematic_model_;

  // distance field params
  double df_size_x_, df_size_y_, df_size_z_;
  double df_resolution_;
  double df_collision_tolerance_;
  double df_max_propagation_distance_;

//  boost::shared_ptr<const collision_detection::CollisionRobot> collision_robot_; /**< standard robot collision checker */
//  boost::shared_ptr<const collision_detection::CollisionWorld> collision_world_; /**< standard robot -> world collision checker */
//  boost::shared_ptr<const collision_detection::CollisionRobotDistanceField> collision_robot_df_;    /**< distance field robot collision checker */
//  boost::shared_ptr<const collision_detection::CollisionWorldDistanceField> collision_world_df_;    /**< distance field robot -> world collision checker */

  void copyObjects(const boost::shared_ptr<const collision_detection::CollisionWorld>& from_world,
                   const boost::shared_ptr<collision_detection::CollisionWorld>& to_world) const;
};

} /* namespace stomp_moveit_interface */
#endif /* STOMP_PLANNER_H_ */
