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
  virtual std::string getDescription(void) const { return "StompPlanner"; }

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

};

} /* namespace stomp_moveit_interface */
#endif /* STOMP_PLANNER_H_ */
