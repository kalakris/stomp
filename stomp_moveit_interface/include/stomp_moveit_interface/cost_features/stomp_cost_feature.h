/*
 * stomp_cost_feature.h
 *
 *  Created on: Aug 31, 2012
 *      Author: kalakris
 */

#ifndef STOMP_COST_FEATURE_H_
#define STOMP_COST_FEATURE_H_

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_distance_field/collision_world_distance_field.h>
#include <moveit/collision_distance_field/collision_robot_distance_field.h>
#include <stomp_moveit_interface/stomp_trajectory.h>

namespace stomp_moveit_interface
{

class StompCostFeature
{
public:
  StompCostFeature(){};
  virtual ~StompCostFeature(){};

  bool initialize(XmlRpc::XmlRpcValue& config,
                  boost::shared_ptr<collision_detection::CollisionRobot> collision_robot,
                  boost::shared_ptr<collision_detection::CollisionWorld> collision_world,
                  boost::shared_ptr<collision_detection::CollisionRobotDistanceField> collision_robot_df,
                  boost::shared_ptr<collision_detection::CollisionWorldDistanceField> collision_world_df);

  virtual int getNumValues() const = 0;
  virtual void computeValuesAndGradients(const boost::shared_ptr<StompTrajectory const>& trajectory,
                                         Eigen::MatrixXd& feature_values,         // num_features x num_time_steps
                                         bool compute_gradients,
                                         std::vector<Eigen::MatrixXd>& gradients, // [num_features] num_joints x num_time_steps
                                         bool& state_validity) const = 0;
  virtual std::string getName() const = 0;
  virtual void getNames(std::vector<std::string>& names) const;

protected:
  virtual bool initialize(XmlRpc::XmlRpcValue& config)=0;

  boost::shared_ptr<collision_detection::CollisionRobot> collision_robot_; /**< standard robot collision checker */
  boost::shared_ptr<collision_detection::CollisionWorld> collision_world_; /**< standard robot -> world collision checker */
  boost::shared_ptr<collision_detection::CollisionRobotDistanceField> collision_robot_df_;    /**< distance field robot collision checker */
  boost::shared_ptr<collision_detection::CollisionWorldDistanceField> collision_world_df_;    /**< distance field robot -> world collision checker */

};

} /* namespace stomp_moveit_interface */
#endif /* STOMP_COST_FEATURE_H_ */
