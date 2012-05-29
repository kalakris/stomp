/*
 * stomp_optimization_task.cpp
 *
 *  Created on: May 24, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/stomp_optimization_task.h>
#include <usc_utilities/param_server.h>
#include <stomp_ros_interface/collision_feature.h>

namespace stomp_ros_interface
{

StompOptimizationTask::StompOptimizationTask(ros::NodeHandle node_handle):
    node_handle_(node_handle)
{
}

StompOptimizationTask::~StompOptimizationTask()
{
}

bool StompOptimizationTask::initialize(int num_threads)
{
  num_threads_ = num_threads;
  //usc_utilities::read(node_handle_, "num_time_steps", num_time_steps_);
  //usc_utilities::read(node_handle_, "movement_duration", movement_duration_);
  usc_utilities::read(node_handle_, "planning_group", planning_group_);
  usc_utilities::read(node_handle_, "reference_frame", reference_frame_);

  // create the feature set
  feature_set_.reset(new learnable_cost_function::FeatureSet());

  // create features and add them
  boost::shared_ptr<learnable_cost_function::Feature> collision_feature(new CollisionFeature());
  feature_set_->addFeature(collision_feature);

  control_cost_weight_ = 0.0;

  // TODO remove initial value hardcoding here
  feature_weights_=Eigen::VectorXd::Ones(feature_set_->getNumValues());

  // initialize per-thread-data
  per_thread_data_.resize(num_threads);
  double max_radius_clearance = 0.0;
  for (int i=0; i<num_threads; ++i)
  {
    per_thread_data_[i].robot_model_.reset(new StompRobotModel(node_handle_));
    per_thread_data_[i].robot_model_->init(reference_frame_);
    per_thread_data_[i].planning_group_ = per_thread_data_[i].robot_model_->getPlanningGroup(planning_group_);
    num_dimensions_ = per_thread_data_[i].planning_group_->num_joints_;
    max_radius_clearance = per_thread_data_[i].robot_model_->getMaxRadiusClearance();
  }
  collision_space_.reset(new StompCollisionSpace(node_handle_));
  collision_space_->init(max_radius_clearance, reference_frame_);

  return true;
}

bool StompOptimizationTask::execute(std::vector<Eigen::VectorXd>& parameters,
                     Eigen::VectorXd& costs,
                     Eigen::MatrixXd& weighted_feature_values,
                     const int iteration_number,
                     const int rollout_number,
                     int thread_id)
{
  computeFeatures(parameters, per_thread_data_[thread_id].features_, thread_id);
  computeCosts(per_thread_data_[thread_id].features_, costs, weighted_feature_values);

  // copying it to per_rollout_data
  PerThreadData* rdata = &noiseless_rollout_data_;
  if (rollout_number >= 0)
  {
    if ((int)noisy_rollout_data_.size() <= rollout_number)
    {
      noisy_rollout_data_.resize(rollout_number+1);
    }
    rdata = &(noisy_rollout_data_[rollout_number]);
  }
  *rdata = per_thread_data_[thread_id];
  // duplicate the cost function inputs
  for (unsigned int i=0; i<rdata->cost_function_input_.size(); ++i)
  {
    boost::shared_ptr<StompCostFunctionInput> input(new StompCostFunctionInput(
        collision_space_, rdata->robot_model_, rdata->planning_group_));
    *input = *rdata->cost_function_input_[i];
    rdata->cost_function_input_[i] = input;
  }
  return true;
}


void StompOptimizationTask::computeFeatures(std::vector<Eigen::VectorXd>& parameters,
                     Eigen::MatrixXd& features,
                     int thread_id)
{
  // prepare the cost function input
  std::vector<double> temp_features(feature_set_->getNumValues());
  std::vector<Eigen::VectorXd> temp_gradients(feature_set_->getNumValues());

  bool state_validity;
  for (int t=0; t<num_time_steps_; ++t)
  {
    for (int d=0; d<num_dimensions_; ++d)
    {
      per_thread_data_[thread_id].cost_function_input_[t]->joint_angles_(d) = parameters[d](t);
    }
    per_thread_data_[thread_id].cost_function_input_[t]->doFK(per_thread_data_[thread_id].planning_group_->fk_solver_);

    feature_set_->computeValuesAndGradients(per_thread_data_[thread_id].cost_function_input_[t],
                                            temp_features, false, temp_gradients, state_validity);
    for (unsigned int f=0; f<temp_features.size(); ++f)
    {
      features(t,f) = temp_features[f];
    }
  }
}

void StompOptimizationTask::computeCosts(const Eigen::MatrixXd& features, Eigen::VectorXd& costs, Eigen::MatrixXd& weighted_feature_values) const
{
  weighted_feature_values = features; // just to initialize the size
  for (int t=0; t<num_time_steps_; ++t)
  {
    weighted_feature_values.row(t) = (features.row(t).array() * feature_weights_.array().transpose()).matrix();
  }
  costs = weighted_feature_values.rowwise().sum();
}


bool StompOptimizationTask::getPolicy(boost::shared_ptr<stomp::CovariantMovementPrimitive>& policy)
{
  policy = policy_;
  return true;
}

bool StompOptimizationTask::setPolicy(const boost::shared_ptr<stomp::CovariantMovementPrimitive> policy)
{
  policy_ = policy;
  return true;
}

double StompOptimizationTask::getControlCostWeight()
{
  return control_cost_weight_;
}

void StompOptimizationTask::setPlanningScene(const arm_navigation_msgs::PlanningScene& scene)
{
  collision_space_->setPlanningScene(scene);
}

void StompOptimizationTask::setMotionPlanRequest(const arm_navigation_msgs::MotionPlanRequest& request)
{
  // TODO this is obviously not complete

  // get the start and goal positions from the message
  std::vector<double> start(num_dimensions_, 0.0);
  std::vector<double> goal(num_dimensions_, 0.0);

  const StompRobotModel::StompPlanningGroup* group = per_thread_data_[0].planning_group_;
  start = group->getJointArrayFromJointState(request.start_state.joint_state);
  goal = group->getJointArrayFromGoalConstraints(request.goal_constraints);

//  ROS_INFO("start, goal:");
//  for (int i=0; i<num_dimensions_; ++i)
//  {
//    ROS_INFO("%f -> %f", start[i], goal[i]);
//  }

  movement_duration_ = request.expected_path_duration.toSec();
  num_time_steps_ = lrint(movement_duration_/request.expected_path_dt.toSec()) + 1;
  if (num_time_steps_ < 10)
    num_time_steps_ = 10; // may be a hack, but we need some room to plan in :)
  if (num_time_steps_ > 500)
    num_time_steps_ = 500; // may start to get too slow / run out of memory at this point

  for (int i=0; i<num_threads_; ++i)
  {
    per_thread_data_[i].cost_function_input_.resize(num_time_steps_);
    for (int t=0; t<num_time_steps_; ++t)
    {
      per_thread_data_[i].cost_function_input_[t].reset(new StompCostFunctionInput(
          collision_space_, per_thread_data_[i].robot_model_, per_thread_data_[i].planning_group_));
    }
    per_thread_data_[i].features_ = Eigen::MatrixXd(num_time_steps_, feature_set_->getNumValues());
  }

  // create the derivative costs
  std::vector<Eigen::MatrixXd> derivative_costs(num_dimensions_,
                                                Eigen::MatrixXd(num_time_steps_ + 2*stomp::TRAJECTORY_PADDING, stomp::NUM_DIFF_RULES));
  std::vector<Eigen::VectorXd> initial_trajectory(num_dimensions_,
                                                  Eigen::VectorXd::Zero(num_time_steps_ + 2*stomp::TRAJECTORY_PADDING));

  for (int d=0; d<num_dimensions_; ++d)
  {
    derivative_costs[d].col(stomp::STOMP_ACCELERATION) = Eigen::VectorXd::Ones(num_time_steps_ + 2*stomp::TRAJECTORY_PADDING);
    initial_trajectory[d] = Eigen::VectorXd::Zero(num_time_steps_ + 2*stomp::TRAJECTORY_PADDING);
    initial_trajectory[d].head(stomp::TRAJECTORY_PADDING) = Eigen::VectorXd::Ones(stomp::TRAJECTORY_PADDING) * start[d];
    initial_trajectory[d].tail(stomp::TRAJECTORY_PADDING) = Eigen::VectorXd::Ones(stomp::TRAJECTORY_PADDING) * goal[d];
  }

  policy_.reset(new stomp::CovariantMovementPrimitive());
  policy_->initialize(num_time_steps_,
                      num_dimensions_,
                      movement_duration_,
                      derivative_costs,
                      initial_trajectory);
  policy_->setToMinControlCost();
  policy_->writeToFile(std::string("/tmp/test.txt"));

}

void StompOptimizationTask::setFeatureWeights(std::vector<double> weights)
{
  ROS_ASSERT((int)weights.size() == feature_set_->getNumValues());
  feature_weights_ = Eigen::VectorXd::Zero(weights.size());
  for (unsigned int i=0; i<weights.size(); ++i)
  {
    feature_weights_(i) = weights[i];
  }
}

void StompOptimizationTask::getRolloutData(PerThreadData& noiseless_rollout, std::vector<PerThreadData>& noisy_rollouts)
{
  noiseless_rollout = noiseless_rollout_data_;
  noisy_rollouts = noisy_rollout_data_;
}

void StompOptimizationTask::publishTrajectoryMarkers(ros::Publisher& viz_pub)
{
  noiseless_rollout_data_.publishMarkers(viz_pub, 0, true);
  for (unsigned int i=0; i<noisy_rollout_data_.size(); ++i)
  {
    noisy_rollout_data_[i].publishMarkers(viz_pub, i, false);
  }
}

void StompOptimizationTask::PerThreadData::publishMarkers(ros::Publisher& viz_pub, int id, bool noiseless)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = robot_model_->getReferenceFrame();
  marker.header.stamp = ros::Time();
  marker.ns = noiseless ? "noiseless":"noisy";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points.resize(cost_function_input_.size());
  marker.colors.resize(cost_function_input_.size());
  for (unsigned int t=0; t<cost_function_input_.size(); ++t)
  {
    KDL::Frame& v = cost_function_input_[t]->segment_frames_[planning_group_->end_effector_segment_index_];
    marker.points[t].x = v.p.x();
    marker.points[t].y = v.p.y();
    marker.points[t].z = v.p.z();
    marker.colors[t].a = noiseless ? 1.0 : 0.5;
    marker.colors[t].r = 0.0;
    marker.colors[t].g = 0.0;
    marker.colors[t].b = 1.0;
  }
  marker.scale.x = noiseless ? 0.02 : 0.01;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  viz_pub.publish(marker);
}

} /* namespace stomp_ros_interface */
