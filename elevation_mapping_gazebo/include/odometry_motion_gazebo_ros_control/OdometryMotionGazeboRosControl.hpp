/*
 * OdometryMotionGazeboRosControl.hpp
 *
 *  Created on: Sep 13, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <urdf/model.h>

// STD
#include <random>
#include <unordered_map>

// Eigen
#include <Eigen/Core>

// Kindr
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>

namespace gazebo {

struct EnumClassHash
{
  template<typename T>
  std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};

enum class MotionDirection
{
  X,
  Y,
  Z,
  Tilt,
  Yaw
};

typedef kindr::Position3D Position;
typedef kindr::HomogeneousTransformationPosition3RotationQuaternionD Pose;
typedef kindr::TwistLocalD Twist;
typedef kindr::Velocity3D LinearVelocity;
typedef kindr::LocalAngularVelocityPD AngularVelocity;
typedef Eigen::Matrix<double, 3, 3> Covariance;
typedef Eigen::Matrix<double, 6, 6> PoseCovariance;
typedef Eigen::Matrix<double, 4, 4> ReducedCovariance;
typedef Eigen::Matrix<double, 4, 4> Jacobian;
typedef std::unordered_map<MotionDirection, double, EnumClassHash> MotionUncertainty;

/** The GazeboRosControl class interfaces StarlETH locomotion controller with
 Gazebo simulator.
 \brief Gazebo ROS interface to StarlETH locomotion controller.
 */
class OdometryMotionGazeboRosControl : public ModelPlugin
{
 public:
  /// Constructs object
  OdometryMotionGazeboRosControl();
  /// Destructor
  virtual ~OdometryMotionGazeboRosControl();

  /// Implements Gazebo virtual load function
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  /// Overrides Gazebo init function
  virtual void Init();
  /// Overrides Gazebo reset function
  virtual void Reset();

 protected:
  /// Update callback
  void Update();
  /// Reads parameters from the parameter server
  void readParameters();
  /// Reads simulation state
  void readSimulation();
  void simulateMotionInOdom();
  /// Writes simulation state
  void writeSimulation();
  void computeTwistInInertial(const Pose& robotPose, const Twist& twistInBase,
                              Twist & twistInWorld);
  /// Callback for the joint control
  void twistCommandCallback(const geometry_msgs::TwistStamped& twist);
  /// Publishes pose over ROS
  void publishPoses();

  /// ROS node handle.
  std::shared_ptr<ros::NodeHandle> nodeHandle_;
  /// ROS base namespace for this plugin
  std::string namespace_;
  /// ROS robot description parameter name
  std::string robotDescriptionParamName_;
  /// Robot base link
  std::string robotBaseLink_;

  /// TF Publisher.
  tf2_ros::TransformBroadcaster tfBroadcaster_;
  /// ROS pose publishers
  ros::Publisher poseInOdomPublisher_;
  ros::Publisher poseInMapPublisher_;
  std::string poseInOdomPublisherTopic_;
  std::string poseInMapPublisherTopic_;
  /// ROS pose frames
  std::string baseFrameId_;
  std::string odomFrameId_;
  std::string mapFrameId_;
  /// Ignore additive noise for simulated motion.
  bool ignoreNoiseForMotion_;

  /// Model
  physics::ModelPtr model_;
  /// World update event
  event::ConnectionPtr updateConnection_;
  /// Robot base link pose (transforms vectors from base to world).
  Pose robotPoseInOdom_;
  Pose robotPoseInMap_;
  PoseCovariance robotPoseCovariance_;
  /// Robot base link twist in base frame.
  Twist robotTwist_;
  /// ROS twist command subscriber.
  ros::Subscriber twistCommandSubscriber_;
  /// ROS twist subscriber topic name.
  std::string twistCommandTopic_;
  std::string poseInOdomTopic_;
  std::string poseInMapTopic_;
  /// Period for publishing simulation state
  common::Time statePublisherPeriod_;

  MotionUncertainty twistNoiseDenistyParameter_;
  MotionUncertainty currentTwistNoiseDensity_;

  /// Current inter-update simulation time
  double currentUpdateDuration_;
  /// Last Gazebo update time
  common::Time lastUpdateTime_;

  std::default_random_engine randomNumberGenerator_;
};

// This is sovled in newer versions of Gazebo.
namespace math {
  extern const Matrix3 ZERO;
  extern const Matrix3 IDENTITY;
}

}
