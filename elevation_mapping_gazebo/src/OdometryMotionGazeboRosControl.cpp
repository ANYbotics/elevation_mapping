/*
 * OdometryMotionGazeboRosControl.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "../include/odometry_motion_gazebo_ros_control/OdometryMotionGazeboRosControl.hpp"

namespace gazebo {

OdometryMotionGazeboRosControl::OdometryMotionGazeboRosControl()
    : twistNoiseDenistyParameter_ { { MotionDirection::X, 0.0 },
          { MotionDirection::Y, 0.0 }, { MotionDirection::Z, 0.0 }, { MotionDirection::Yaw, 0.0 } },
      currentTwistNoiseDensity_(twistNoiseDenistyParameter_)
{
}

OdometryMotionGazeboRosControl::~OdometryMotionGazeboRosControl()
{
}

void OdometryMotionGazeboRosControl::Init()
{
  Reset();
}

void OdometryMotionGazeboRosControl::Reset()
{
  robotLinearVelocity_.Set(0.0, 0.0, 0.0);
  robotAngularVelocity_.Set(0.0, 0.0, 0.0);
  lastUpdateTime_ = common::Time();
  currentUpdateDuration_ = 0.0;
  readSimulation();
  robotPoseInOdom_ = robotPoseInMap_;
}

void OdometryMotionGazeboRosControl::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Compulsory check to see if ROS is correctly initialized.
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM(
        "A ROS node for Gazebo has not been initialized, " "unable to load plugin." << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' " "in the gazebo_ros package)");
    return;
  }

  // Read parameters from the SDF model.
  if (sdf->HasElement("robotNamespace"))
    // Currently unused.
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    namespace_.clear();
  if (sdf->HasElement("robotBaseLink"))
    robotBaseLink_ = sdf->GetElement("robotBaseLink")->Get<std::string>();
  else
    robotBaseLink_ = "base_link";
  if (sdf->HasElement("robotDescription"))
    robotDescriptionParamName_ = sdf->GetElement("robotDescription")->Get<std::string>();
  else
    robotDescriptionParamName_ = "robot_description";
  double statePublisherRate;

  model_ = model;

  // Create ROS node handle.
  nodeHandle_ = std::make_shared<ros::NodeHandle>("~");
  ROS_INFO_NAMED("odometry_motion_gazebo_ros_control", "Starting odometry_motion_gazebo_ros_control plugin in namespace: %s",
                 nodeHandle_->getNamespace().c_str());

  // Read configuration parameters for the plugin.
  readParameters();

  twistCommandSubscriber_ = nodeHandle_->subscribe(
      twistCommandTopic_, 1, &OdometryMotionGazeboRosControl::twistCommandCallback, this);

  poseInOdomPublisher_ = nodeHandle_->advertise<geometry_msgs::PoseWithCovarianceStamped>(poseInOdomTopic_, 1);
  poseInMapPublisher_ = nodeHandle_->advertise<geometry_msgs::PoseWithCovarianceStamped>(poseInMapTopic_, 1);

  // Reset simulation variables.
  Reset();

  // Connect to world updates from Gazebo.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&OdometryMotionGazeboRosControl::Update, this));
}

void OdometryMotionGazeboRosControl::Update()
{
  auto now = model_->GetWorld()->GetSimTime();
  currentUpdateDuration_ = (now - lastUpdateTime_).Double();
  lastUpdateTime_ = now;
  readSimulation();
  simulateMotionInOdom();
  writeSimulation();
  publishPoses();
}

void OdometryMotionGazeboRosControl::readParameters()
{
  nodeHandle_->param<std::string>("frame_ids/base", baseFrameId_, "base_link");
  nodeHandle_->param<std::string>("frame_ids/odometry", odomFrameId_, "odom");
  nodeHandle_->param<std::string>("frame_ids/map", mapFrameId_, "map");
  nodeHandle_->param<std::string>("topics/pose_in_odom", poseInOdomTopic_, "/pose_in_odom");
  nodeHandle_->param<std::string>("topics/pose_in_map", poseInMapTopic_, "/pose_in_map");
  nodeHandle_->param<std::string>("topics/twist_command", twistCommandTopic_, "/twist");
  nodeHandle_->param<std::string>("topics/twist_command", twistCommandTopic_, "/twist");
  nodeHandle_->param("twist_noise_density/linear/x", twistNoiseDenistyParameter_[MotionDirection::X], 0.0); // [m^2/s^2 * s]
  nodeHandle_->param("twist_noise_density/linear/y", twistNoiseDenistyParameter_[MotionDirection::Y], 0.0);
  nodeHandle_->param("twist_noise_density/linear/x", twistNoiseDenistyParameter_[MotionDirection::X], 0.0);
  nodeHandle_->param("twist_noise_density/angular/z", twistNoiseDenistyParameter_[MotionDirection::Yaw], 0.0); // [rad^2/s^2 * s]
}

void OdometryMotionGazeboRosControl::readSimulation()
{
  auto baseLink = model_->GetLink(robotBaseLink_);
  if (!baseLink) {
    ROS_ERROR_STREAM_NAMED("odometry_motion_gazebo_ros_control",
                           "Base link " << robotBaseLink_ << " does not exist in Gazebo.");
    return;
  }
  robotPoseInMap_ = baseLink->GetWorldPose();
}

void OdometryMotionGazeboRosControl::publishPoses()
{
  ros::Time timeStamp(lastUpdateTime_.sec, lastUpdateTime_.nsec);

  if (poseInMapPublisher_.getNumSubscribers() > 0) {
    geometry_msgs::PoseWithCovarianceStamped pose;
    convertPoseToPoseMsg(robotPoseInMap_, pose.pose.pose);
    pose.header.stamp = timeStamp;
    pose.header.frame_id = mapFrameId_;
    poseInMapPublisher_.publish(pose);
  }

  if (poseInMapPublisher_.getNumSubscribers() > 0) {
    geometry_msgs::PoseWithCovarianceStamped pose;
    convertPoseToPoseMsg(robotPoseInOdom_, pose.pose.pose);
    pose.header.stamp = timeStamp;
    pose.header.frame_id = odomFrameId_;
    poseInOdomPublisher_.publish(pose);
  }

  geometry_msgs::TransformStamped transformMapToOdom;
  transformMapToOdom.header.stamp = timeStamp;
  transformMapToOdom.header.frame_id = mapFrameId_;
  transformMapToOdom.child_frame_id = odomFrameId_;
  math::Pose odomPoseInMap = robotPoseInMap_ - robotPoseInOdom_;
  convertPoseToTransformMsg(odomPoseInMap, transformMapToOdom.transform);
  tfBroadcaster_.sendTransform(transformMapToOdom);

  geometry_msgs::TransformStamped transformOdomToBase;
  transformOdomToBase.header.stamp = timeStamp;
  transformOdomToBase.header.frame_id = odomFrameId_;
  transformOdomToBase.child_frame_id = baseFrameId_;
  convertPoseToTransformMsg(robotPoseInOdom_, transformOdomToBase.transform);
  tfBroadcaster_.sendTransform(transformOdomToBase);
}

void OdometryMotionGazeboRosControl::twistCommandCallback(const geometry_msgs::TwistStamped& twist)
{
  if (baseFrameId_ != twist.header.frame_id) {
    ROS_WARN_STREAM("Twist message must be defined in the robot base frame (" << baseFrameId_ << ")!");
    return;
  }
  robotLinearVelocity_.x = twist.twist.linear.x;
  robotLinearVelocity_.y = twist.twist.linear.y;
  robotLinearVelocity_.z = twist.twist.linear.z;
  robotAngularVelocity_.x = twist.twist.angular.x;
  robotAngularVelocity_.y = twist.twist.angular.y;
  robotAngularVelocity_.z = twist.twist.angular.z;
}

void OdometryMotionGazeboRosControl::computeTwistInInertial(
    const math::Pose& robotPose, const math::Vector3& linearVelocityInBase,
    const math::Vector3& angularVelocityInBase, math::Vector3& linearVelocityInWorld,
    math::Vector3& angularVelocityInWorld)
{
  linearVelocityInWorld = robotPose.rot.RotateVector(linearVelocityInBase);
  angularVelocityInWorld = robotPose.rot.RotateVector(angularVelocityInBase);
}

void OdometryMotionGazeboRosControl::simulateMotionInOdom()
{
  // Update current noise density.
  MotionUncertainty standardDeviation;
  for (auto& noiseDensity : currentTwistNoiseDensity_) {
    noiseDensity.second = twistNoiseDenistyParameter_[noiseDensity.first];
    standardDeviation[noiseDensity.first] = sqrt(noiseDensity.second / currentUpdateDuration_);
  }

  // Compute noise.
  math::Vector3 linearVelocityNoise, angularVelocityNoise;
  linearVelocityNoise.x = std::normal_distribution<double>(0.0, standardDeviation[MotionDirection::X])(randomNumberGenerator_);
  linearVelocityNoise.y = std::normal_distribution<double>(0.0, standardDeviation[MotionDirection::Y])(randomNumberGenerator_);
  linearVelocityNoise.z = std::normal_distribution<double>(0.0, standardDeviation[MotionDirection::Z])(randomNumberGenerator_);
  angularVelocityNoise.z = std::normal_distribution<double>(0.0, standardDeviation[MotionDirection::Yaw])(randomNumberGenerator_);

  // Add noise and compute velocity in odom frame.
  math::Vector3 linearVelocityInBase = robotLinearVelocity_ + linearVelocityNoise;
  math::Vector3 linearVelocityInWorld, angularVelocityInWorld;
  computeTwistInInertial(robotPoseInOdom_, linearVelocityInBase, robotAngularVelocity_,
                         linearVelocityInWorld, angularVelocityInWorld);
  angularVelocityInWorld += angularVelocityNoise;

  // Integrate.
  robotPoseInOdom_.pos += currentUpdateDuration_ * linearVelocityInBase;
}

void OdometryMotionGazeboRosControl::writeSimulation()
{
  math::Vector3 linearVelocityInWorld, angularVelocityInWorld;
  computeTwistInInertial(robotPoseInMap_, robotLinearVelocity_, robotAngularVelocity_,
                         linearVelocityInWorld, angularVelocityInWorld);
  model_->SetWorldTwist(linearVelocityInWorld, angularVelocityInWorld);
}

void OdometryMotionGazeboRosControl::convertPoseToPoseMsg(const math::Pose& pose, geometry_msgs::Pose& poseMsg)
{
  poseMsg.position.x = pose.pos[0];
  poseMsg.position.y = pose.pos[1];
  poseMsg.position.z = pose.pos[2];
  poseMsg.orientation.x = pose.rot.x;
  poseMsg.orientation.y = pose.rot.y;
  poseMsg.orientation.z = pose.rot.z;
  poseMsg.orientation.w = pose.rot.w;
}

void OdometryMotionGazeboRosControl::convertPoseToTransformMsg(const math::Pose& pose, geometry_msgs::Transform& transformMsg)
{
  transformMsg.translation.x = pose.pos[0];
  transformMsg.translation.y = pose.pos[1];
  transformMsg.translation.z = pose.pos[2];
  transformMsg.rotation.x = pose.rot.x;
  transformMsg.rotation.y = pose.rot.y;
  transformMsg.rotation.z = pose.rot.z;
  transformMsg.rotation.w = pose.rot.w;
}

GZ_REGISTER_MODEL_PLUGIN(OdometryMotionGazeboRosControl)
}
