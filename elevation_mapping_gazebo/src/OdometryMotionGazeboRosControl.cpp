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
  lastUpdateTime_ = common::Time();
  currentUpdateDuration_ = 0.0;
  robotPoseCovariance_.setZero();
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
  nodeHandle_->param("twist_noise_density/linear/z", twistNoiseDenistyParameter_[MotionDirection::Z], 0.0);
  nodeHandle_->param("twist_noise_density/angular/z", twistNoiseDenistyParameter_[MotionDirection::Yaw], 0.0); // [rad^2/s^2 * s]
  nodeHandle_->param("ignore_noise_for_motion", ignoreNoiseForMotion_, false);
}

void OdometryMotionGazeboRosControl::readSimulation()
{
  auto baseLink = model_->GetLink(robotBaseLink_);
  if (!baseLink) {
    ROS_ERROR_STREAM_NAMED("odometry_motion_gazebo_ros_control",
                           "Base link " << robotBaseLink_ << " does not exist in Gazebo.");
    return;
  }
  robotPoseInMap_.getPosition().x() = baseLink->GetWorldPose().pos.x;
  robotPoseInMap_.getPosition().y() = baseLink->GetWorldPose().pos.y;
  robotPoseInMap_.getPosition().z() = baseLink->GetWorldPose().pos.z;
  robotPoseInMap_.getRotation().x() = baseLink->GetWorldPose().rot.x;
  robotPoseInMap_.getRotation().y() = baseLink->GetWorldPose().rot.y;
  robotPoseInMap_.getRotation().z() = baseLink->GetWorldPose().rot.z;
  robotPoseInMap_.getRotation().w() = baseLink->GetWorldPose().rot.w;
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
  Twist twistNoise;
  twistNoise.getTranslationalVelocity().x() = std::normal_distribution<double>(0.0, standardDeviation[MotionDirection::X])(randomNumberGenerator_);
  twistNoise.getTranslationalVelocity().y() = std::normal_distribution<double>(0.0, standardDeviation[MotionDirection::Y])(randomNumberGenerator_);
  twistNoise.getTranslationalVelocity().z() = std::normal_distribution<double>(0.0, standardDeviation[MotionDirection::Z])(randomNumberGenerator_);
  twistNoise.getRotationalVelocity().z() = std::normal_distribution<double>(0.0, standardDeviation[MotionDirection::Yaw])(randomNumberGenerator_);

  // Add noise and compute velocity in odom frame.
  Twist twistInBase;
  twistInBase.getTranslationalVelocity() = robotTwist_.getTranslationalVelocity();
  if (!ignoreNoiseForMotion_) twistInBase.getTranslationalVelocity() += twistNoise.getTranslationalVelocity();
  twistInBase.getRotationalVelocity() = robotTwist_.getRotationalVelocity();
  Twist twistInWorld;
  computeTwistInInertial(robotPoseInOdom_, twistInBase, twistInWorld);
  if (!ignoreNoiseForMotion_) twistInWorld.getRotationalVelocity() += twistNoise.getRotationalVelocity();

  // Compute pose through integration.
  robotPoseInOdom_.getPosition() += Position(currentUpdateDuration_ * twistInWorld.getTranslationalVelocity());
  robotPoseInOdom_.getRotation() = robotPoseInOdom_.getRotation().boxPlus(currentUpdateDuration_ * twistInWorld.getRotationalVelocity().vector());

  // Rotation matrix of z-align frame R_I_tilde_B.
  const double yawRotationAngle = kindr::RotationVectorPD(robotPoseInOdom_.getRotation()).z();
  const kindr::RotationMatrixPD transformBaseToOdom(kindr::RotationVectorPD(Eigen::Vector3d::UnitZ() * yawRotationAngle));

  // Compute reduced pose covariance.
  // Density to covariance is computer at later step below.
  const Eigen::Vector4d reducedPoseCovarianceDiagonal(
      currentTwistNoiseDensity_[MotionDirection::X],
      currentTwistNoiseDensity_[MotionDirection::Y],
      currentTwistNoiseDensity_[MotionDirection::Z],
      currentTwistNoiseDensity_[MotionDirection::Yaw]);
  const ReducedCovariance reducedPoseCovariance(reducedPoseCovarianceDiagonal.asDiagonal());
  Jacobian jacobian = Jacobian::Zero();
  jacobian.topLeftCorner(3, 3) = transformBaseToOdom.matrix();
  jacobian(3, 3) = 1.0;
  const ReducedCovariance reducedPoseCovarianceUpdate(
      currentUpdateDuration_ * jacobian * reducedPoseCovariance * jacobian.transpose());

  // Full pose covariance.
  robotPoseCovariance_.topLeftCorner(3, 3) += reducedPoseCovarianceUpdate.topLeftCorner(3, 3);
  robotPoseCovariance_(5, 5) += reducedPoseCovarianceUpdate(3, 3);
}

void OdometryMotionGazeboRosControl::writeSimulation()
{
  Twist twistInWorld;
  computeTwistInInertial(robotPoseInMap_, robotTwist_, twistInWorld);
  math::Vector3 linearVelocityInWorld, angularVelocityInWorld;
  linearVelocityInWorld.x = twistInWorld.getTranslationalVelocity().x();
  linearVelocityInWorld.y = twistInWorld.getTranslationalVelocity().y();
  linearVelocityInWorld.z = twistInWorld.getTranslationalVelocity().z();
  angularVelocityInWorld.x = twistInWorld.getRotationalVelocity().x();
  angularVelocityInWorld.y = twistInWorld.getRotationalVelocity().y();
  angularVelocityInWorld.z = twistInWorld.getRotationalVelocity().z();
  model_->SetWorldTwist(linearVelocityInWorld, angularVelocityInWorld);
}

void OdometryMotionGazeboRosControl::computeTwistInInertial(const Pose& robotPose,
                                                            const Twist& twistInBase,
                                                            Twist & twistInWorld)
{
  twistInWorld.getTranslationalVelocity() = robotPose.getRotation().rotate(twistInBase.getTranslationalVelocity());
  twistInWorld.getRotationalVelocity() = robotPose.getRotation().rotate(twistInBase.getRotationalVelocity());
}

void OdometryMotionGazeboRosControl::twistCommandCallback(const geometry_msgs::TwistStamped& twist)
{
  if (baseFrameId_ != twist.header.frame_id) {
    ROS_WARN_STREAM("Twist message must be defined in the robot base frame (" << baseFrameId_ << ")!");
    return;
  }
  kindr_ros::convertFromRosGeometryMsg(twist.twist, robotTwist_);
}

void OdometryMotionGazeboRosControl::publishPoses()
{
  ros::Time timeStamp(lastUpdateTime_.sec, lastUpdateTime_.nsec);

  if (poseInMapPublisher_.getNumSubscribers() > 0) {
    geometry_msgs::PoseWithCovarianceStamped pose;
    kindr_ros::convertToRosGeometryMsg(robotPoseInMap_, pose.pose.pose);
    pose.header.stamp = timeStamp;
    pose.header.frame_id = mapFrameId_;
    poseInMapPublisher_.publish(pose);
  }

  if (poseInOdomPublisher_.getNumSubscribers() > 0) {
    geometry_msgs::PoseWithCovarianceStamped pose;
    kindr_ros::convertToRosGeometryMsg(robotPoseInOdom_, pose.pose.pose);
    pose.header.stamp = timeStamp;
    pose.header.frame_id = odomFrameId_;
    std::copy(robotPoseCovariance_.data(), robotPoseCovariance_.data() + 36, pose.pose.covariance.begin());
    poseInOdomPublisher_.publish(pose);
  }

  geometry_msgs::TransformStamped transformMapToOdom;
  transformMapToOdom.header.stamp = timeStamp;
  transformMapToOdom.header.frame_id = mapFrameId_;
  transformMapToOdom.child_frame_id = odomFrameId_;
  const Pose robotPoseInOdomInverse(
      robotPoseInOdom_.getRotation().inverseRotate(-robotPoseInOdom_.getPosition()),
      robotPoseInOdom_.getRotation().inverted());
  Pose odomPoseInMap = robotPoseInMap_ * robotPoseInOdomInverse;
  kindr_ros::convertToRosGeometryMsg(odomPoseInMap, transformMapToOdom.transform);
  tfBroadcaster_.sendTransform(transformMapToOdom);

  geometry_msgs::TransformStamped transformOdomToBase;
  transformOdomToBase.header.stamp = timeStamp;
  transformOdomToBase.header.frame_id = odomFrameId_;
  transformOdomToBase.child_frame_id = baseFrameId_;
  kindr_ros::convertToRosGeometryMsg(robotPoseInOdom_, transformOdomToBase.transform);
  tfBroadcaster_.sendTransform(transformOdomToBase);
}

GZ_REGISTER_MODEL_PLUGIN(OdometryMotionGazeboRosControl)
}
