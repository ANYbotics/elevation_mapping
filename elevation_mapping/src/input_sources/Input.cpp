/*
 *  Input.cpp
 *
 *  Created on: Oct 06, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#include <memory>

#include "elevation_mapping/input_sources/Input.hpp"

#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

namespace elevation_mapping {

Input::Input(ros::NodeHandle nh) : nodeHandle_(nh) {}

bool Input::configure(std::string name, const XmlRpc::XmlRpcValue& configuration,
                      const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters) {
  // Configuration Guards.
  if (configuration.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR(
        "Input source must be specified as map, but is "
        "XmlRpcType:%d.",
        configuration.getType());
    return false;
  }

  Parameters parameters;

  // Check Optional enabled parameter.
  if (configuration.hasMember("enabled")) {
    if (configuration["enabled"].getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
      ROS_ERROR(
          "Could not configure input source %s because parameter 'enabled' has the "
          "wrong type.",
          name.c_str());
      return false;
    }

    parameters.isEnabled_ = static_cast<bool>(configuration["enabled"]);
  }

  // Check that configuration exist and has an appropriate type.
  using nameAndType = std::pair<std::string, XmlRpc::XmlRpcValue::Type>;
  for (const nameAndType& member : std::vector<nameAndType>{{"type", XmlRpc::XmlRpcValue::TypeString},
                                                            {"topic", XmlRpc::XmlRpcValue::TypeString},
                                                            {"queue_size", XmlRpc::XmlRpcValue::TypeInt},
                                                            {"publish_on_update", XmlRpc::XmlRpcValue::TypeBoolean},
                                                            {"sensor_processor", XmlRpc::XmlRpcValue::TypeStruct}}) {
    if (!configuration.hasMember(member.first)) {
      ROS_ERROR("Could not configure input source %s because no %s was given.", name.c_str(), member.first.c_str());
      return false;
    }
    if (configuration[member.first].getType() != member.second) {
      ROS_ERROR(
          "Could not configure input source %s because member %s has the "
          "wrong type.",
          name.c_str(), member.first.c_str());
      return false;
    }
  }

  parameters.name_ = name;
  parameters.type_ = static_cast<std::string>(configuration["type"]);
  parameters.topic_ = static_cast<std::string>(configuration["topic"]);
  const int& queueSize = static_cast<int>(configuration["queue_size"]);
  if (queueSize >= 0) {
    parameters.queueSize_ = static_cast<unsigned int>(queueSize);
  } else {
    ROS_ERROR("The specified queue_size is negative.");
    return false;
  }
  parameters.publishOnUpdate_ = static_cast<bool>(configuration["publish_on_update"]);

  parameters_.setData(parameters);

  // SensorProcessor
  if (!configureSensorProcessor(name, configuration, generalSensorProcessorParameters)) {
    return false;
  }

  ROS_DEBUG("Configured %s:%s @ %s (publishing_on_update: %s), using %s to process data.\n", parameters.type_.c_str(),
            parameters.name_.c_str(), nodeHandle_.resolveName(parameters.topic_).c_str(), parameters.publishOnUpdate_ ? "true" : "false",
            static_cast<std::string>(configuration["sensor_processor"]["type"]).c_str());
  return true;
}

std::string Input::getSubscribedTopic() const {
  const Parameters parameters{parameters_.getData()};
  return nodeHandle_.resolveName(parameters.topic_);
}

bool Input::configureSensorProcessor(std::string name, const XmlRpc::XmlRpcValue& parameters,
                                     const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters) {
  if (!parameters["sensor_processor"].hasMember("type")) {
    ROS_ERROR("Could not configure sensor processor of input source %s because no type was given.", name.c_str());
    return false;
  }
  if (parameters["sensor_processor"]["type"].getType() != XmlRpc::XmlRpcValue::TypeString) {
    ROS_ERROR(
        "Could not configure sensor processor of input source %s because the member 'type' has the "
        "wrong type.",
        name.c_str());
    return false;
  }
  std::string sensorType = static_cast<std::string>(parameters["sensor_processor"]["type"]);
  if (sensorType == "structured_light") {
    sensorProcessor_ = std::make_unique<StructuredLightSensorProcessor>(nodeHandle_, generalSensorProcessorParameters);
  } else if (sensorType == "stereo") {
    sensorProcessor_ = std::make_unique<StereoSensorProcessor>(nodeHandle_, generalSensorProcessorParameters);
  } else if (sensorType == "laser") {
    sensorProcessor_ = std::make_unique<LaserSensorProcessor>(nodeHandle_, generalSensorProcessorParameters);
  } else if (sensorType == "perfect") {
    sensorProcessor_ = std::make_unique<PerfectSensorProcessor>(nodeHandle_, generalSensorProcessorParameters);
  } else {
    ROS_ERROR("The sensor type %s is not available.", sensorType.c_str());
    return false;
  }

  return sensorProcessor_->readParameters();
}

}  // namespace elevation_mapping
