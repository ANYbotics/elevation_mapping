/*
 *  InputSourceManager.cpp
 *
 *  Created on: Oct 02, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/input_sources/InputSourceManager.hpp"
#include "elevation_mapping/ElevationMapping.hpp"

namespace elevation_mapping {

InputSourceManager::InputSourceManager(const ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {}

bool InputSourceManager::configureFromRos(const std::string& inputSourcesNamespace) {
  XmlRpc::XmlRpcValue inputSourcesConfiguration;
  if (!nodeHandle_.getParam(inputSourcesNamespace, inputSourcesConfiguration)) {
    ROS_WARN(
        "Could not load the input sources configuration from parameter\n "
        "%s, are you sure it was pushed to the parameter server? Assuming\n "
        "that you meant to leave it empty. Not subscribing to any inputs!\n",
        nodeHandle_.resolveName(inputSourcesNamespace).c_str());
    return false;
  }
  return configure(inputSourcesConfiguration, inputSourcesNamespace);
}

bool InputSourceManager::configure(const XmlRpc::XmlRpcValue& config, const std::string& sourceConfigurationName) {
  if (config.getType() == XmlRpc::XmlRpcValue::TypeArray &&
      config.size() == 0) {  // Use Empty array as special case to explicitly configure no inputs.
    return true;
  }

  if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR(
        "%s: The input sources specification must be a struct. but is of "
        "of XmlRpcType %d",
        sourceConfigurationName.c_str(), config.getType());
    ROS_ERROR("The xml passed in is formatted as follows:\n %s", config.toXml().c_str());
    return false;
  }

  bool successfulConfiguration = true;
  std::set<std::string> subscribedTopics;
  SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{nodeHandle_.param("robot_base_frame_id", std::string("/robot")),
                                                                      nodeHandle_.param("map_frame_id", std::string("/map"))};
  // Configure all input sources in the list.
  for (const auto& inputConfig : config) {
    Input source{(ros::NodeHandle(nodeHandle_.resolveName(sourceConfigurationName + "/" + inputConfig.first)))};

    const bool configured{source.configure(inputConfig.first, inputConfig.second, generalSensorProcessorConfig)};
    if (!configured) {
      successfulConfiguration = false;
      continue;
    }

    if (!source.isEnabled()) {
      continue;
    }

    const std::string subscribedTopic{source.getSubscribedTopic()};
    const bool topicIsUnique{subscribedTopics.insert(subscribedTopic).second};

    if (topicIsUnique) {
      sources_.push_back(std::move(source));
    } else {
      ROS_WARN(
          "The input sources specification tried to subscribe to %s "
          "multiple times. Only subscribing once.",
          subscribedTopic.c_str());
      successfulConfiguration = false;
    }
  }

  return successfulConfiguration;
}

int InputSourceManager::getNumberOfSources() {
  return static_cast<int>(sources_.size());
}

}  // namespace elevation_mapping