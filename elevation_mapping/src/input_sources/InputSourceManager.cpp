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
  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR(
        "%s: The input sources specification must be a list. but is of "
        "of XmlRpcType %d",
        sourceConfigurationName.c_str(), config.getType());
    ROS_ERROR("The xml passed in is formatted as follows:\n %s", config.toXml().c_str());
    return false;
  }

  bool successfulConfiguration = true;
  std::set<std::string> subscribedTopics;

  // Configure all input sources in the list.
  for (int i = 0; i < config.size(); ++i) {
    Input source{nodeHandle_};

    bool configured = source.configure(config[i]);
    if (!configured) {
      successfulConfiguration = false;
      continue;
    }

    std::string subscribedTopic = source.getSubscribedTopic();
    bool topicIsUnique = subscribedTopics.insert(subscribedTopic).second;

    if (topicIsUnique) {
      sources_.push_back(source);
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