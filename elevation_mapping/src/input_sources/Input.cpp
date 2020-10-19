/*
 *  Input.cpp
 *
 *  Created on: Oct 06, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/input_sources/Input.hpp"

namespace elevation_mapping {

Input::Input(ros::NodeHandle& nh) : nodeHandle_(nh), queueSize_(0), publishOnUpdate_(true) {}

bool Input::configure(const XmlRpc::XmlRpcValue& parameters) {
  // Configuration Guards.
  if (parameters.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR(
        "Input source must be specified as map, but is "
        "XmlRpcType:%d.",
        parameters.getType());
    return false;
  }

  // Check that parameters exist and has an appropriate type.
  using nameAndType = std::pair<std::string, XmlRpc::XmlRpcValue::Type>;
  for (const nameAndType& member : std::vector<nameAndType>{{"name", XmlRpc::XmlRpcValue::TypeString},
                                                            {"type", XmlRpc::XmlRpcValue::TypeString},
                                                            {"topic", XmlRpc::XmlRpcValue::TypeString},
                                                            {"queue_size", XmlRpc::XmlRpcValue::TypeInt},
                                                            {"publish_on_update", XmlRpc::XmlRpcValue::TypeBoolean}}) {
    if (!parameters.hasMember(member.first)) {
      ROS_ERROR("Could not configure input source because no %s was given.", member.first.c_str());
      return false;
    }
    if (parameters[member.first].getType() != member.second) {
      ROS_ERROR(
          "Could not configure input source because member %s has the "
          "wrong type.",
          member.first.c_str());
      return false;
    }
  }

  name_ = static_cast<std::string>(parameters["name"]);
  type_ = static_cast<std::string>(parameters["type"]);
  topic_ = static_cast<std::string>(parameters["topic"]);
  const int& queueSize = static_cast<int>(parameters["queue_size"]);
  if (queueSize >= 0) {
    queueSize_ = static_cast<const unsigned int>(queueSize);
  } else {
    ROS_ERROR("The specified queue_size is negative.");
    return false;
  }
  publishOnUpdate_ = static_cast<bool>(parameters["publish_on_update"]);
  ROS_DEBUG("Configured %s:%s @ %s (publishing_on_update: %s)\n", type_.c_str(), name_.c_str(), nodeHandle_.resolveName(topic_).c_str(),
            publishOnUpdate_ ? "true" : "false");
  return true;
}

std::string Input::getSubscribedTopic() const {
  return nodeHandle_.resolveName(topic_);
}

}  // namespace elevation_mapping