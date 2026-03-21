// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ros_gz_bridge/ros_gz_bridge.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "bridge_handle_ros_to_gz.hpp"
#include "bridge_handle_gz_to_ros.hpp"
#include "get_mappings.hpp"

#include <rclcpp/expand_topic_or_service_name.hpp>

namespace ros_gz_bridge
{

RosGzBridge::RosGzBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("ros_gz_bridge", options)
{
  gz_node_ = std::make_shared<gz::transport::Node>();

  this->declare_parameter<int>("subscription_heartbeat", 1000);
  this->declare_parameter<std::string>("config_file", "");
  this->declare_parameter<bool>("lazy", kDefaultLazy);
  this->declare_parameter<bool>("expand_gz_topic_names", false);
  this->declare_parameter<bool>("override_timestamps_with_wall_time", false);
  this->declare_parameter<std::string>("override_frame_id", "");
  this->declare_parameter("bridge_names", std::vector<std::string>());
  const auto names = this->get_parameter("bridge_names").as_string_array();

  using rclcpp::PARAMETER_STRING;
  using rclcpp::PARAMETER_NOT_SET;

  for (const auto & name : names) {
    const auto prefix = "bridges." + name + ".";

    this->declare_parameter(prefix + "ros_type_name", PARAMETER_STRING);

    const auto ros_topic_name = this->declare_parameter(prefix + "ros_topic_name", "");
    const auto service_name = this->declare_parameter(prefix + "service_name", "");
    const auto is_topic = !ros_topic_name.empty();
    const auto is_service = !service_name.empty();

    if (is_topic == is_service) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Bridge %s needs to set exactly one of ros_topic_name or service_name.", name.c_str());
      continue;
    }

    if (is_topic) {
      const auto gz_topic_name = this->declare_parameter(prefix + "gz_topic_name",
        PARAMETER_STRING);
      if (gz_topic_name.get_type() == PARAMETER_NOT_SET) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Bridge %s does not set required parameter gz_topic_name.", name.c_str());
        continue;
      }

      this->declare_parameter(prefix + "gz_type_name", PARAMETER_STRING);

      this->declare_parameter(prefix + "direction", "BIDIRECTIONAL");
      // Queue sizes default to 10 if qos_profile is not set.
      // If it is defined, they are applied only if they are non-negative.
      this->declare_parameter(prefix + "publisher_queue", -1);
      this->declare_parameter(prefix + "subscriber_queue", -1);
      this->declare_parameter(prefix + "lazy", this->get_parameter("lazy").as_bool());
      this->declare_parameter(prefix + "qos_profile", "");
      this->declare_parameter(prefix + "frame_id", "");
    } else {
      const auto gz_req_type = this->declare_parameter(prefix + "gz_req_type_name",
        PARAMETER_STRING);
      if (gz_req_type.get_type() == PARAMETER_NOT_SET) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Bridge %s does not set required parameter gz_req_type_name.", name.c_str());
        continue;
      }
      const auto gz_rep_type = this->declare_parameter(prefix + "gz_rep_type_name",
        PARAMETER_STRING);
      if (gz_rep_type.get_type() == PARAMETER_NOT_SET) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Bridge %s does not set required parameter gz_rep_type_name.", name.c_str());
        continue;
      }
    }
  }

  int heartbeat;
  this->get_parameter("subscription_heartbeat", heartbeat);
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(heartbeat),
    std::bind(&RosGzBridge::spin, this));
}

void RosGzBridge::spin()
{
  if (handles_.empty()) {
    std::string config_file;
    this->get_parameter("config_file", config_file);
    bool expand_names;
    this->get_parameter("expand_gz_topic_names", expand_names);
    const std::string ros_ns = this->get_namespace();
    const std::string ros_node_name = this->get_name();

    bool lazy;
    this->get_parameter("lazy", lazy);

    // Add bridges from config file
    if (!config_file.empty()) {
      auto entries = readFromYamlFile(config_file);
      for (auto & entry : entries) {
        if (expand_names) {
          entry.gz_topic_name = rclcpp::expand_topic_or_service_name(
            entry.gz_topic_name, ros_node_name, ros_ns, false);
        }
        entry.is_lazy = entry.is_lazy.value_or(lazy);
        if (entry.service_name.empty()) {
          this->add_bridge(entry);
        } else {
          this->add_service_bridge(
            entry.ros_type_name.value(),
            entry.gz_req_type_name,
            entry.gz_rep_type_name,
            entry.service_name);
        }
      }
    }

    // Add bridges from parameters
    const auto names = this->get_parameter("bridge_names").as_string_array();
    for (const auto & name : names) {
      const auto prefix = "bridges." + name + ".";
      if (!this->get_parameter(prefix + "ros_topic_name").as_string().empty()) {
        const auto directionStr = this->get_parameter(prefix + "direction").as_string();
        BridgeDirection direction {BridgeDirection::NONE};
        if (directionStr == "NONE") {
          direction = BridgeDirection::NONE;
        } else if (directionStr == "BIDIRECTIONAL") {
          direction = BridgeDirection::BIDIRECTIONAL;
        } else if (directionStr == "GZ_TO_ROS") {
          direction = BridgeDirection::GZ_TO_ROS;
        } else if (directionStr == "ROS_TO_GZ") {
          direction = BridgeDirection::ROS_TO_GZ;
        } else {
          RCLCPP_ERROR(
            this->get_logger(),
            "Bridge %s defines unknown direction %s.",
            name.c_str(), directionStr.c_str());
          continue;
        }

        const auto ros_type_name_str = this->get_parameter(prefix + "ros_type_name").as_string();
        std::optional<std::string> ros_type_name;
        if (!ros_type_name_str.empty()) {
          ros_type_name = ros_type_name_str;
        }

        const auto gz_type_name_str = this->get_parameter(prefix + "gz_type_name").as_string();
        std::optional<std::string> gz_type_name;
        if (!gz_type_name_str.empty()) {
          gz_type_name = gz_type_name_str;
        }

        const auto qos_profile_str = this->get_parameter(prefix + "qos_profile").as_string();
        std::optional<rclcpp::QoS> qos_profile;
        if (!qos_profile_str.empty()) {
          try {
            qos_profile = parseQoS(qos_profile_str);
          } catch (const std::invalid_argument & e) {
            RCLCPP_ERROR(
              this->get_logger(),
              "Bridge %s defines unknown QoS profile %s.",
              name.c_str(), qos_profile_str.c_str());
            continue;
          }
        }

        const auto pub_queue_size_int = this->get_parameter(prefix + "publisher_queue").as_int();
        std::optional<size_t> pub_queue_size;
        if (pub_queue_size_int >= 0) {
          pub_queue_size = pub_queue_size_int;
        } else if (!qos_profile.has_value()) {
          pub_queue_size = kDefaultPublisherQueue;
        }

        const auto sub_queue_size_int = this->get_parameter(prefix + "subscriber_queue").as_int();
        std::optional<size_t> sub_queue_size;
        if (sub_queue_size_int >= 0) {
          sub_queue_size = sub_queue_size_int;
        } else if (!qos_profile.has_value()) {
          sub_queue_size = kDefaultSubscriberQueue;
        }

        BridgeConfig config {
          ros_type_name,
          this->get_parameter(prefix + "ros_topic_name").as_string(),
          gz_type_name,
          this->get_parameter(prefix + "gz_topic_name").as_string(),
          direction,
          pub_queue_size,
          sub_queue_size,
          this->get_parameter(prefix + "lazy").as_bool(),
          qos_profile,
          {},
          {},
          {},
          this->get_parameter(prefix + "frame_id").as_string()
        };
        if (expand_names) {
          config.gz_topic_name = rclcpp::expand_topic_or_service_name(
            config.gz_topic_name, ros_node_name, ros_ns, false);
        }

        this->add_bridge(config);
      } else {
        this->add_service_bridge(
          this->get_parameter(prefix + "ros_type_name").as_string(),
          this->get_parameter(prefix + "gz_req_type_name").as_string(),
          this->get_parameter(prefix + "gz_rep_type_name").as_string(),
          this->get_parameter(prefix + "service_name").as_string());
      }
    }
  }
  for (auto & bridge : handles_) {
    bridge->Spin();
  }
}

void RosGzBridge::add_bridge(const BridgeConfig & input_config)
{
  // Resolve the laziness: if the caller left is_lazy as nullopt, inherit the
  // node-level "lazy" parameter so that the effective value is always explicit.
  BridgeConfig config = input_config;
  if (!config.is_lazy.has_value()) {
    bool node_lazy = kDefaultLazy;
    this->get_parameter("lazy", node_lazy);
    config.is_lazy = node_lazy;
  }

  if (!complete_bridge_type(config)) {
    return;
  }
  const auto & ros_type_name = config.ros_type_name.value();
  const auto & gz_type_name = config.gz_type_name.value();

  bool gz_to_ros = false;
  bool ros_to_gz = false;

  if (config.direction == BridgeDirection::GZ_TO_ROS) {
    gz_to_ros = true;
  }

  if (config.direction == BridgeDirection::ROS_TO_GZ) {
    ros_to_gz = true;
  }

  if (config.direction == BridgeDirection::BIDIRECTIONAL) {
    ros_to_gz = true;
    gz_to_ros = true;
  }

  try {
    if (gz_to_ros) {
      RCLCPP_INFO(
        this->get_logger(),
        "Creating GZ->ROS Bridge: [%s (%s) -> %s (%s)] (Lazy %d)",
        config.gz_topic_name.c_str(), gz_type_name.c_str(),
        config.ros_topic_name.c_str(), ros_type_name.c_str(),
        config.is_lazy.value_or(kDefaultLazy));
      handles_.push_back(
        std::make_unique<ros_gz_bridge::BridgeHandleGzToRos>(
          shared_from_this(), gz_node_,
          config));

      handles_.back()->Start();
    }

    if (ros_to_gz) {
      RCLCPP_INFO(
        this->get_logger(),
        "Creating ROS->GZ Bridge: [%s (%s) -> %s (%s)] (Lazy %d)",
        config.ros_topic_name.c_str(), ros_type_name.c_str(),
        config.gz_topic_name.c_str(), gz_type_name.c_str(),
        config.is_lazy.value_or(kDefaultLazy));
      handles_.push_back(
        std::make_unique<ros_gz_bridge::BridgeHandleRosToGz>(
          shared_from_this(), gz_node_,
          config));

      handles_.back()->Start();
    }
  } catch (std::runtime_error & _e) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to create a bridge for topic [%s] with ROS2 type [%s] "
      "to topic [%s] with Gazebo Transport type [%s]: %s",
      config.ros_topic_name.c_str(),
      ros_type_name.c_str(),
      config.gz_topic_name.c_str(),
      gz_type_name.c_str(),
      _e.what());
  }
}

void RosGzBridge::add_service_bridge(
  const std::string & ros_type_name,
  const std::string & gz_req_type_name,
  const std::string & gz_rep_type_name,
  const std::string & service_name)
{
  try {
    RCLCPP_INFO(
      this->get_logger(),
      "Creating ROS->GZ service bridge [%s (%s -> %s/%s)]",
      service_name.c_str(), ros_type_name.c_str(),
      gz_req_type_name.c_str(), gz_rep_type_name.c_str());
    auto factory = get_service_factory(ros_type_name, gz_req_type_name, gz_rep_type_name);
    services_.push_back(factory->create_ros_service(shared_from_this(), gz_node_, service_name));
  } catch (std::runtime_error & _e) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to create a bridge for service [%s] with ROS2 type [%s] "
      " and Gazebo types [%s/%s]: %s",
      service_name.c_str(), ros_type_name.c_str(),
      gz_req_type_name.c_str(), gz_rep_type_name.c_str(),
      _e.what());
  }
}


std::vector<std::string> RosGzBridge::get_ros_topic_types(
  const std::string & topic_name) const
{
  std::vector<std::string> types;
  auto topic_names_and_types = this->get_topic_names_and_types();
  auto it = topic_names_and_types.find(topic_name);
  if (it == topic_names_and_types.end()) {
    return types;
  }

  for (const auto & type : it->second) {
    if (!type.empty()) {
      auto it = std::find(types.begin(), types.end(), type);
      if (it == types.end()) {
        types.push_back(type);
      }
    }
  }
  return types;
}

std::vector<std::string> RosGzBridge::get_gz_topic_types(
  const std::string & topic_name) const
{
  std::vector<std::string> types;

  std::vector<gz::transport::MessagePublisher> publishers;
  std::vector<gz::transport::MessagePublisher> subscribers;

  if (!gz_node_->TopicInfo(topic_name, publishers, subscribers)) {
    return types;
  }

  for (const auto & pub : publishers) {
    const auto & type = pub.MsgTypeName();
    if (!type.empty()) {
      auto it = std::find(types.begin(), types.end(), type);
      if (it == types.end()) {
        types.push_back(type);
      }
    }
  }

  for (const auto & sub : subscribers) {
    const auto & type = sub.MsgTypeName();
    if (!type.empty()) {
      auto it = std::find(types.begin(), types.end(), type);
      if (it == types.end()) {
        types.push_back(type);
      }
    }
  }

  return types;
}

bool RosGzBridge::complete_gz_type_from_ros_type(BridgeConfig & config)
{
  std::vector<std::string> possible_gz_types;
  if (!get_ros_to_gz_mapping(config.ros_type_name.value(), possible_gz_types)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "No Gazebo type mapping found for ROS type [%s] for bridge [%s -> %s].",
      config.ros_type_name->c_str(),
      config.ros_topic_name.c_str(),
      config.gz_topic_name.c_str());
    return false;
  }

  if (possible_gz_types.size() == 1) {
    config.gz_type_name = possible_gz_types[0];
    RCLCPP_INFO(
      this->get_logger(),
      "Inferred Gazebo type [%s] for ROS type [%s] for bridge [%s -> %s].",
      config.gz_type_name->c_str(),
      config.ros_type_name->c_str(),
      config.ros_topic_name.c_str(),
      config.gz_topic_name.c_str());
    return true;
  }

  std::vector<std::string> gz_types_on_topic = get_gz_topic_types(config.gz_topic_name);
  if (gz_types_on_topic.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Multiple possible Gazebo types for ROS type [%s] for bridge [%s -> %s], but no "
      "publishers or subscribers found on Gazebo topic [%s] to help disambiguate.",
      config.ros_type_name->c_str(),
      config.ros_topic_name.c_str(),
      config.gz_topic_name.c_str(),
      config.gz_topic_name.c_str());
    return false;
  }

  std::vector<std::string> matched_gz_types;
  for (const auto & possible_gz_type : possible_gz_types) {
    if (std::find(gz_types_on_topic.begin(), gz_types_on_topic.end(), possible_gz_type) != gz_types_on_topic.end()) {
      matched_gz_types.push_back(possible_gz_type);
    }
  }

  if (matched_gz_types.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Multiple possible Gazebo types for ROS type [%s] for bridge [%s -> %s], but no "
      "matching types found on Gazebo topic [%s] to help disambiguate.",
      config.ros_type_name->c_str(),
      config.ros_topic_name.c_str(),
      config.gz_topic_name.c_str(),
      config.gz_topic_name.c_str());
    return false;
  }
  else if (matched_gz_types.size() == 1) {
    config.gz_type_name = matched_gz_types[0];
    RCLCPP_INFO(
      this->get_logger(),
      "Inferred Gazebo type [%s] for ROS type [%s] for bridge [%s -> %s].",
      config.gz_type_name->c_str(),
      config.ros_type_name->c_str(),
      config.ros_topic_name.c_str(),
      config.gz_topic_name.c_str());
    return true;
  }

  RCLCPP_ERROR(
    this->get_logger(),
    "Multiple possible Gazebo types for ROS type [%s] for bridge [%s -> %s], and "
    "%zu matching types found on Gazebo topic. "
    "Please specify the Gazebo type explicitly.",
    config.ros_type_name->c_str(),
    config.ros_topic_name.c_str(),
    config.gz_topic_name.c_str(),
    matched_gz_types.size());

    return false;
}


bool RosGzBridge::complete_ros_type_from_gz_type(BridgeConfig & config)
{
  std::vector<std::string> possible_ros_types;
  if (!get_gz_to_ros_mapping(config.gz_type_name.value(), possible_ros_types)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "No ROS type mapping found for Gazebo type [%s] for bridge [%s -> %s].",
      config.gz_type_name->c_str(),
      config.ros_topic_name.c_str(),
      config.gz_topic_name.c_str());
    return false;
  }

  if (possible_ros_types.size() == 1) {
    config.ros_type_name = possible_ros_types[0];
    RCLCPP_INFO(
      this->get_logger(),
      "Inferred ROS type [%s] for Gazebo type [%s] for bridge [%s -> %s].",
      config.ros_type_name->c_str(),
      config.gz_type_name->c_str(),
      config.ros_topic_name.c_str(),
      config.gz_topic_name.c_str());
    return true;
  }

  std::vector<std::string> ros_types_on_topic = get_ros_topic_types(config.ros_topic_name);
  if (ros_types_on_topic.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Multiple possible ROS types for Gazebo type [%s] for bridge [%s -> %s], but no "
      "publishers or subscribers found on ROS topic [%s] to help disambiguate.",
      config.gz_type_name->c_str(),
      config.ros_topic_name.c_str(),
      config.gz_topic_name.c_str(),
      config.ros_topic_name.c_str());
    return false;
  }

  std::vector<std::string> matched_ros_types;
  for (const auto & possible_ros_type : possible_ros_types) {
    if (std::find(ros_types_on_topic.begin(), ros_types_on_topic.end(), possible_ros_type) != ros_types_on_topic.end()) {
      matched_ros_types.push_back(possible_ros_type);
    }
  }

  if (matched_ros_types.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Multiple possible ROS types for Gazebo type [%s] for bridge [%s -> %s], but no "
      "matching types found on ROS topic [%s] to help disambiguate.",
      config.gz_type_name->c_str(),
      config.ros_topic_name.c_str(),
      config.gz_topic_name.c_str(),
      config.ros_topic_name.c_str());
    return false;
  }
  else if (matched_ros_types.size() == 1) {
    config.ros_type_name = matched_ros_types[0];
    RCLCPP_INFO(
      this->get_logger(),
      "Inferred ROS type [%s] for Gazebo type [%s] for bridge [%s -> %s].",
      config.ros_type_name->c_str(),
      config.gz_type_name->c_str(),
      config.ros_topic_name.c_str(),
      config.gz_topic_name.c_str());
    return true;
  }

  RCLCPP_ERROR(
    this->get_logger(),
    "Multiple possible ROS types for Gazebo type [%s] for bridge [%s -> %s], and "
    "%zu matching types found on ROS topic. "
    "Please specify the ROS type explicitly.",
    config.gz_type_name->c_str(),
    config.ros_topic_name.c_str(),
    config.gz_topic_name.c_str(),
    matched_ros_types.size());

    return false;
}

bool RosGzBridge::complete_types_from_runtime_topics(BridgeConfig & config)
{
  std::vector<std::string> ros_types_on_topic = get_ros_topic_types(config.ros_topic_name);
  std::vector<std::string> gz_types_on_topic = get_gz_topic_types(config.gz_topic_name);

  // TODO(C88-YQ): 
  if (ros_types_on_topic.empty() || gz_types_on_topic.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Cannot complete bridge [%s -> %s] because no publishers or subscribers found on "
      "ROS topic [%s] or Gazebo topic [%s].",
      config.ros_topic_name.c_str(),
      config.gz_topic_name.c_str(),
      config.ros_topic_name.c_str(),
      config.gz_topic_name.c_str());
    return false;
  }

  std::string matched_ros_type;
  std::string matched_gz_type;
  int match_count = 0;
  for (const auto & ros_type : ros_types_on_topic) {
    if (match_count > 1) {
      break;
    }

    std::vector<std::string> possible_gz_types;
    if (!get_ros_to_gz_mapping(ros_type, possible_gz_types)) {
      continue;
    }

    for (const auto & gz_type : gz_types_on_topic) {
      if (std::find(possible_gz_types.begin(), possible_gz_types.end(), gz_type) != possible_gz_types.end()) {
        matched_ros_type = ros_type;
        matched_gz_type = gz_type;
        match_count++;
      }
    }
  }

  if (match_count == 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Cannot complete bridge [%s -> %s] because no matching types found on ROS topic [%s] "
      "and Gazebo topic [%s].",
      config.ros_topic_name.c_str(),
      config.gz_topic_name.c_str(),
      config.ros_topic_name.c_str(),
      config.gz_topic_name.c_str());
    return false;
  } else if (match_count == 1) {
    config.ros_type_name = matched_ros_type;
    config.gz_type_name = matched_gz_type;
    RCLCPP_INFO(
      this->get_logger(),
      "Inferred ROS type [%s] and Gazebo type [%s] for bridge [%s -> %s] from runtime topic information.",
      config.ros_type_name->c_str(),
      config.gz_type_name->c_str(),
      config.ros_topic_name.c_str(),
      config.gz_topic_name.c_str());
    return true;
  }

  RCLCPP_ERROR(
    this->get_logger(),
    "Cannot complete bridge [%s -> %s] because multiple matching types found on ROS topic [%s] "
    "and Gazebo topic [%s]. Please specify the types explicitly.",
    config.ros_topic_name.c_str(),
    config.gz_topic_name.c_str(),
    config.ros_topic_name.c_str(),
    config.gz_topic_name.c_str());
  return false;
}

bool RosGzBridge::complete_bridge_type(BridgeConfig & config)
{
  const bool has_ros_type =
    config.ros_type_name.has_value() && !config.ros_type_name->empty();
  const bool has_gz_type =
    config.gz_type_name.has_value() && !config.gz_type_name->empty();
  
  if (has_ros_type && has_gz_type) {
    return true;
  }

  if (has_ros_type) {
    return complete_gz_type_from_ros_type(config);
  }

  if (has_gz_type) {
    return complete_ros_type_from_gz_type(config);
  }

  return complete_types_from_runtime_topics(config);
}

}  // namespace ros_gz_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros_gz_bridge::RosGzBridge)
