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

#ifndef ROS_GZ_BRIDGE__ROS_GZ_BRIDGE_HPP_
#define ROS_GZ_BRIDGE__ROS_GZ_BRIDGE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <gz/msgs/config.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/node.hpp>
#include "ros_gz_bridge/bridge_config.hpp"

namespace ros_gz_bridge
{
/// \brief Result of attempting to resolve bridge message types
enum class TypeResolutionResult
{
  /// \brief Types were successfully resolved and written to the config
  RESOLVED,

  /// \brief Insufficient information at the moment; should retry later
  PENDING,

  /// \brief Resolution is impossible due to conflict or ambiguity
  FAILED
};

/// Forward declarations
class BridgeHandle;

/// \brief Component container for the ROS-GZ Bridge
class RosGzBridge : public rclcpp::Node
{
public:
  /// \brief Constructor
  /// \param[in] options options control creation of the ROS 2 node
  explicit RosGzBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// \brief Register a bridge configuration for type resolution.
  /// \param[in] config Bridge configuration to register.
  void register_bridge(BridgeConfig & config);

  /// \brief Add a new ROS-GZ bridge to the node
  /// \param[in] config Parameters to control creation of a new bridge
  void add_bridge(const BridgeConfig & config);

  /// \brief Attempt to resolve and create pending bridges.
  void process_pending_bridges();

  /// \brief Create a new ROS-GZ bridge for a service
  /// \param[in] ros_type_name Name of the ROS service (eg ros_gz_interfaces/srv/ControlWorld)
  /// \param[in] gz_req_type_name Gazebo service request type
  /// \param[in] gz_rep_type_name Gazebo service response type
  /// \param[in] service_name Address of the service to be bridged
  void add_service_bridge(
    const std::string & ros_type_name,
    const std::string & gz_req_type_name,
    const std::string & gz_rep_type_name,
    const std::string & service_name);

  /// \brief Get all unique ROS message types currently observed on the topic.
  /// \param[in] topic_name Name of the ROS topic.
  /// \return A list of unique message type names. Returns empty if the topic
  /// is not found or no types are available.
  std::vector<std::string> get_ros_topic_types(const std::string & topic_name) const;

  /// \brief Get all unique Gazebo message types currently observed on the topic.
  /// \param[in] topic_name Name of the Gazebo topic.
  /// \return A list of unique message type names collected from publishers
  /// and subscribers. Returns empty if no information is available.
  std::vector<std::string> get_gz_topic_types(const std::string & topic_name) const;

  /// \brief Infer the Gazebo message type from a given ROS message type.
  /// \param[in,out] config Bridge configuration to complete.
  /// \return Resolution result indicating resolved, pending, or failure.
  TypeResolutionResult complete_gz_type_from_ros_type(BridgeConfig & config);

  /// \brief Infer the ROS message type from a given Gazebo message type.
  /// \param[in,out] config Bridge configuration to complete.
  /// \return Resolution result indicating resolved, pending, or failure.
  TypeResolutionResult complete_ros_type_from_gz_type(BridgeConfig & config);

  /// \brief Infer both ROS and Gazebo message types using runtime topic information.
  /// \param[in,out] config Bridge configuration to complete.
  /// \return Resolution result indicating resolved, pending, or failure.
  TypeResolutionResult complete_types_from_runtime_topics(BridgeConfig & config);

  /// \brief Complete a bridge configuration when one message type is missing
  /// \param[in,out] config Bridge configuration to complete
  /// \return Resolution result indicating resolved, pending, or failure.
  TypeResolutionResult complete_bridge_type(BridgeConfig & config);

protected:
  /// \brief Periodic callback to check connectivity and liveliness
  void spin();

protected:
  /// \brief Pointer to Gazebo node used to create publishers/subscribers
  std::shared_ptr<gz::transport::Node> gz_node_;

  /// \brief List of bridge handles
  std::vector<std::shared_ptr<ros_gz_bridge::BridgeHandle>> handles_;

  /// \brief List of bridged ROS services
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;

  /// \brief Timer to control periodic callback
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  /// \brief List of bridges waiting for type resolution
  std::vector<BridgeConfig> pending_bridges_;

  /// \brief Flag to indicate whether the configuration has been loaded
  bool config_loaded_;
};
}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__ROS_GZ_BRIDGE_HPP_
