// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory_with_ns.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>
#include <gz/msgs/world_control_state.pb.h>
#include <gz/transport/Node.hh>

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <simulation_interfaces/msg/result.hpp>
#include <simulation_interfaces/srv/spawn_entity.hpp>

#include "gz_simulation_interfaces/gazebo_proxy.hpp"
#include "gz_simulation_interfaces/services/spawn_entity.hpp"

using namespace std::chrono_literals;

TEST(GzSimulationInterfacesSpawnEntityTest, ForwardsNamespaceToGazeboRequest)
{
  rclcpp::init(0, nullptr);

  const std::string world_name = "test_world";
  const std::string expected_namespace = "robot_1";
  const std::string gz_prefix = "world/" + world_name + "/";

  gz::transport::Node gz_node;

  std::function<bool(gz::msgs::StringMsg_V &)> worlds_cb =
    [&world_name](gz::msgs::StringMsg_V & response) {
      response.add_data(world_name);
      return true;
    };

  ASSERT_TRUE(gz_node.Advertise("gazebo/worlds", worlds_cb));

  std::function<bool(gz::msgs::SerializedStepMap &)> state_cb =
    [](gz::msgs::SerializedStepMap & response) {
      (void)response;
      return true;
    };

  ASSERT_TRUE(gz_node.Advertise("world/test_world/state", state_cb));

  std::function<bool(
    const gz::msgs::WorldControlState &,
    gz::msgs::Boolean &)> control_state_cb =
    [](const gz::msgs::WorldControlState &,
      gz::msgs::Boolean & response) {
      response.set_data(true);
      return true;
    };

  ASSERT_TRUE(gz_node.Advertise("world/test_world/control/state", control_state_cb));

  std::mutex received_request_mutex;
  std::string received_namespace;
  bool received_request = false;

  std::function<bool(
    const gz::msgs::EntityFactoryWithNs &,
    gz::msgs::Boolean &)> create_cb =
    [&](const gz::msgs::EntityFactoryWithNs & request,
      gz::msgs::Boolean & response) {
      {
        std::lock_guard<std::mutex> lock(received_request_mutex);
        received_namespace = request.entity_namespace();
        received_request = true;
      }

      response.set_data(true);
      return true;
    };

  ASSERT_TRUE(gz_node.Advertise("world/test_world/create_with_ns/blocking", create_cb));

  auto ros_node = std::make_shared<rclcpp::Node>("gz_server");
  auto gz_proxy =
    std::make_shared<ros_gz_sim::gz_simulation_interfaces::GazeboProxy>("", ros_node);
  auto spawn_entity =
    std::make_shared<ros_gz_sim::gz_simulation_interfaces::services::SpawnEntity>(
    ros_node, gz_proxy);
  (void)spawn_entity;

  auto client =
    ros_node->create_client<simulation_interfaces::srv::SpawnEntity>("spawn_entity");
  ASSERT_TRUE(client->wait_for_service(5s));

  auto request =
    std::make_shared<simulation_interfaces::srv::SpawnEntity::Request>();
  request->name = "test_model";
  request->entity_namespace = expected_namespace;
  request->entity_resource.resource_string =
    "<sdf version='1.12'><model name='test_model'>"
    "<link name='link'/></model></sdf>";

  auto future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(ros_node, future, 5s),
    rclcpp::FutureReturnCode::SUCCESS);

  const auto response = future.get();
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(
    response->result.result,
    simulation_interfaces::msg::Result::RESULT_OK);

  {
    std::lock_guard<std::mutex> lock(received_request_mutex);
    ASSERT_TRUE(received_request);
    EXPECT_EQ(received_namespace, expected_namespace);
  }

  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
