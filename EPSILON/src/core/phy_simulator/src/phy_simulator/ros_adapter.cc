/**
 * @file ros_adapter.cc
 * @brief
 * @version 0.1
 * @date 2019-03-18
 *
 */
#include "phy_simulator/ros_adapter.h"

namespace phy_simulator {

RosAdapter::RosAdapter() {}

RosAdapter::RosAdapter(std::shared_ptr<rclcpp::Node> node) : node_(node) {
  arena_info_pub_ = node_->create_publisher<vehicle_msgs::msg::ArenaInfo>("arena_info", 10);
  arena_info_static_pub_ =
      node_->create_publisher<vehicle_msgs::msg::ArenaInfoStatic>("arena_info_static", 10);
  arena_info_dynamic_pub_ =
      node_->create_publisher<vehicle_msgs::msg::ArenaInfoDynamic>("arena_info_dynamic", 10);
}

void RosAdapter::PublishDataWithStamp(const rclcpp::Time &stamp) {
  vehicle_msgs::msg::ArenaInfo msg;
  vehicle_msgs::Encoder::GetRosArenaInfoFromSimulatorData(
      p_phy_sim_->lane_net(), p_phy_sim_->vehicle_set(),
      p_phy_sim_->obstacle_set(), stamp, std::string("map"), &msg);
  arena_info_pub_->publish(msg);
}

void RosAdapter::PublishStaticDataWithStamp(const rclcpp::Time &stamp) {
  vehicle_msgs::msg::ArenaInfoStatic msg;
  vehicle_msgs::Encoder::GetRosArenaInfoStaticFromSimulatorData(
      p_phy_sim_->lane_net(), p_phy_sim_->obstacle_set(), stamp,
      std::string("map"), &msg);
  arena_info_static_pub_->publish(msg);
}

void RosAdapter::PublishDynamicDataWithStamp(const rclcpp::Time &stamp) {
  vehicle_msgs::msg::ArenaInfoDynamic msg;
  vehicle_msgs::Encoder::GetRosArenaInfoDynamicFromSimulatorData(
      p_phy_sim_->vehicle_set(), stamp, std::string("map"), &msg);
  arena_info_dynamic_pub_->publish(msg);
}

}  // namespace phy_simulator
