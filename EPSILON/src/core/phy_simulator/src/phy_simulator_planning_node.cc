#include <stdlib.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include "vehicle_msgs/msg/control_signal.hpp"
#include "vehicle_msgs/decoder.h"

#include "phy_simulator/basics.h"
#include "phy_simulator/phy_simulator.h"
#include "phy_simulator/ros_adapter.h"
#include "phy_simulator/visualizer.h"

using namespace phy_simulator;

DECLARE_BACKWARD;
const double simulation_rate = 500.0;
const double gt_msg_rate = 100.0;
const double gt_static_msg_rate = 10.0;
const double visualization_msg_rate = 20.0;

common::VehicleControlSignalSet _signal_set;
std::vector<rclcpp::Subscription<vehicle_msgs::msg::ControlSignal>::SharedPtr> _ros_sub;

Vec3f initial_state(0, 0, 0);
bool flag_rcv_initial_state = false;

Vec3f goal_state(0, 0, 0);
bool flag_rcv_goal_state = false;

void CtrlSignalCallback(const vehicle_msgs::msg::ControlSignal::SharedPtr msg, int index) {
  common::VehicleControlSignal ctrl;
  vehicle_msgs::Decoder::GetControlSignalFromRosControlSignal(*msg, &ctrl);
  _signal_set.signal_set[index] = ctrl;
}

void InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  common::VisualizationUtil::Get3DofStateFromRosPose(msg->pose.pose, &initial_state);
  flag_rcv_initial_state = true;
}

void NavGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  common::VisualizationUtil::Get3DofStateFromRosPose(msg->pose, &goal_state);
  flag_rcv_goal_state = true;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("phy_simulator_node");

  std::string vehicle_info_path;
  if (!node->get_parameter("vehicle_info_path", vehicle_info_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get param vehicle_info_path");
    assert(false);
  }

  std::string map_path;
  if (!node->get_parameter("map_path", map_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get param map_path");
    assert(false);
  }

  std::string lane_net_path;
  if (!node->get_parameter("lane_net_path", lane_net_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get param lane_net_path");
    assert(false);
  }

  PhySimulation phy_sim(vehicle_info_path, map_path, lane_net_path);

  RosAdapter ros_adapter(node);
  ros_adapter.set_phy_sim(&phy_sim);

  Visualizer visualizer(node);
  visualizer.set_phy_sim(&phy_sim);

  auto vehicle_ids = phy_sim.vehicle_ids();
  int num_vehicles = static_cast<int>(vehicle_ids.size());
  _ros_sub.resize(num_vehicles);

  for (int i = 0; i < num_vehicles; i++) {
    auto vehicle_id = vehicle_ids[i];
    std::string topic_name = std::string("/ctrl/agent_") + std::to_string(vehicle_id);
    RCLCPP_INFO(node->get_logger(), "subscribing to %s", topic_name.c_str());
    _ros_sub[i] = node->create_subscription<vehicle_msgs::msg::ControlSignal>(
        topic_name, 10, [vehicle_id](const vehicle_msgs::msg::ControlSignal::SharedPtr msg) {
          CtrlSignalCallback(msg, vehicle_id);
        });
  }

  for (auto& vehicle_id : vehicle_ids) {
    common::VehicleControlSignal default_signal;
    _signal_set.signal_set.insert(std::make_pair(vehicle_id, default_signal));
  }

  auto ini_pos_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10, InitialPoseCallback);
  auto goal_pos_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 10, NavGoalCallback);

  auto clock = node->get_clock();
  auto rate = rclcpp::Rate(simulation_rate);
  auto next_gt_pub_time = clock->now();
  auto next_gt_static_pub_time = next_gt_pub_time;
  auto next_vis_pub_time = clock->now();

  RCLCPP_INFO(node->get_logger(), "[PhySimulation] Initialization finished, waiting for callback");

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    phy_sim.UpdateSimulatorUsingSignalSet(_signal_set, 1.0 / simulation_rate);

    auto tnow = clock->now();
    if (tnow >= next_gt_pub_time) {
      next_gt_pub_time += rclcpp::Duration(1.0 / gt_msg_rate);
      ros_adapter.PublishDynamicDataWithStamp(tnow);
    }

    if (tnow >= next_gt_static_pub_time) {
      next_gt_static_pub_time += rclcpp::Duration(1.0 / gt_static_msg_rate);
      ros_adapter.PublishStaticDataWithStamp(tnow);
    }

    if (tnow >= next_vis_pub_time) {
      next_vis_pub_time += rclcpp::Duration(1.0 / visualization_msg_rate);
      visualizer.VisualizeDataWithStamp(tnow);
    }

    rate.sleep();
  }

  _ros_sub.clear();
  rclcpp::shutdown();
  return 0;
}
