#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
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

void CtrlSignalCallback(const vehicle_msgs::msg::ControlSignal::SharedPtr msg,
                        int index) {
  common::VehicleControlSignal ctrl;
  vehicle_msgs::Decoder::GetControlSignalFromRosControlSignal(*msg, &ctrl);
  _signal_set.signal_set[index] = ctrl;
}

void InitialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  common::VisualizationUtil::Get3DofStateFromRosPose(msg->pose.pose,
                                                     &initial_state);
  flag_rcv_initial_state = true;
}

void NavGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  common::VisualizationUtil::Get3DofStateFromRosPose(msg->pose, &goal_state);
  flag_rcv_goal_state = true;
}

int main(int argc, char** argv) {
  // 使用了 "~" 作为匿名名称，这意味着每次启动节点时都会生成一个唯一的名称
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("phy_simulator_planning_node");

  std::string vehicle_info_path;
  if (!nh->get_parameter("vehicle_info_path", vehicle_info_path)) {
    RCLCPP_ERROR(nh->get_logger(), "Failed to get param %s", vehicle_info_path.c_str());
    assert(false);
  }
  std::string map_path;
  if (!nh->get_parameter("map_path", map_path)) {
    RCLCPP_ERROR(nh->get_logger(), "Failed to get param %s", map_path.c_str());
    assert(false);
  }
  std::string lane_net_path;
  if (!nh->get_parameter("lane_net_path", lane_net_path)) {
    RCLCPP_ERROR(nh->get_logger(), "Failed to get param %s", lane_net_path.c_str());
    assert(false);
  }

  PhySimulation phy_sim(vehicle_info_path, map_path, lane_net_path);

  RosAdapter ros_adapter(nh);
  ros_adapter.set_phy_sim(&phy_sim);

  Visualizer visualizer;
  visualizer.set_phy_sim(&phy_sim);

  auto vehicle_ids = phy_sim.vehicle_ids();
  int num_vehicles = static_cast<int>(vehicle_ids.size());
  _ros_sub.resize(num_vehicles);

  for (int i = 0; i < num_vehicles; i++) {
    auto vehicle_id = vehicle_ids[i];
    std::string topic_name =
        std::string("/ctrl/agent_") + std::to_string(vehicle_id);
    printf("subscribing to %s\n", topic_name.c_str());
    _ros_sub[i] = nh->create_subscription<vehicle_msgs::msg::ControlSignal>(
        topic_name, 10, [vehicle_id](const vehicle_msgs::msg::ControlSignal::SharedPtr msg) {
          CtrlSignalCallback(msg, vehicle_id);
        });
  }

  for (auto& vehicle_id : vehicle_ids) {
    common::VehicleControlSignal default_signal;
    _signal_set.signal_set.insert(std::pair<int, common::VehicleControlSignal>(
        vehicle_id, default_signal));
  }

  auto ini_pos_sub = nh->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10, InitialPoseCallback);
  auto goal_pos_sub = nh->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 10, NavGoalCallback);

  rclcpp::Rate rate(simulation_rate);
  rclcpp::Time next_gt_pub_time = rclcpp::Clock(RCL_ROS_TIME).now();
  rclcpp::Time next_gt_static_pub_time = next_gt_pub_time;
  rclcpp::Time next_vis_pub_time = rclcpp::Clock(RCL_ROS_TIME).now();

  std::cout << "[PhySimulation] Initialization finished, waiting for callback"
            << std::endl;

  while (rclcpp::ok()) {
    rclcpp::spin_some(nh);

    phy_sim.UpdateSimulatorUsingSignalSet(_signal_set, 1.0 / simulation_rate);

    rclcpp::Time tnow = rclcpp::Clock(RCL_ROS_TIME).now();
    if (tnow >= next_gt_pub_time) {
      next_gt_pub_time += rclcpp::Duration::from_seconds(1.0 / gt_msg_rate);
      ros_adapter.PublishDynamicDataWithStamp(tnow);
    }

    if (tnow >= next_gt_static_pub_time) {
      next_gt_static_pub_time += rclcpp::Duration::from_seconds(1.0 / gt_static_msg_rate);
      ros_adapter.PublishStaticDataWithStamp(tnow);
    }

    if (tnow >= next_vis_pub_time) {
      next_vis_pub_time += rclcpp::Duration::from_seconds(1.0 / visualization_msg_rate);
      visualizer.VisualizeDataWithStamp(tnow);
    }

    rate.sleep();
  }

  _ros_sub.clear();
  rclcpp::shutdown();
  return 0;
}
