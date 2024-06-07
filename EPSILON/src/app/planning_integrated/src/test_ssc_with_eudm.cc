/**
 * @file test_ssc_with_mpdm.cc
 * @author HKUST Aerial Robotics Group (lzhangbz@ust.hk)
 * @brief
 * @version 0.1
 * @date 2020-09-21
 * @copyright Copyright (c) 2020
 */
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>

#include <chrono>
#include <iostream>

#include "behavior_planner/behavior_server_ros.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "ssc_planner/ssc_server_ros.h"

DECLARE_BACKWARD;
double ssc_planner_work_rate = 20.0;
double bp_work_rate = 20.0;

planning::SscPlannerServer* p_ssc_server_{nullptr};
planning::BehaviorPlannerServer* p_bp_server_{nullptr};

int BehaviorUpdateCallback(
    const semantic_map_manager::SemanticMapManager& smm) {
  if (p_ssc_server_) p_ssc_server_->PushSemanticMap(smm);
  return 0;
}

int SemanticMapUpdateShortcut(
    const semantic_map_manager::SemanticMapManager& smm) {
  if (p_bp_server_) p_bp_server_->PushSemanticMap(smm);
  return 0;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_ssc_with_mpdm");

  int ego_id;
  node->declare_parameter("ego_id", 0);
  node->get_parameter("ego_id", ego_id);

  std::string agent_config_path;
  node->declare_parameter("agent_config_path", "");
  node->get_parameter("agent_config_path", agent_config_path);

  std::string ssc_config_path;
  node->declare_parameter("ssc_config_path", "");
  node->get_parameter("ssc_config_path", ssc_config_path);

  semantic_map_manager::SemanticMapManager semantic_map_manager(
      ego_id, agent_config_path);
  semantic_map_manager::RosAdapter smm_ros_adapter(node, &semantic_map_manager);
  smm_ros_adapter.BindMapUpdateShortcut(SemanticMapUpdateShortcut);

  double desired_vel;
  node->declare_parameter("desired_vel", 6.0);
  node->get_parameter("desired_vel", desired_cloud_vel);

  p_bp_server_ = new planning::BehaviorPlannerServer(node, bp_work_rate, ego_id);
  p_bp_server_->set_user_desired_velocity(desired_cloud_vel);
  p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateCallback);
  p_bp_server_->set_autonomous_level(3);
  p_bp_server_->enable_hmi_interface();

  p_ssc_server_ =
      new planning::SscPlannerServer(node, ssc_planner_work_rate, ego_id);

  p_ssc_server_->Init(ssc_config_path);
  p_bp_server_->Init();
  smm_ros_adapter.Init();

  p_bp_server_->Start();
  p_ssc_server_->Start();

  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  return 0;
}
