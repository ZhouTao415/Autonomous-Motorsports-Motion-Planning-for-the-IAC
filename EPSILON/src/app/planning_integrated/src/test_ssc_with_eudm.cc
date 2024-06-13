#include <chrono>
#include <iostream>
#include <memory>

#include "eudm_planner/eudm_server_ros.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "ssc_planner/ssc_server_ros.h"
#include "rclcpp/rclcpp.hpp"

DECLARE_BACKWARD;
double ssc_planner_work_rate = 20.0;
double bp_work_rate = 20.0;

std::shared_ptr<planning::SscPlannerServer> p_ssc_server_{nullptr};
std::shared_ptr<planning::EudmPlannerServer> p_bp_server_{nullptr};

int BehaviorUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
  if (p_ssc_server_) p_ssc_server_->PushSemanticMap(smm);
  return 0;
}

int SemanticMapUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
  if (p_bp_server_) p_bp_server_->PushSemanticMap(smm);
  return 0;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("planning_integrated");
  auto options = rclcpp::NodeOptions();

  RCLCPP_INFO(node->get_logger(), "Node name: %s", node->get_name());

  node->declare_parameter<int>("ego_id", 0);
  node->declare_parameter<std::string>("agent_config_path", "");
  node->declare_parameter<std::string>("bp_config_path", "");
  node->declare_parameter<std::string>("ssc_config_path", "");
  node->declare_parameter<double>("desired_vel", 6.0);
  node->declare_parameter<bool>("use_sim_state", true);

  int ego_id;
  if (!node->get_parameter("ego_id", ego_id)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get param %d", ego_id);
    assert(false);
  }

  std::string agent_config_path;
  if (!node->get_parameter("agent_config_path", agent_config_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get param agent_config_path %s", 
                                      agent_config_path.c_str());
    assert(false);
  }

  std::string bp_config_path;
  if (!node->get_parameter("bp_config_path", bp_config_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get param bp_config_path %s", 
                                      bp_config_path.c_str());
    assert(false);
  }

  std::string ssc_config_path;
  if (!node->get_parameter("ssc_config_path", ssc_config_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get param ssc_config_path %s",
                                      ssc_config_path.c_str());
    assert(false);
  }


  semantic_map_manager::SemanticMapManager semantic_map_manager(ego_id, agent_config_path);
  semantic_map_manager::RosAdapter smm_ros_adapter(options, &semantic_map_manager);
  smm_ros_adapter.BindMapUpdateCallback(SemanticMapUpdateCallback);

  double desired_vel;
  node->get_parameter("desired_vel", desired_vel);

  // Declare bp
  p_bp_server_ = std::make_shared<planning::EudmPlannerServer>(options, bp_work_rate, ego_id);
  p_bp_server_->set_user_desired_velocity(desired_vel);
  p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateCallback);

  p_ssc_server_ = std::make_shared<planning::SscPlannerServer>(options, ssc_planner_work_rate, ego_id);

  p_ssc_server_->Init(ssc_config_path);
  p_bp_server_->Init(bp_config_path);
  smm_ros_adapter.Init();

  p_bp_server_->Start();
  p_ssc_server_->Start();


  // Logging parameters for verification
  RCLCPP_INFO(node->get_logger(), "ego_id: %d", ego_id);
  RCLCPP_INFO(node->get_logger(), "agent_config_path: %s", agent_config_path.c_str());
  RCLCPP_INFO(node->get_logger(), "bp_config_path: %s", bp_config_path.c_str());
  RCLCPP_INFO(node->get_logger(), "ssc_config_path: %s", ssc_config_path.c_str());
  RCLCPP_INFO(node->get_logger(), "desired_vel: %f", desired_vel);
  // RCLCPP_INFO(node->get_logger(), "use_sim_state: %s", use_sim_state ? "true" : "false");


  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
