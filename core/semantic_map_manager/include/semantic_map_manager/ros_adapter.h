#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_

#include <assert.h>

#include <functional>
#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "rclcpp/rclcpp.hpp"
#include "semantic_map_manager/data_renderer.h"
#include "vehicle_msgs/decoder.h"

#include "vehicle_msgs/msg/arena_info.hpp"
#include "vehicle_msgs/msg/arena_info_static.hpp"
#include "vehicle_msgs/msg/arena_info_dynamic.hpp"

namespace semantic_map_manager {

class RosAdapter {
 public:
  using GridMap2D = common::GridMapND<uint8_t, 2>;

  RosAdapter(std::shared_ptr<rclcpp::Node> node, SemanticMapManager* ptr_smm)
      : node_(node), p_smm_(ptr_smm), p_data_renderer_(new DataRenderer(ptr_smm)) {
    Init();
  }

  ~RosAdapter() {
    delete p_data_renderer_;
  }

  void BindMapUpdateCallback(std::function<int(const SemanticMapManager&)> fn);

  void Init();  // Make Init public

 private:
  void ArenaInfoCallback(const vehicle_msgs::msg::ArenaInfo::SharedPtr msg);

  void ArenaInfoStaticCallback(const vehicle_msgs::msg::ArenaInfoStatic::SharedPtr msg);

  void ArenaInfoDynamicCallback(const vehicle_msgs::msg::ArenaInfoDynamic::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;

  // communicate with phy simulator
  rclcpp::Subscription<vehicle_msgs::msg::ArenaInfo>::SharedPtr arena_info_sub_;
  rclcpp::Subscription<vehicle_msgs::msg::ArenaInfoStatic>::SharedPtr arena_info_static_sub_;
  rclcpp::Subscription<vehicle_msgs::msg::ArenaInfoDynamic>::SharedPtr arena_info_dynamic_sub_;

  common::Vehicle ego_vehicle_;
  common::VehicleSet vehicle_set_;
  common::LaneNet lane_net_;
  common::ObstacleSet obstacle_set_;

  DataRenderer* p_data_renderer_;
  SemanticMapManager* p_smm_;

  bool get_arena_info_static_ = false;

  bool has_callback_binded_ = false;
  std::function<int(const SemanticMapManager&)> private_callback_fn_;
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_
