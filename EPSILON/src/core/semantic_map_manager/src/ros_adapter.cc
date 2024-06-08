#include "semantic_map_manager/ros_adapter.h"

namespace semantic_map_manager {

void RosAdapter::Init() {
  // communicate with phy simulator
  arena_info_sub_ = this->create_subscription<vehicle_msgs::msg::ArenaInfo>(
      "arena_info", 2, std::bind(&RosAdapter::ArenaInfoCallback, this, std::placeholders::_1));
  arena_info_static_sub_ = this->create_subscription<vehicle_msgs::msg::ArenaInfoStatic>(
      "arena_info_static", 2, std::bind(&RosAdapter::ArenaInfoStaticCallback, this, std::placeholders::_1));
  arena_info_dynamic_sub_ = this->create_subscription<vehicle_msgs::msg::ArenaInfoDynamic>(
      "arena_info_dynamic", 2, std::bind(&RosAdapter::ArenaInfoDynamicCallback, this, std::placeholders::_1));
}

void RosAdapter::ArenaInfoCallback(const vehicle_msgs::msg::ArenaInfo::SharedPtr msg) {
  rclcpp::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfo(
      *msg, &time_stamp, &lane_net_, &vehicle_set_, &obstacle_set_);
  p_data_renderer_->Render(time_stamp.seconds(), lane_net_, vehicle_set_, obstacle_set_);
  if (has_callback_binded_) {
    private_callback_fn_(*p_smm_);
  }
}

void RosAdapter::ArenaInfoStaticCallback(const vehicle_msgs::msg::ArenaInfoStatic::SharedPtr msg) {
  rclcpp::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoStatic(
      *msg, &time_stamp, &lane_net_, &obstacle_set_);
  get_arena_info_static_ = true;
}

void RosAdapter::ArenaInfoDynamicCallback(const vehicle_msgs::msg::ArenaInfoDynamic::SharedPtr msg) {
  rclcpp::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoDynamic(
      *msg, &time_stamp, &vehicle_set_);

  if (get_arena_info_static_) {
    p_data_renderer_->Render(time_stamp.seconds(), lane_net_, vehicle_set_, obstacle_set_);
    if (has_callback_binded_) {
      private_callback_fn_(*p_smm_);
    }
  }
}

void RosAdapter::BindMapUpdateCallback(std::function<int(const SemanticMapManager&)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

}  // namespace semantic_map_manager
