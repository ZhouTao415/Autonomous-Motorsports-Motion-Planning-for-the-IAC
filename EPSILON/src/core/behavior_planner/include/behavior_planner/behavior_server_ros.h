#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_SERVER_ROS2_H__
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_SERVER_ROS2_H__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vehicle_msgs/encoder.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <chrono>
#include <functional>
#include <numeric>
#include <thread>

#include "behavior_planner/behavior_planner.h"
#include "behavior_planner/map_adapter.h"
#include "behavior_planner/visualizer.h"
#include "semantic_map_manager/semantic_map_manager.h"

#include "common/basics/tic_toc.h"
#include "common/visualization/common_visualization_util.h"
#include "moodycamel/atomicops.h"
#include "moodycamel/readerwriterqueue.h"

namespace planning {

class BehaviorPlannerServer : public rclcpp::Node {
 public:
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;

  struct Config {
    int kInputBufferSize{100};
  };

  static std::shared_ptr<BehaviorPlannerServer> Create(std::shared_ptr<rclcpp::Node> node, double work_rate, int ego_id) {
    auto server = std::shared_ptr<BehaviorPlannerServer>(new BehaviorPlannerServer(node, work_rate, ego_id));
    server->p_visualizer_ = std::make_unique<BehaviorPlannerVisualizer>(server, &(server->bp_), ego_id);
    server->p_input_smm_buff_ = std::make_unique<moodycamel::ReaderWriterQueue<SemanticMapManager>>(server->config_.kInputBufferSize);
    return server;
  }

  void PushSemanticMap(const SemanticMapManager &smm);
  void BindBehaviorUpdateCallback(std::function<int(const SemanticMapManager &)> fn);

  void set_autonomous_level(int level);
  void set_user_desired_velocity(const decimal_t desired_vel);
  void set_aggressive_level(int level);

  decimal_t user_desired_velocity() const;
  decimal_t reference_desired_velocity() const;

  void enable_hmi_interface();
  void Init();
  void Start();
  BehaviorPlannerServer(std::shared_ptr<rclcpp::Node> node, double work_rate, int ego_id)
      : Node("behavior_planner_server", node->get_node_options()), work_rate_(work_rate), ego_id_(ego_id), node_(node) {}

 private:

  void PlanCycleCallback();
  void JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);
  void PublishData();
  void Replan();
  void MainThread();

  Config config_;

  BehaviorPlanner bp_;
  BehaviorPlannerMapAdapter map_adapter_;
  std::unique_ptr<BehaviorPlannerVisualizer> p_visualizer_;

  TicToc time_profile_tool_;
  decimal_t global_init_stamp_{0.0};

  // ros related
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  double work_rate_;
  int ego_id_;
  std::shared_ptr<rclcpp::Node> node_;

  // input buffer
  std::unique_ptr<moodycamel::ReaderWriterQueue<SemanticMapManager>> p_input_smm_buff_;

  bool has_callback_binded_ = false;
  std::function<int(const SemanticMapManager &)> private_callback_fn_;

  bool is_hmi_enabled_ = false;
};

}  // namespace planning

#endif  // _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_SERVER_ROS2_H__
