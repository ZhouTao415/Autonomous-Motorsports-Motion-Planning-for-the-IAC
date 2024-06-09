#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_SERVER_ROS_H__
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_SERVER_ROS_H__

#include <rclcpp/rclcpp.hpp>
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

#include <sensor_msgs/msg/joy.hpp>
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vehicle_msgs/encoder.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace planning {

class BehaviorPlannerServer : public rclcpp::Node {
 public:
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;

  struct Config {
    int kInputBufferSize{100};
  };

  BehaviorPlannerServer(const rclcpp::NodeOptions &options, int ego_id);

  BehaviorPlannerServer(const rclcpp::NodeOptions &options, double work_rate, int ego_id);

  void PushSemanticMap(const SemanticMapManager &smm);

  void BindBehaviorUpdateCallback(
      std::function<int(const SemanticMapManager &)> fn);

  /**
   * @brief set the level of autonomous driving
   */
  void set_autonomous_level(int level);

  /**
   * @brief set desired velocity
   */
  void set_user_desired_velocity(const double desired_vel);

  void set_aggressive_level(int level);

  double user_desired_velocity() const;

  double reference_desired_velocity() const;

  void enable_hmi_interface();

  void Init();

  void Start();

 private:
  void PlanCycleCallback();

  void JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr &msg);

  void Replan();

  void PublishData();

  void MainThread();

  Config config_;

  BehaviorPlanner bp_;
  BehaviorPlannerMapAdapter map_adapter_;
  BehaviorPlannerVisualizer *p_visualizer_;

  TicToc time_profile_tool_;
  double global_init_stamp_{0.0};

  // ros related
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  double work_rate_;
  int ego_id_;

  // input buffer
  moodycamel::ReaderWriterQueue<SemanticMapManager> *p_input_smm_buff_;

  bool has_callback_binded_ = false;
  std::function<int(const SemanticMapManager &)> private_callback_fn_;

  bool is_hmi_enabled_ = false;
};

}  // namespace planning

#endif
