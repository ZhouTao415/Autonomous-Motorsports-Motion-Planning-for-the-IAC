#ifndef _CORE_EUDM_PLANNER_INC_EUDM_SERVER_ROS_H__
#define _CORE_EUDM_PLANNER_INC_EUDM_SERVER_ROS_H__

#include <sensor_msgs/msg/joy.hpp>

#include <chrono>
#include <functional>
#include <numeric>
#include <thread>

#include "common/basics/tic_toc.h"
#include "common/visualization/common_visualization_util.h"
#include "eudm_planner/dcp_tree.h"
#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/eudm_manager.h"
#include "eudm_planner/eudm_planner.h"
#include "eudm_planner/map_adapter.h"
#include "eudm_planner/visualizer.h"
#include "moodycamel/atomicops.h"
#include "moodycamel/readerwriterqueue.h"
#include <rclcpp/rclcpp.hpp>
#include "semantic_map_manager/semantic_map_manager.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vehicle_msgs/encoder.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace planning {
class EudmPlannerServer : public rclcpp::Node {
 public:
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;
  using DcpAction = DcpTree::DcpAction;
  using DcpLonAction = DcpTree::DcpLonAction;
  using DcpLatAction = DcpTree::DcpLatAction;

  struct Config {
    int kInputBufferSize{100};
  };

  EudmPlannerServer(const rclcpp::NodeOptions &options, int ego_id);

  EudmPlannerServer(const rclcpp::NodeOptions &options, double work_rate, int ego_id);

  void PushSemanticMap(const SemanticMapManager &smm);

  void BindBehaviorUpdateCallback(
      std::function<int(const SemanticMapManager &)> fn);

  void set_user_desired_velocity(const decimal_t desired_vel);

  decimal_t user_desired_velocity() const;

  void Init(const std::string &bp_config_path);

  void Start();

 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_; 

  void PlanCycleCallback();

  void JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);
  

  void Replan();

  void PublishData();

  void MainThread();

  ErrorType GetCorrespondingActionInActionSequence(
      const decimal_t &t, const std::vector<DcpAction> &action_seq,
      DcpAction *a) const;

  Config config_;

  EudmManager bp_manager_;
  EudmPlannerVisualizer *p_visualizer_;

  SemanticMapManager smm_;

  planning::eudm::Task task_;
  bool use_sim_state_ = true;

  double work_rate_{20.0};
  int ego_id_;

  // input buffer
  moodycamel::ReaderWriterQueue<SemanticMapManager> *p_input_smm_buff_;

  bool has_callback_binded_ = false;
  std::function<int(const SemanticMapManager &)> private_callback_fn_;
};

}  // namespace planning

#endif
