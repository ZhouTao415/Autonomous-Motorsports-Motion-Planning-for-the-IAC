#include "eudm_planner/eudm_server_ros.h"

namespace planning {

EudmPlannerServer::EudmPlannerServer(const rclcpp::NodeOptions &options, int ego_id)
    : Node("eudm_planner_server", options), work_rate_(20.0), ego_id_(ego_id) {
  p_visualizer_ = new EudmPlannerVisualizer(options, &bp_manager_, ego_id);
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(config_.kInputBufferSize);
  task_.user_perferred_behavior = 0;
}

EudmPlannerServer::EudmPlannerServer(const rclcpp::NodeOptions &options, double work_rate, int ego_id)
    : Node("eudm_planner_server", options), work_rate_(work_rate), ego_id_(ego_id) {
  p_visualizer_ = new EudmPlannerVisualizer(options, &bp_manager_, ego_id);
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(config_.kInputBufferSize);
  task_.user_perferred_behavior = 0;
}

void EudmPlannerServer::PushSemanticMap(const SemanticMapManager &smm) {
  if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm);
}

void EudmPlannerServer::PublishData() {
  p_visualizer_->PublishDataWithStamp(rclcpp::Clock(RCL_ROS_TIME).now());
}

void EudmPlannerServer::Init(const std::string &bp_config_path) {
  bp_manager_.Init(bp_config_path, work_rate_);
  auto joy_callback = std::bind(&EudmPlannerServer::JoyCallback, this, std::placeholders::_1);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, joy_callback);
  this->declare_parameter("use_sim_state", use_sim_state_);
  this->get_parameter("use_sim_state", use_sim_state_);
  p_visualizer_->Init();
  p_visualizer_->set_use_sim_state(use_sim_state_);
}

void EudmPlannerServer::JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
  int msg_id;
  if (std::string("").compare(msg->header.frame_id) == 0) {
    msg_id = 0;
  } else {
    msg_id = std::stoi(msg->header.frame_id);
  }
  if (msg_id != ego_id_) return;

  if (msg->buttons[0] == 0 && msg->buttons[1] == 0 && msg->buttons[2] == 0 &&
      msg->buttons[3] == 0 && msg->buttons[4] == 0 && msg->buttons[5] == 0 &&
      msg->buttons[6] == 0)
    return;

  if (msg->buttons[2] == 1) {
    if (task_.user_perferred_behavior != -1) {
      task_.user_perferred_behavior = -1;
    } else {
      task_.user_perferred_behavior = 0;
    }
  } else if (msg->buttons[1] == 1) {
    if (task_.user_perferred_behavior != 1) {
      task_.user_perferred_behavior = 1;
    } else {
      task_.user_perferred_behavior = 0;
    }
  } else if (msg->buttons[3] == 1) {
    task_.user_desired_vel = task_.user_desired_vel + 1.0;
  } else if (msg->buttons[0] == 1) {
    task_.user_desired_vel = std::max(task_.user_desired_vel - 1.0, 0.0);
  } else if (msg->buttons[4] == 1) {
    task_.lc_info.forbid_lane_change_left = !task_.lc_info.forbid_lane_change_left;
  } else if (msg->buttons[5] == 1) {
    task_.lc_info.forbid_lane_change_right = !task_.lc_info.forbid_lane_change_right;
  } else if (msg->buttons[6] == 1) {
    task_.is_under_ctrl = !task_.is_under_ctrl;
  }
}

void EudmPlannerServer::Start() {
  std::thread(&EudmPlannerServer::MainThread, this).detach();
  task_.is_under_ctrl = true;
}

void EudmPlannerServer::MainThread() {
  using namespace std::chrono;
  system_clock::time_point current_start_time{system_clock::now()};
  system_clock::time_point next_start_time{current_start_time};
  const milliseconds interval{static_cast<int>(1000.0 / work_rate_)};
  while (rclcpp::ok()) {
    current_start_time = system_clock::now();
    next_start_time = current_start_time + interval;
    PlanCycleCallback();
    std::this_thread::sleep_until(next_start_time);
  }
}

void EudmPlannerServer::PlanCycleCallback() {
  if (p_input_smm_buff_ == nullptr) return;

  bool has_updated_map = false;
  while (p_input_smm_buff_->try_dequeue(smm_)) {
    has_updated_map = true;
  }

  if (!has_updated_map) return;

  auto map_ptr = std::make_shared<semantic_map_manager::SemanticMapManager>(smm_);

  decimal_t replan_duration = 1.0 / work_rate_;
  double stamp = std::floor(smm_.time_stamp() / replan_duration) * replan_duration;

  if (bp_manager_.Run(stamp, map_ptr, task_) == kSuccess) {
    common::SemanticBehavior behavior;
    bp_manager_.ConstructBehavior(&behavior);
    smm_.set_ego_behavior(behavior);
  }

  if (has_callback_binded_) {
    private_callback_fn_(smm_);
  }

  PublishData();
}

void EudmPlannerServer::BindBehaviorUpdateCallback(
    std::function<int(const SemanticMapManager &)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

void EudmPlannerServer::set_user_desired_velocity(const decimal_t desired_vel) {
  task_.user_desired_vel = desired_vel;
}

decimal_t EudmPlannerServer::user_desired_velocity() const {
  return task_.user_desired_vel;
}

}  // namespace planning
