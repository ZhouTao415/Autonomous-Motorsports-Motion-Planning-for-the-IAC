#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_ROS_ADAPTER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_ROS_ADAPTER_H_

#include <assert.h>
#include <functional>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"
#include "eudm_planner/eudm_manager.h"
#include "eudm_planner/eudm_planner.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace planning {

class EudmPlannerVisualizer {
 public:
  EudmPlannerVisualizer(std::shared_ptr<rclcpp::Node> node, EudmManager* p_bp_manager, int ego_id)
      : node_(node), ego_id_(ego_id) {
    assert(p_bp_manager != nullptr);
    p_bp_manager_ = p_bp_manager;
  }

  void Init() {
    std::string forward_traj_topic = std::string("/vis/agent_") +
                                     std::to_string(ego_id_) +
                                     std::string("/forward_trajs");

    forward_traj_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(forward_traj_topic, 1);
  }

  void PublishDataWithStamp(const rclcpp::Time& stamp) {
    VisualizeForwardTrajectories(stamp);
  }

  void VisualizeForwardTrajectories(const rclcpp::Time& stamp) {
    auto forward_trajs = p_bp_manager_->planner().forward_trajs();
    int processed_winner_id = p_bp_manager_->processed_winner_id();
    int original_winner_id = p_bp_manager_->original_winner_id();
    visualization_msgs::msg::MarkerArray traj_list_marker;
    common::ColorARGB traj_color(0.5, 0.5, 0.5, 0.5);
    double traj_z = 0.3;
    for (int i = 0; i < static_cast<int>(forward_trajs.size()); ++i) {
      if (i == processed_winner_id) {
        traj_color = common::cmap.at("gold");
        traj_z = 0.4;
      } else if (i == original_winner_id) {
        traj_color = common::cmap.at("spring green");
        traj_z = 0.4;
      } else {
        traj_color = common::ColorARGB(0.5, 0.5, 0.5, 0.5);
        traj_z = 0.3;
      }
      std::vector<common::Point> points;
      for (const auto& v : forward_trajs[i]) {
        common::Point pt(v.state().vec_position(0), v.state().vec_position(1));
        pt.z = traj_z;
        points.push_back(pt);
        visualization_msgs::msg::Marker point_marker;
        common::VisualizationUtil::GetRosMarkerCylinderUsingPoint(
            common::Point(pt), Vec3f(0.5, 0.5, 0.1), traj_color, 0,
            &point_marker);
        traj_list_marker.markers.push_back(point_marker);
      }
      visualization_msgs::msg::Marker line_marker;
      common::VisualizationUtil::GetRosMarkerLineStripUsingPoints(
          points, Vec3f(0.1, 0.1, 0.1), traj_color, 0, &line_marker);
      traj_list_marker.markers.push_back(line_marker);
    }
    int num_markers = static_cast<int>(traj_list_marker.markers.size());
    common::VisualizationUtil::FillHeaderIdInMarkerArray(
        stamp, std::string("map"), last_forward_trajs_marker_cnt_,
        &traj_list_marker);
    last_forward_trajs_marker_cnt_ = num_markers;
    forward_traj_vis_pub_->publish(traj_list_marker);
  }

  void set_use_sim_state(bool use_sim_state) { use_sim_state_ = use_sim_state; }

 private:
  std::shared_ptr<rclcpp::Node> node_;
  int ego_id_;
  bool use_sim_state_ = true;

  int last_forward_trajs_marker_cnt_ = 0;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr forward_traj_vis_pub_;
  EudmManager* p_bp_manager_{nullptr};

};

}  // namespace planning

#endif  // _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_ROS_ADAPTER_H_
