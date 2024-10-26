// Your converted code
#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_ROS_ADAPTER_H_
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_ROS_ADAPTER_H_

#include <assert.h>
#include <functional>
#include <iostream>
#include <vector>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "vehicle_msgs/decoder.h"

#include "behavior_planner/behavior_planner.h"
#include "common/basics/basics.h"
#include "common/basics/semantics.h"

namespace planning {

class BehaviorPlannerVisualizer {
 public:
  BehaviorPlannerVisualizer(std::shared_ptr<rclcpp::Node> node, BehaviorPlanner* ptr_bp, int ego_id)
      : node_(node), ego_id_(ego_id) {
    p_bp_ = ptr_bp;
  }

  void Init() {
    std::string forward_traj_topic = std::string("/vis/agent_") +
                                     std::to_string(ego_id_) +
                                     std::string("/forward_trajs");
    forward_traj_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(forward_traj_topic, 1);
  }

  void PublishDataWithStamp(const rclcpp::Time& stamp) {
    if (p_bp_ == nullptr) return;
    auto forward_trajs = p_bp_->forward_trajs();
    visualization_msgs::msg::MarkerArray traj_list_marker;
    common::ColorARGB traj_color = common::cmap.at("gold");
    for (const auto& traj : forward_trajs) {
      std::vector<common::Point> points;
      for (const auto& v : traj) {
        common::Point pt(v.state().vec_position(0), v.state().vec_position(1));
        pt.z = 0.3;
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

 private:
  // rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Node> node_;
  int ego_id_;

  int last_forward_trajs_marker_cnt_ = 0;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr forward_traj_vis_pub_;

  BehaviorPlanner* p_bp_{nullptr};
};

}  // namespace planning

#endif  // _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_ROS_ADAPTER_H_
