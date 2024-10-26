#ifndef _UTIL_SSC_PLANNER_INC_VISUALIZER_H_
#define _UTIL_SSC_PLANNER_INC_VISUALIZER_H_

#include <assert.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/primitive/frenet_primitive.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"
#include "ssc_planner/ssc_planner.h"

namespace planning {

class SscVisualizer {
 public:
  SscVisualizer(rclcpp::Node::SharedPtr node, int node_id);
  ~SscVisualizer() {}

  void VisualizeDataWithStamp(const rclcpp::Time &stamp, const SscPlanner &planner);

 private:
  void VisualizeSscMap(const rclcpp::Time &stamp, const SscMap *p_ssc_map);
  void VisualizeEgoVehicleInSscSpace(const rclcpp::Time &stamp, const common::FsVehicle &fs_ego_vehicle);
  void VisualizeForwardTrajectoriesInSscSpace(
      const rclcpp::Time &stamp, const vec_E<vec_E<common::FsVehicle>> &trajs,
      const SscMap *p_ssc_map);
  void VisualizeQpTrajs(const rclcpp::Time &stamp, const vec_E<common::BezierSpline<5, 2>> &trajs);
  void VisualizeSurroundingVehicleTrajInSscSpace(
      const rclcpp::Time &stamp,
      const vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>> &trajs_set,
      const SscMap *p_ssc_map);
  void VisualizeCorridorsInSscSpace(
      const rclcpp::Time &stamp, const vec_E<common::DrivingCorridor> corridor_vec,
      const SscMap *p_ssc_map);

  int last_traj_list_marker_cnt_ = 0;
  int last_surrounding_vehicle_marker_cnt_ = 0;

  rclcpp::Node::SharedPtr node_;
  int node_id_;

  decimal_t start_time_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ssc_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ego_vehicle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr forward_trajs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sur_vehicle_trajs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr qp_pub_;

  int last_corridor_mk_cnt = 0;
  int last_qp_traj_mk_cnt = 0;
  int last_sur_vehicle_traj_mk_cnt = 0;
  int last_forward_traj_mk_cnt = 0;
};  // SscVisualizer
}  // namespace planning

#endif  // _UTIL_SSC_PLANNER_INC_VISUALIZER_H_
