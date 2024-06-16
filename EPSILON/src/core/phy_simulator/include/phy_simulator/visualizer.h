#ifndef _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_VISUALIZER_H_
#define _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_VISUALIZER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"

#include "phy_simulator/phy_simulator.h"

namespace phy_simulator {

class Visualizer {
 public:
  Visualizer() {}
  Visualizer(rclcpp::Node::SharedPtr node);
  ~Visualizer() {}

  void set_phy_sim(PhySimulation *p_phy_sim) { p_phy_sim_ = p_phy_sim; }

  void VisualizeData();
  void VisualizeDataWithStamp(const rclcpp::Time &stamp);
  void SendTfWithStamp(const rclcpp::Time &stamp);

 private:
  void VisualizeVehicleSet(const rclcpp::Time &stamp,
                           const common::VehicleSet &vehicle_set);
  void VisualizeLaneNet(const rclcpp::Time &stamp,
                        const common::LaneNet &lane_net);
  void VisualizeObstacleSet(const rclcpp::Time &stamp,
                            const common::ObstacleSet &Obstacle_set);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vehicle_set_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_net_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_set_pub_;

  PhySimulation *p_phy_sim_;
};  // Visualizer

}  // namespace phy_simulator

#endif  // _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_VISUALIZER_H_
