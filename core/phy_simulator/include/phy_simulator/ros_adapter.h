/**
 * @file ros_adapter.h
 * @brief
 * @version 0.1
 * @date 2019-03-18
 *
 */
#ifndef _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ROS_ADAPTER_H_
#define _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ROS_ADAPTER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "vehicle_msgs/encoder.h"

#include "common/basics/basics.h"
#include "common/basics/semantics.h"

#include "phy_simulator/phy_simulator.h"

namespace phy_simulator {

class RosAdapter {
 public:
  /**
   * @brief Default constructor
   */
  RosAdapter();

  /**
   * @brief Construct a new RosAdapter object
   *
   * @param node node 
   */
  RosAdapter(std::shared_ptr<rclcpp::Node> node);

  void set_phy_sim(PhySimulation *p_phy_sim) { p_phy_sim_ = p_phy_sim; }

  /**
   * @brief Publish data of simulator with time stamp
   *
   * @param stamp ROS time stamp
   */
  void PublishDataWithStamp(const rclcpp::Time &stamp);

  /**
   * @brief Publish dynamic data of simulator with time stamp
   *
   * @param stamp ROS time stamp
   */
  void PublishDynamicDataWithStamp(const rclcpp::Time &stamp);

  /**
   * @brief Publish static data of simulator with time stamp
   *
   * @param stamp ROS time stamp
   */
  void PublishStaticDataWithStamp(const rclcpp::Time &stamp);

 private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<vehicle_msgs::msg::ArenaInfo>::SharedPtr arena_info_pub_;
  rclcpp::Publisher<vehicle_msgs::msg::ArenaInfoStatic>::SharedPtr arena_info_static_pub_;
  rclcpp::Publisher<vehicle_msgs::msg::ArenaInfoDynamic>::SharedPtr arena_info_dynamic_pub_;

  PhySimulation *p_phy_sim_;
};  // RosAdapter
}  // namespace phy_simulator

#endif  // _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ROS_ADAPTER_H_
