#pragma once

#include "STDescCore.h"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

// ROS-specific functions for parameter reading and visualization
void read_parameters(std::shared_ptr<rclcpp::Node> node, ConfigSetting &config_setting);

void publish_std_pairs(
    const std::vector<std::pair<STDesc, STDesc>> &match_std_pairs,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr std_publisher);