// Copyright 2023 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef PATH_TO_TRAJECTORY__PATH_TO_TRAJECTORY_HPP_
#define PATH_TO_TRAJECTORY__PATH_TO_TRAJECTORY_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class PathToTrajectory : public rclcpp::Node
{
public:
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

public:
  PathToTrajectory();

private:
  void trajectoryCallback(const Trajectory::SharedPtr msg);
  void avoidObstacles(Trajectory & trajectory);
  void smoothTrajectory(Trajectory & trajectory);
  void publishDebugMarkers(const Trajectory & trajectory);
  void publishDistanceInfo(const Trajectory & trajectory);

  rclcpp::Subscription<Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<MarkerArray>::SharedPtr objects_sub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr distance_info_pub_;

  // 障害物の位置情報を保持するメンバ変数
  MarkerArray::SharedPtr objects_;
};

#endif  // PATH_TO_TRAJECTORY__PATH_TO_TRAJECTORY_HPP_
