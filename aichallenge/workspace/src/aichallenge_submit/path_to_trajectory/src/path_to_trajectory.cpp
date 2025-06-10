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

#include "path_to_trajectory/path_to_trajectory.hpp"

PathToTrajectory::PathToTrajectory() : Node("path_to_trajectory_node")
{
  using std::placeholders::_1;
  const auto rv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

  // パスとトラジェクトリのパブリッシャ・サブスクライバ
  trajectory_pub_ = this->create_publisher<Trajectory>("output", rv_qos);
  path_sub_ = this->create_subscription<PathWithLaneId>(
    "input", rv_qos, std::bind(&PathToTrajectory::pathCallback, this, _1));

  // 障害物の位置情報を受け取るサブスクライバ
  objects_sub_ = this->create_subscription<MarkerArray>(
    "/aichallenge/objects_marker", rv_qos, [this](const MarkerArray::SharedPtr msg) {
      objects_ = msg;

      // 障害物の総数を出力
      RCLCPP_INFO(this->get_logger(), "Received objects (total: %zu)", objects_->markers.size());

      // 各障害物の詳細情報をDEBUGレベルで出力
      for (size_t i = 0; i < objects_->markers.size(); ++i) {
        const auto & marker = objects_->markers[i];
        RCLCPP_DEBUG(
          this->get_logger(),
          "Object[%zu]: position=(%.2f, %.2f, %.2f), "
          "diameter=%.2f, frame_id=%s",
          i, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z,
          marker.scale.x,  // x=y=直径
          marker.header.frame_id.c_str());
      }
    });

  RCLCPP_INFO(this->get_logger(), "Path to trajectory node has been initialized.");
}

void PathToTrajectory::pathCallback(const PathWithLaneId::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received path with %zu points", msg->points.size());

  Trajectory trajectory;
  trajectory.header = msg->header;
  for (auto & path_point_with_lane_id : msg->points) {
    TrajectoryPoint trajectory_point;
    trajectory_point.pose = path_point_with_lane_id.point.pose;
    trajectory_point.longitudinal_velocity_mps =
      path_point_with_lane_id.point.longitudinal_velocity_mps;
    trajectory.points.emplace_back(std::move(trajectory_point));
  }

  // RCLCPP_DEBUG(
  //   this->get_logger(),
  //   "Publishing trajectory with %zu points (frame_id: %s)",
  //   trajectory.points.size(),
  //   trajectory.header.frame_id.c_str());

  trajectory_pub_->publish(trajectory);
}

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathToTrajectory>());
  rclcpp::shutdown();
  return 0;
}
