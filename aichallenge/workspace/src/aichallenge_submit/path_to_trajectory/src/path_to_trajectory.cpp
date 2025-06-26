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

#include <cmath>
#include <algorithm>

PathToTrajectory::PathToTrajectory() : Node("path_to_trajectory_node")
{
  using std::placeholders::_1;
  // QoS設定をBEST_EFFORTに変更してパブリッシャーと一致させる
  const auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .best_effort()
    .durability_volatile();

  RCLCPP_INFO(this->get_logger(), "Initializing PathToTrajectory node...");

  // パスとトラジェクトリのパブリッシャ・サブスクライバ
  trajectory_pub_ = this->create_publisher<Trajectory>("output", best_effort_qos);
  RCLCPP_INFO(this->get_logger(), "Created publisher for /output topic");
  
  // Trajectoryメッセージをサブスクライブするように変更
  trajectory_sub_ = this->create_subscription<Trajectory>(
    "input", best_effort_qos, std::bind(&PathToTrajectory::trajectoryCallback, this, _1));
  RCLCPP_INFO(this->get_logger(), "Created subscription for /input topic (Trajectory)");

  // 障害物の位置情報を受け取るサブスクライバ
  objects_sub_ = this->create_subscription<MarkerArray>(
    "/aichallenge/objects_marker", best_effort_qos, [this](const MarkerArray::SharedPtr msg) {
      objects_ = msg;

      RCLCPP_INFO(this->get_logger(), "Received %zu objects", objects_->markers.size());

      // 各障害物の詳細情報をINFOレベルで出力
      for (size_t i = 0; i < objects_->markers.size(); ++i) {
        const auto & marker = objects_->markers[i];
        RCLCPP_INFO(
          this->get_logger(),
          "Object[%zu]: position=(%.2f, %.2f, %.2f), "
          "diameter=%.2f, frame_id=%s",
          i, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z,
          marker.scale.x,  // x=y=直径
          marker.header.frame_id.c_str());
      }
    });
  RCLCPP_INFO(this->get_logger(), "Created subscription for /aichallenge/objects_marker topic");

  RCLCPP_INFO(this->get_logger(), "PathToTrajectory node initialized successfully");
}

void PathToTrajectory::trajectoryCallback(const Trajectory::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "=== trajectoryCallback EXECUTED ===");
  RCLCPP_DEBUG(this->get_logger(), "Received trajectory with %zu points", msg->points.size());
  RCLCPP_DEBUG(this->get_logger(), "Header frame_id: %s", msg->header.frame_id.c_str());
  RCLCPP_DEBUG(this->get_logger(), "Header timestamp: %d.%d", msg->header.stamp.sec, msg->header.stamp.nanosec);
  
  // 最初の5ポイントの位置情報を出力
  for (size_t i = 0; i < std::min(size_t(5), msg->points.size()); ++i) {
    const auto & point = msg->points[i];
    RCLCPP_DEBUG(this->get_logger(), 
      "Trajectory[%zu]: position=(%.2f, %.2f, %.2f)", 
      i, point.pose.position.x, point.pose.position.y, point.pose.position.z);
  }
  
  // 受信したトラジェクトリをコピー
  Trajectory trajectory = *msg;

  RCLCPP_DEBUG(this->get_logger(), "Processing trajectory with %zu points", trajectory.points.size());

  // 障害物回避処理を実行
  avoidObstacles(trajectory);

  trajectory_pub_->publish(trajectory);
  RCLCPP_DEBUG(this->get_logger(), "Published processed trajectory to /output");
}

void PathToTrajectory::avoidObstacles(Trajectory &trajectory) {
  // Check if objects are available
  if (!objects_) {
    RCLCPP_DEBUG(this->get_logger(), "No objects available for obstacle avoidance");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "=== avoidObstacles EXECUTED ===");
  RCLCPP_INFO(this->get_logger(), "Processing %zu trajectory points against %zu objects", 
              trajectory.points.size(), objects_->markers.size());

  constexpr double SAFETY_DISTANCE = 4.0; // Increased safety distance in meters
  constexpr double MAX_STEERING_OFFSET = 3.4; // Maximum steering offset in meters

  int avoidance_count = 0;

  // Iterate through the trajectory points
  for (auto & traj_point : trajectory.points) {
    for (const auto & marker : objects_->markers) {
      double obj_x = marker.pose.position.x;
      double obj_y = marker.pose.position.y;
      double car_x = traj_point.pose.position.x;
      double car_y = traj_point.pose.position.y;

      // Check if the object is close to the trajectory point
      double distance = std::hypot(car_x - obj_x, car_y - obj_y);
      
      if (distance < SAFETY_DISTANCE) {
        avoidance_count++;
        RCLCPP_INFO(this->get_logger(),
          "AVOIDANCE: Point(%.2f, %.2f) - Object(%.2f, %.2f) = %.2fm", 
          car_x, car_y, obj_x, obj_y, distance);
        
        // Apply a larger steering offset based on proximity to the obstacle
        double steering_offset = MAX_STEERING_OFFSET * (SAFETY_DISTANCE - distance) / SAFETY_DISTANCE;
        
        // Determine direction to steer
        double direction = (obj_y > car_y) ? -1.0 : 1.0; // Steer right if object is on the left, otherwise steer left

        // Apply the steering offset
        traj_point.pose.position.y += direction * steering_offset;

        // Adjust the x position to ensure smooth transition
        traj_point.pose.position.x += (obj_x > car_x) ? -steering_offset / 2 : steering_offset / 2;

        // Adjust velocity and other parameters for smoother transition
        traj_point.longitudinal_velocity_mps *= (1.0 - distance / SAFETY_DISTANCE);
        traj_point.lateral_velocity_mps = direction * steering_offset;

        // Adjust acceleration to be higher when closer to the obstacle
        traj_point.acceleration_mps2 = (distance < 1.0) ? MAX_STEERING_OFFSET : MAX_STEERING_OFFSET / distance;

        traj_point.heading_rate_rps = direction * std::atan2(traj_point.pose.position.y - car_y, traj_point.pose.position.x - car_x);
        traj_point.front_wheel_angle_rad = direction * steering_offset / 2;
        traj_point.rear_wheel_angle_rad = -direction * steering_offset / 2;

        // Set time_from_start for smoother transition
        traj_point.time_from_start.sec = static_cast<int32_t>(distance / traj_point.longitudinal_velocity_mps);
        traj_point.time_from_start.nanosec = static_cast<uint32_t>((distance / traj_point.longitudinal_velocity_mps - traj_point.time_from_start.sec) * 1e9);
      }
    }
  }

  if (avoidance_count > 0) {
    RCLCPP_INFO(this->get_logger(), "Applied obstacle avoidance to %d trajectory points", avoidance_count);
  } else {
    RCLCPP_INFO(this->get_logger(), "No trajectory points needed obstacle avoidance");
  }
}

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("path_to_trajectory"), "Starting PathToTrajectory node");
  rclcpp::spin(std::make_shared<PathToTrajectory>());
  rclcpp::shutdown();
  return 0;
}
