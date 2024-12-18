// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
* @file ground_truth_visualizer.hpp
*
* Node that subscribes to the ground truth topic and publishes the ground truth path
*
* @authors Rodrigo Da Silva Gomez
*/

#ifndef GROUND_TRUTH_VISUALIZER__GROUND_TRUTH_VISUALIZER_HPP_
#define GROUND_TRUTH_VISUALIZER__GROUND_TRUTH_VISUALIZER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace ground_truth_visualizer
{
class GroundTruthVisualizer : public rclcpp::Node
{
public:
  explicit GroundTruthVisualizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GroundTruthVisualizer();

private:
  std::string sub_ground_truth_pose_topic_ = "/pose";
  std::string sub_ground_truth_point_topic_ = "/point";
  std::string pub_ground_truth_pose_topic_ = "/ground_truth/pose";
  std::string pub_ground_truth_path_topic_ = "/ground_truth/path";

  geometry_msgs::msg::PoseStamped starting_pose_;

  Eigen::Isometry3d starting_pose_transform_;

  std::string frame_id_ = "earth";

  std::shared_ptr<nav_msgs::msg::Path> ground_truth_path_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ground_truth_point_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ground_truth_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_pose_pub_;

  void groundTruthPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void groundTruthPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

private:
  /**
   * @brief Get the modified options object
   *
   * @param options
   * @return rclcpp::NodeOptions
   */
  static rclcpp::NodeOptions get_modified_options(const rclcpp::NodeOptions & options);
}; // class GroundTruthVisualizer
}  // namespace ground_truth_visualizer

#endif // GROUND_TRUTH_VISUALIZER__GROUND_TRUTH_VISUALIZER_HPP_
