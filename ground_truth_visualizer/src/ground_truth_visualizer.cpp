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
* @file ground_truth_visualizer_node.cpp
*
* Node that subscribes to the ground truth topic and publishes the ground truth path
*
* @authors Rodrigo Da Silva Gomez
*/

#include "ground_truth_visualizer.hpp"
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>

namespace ground_truth_visualizer
{
GroundTruthVisualizer::GroundTruthVisualizer(
  const rclcpp::NodeOptions & options)
: Node("ground_truth_visualizer", get_modified_options(options))
{
  // Get parameters
  this->get_parameter("sub_ground_truth_pose_topic", sub_ground_truth_pose_topic_);
  this->get_parameter("sub_ground_truth_point_topic", sub_ground_truth_point_topic_);
  this->get_parameter("pub_ground_truth_pose_topic", pub_ground_truth_pose_topic_);
  this->get_parameter("pub_ground_truth_path_topic", pub_ground_truth_path_topic_);
  this->get_parameter("frame_id", frame_id_);

  double px = this->get_parameter("starting_pose.position.x").as_double();
  double py = this->get_parameter("starting_pose.position.y").as_double();
  double pz = this->get_parameter("starting_pose.position.z").as_double();
  double ow = this->get_parameter("starting_pose.orientation.w").as_double();
  double ox = this->get_parameter("starting_pose.orientation.x").as_double();
  double oy = this->get_parameter("starting_pose.orientation.y").as_double();
  double oz = this->get_parameter("starting_pose.orientation.z").as_double();

  starting_pose_ = geometry_msgs::msg::PoseStamped();
  starting_pose_.header.frame_id = frame_id_;
  starting_pose_.header.stamp = this->now();
  starting_pose_.pose.position.x = px;
  starting_pose_.pose.position.y = py;
  starting_pose_.pose.position.z = pz;
  starting_pose_.pose.orientation.w = ow;
  starting_pose_.pose.orientation.x = ox;
  starting_pose_.pose.orientation.y = oy;
  starting_pose_.pose.orientation.z = oz;

  auto rotation_matrix = Eigen::Quaterniond(ow, ox, oy, oz).toRotationMatrix();
  auto translation_vector = Eigen::Vector3d(px, py, pz);
  auto transformation_matrix = Eigen::Isometry3d::Identity();

  transformation_matrix.translation() = translation_vector;
  transformation_matrix.linear() = rotation_matrix;

  starting_pose_transform_ = transformation_matrix.inverse();
  std::cout << "Starting pose: \n" << starting_pose_transform_.matrix() << std::endl;

  // transformation_matrix.block<3, 3>(0, 0) = rotation_matrix;
  // transformation_matrix.block<3, 1>(0, 3) = translation_vector;

  // auto exaple_point = Eigen::Vector4d(1, 0, 0, 1);
  // auto transformed_point = transformation_matrix * exaple_point;


  // Print parameters
  RCLCPP_INFO(
    this->get_logger(), "sub_ground_truth_pose_topic: %s", sub_ground_truth_pose_topic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "sub_ground_truth_point_topic: %s",
    sub_ground_truth_point_topic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "pub_ground_truth_pose_topic: %s", pub_ground_truth_pose_topic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "pub_ground_truth_path_topic: %s", pub_ground_truth_path_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());

  // Initialize publishers
  ground_truth_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    pub_ground_truth_path_topic_,
    rclcpp::QoS(10));

  ground_truth_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    pub_ground_truth_pose_topic_,
    rclcpp::QoS(10));

  // Initialize subscribers
  if (!sub_ground_truth_pose_topic_.empty()) {
    ground_truth_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      sub_ground_truth_pose_topic_,
      rclcpp::QoS(10),
      std::bind(&GroundTruthVisualizer::groundTruthPoseCallback, this, std::placeholders::_1));
  }
  if (!sub_ground_truth_point_topic_.empty()) {
    ground_truth_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      sub_ground_truth_point_topic_,
      rclcpp::QoS(10),
      std::bind(&GroundTruthVisualizer::groundTruthPointCallback, this, std::placeholders::_1));
  }

  // Initialize path
  ground_truth_path_ = std::make_shared<nav_msgs::msg::Path>();
  ground_truth_path_->header.frame_id = frame_id_;
  ground_truth_path_->header.stamp = this->now();
  ground_truth_path_->poses.clear();
}

GroundTruthVisualizer::~GroundTruthVisualizer() {}

void GroundTruthVisualizer::groundTruthPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.header.frame_id = frame_id_;
  // pose.pose = msg->pose;

  Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Vector3d transformed_position = starting_pose_transform_ * position;

  pose.pose.position.x = transformed_position.x();
  pose.pose.position.y = transformed_position.y();
  pose.pose.position.z = transformed_position.z();

  ground_truth_pose_pub_->publish(pose);

  ground_truth_path_->poses.emplace_back(pose);
  ground_truth_path_->header.stamp = msg->header.stamp;
  ground_truth_pub_->publish(*ground_truth_path_);
}

void GroundTruthVisualizer::groundTruthPointCallback(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.header.frame_id = frame_id_;
  // pose.pose.position = msg->point;

  Eigen::Vector3d position(msg->point.x, msg->point.y, msg->point.z);
  Eigen::Vector3d transformed_position = starting_pose_transform_ * position;

  pose.pose.position.x = transformed_position.x();
  pose.pose.position.y = transformed_position.y();
  pose.pose.position.z = transformed_position.z();

  geometry_msgs::msg::PoseStamped transformed_pose;


  ground_truth_pose_pub_->publish(pose);

  ground_truth_path_->poses.emplace_back(pose);
  ground_truth_path_->header.stamp = msg->header.stamp;
  ground_truth_pub_->publish(*ground_truth_path_);
}

rclcpp::NodeOptions GroundTruthVisualizer::get_modified_options(const rclcpp::NodeOptions & options)
{
  rclcpp::NodeOptions modified_options = options;
  modified_options.allow_undeclared_parameters(true);
  modified_options.automatically_declare_parameters_from_overrides(true);
  return modified_options;
}

} // namespace ground_truth_visualizer
