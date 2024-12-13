#pragma once

// #include <ros/ros.h>
// #include <std_msgs/Header.h>
// #include <std_msgs/Float32.h>
// #include <std_msgs/Bool.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/PointCloud.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/image_encodings.h>
// #include <nav_msgs/Path.h>
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/PointStamped.h>
// #include <visualization_msgs/Marker.h>
// #include <tf/transform_broadcaster.h>
// Same but in ros2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
#include "../estimator.h"
#include "../parameters.h"
#include <fstream>

// extern ros::Publisher pub_odometry;
// extern ros::Publisher pub_path, pub_pose;
// extern ros::Publisher pub_cloud, pub_map;
// extern ros::Publisher pub_key_poses;
// extern ros::Publisher pub_ref_pose, pub_cur_pose;
// extern ros::Publisher pub_key;
// extern nav_msgs::Path path;
// extern ros::Publisher pub_pose_graph;
// Same but in ros2
extern rclcpp::Publisher < nav_msgs::msg::Odometry > ::SharedPtr pub_odometry;
extern rclcpp::Publisher < nav_msgs::msg::Path > ::SharedPtr pub_path;
extern rclcpp::Publisher < geometry_msgs::msg::PoseStamped > ::SharedPtr pub_pose;
extern rclcpp::Publisher < sensor_msgs::msg::PointCloud > ::SharedPtr pub_cloud;
extern rclcpp::Publisher < sensor_msgs::msg::PointCloud > ::SharedPtr pub_map;
extern rclcpp::Publisher < visualization_msgs::msg::Marker > ::SharedPtr pub_key_poses;
extern rclcpp::Publisher < geometry_msgs::msg::PoseStamped > ::SharedPtr pub_ref_pose;
extern rclcpp::Publisher < geometry_msgs::msg::PoseStamped > ::SharedPtr pub_cur_pose;
extern rclcpp::Publisher < visualization_msgs::msg::Marker > ::SharedPtr pub_key;
extern rclcpp::Publisher < visualization_msgs::msg::MarkerArray > ::SharedPtr pub_pose_graph;
extern rclcpp::Publisher < sensor_msgs::msg::Image > ::SharedPtr pub_relocalization_image;
extern int IMAGE_ROW, IMAGE_COL;

// void registerPub(ros::NodeHandle & n);
// Same but in ros2
void registerPub(rclcpp::Node::SharedPtr & n);

void pubLatestOdometry(
  const Eigen::Vector3d & P, const Eigen::Quaterniond & Q,
  // const Eigen::Vector3d & V, const std_msgs::msg::Header & header);
  // Same but in ros2
  const Eigen::Vector3d & V, const std_msgs::msg::Header & header);

void printStatistics(const Estimator & estimator, double t);

// void pubOdometry(const Estimator & estimator, const std_msgs::msg::Header & header);
// Same but in ros2
void pubOdometry(const Estimator & estimator, const std_msgs::msg::Header & header);

void pubInitialGuess(const Estimator & estimator, const std_msgs::msg::Header & header);

void pubKeyPoses(const Estimator & estimator, const std_msgs::msg::Header & header);

void pubCameraPose(const Estimator & estimator, const std_msgs::msg::Header & header);

void pubPointCloud(const Estimator & estimator, const std_msgs::msg::Header & header);

void pubTF(const Estimator & estimator, const std_msgs::msg::Header & header);

void pubKeyframe(const Estimator & estimator);

void pubRelocalization(const Estimator & estimator);
