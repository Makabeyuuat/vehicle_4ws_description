#ifndef CALLBACK_HPP
#define CALLBACK_HPP

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

// グローバル変数の宣言（各トピックの絶対位置を保持）
extern geometry_msgs::PoseStamped Body_link_pose;
extern bool Body_link_pose_received;

extern geometry_msgs::PoseStamped v1_front_left_steering_pose;
extern bool v1_front_left_steering_pose_received;


// コールバック関数の宣言
void trueBodyLinkCallback(const nav_msgs::Odometry::ConstPtr& msg);
void trueV1FrontLeftSteeringCallback(const nav_msgs::Odometry::ConstPtr& msg);
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

#endif // CALLBACK_HPP
