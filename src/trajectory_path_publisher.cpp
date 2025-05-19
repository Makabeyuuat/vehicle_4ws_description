#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include "initial.hpp"
#include "mathFunc.h"   // Rx, Ry
#include "Bezier.h"     // qs, Bx, By

int main(int argc, char** argv) {
  // 1) ROS 初期化
  ros::init(argc, argv, "trajectory_path_publisher");
  ros::NodeHandle nh;

  // 2) PoseArray パブリッシャー
  ros::Publisher pa_pub = nh.advertise<geometry_msgs::PoseArray>(
    "trajectory", 1, true);

  // 3) qs と R の計算
  qs[0] = 0.0;
  for (int i = 1; i < Q; ++i) {
    qs[i] = qs[i - 1] + 0.00001;
  }
  for (int i = 0; i < Q; ++i) {
    R[i][0] = Rx(Bx, qs, i);
    R[i][1] = Ry(By, qs, i);
  }

  // 4) PoseArray メッセージ作成
  geometry_msgs::PoseArray pa;
  pa.header.frame_id = "world";
  pa.header.stamp    = ros::Time::now();
  pa.poses.reserve(Q);

  for (int i = 0; i < Q; ++i) {
    geometry_msgs::Pose p;
    p.position.x = R[i][0];
    p.position.y = R[i][1];
    p.position.z = 0.1;
    // 姿勢は単位クォータニオン
    p.orientation.w = 1.0;
    p.orientation.x = 0.0;
    p.orientation.y = 0.0;
    p.orientation.z = 0.0;
    pa.poses.push_back(p);
  }

  // 5) プラグイン購読準備のため少し待機してパブリッシュ
  ros::Duration(0.5).sleep();
  pa_pub.publish(pa);

  // 6) スピンしてノードを生存
  ros::spin();
  return 0;
}
