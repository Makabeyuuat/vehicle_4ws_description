#include <ros/ros.h>
#include <cmath>
#include <vector>
#include "vehicle.hpp"       // Vehicle クラスの宣言
#include "callback.hpp"     // コールバック関数の宣言
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define PI 3.141592653589793

int main(int argc, char** argv)
{
    ros::init(argc, argv, "steering_desired_controller");
    ros::NodeHandle nh;

    // ← ここで joint_states も購読します
    ros::Subscriber joint_state_sub    = nh.subscribe("/vehicle_4ws/joint_states", 10, jointStateCallback);

    // 各トピックを別々のコールバック関数で購読
    ros::Subscriber true_bodylink_sub = nh.subscribe("/vehicle_4ws/true_body_link", 10, trueBodyLinkCallback);
    // ros::Subscriber true_carrier_sub = nh.subscribe("/vehicle_4ws/true_carrier", 10, trueCarrierCallback);
    // ros::Subscriber true_linkage_point1_sub = nh.subscribe("/vehicle_4ws/true_linkage_point1", 10, trueLinkagePoint1Callback);
    // ros::Subscriber true_linkage_point2_sub = nh.subscribe("/vehicle_4ws/true_linkage_point2", 10, trueLinkagePoint2Callback);
    // ros::Subscriber true_linkage_point3_sub = nh.subscribe("/vehicle_4ws/true_linkage_point3", 10, trueLinkagePoint3Callback);
    // ros::Subscriber true_v1_front_sub = nh.subscribe("/vehicle_4ws/true_v1_front", 10, trueV1FrontLeftSteeringCallback);
    // ros::Subscriber true_v1_rear_sub = nh.subscribe("/vehicle_4ws/true_v1_rear", 10, trueV1RearLeftSteeringCallback);

    // Vehicle クラスのインスタンス生成（v1, v2, v3 の3台）
    Vehicle vehicle1(nh, "v1");

    ros::Rate loop_rate(50); // 50Hz の制御ループ

    while (ros::ok()) {
        double current_time = ros::Time::now().toSec();
        double desired_steering = 0.5 * sin(current_time);

        // 各車両へ steering コマンドと車輪の回転速度コマンドを送信
        vehicle1.publishSteeringCommand(desired_steering, desired_steering);
       
        double wheel_command = 3.5;
        vehicle1.publishWheelCommand(wheel_command, wheel_command);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

