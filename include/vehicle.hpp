#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>

// Vehicleクラスは、1台の車両に対する車輪とステアリングの制御を担当します。
class Vehicle {
public:
    // コンストラクタ
    // nh: ROSノードハンドル（ROSの各種機能へのアクセスに必要）
    // vehicle_id: 車両の識別子（例："v1", "v2", "v3"）で、トピック名の一部に使用
    Vehicle(ros::NodeHandle& nh, const std::string& vehicle_id);

    // 車両のsteeringジョイント全体に対してコマンドを送信するメソッド
    void publishSteeringCommand(double front_left_steering_value, double front_right_steering_value);

    // 車両の全ての車輪に対して回転速度コマンドを送信するメソッド
    void publishWheelCommand(double rear_left_rotation_value, double rear_right_rotation_value);

private:
    // 各車輪用のパブリッシャー
    // ros::Publisher front_left_wheel_pub;
    // ros::Publisher front_right_wheel_pub;
    ros::Publisher rear_left_wheel_pub;
    ros::Publisher rear_right_wheel_pub;

    // 各steering用のパブリッシャー
    ros::Publisher front_left_steering_pub;
    ros::Publisher front_right_steering_pub;

    // 車両識別子（トピック名を生成するために使用）
    std::string vehicle_id_;
};

#endif 