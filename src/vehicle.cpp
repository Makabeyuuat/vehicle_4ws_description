#include "vehicle.hpp"

// コンストラクタの実装
Vehicle::Vehicle(ros::NodeHandle& nh, const std::string& vehicle_id) : vehicle_id_(vehicle_id) {
    // トピック名の接頭辞（名前空間）
    std::string ns = "/vehicle_4ws/";

    // 車輪用パブリッシャーの初期化
    // front_left_wheel_pub = nh.advertise<std_msgs::Float64>(ns + "front_left_wheel_velocity_controller/command", 1);
    // front_right_wheel_pub = nh.advertise<std_msgs::Float64>(ns + "front_right_wheel_velocity_controller/command", 1);
    rear_left_wheel_pub = nh.advertise<std_msgs::Float64>(ns + "rear_left_wheel_velocity_controller/command", 1);
    rear_right_wheel_pub = nh.advertise<std_msgs::Float64>(ns + "rear_right_wheel_velocity_controller/command", 1);

    // STEERING用パブリッシャーの初期化
    front_left_steering_pub = nh.advertise<std_msgs::Float64>(ns + "front_left_steering_position_controller/command", 1);
    front_right_steering_pub = nh.advertise<std_msgs::Float64>(ns + "front_right_steering_position_controller/command", 1);
}

// 各steeringジョイントへ指令値を送信するメソッド
void Vehicle::publishSteeringCommand(double front_left_steering_value, double front_right_steering_value) {
    std_msgs::Float64 msg1, msg2;
    msg1.data = front_left_steering_value;
    msg2.data = front_right_steering_value;
    front_left_steering_pub.publish(msg1);
    front_right_steering_pub.publish(msg2);
}

// 各車輪へ回転速度コマンドを送信するメソッド
void Vehicle::publishWheelCommand(double rear_left_rotation_value, double rear_right_rotation_value) {
    std_msgs::Float64 msg1, msg2, msg3, msg4;
    // msg1.data = front_left_rotation_value;
    // msg2.data = front_right_rotation_value;
    msg3.data = rear_left_rotation_value;
    msg4.data = rear_right_rotation_value;
    // front_left_wheel_pub.publish(msg1);
    // front_right_wheel_pub.publish(msg2);
    rear_left_wheel_pub.publish(msg3);
    rear_right_wheel_pub.publish(msg4);
}


