#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include "initial.hpp"
#include "mathFunc.h"		// 数学関数のヘッダーファイル
#include "Bezier.h"			// Bezier 曲線の関数
#include "vehicle.hpp"       // Vehicle クラスの宣言
#include "callback.hpp"     // コールバック関数の宣言
#include "initial.hpp"		// 初期値設定のヘッダーファイル
#include "getInputValue.hpp"
#include "dynamics_integrator.hpp"

using namespace std;


Search searchP(std::vector<double>& x);
Search searchPP(std::vector<double>& x);



int main(int argc, char** argv) {
    ros::init(argc, argv, "steering_desired_controller");
    ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	// jointをsubscribe
    ros::Subscriber joint_state_sub    = nh.subscribe("/vehicle_4ws/joint_states", 10, jointStateCallback);
    ros::Subscriber true_bodylink_sub = nh.subscribe("/vehicle_4ws/true_body_link", 10, trueBodyLinkCallback);
	ros::Subscriber front_left_steering_sub = nh.subscribe("/vehicle_4ws/true_front_left_steering_link", 10, trueV1FrontLeftSteeringCallback);


	PIDGains driveGains{1.0, 0.1, 0.01};  // (Kp, Ki, Kd)
    PIDGains steerGains{0.5, 0.05, 0.005};
    // Vehicle クラスのインスタンス生成
    Vehicle vehicle1(nh, "v1");
	//制御入力のクラスをインスタンス化
	getInputValue getInputValue(0.01);
	//動力学計算のクラスをインスタンス化
	DynamicsIntegrator integrator(
        M_mass, I_theta, lv, GRAV, rho,
        driveGains, steerGains,
        0.01
    );

	//gazebo上の初期値をx_oldに代入
	while (ros::ok() && !(got_body_pose && got_steering_pose)) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	//デバッグ用ログ出力
	ROS_INFO("Locking initial pose and calling initial(): x=%.3f, y=%.3f, body_yaw=%.3f ,steering=%.3f",
           true_body_pos[0], true_body_pos[1], true_body_yaw, true_steering);
	ROS_INFO("Locking initial pose and calling initial(): xdot=%.3f, ydot=%.3f, thetadot=%.3f ,phidot=%.3f",
           true_body_pos[0], true_body_pos[1], true_body_yaw, true_steering);
		   
	//初期値を設定
	initial(t_max, h);


	//曲率計算
	for (int i = 1; i < Q; i++) {
		qs[i] = qs[i - 1] + 0.00001;
	}

	for (int i = 0; i < Q; i++) {

		R[i][0] = Rx(Bx, qs, i);
		R[i][1] = Ry(By, qs, i);
		dRdq[i][0] = d1Rxdq1(Bx, qs, i);
		dRdq[i][1] = d1Rydq1(By, qs, i);

		cs[i][0] = (-(d1Rydq1(By, qs, i) * d2Rxdq2(Bx, qs, i)) + d1Rxdq1(Bx, qs, i) * d2Rydq2(By, qs, i)) /
			Power(Power(d1Rxdq1(Bx, qs, i), 2) + Power(d1Rydq1(By, qs, i), 2), 1.5);


		cs[i][1] = (Power(d1Rydq1(By, qs, i), 2) * (3 * d2Rxdq2(Bx, qs, i) * d2Rydq2(By, qs, i) - d1Rydq1(By, qs, i) * d3Rxdq3(Bx, qs, i))
			- Power(d1Rxdq1(Bx, qs, i), 2) * (3 * d2Rxdq2(Bx, qs, i) * d2Rydq2(By, qs, i) + d1Rydq1(By, qs, i) * d3Rxdq3(Bx, qs, i))
			+ Power(d1Rxdq1(Bx, qs, i), 3) * d3Rydq3(By, qs, i) + d1Rxdq1(Bx, qs, i) * d1Rydq1(By, qs, i) * (3 * Power(d2Rxdq2(Bx, qs, i), 2)
				- 3 * Power(d2Rydq2(By, qs, i), 2) + d1Rydq1(By, qs, i) * d3Rydq3(By, qs, i))) / Power(Power(d1Rxdq1(Bx, qs, i), 2) + Power(d1Rydq1(By, qs, i), 2), 3);


		cs[i][2] = (-(Power(d1Rxdq1(Bx, qs, i), 4) * (4 * d2Rydq2(By, qs, i) * d3Rxdq3(Bx, qs, i)
			+ 6 * d2Rxdq2(Bx, qs, i) * d3Rydq3(By, qs, i) + d1Rydq1(By, qs, i) * d4Rxdq4(Bx, qs, i)))
			+ Power(d1Rxdq1(Bx, qs, i), 2) * d1Rydq1(By, qs, i) * (-15 * Power(d2Rxdq2(Bx, qs, i), 3)
				+ d2Rxdq2(Bx, qs, i) * (39 * Power(d2Rydq2(By, qs, i), 2) - 2 * d1Rydq1(By, qs, i) * d3Rydq3(By, qs, i))
				+ 2 * d1Rydq1(By, qs, i) * (d2Rydq2(By, qs, i) * d3Rxdq3(Bx, qs, i) - d1Rydq1(By, qs, i) * d4Rxdq4(Bx, qs, i)))
			+ Power(d1Rydq1(By, qs, i), 3) * (3 * Power(d2Rxdq2(Bx, qs, i), 3) + d2Rxdq2(Bx, qs, i) * (-15 * Power(d2Rydq2(By, qs, i), 2)
				+ 4 * d1Rydq1(By, qs, i) * d3Rydq3(By, qs, i)) + d1Rydq1(By, qs, i) * (6 * d2Rydq2(By, qs, i) * d3Rxdq3(Bx, qs, i)
					- d1Rydq1(By, qs, i) * d4Rxdq4(Bx, qs, i))) + Power(d1Rxdq1(Bx, qs, i), 5) * d4Rydq4(By, qs, i)
			+ d1Rxdq1(Bx, qs, i) * Power(d1Rydq1(By, qs, i), 2) * (-39 * Power(d2Rxdq2(Bx, qs, i), 2) * d2Rydq2(By, qs, i)
				+ 15 * Power(d2Rydq2(By, qs, i), 3) + 10 * d1Rydq1(By, qs, i) * d2Rxdq2(Bx, qs, i) * d3Rxdq3(Bx, qs, i)
				- 10 * d1Rydq1(By, qs, i) * d2Rydq2(By, qs, i) * d3Rydq3(By, qs, i) + Power(d1Rydq1(By, qs, i), 2) * d4Rydq4(By, qs, i))
			+ Power(d1Rxdq1(Bx, qs, i), 3) * (15 * Power(d2Rxdq2(Bx, qs, i), 2) * d2Rydq2(By, qs, i) - 3 * Power(d2Rydq2(By, qs, i), 3)
				+ 10 * d1Rydq1(By, qs, i) * d2Rxdq2(Bx, qs, i) * d3Rxdq3(Bx, qs, i) - 10 * d1Rydq1(By, qs, i) * d2Rydq2(By, qs, i) * d3Rydq3(By, qs, i)
				+ 2 * Power(d1Rydq1(By, qs, i), 2) * d4Rydq4(By, qs, i))) / Power(Power(d1Rxdq1(Bx, qs, i), 2) + Power(d1Rydq1(By, qs, i), 2), 4.5);

	}


	//全探索
	searchP(x_old);


	//Gazeboのフィードバックをもとに計算
	while(ros::ok()) {
		//ROSのコールバックを処理
		double current_time = ros::Time::now().toSec();
		ros::spinOnce();
	
		//部分探索
		searchPP(x_old);
	
		//制御入力を計算し、それらをルンゲクッタ法で更新
		getInputValue.getU(x_old, sr.j);
		integrator.step(q_map, qdot_map, x_old[4], phidot, u1, u2);
		getInputValue.ddrungeKutta(x_d, x_dd);
		getInputValue.rungeKutta(x_old, x_d);

		
	
	
		//デバッグ用ログ出力
    	ROS_INFO_THROTTLE(1.0, "x_old = [%.3f, %.3f, %.3f, %.3f, %.3f]",
    	    x_old[0], x_old[1], x_old[2], x_old[3], x_old[4]);

    	ROS_INFO_THROTTLE(1.0, "x_input = [%.3f, %.3f, %.3f, %.3f, %.3f]",
    	    x_input[0], x_input[1], x_input[2], x_input[3], x_input[4]);

    	ROS_INFO_THROTTLE(1.0, "sr: j=%d, Psx=%.3f, Psy=%.3f, d=%.3f, Cs=%.3f",
    	    sr.j, sr.Psx, sr.Psy, sr.d, sr.Cs);

    	ROS_INFO_THROTTLE(1.0, "cmd: steering=%.3f, omega_rear=[%.3f, %.3f]\n",
    	    x_input[4], omega_rear[0], omega_rear[1]);

		// 各車両へ steering コマンドと車輪の回転速度コマンドを送信
        vehicle1.publishSteeringCommand(x_old[4],x_old[4]);
        vehicle1.publishWheelCommand(omega_rear[0], omega_rear[1]);

		// ループレートを維持
		loop_rate.sleep();
	}
	return 0;
}



//Ps探索
Search searchP(std::vector<double>& x) {
	int i;
	double dot = 0.0;
	double dist = 0.0;
	double dist0 = DBL_MAX;

	for (i = 0; i < Q; i++) {
		dot = (x[1] - R[i][0]) * (dRdq[i][0] / norm(Bx, By, qs, i)) + (x[2] - R[i][1]) * (dRdq[i][1] / norm(Bx, By, qs, i));

		dist = sqrt(pow((x[1] - R[i][0]), 2) + pow((x[2] - R[i][1]), 2));

		if (-0.0001 < dot && dot < 0.0001) {
			if (dist < dist0) {
				dist0 = dist;
				sr.Psx = R[i][0];
				sr.Psy = R[i][1];
				sr.d = (x[1] - R[i][0]) * (-(dRdq[i][1] / norm(Bx, By, qs, i))) + (x[2] - R[i][1]) * (dRdq[i][0] / norm(Bx, By, qs, i));
				sr.Cs = cs[i][0];
				sr.Cs1 = cs[i][1];
				sr.Cs2 = cs[i][2];
				sr.j = i;
			}
		}
	}
	return(sr);
}

Search searchPP(std::vector<double>& x) {
	int i;
	double dot = 0.0;
	double dist = 0.0;
	double dist0= DBL_MAX;

	if (sr.j < PSdist) {
		for (i = 0; i < sr.j + PSdist; i++) {
			dot = (x[1] - R[i][0]) * (dRdq[i][0] / norm(Bx, By, qs, i)) + (x[2] - R[i][1]) * (dRdq[i][1] / norm(Bx, By, qs, i));

			dist = sqrt(pow((x[1] - R[i][0]), 2) + pow((x[2] - R[i][1]), 2));

			if (-0.0001 < dot && dot < 0.0001) {
				if (dist < dist0) {
					dist0 = dist;
					sr.Psx = R[i][0];
					sr.Psy = R[i][1];
					sr.d = (x[1] - R[i][0]) * (-(dRdq[i][1] / norm(Bx, By, qs, i))) + (x[2] - R[i][1]) * (dRdq[i][0] / norm(Bx, By, qs, i));
					sr.Cs = cs[i][0];
					sr.Cs1 = cs[i][1];
					sr.Cs2 = cs[i][2];
					sr.j = i;
				}
		}
	}
	}else if (sr.j > Q - PSdist) {
		for (i = sr.j - PSdist; i < Q; i++) {
			dot = (x[1] - R[i][0]) * (dRdq[i][0] / norm(Bx, By, qs, i)) + (x[2] - R[i][1]) * (dRdq[i][1] / norm(Bx, By, qs, i));

			dist = sqrt(pow((x[1] - R[i][0]), 2) + pow((x[2] - R[i][1]), 2));

			if (-0.0001 < dot && dot < 0.0001) {
				if (dist < dist0) {
					dist0 = dist;
					sr.Psx = R[i][0];
					sr.Psy = R[i][1];
					sr.d = (x[1] - R[i][0]) * (-(dRdq[i][1] / norm(Bx, By, qs, i))) + (x[2] - R[i][1]) * (dRdq[i][0] / norm(Bx, By, qs, i));
					sr.Cs = cs[i][0];
					sr.Cs1 = cs[i][1];
					sr.Cs2 = cs[i][2];
					sr.j = i;
				}
			}
		}
	}
	else {
		for (i = sr.j - PSdist; i < sr.j+PSdist; i++) {
			dot = (x[1] - R[i][0]) * (dRdq[i][0] / norm(Bx, By, qs, i)) + (x[2] - R[i][1]) * (dRdq[i][1] / norm(Bx, By, qs, i));

			dist = sqrt(pow((x[1] - R[i][0]), 2) + pow((x[2] - R[i][1]), 2));

			if (-0.0001 < dot && dot < 0.0001) {
				if (dist < dist0) {
					dist0 = dist;
					sr.Psx = R[i][0];
					sr.Psy = R[i][1];
					sr.d = (x[1] - R[i][0]) * (-(dRdq[i][1] / norm(Bx, By, qs, i))) + (x[2] - R[i][1]) * (dRdq[i][0] / norm(Bx, By, qs, i));
					sr.Cs = cs[i][0];
					sr.Cs1 = cs[i][1];
					sr.Cs2 = cs[i][2];
					sr.j = i;
				}
			}
		}
	}
	return(sr);
}