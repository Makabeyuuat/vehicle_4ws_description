#include "initial.hpp"
#include <vector>
#include <Eigen/Dense>


//double Thetap[11] = {0.0};

Eigen::Map<Eigen::Vector3d> q_map(q_twist);
Eigen::Map<Eigen::Vector3d> qdot_map(qdot_twist);


void initial(double &t, double &dt) {
   
    t = 66.8;
    dt = 0.01;
    u1 = 0.1;

    

}
