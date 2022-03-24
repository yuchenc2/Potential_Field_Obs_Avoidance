#ifndef BDH_SATYRR_CONTROLLER_HPP_
#define BDH_SATYRR_CONTROLLER_HPP_

#include "mujoco.h"
#include <math.h>
#include "string.h"
#include <vector>

#define q_free_NUM 6
#define q_NUM 12
#define actuator_NUM 6

using namespace std;
#define Hip 1
#define Knee 2
#define Yaw_left 1
#define Yaw_right 2

#define SATYRR_leg  0.55
#define SATYRR_r  0.06
#define SATYRR_length  0.8

class SATYRR_STATE
{
    public:
        SATYRR_STATE();
        
        double q_free[q_free_NUM];
        double q[q_NUM];

        double des_state_[4];
        double state_[4];
        double CoM_[2];
        double CoM_R[2];
        double CoM_L[2];

        double x = 0.0;
        double x_old = 0.0;
        double dx = 0.0;
        double pitch = 0.0;
        double pitch_old = 0.0;
        double dpitch = 0.0;
        double psi = 0.0; //yaw
        double psi_old = 0.0;
        double dpsi = 0.0;
 
        //des
        const double desHip = 0.45588;

        const double gravity = 9.81;
        const double mass_wheel = 0.525989*2;
        const double mass_leg = (0.904864 + 0.394882) * 2; // thigh + shin
        const double mass_torso = 4.05;
        const double mass_upper = (0.2256 + 0.58497 + 0.111225 + 0.24689) * 2; // sholder + arm + shank + shoulder_u
        const double dt = 0.001;
        
        bool getCOM(double q_hip_l, double q_hip_r, double pitch);

};



class SATYRR_controller
{
    public:
        double FxR;
        double applied_torq[actuator_NUM];
        double wheel_torque;
        double yaw_torq;
        const int K_xW = -180; 
        const int K_pitch = -640;
        const int K_dxW = -120;
        const int K_dpitch = -70;
        const int Kp_yaw = 1.9;
        const int Kd_yaw = 0.4;

        SATYRR_controller();
        // double CoM[2]; 
        // bool f_getCOM(double q_hip[], double pitch);
        double f_stabilizationControl(vector<double> tgt, vector<double> state); 
        double f_yawControl(vector<double> tgt, vector<double> state);
        bool f_jointContrl(double q1, double q2, double q_vel1, double q_vel2, double tgt, int K1, int K2, int K3, int K4, int case_);
        // bool f_jointContrl();
        
        double joint_torq_out[2];

}; 


#endif