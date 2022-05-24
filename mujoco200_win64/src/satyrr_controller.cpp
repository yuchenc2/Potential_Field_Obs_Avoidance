#include "satyrr_controller.hpp"


using namespace std;


SATYRR_STATE::SATYRR_STATE()
{
    for(int i=0;i<q_free_NUM;i++)
    {
        q_free[i] = 0.0;
    }

    for(int i=0;i<q_NUM;i++)
    {
        q[i] = 0.0;
    }

    for(int i=0;i<4;i++)
    {
        des_state_[i] = 0.0;
        state_[i] = 0.0;
    }

    for(int i=0;i<2;i++)
    {
        CoM_R[i] = 0.0;
        CoM_L[i] = 0.0;
        CoM_[i] = 0.0;
    }

}

bool SATYRR_STATE::getCOM(double q_hip_l, double q_hip_r, double pitch)
{
    CoM_R[0] = -.01884*cos(pitch) + .07329*sin(pitch) - SATYRR_leg*sin(q_hip_r - pitch) + SATYRR_leg*sin(q_hip_r + pitch);
    CoM_R[1] = .07329*cos(pitch) + .01884*sin(pitch) + SATYRR_leg*cos(q_hip_r+ pitch) + SATYRR_leg*cos(q_hip_r - pitch); 

    CoM_L[0] = -.01884*cos(pitch) + .07329*sin(pitch) - SATYRR_leg*sin(q_hip_l - pitch) + SATYRR_leg*sin(q_hip_l + pitch);
    CoM_L[1] = .07329*cos(pitch) + .01884*sin(pitch) + SATYRR_leg*cos(q_hip_l + pitch) + SATYRR_leg*cos(q_hip_l - pitch);

    CoM_[0] = (CoM_R[0] + CoM_L[0])/2;
    CoM_[1] = (CoM_R[1] + CoM_L[1])/2; 

    return true;
}

SATYRR_controller::SATYRR_controller()
{
    for(int i=0;i<actuator_NUM;i++)
    {
        applied_torq[i] = 0.0;
    }
    FxR = 0.0;
    wheel_torque = 0.0;
    yaw_torq = 0.0;

}

bool SATYRR_controller::f_jointContrl(double q1, double q2, double q_vel1, double q_vel2, double tgt, int K1, int K2, int K3, int K4, int case_)
{
    const double max_torq = 5;

    if(case_ == Hip){
        applied_torq[0] = K3*(q1 - tgt) + K4*(q_vel1 - 0); //Left
        applied_torq[3] = K1*(q2 - tgt) + K2*(q_vel2 - 0); //right
    }
    else if(case_ == Knee){
        applied_torq[1] = K3*(q1 - tgt) + K4*(q_vel1 - 0);
        applied_torq[4] = K1*(q2 - tgt) + K2*(q_vel2 - 0);
    }

    for(int id=0; id<4; id++){
        if (applied_torq[id] > max_torq) applied_torq[id] = max_torq;
        else if (applied_torq[id] < max_torq) applied_torq[id] = -max_torq;
    }
    return true;
}

double SATYRR_controller::f_stabilizationControl(vector<double> tgt, vector<double> state, double pitch_act)
{
    printf("stablization error = %f, %f, %f, %f \n",tgt[0] - state[0],tgt[2] - state[2],0 - state[1],0 - state[3]);
    FxR = K_xW *(tgt[0] - state[0]) + K_dxW*(tgt[2] - state[2]) + K_pitch*(0 - state[1]) + K_dpitch*(0 - state[3]);
    wheel_torque = FxR *  SATYRR_r/2;
    // printf("wheel torq = %f \n", wheel_torque);
    return wheel_torque;
}

double SATYRR_controller::f_yawControl(vector<double> tgt, vector<double> state)
{
   yaw_torq = Kp_yaw *(tgt[0] - state[0]) + Kd_yaw * (tgt[1] - state[1]);
   return yaw_torq;
}