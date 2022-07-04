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
    // printf("stablization error = %f, %f, %f, %f \n",tgt[0] - state[0],tgt[2] - state[2],0 - state[1],0 - state[3]);
    FxR = K_xW *(tgt[0] - state[0]) + K_dxW*(tgt[2] - state[2]) + K_pitch*(0 - state[1]) + K_dpitch*(0 - state[3]);
    wheel_torque = FxR *  SATYRR_r/2.0;
    // printf("wheel torq = %f \n", wheel_torque);
    return wheel_torque;
}

double SATYRR_controller::f_yawControl(vector<double> tgt, vector<double> state)
{
   yaw_torq = Kp_yaw *(tgt[0] - state[0]) + Kd_yaw * (tgt[1] - state[1]);
   return yaw_torq;
}


Traj_Planning::Traj_Planning()
{
    time = 0.0;
    glo_cnt = 1;
    buf_cnt = 0;
    cnt = delay_for_traj;
    on_traj = false;
    y_new = 0.0;

    pos_curr = 0.0;
    vel_curr = 0.0;
    acc_curr = 0.0;
    pos_old = 0.0;
    vel_old = 0.0;
    acc_old = 0.0;
    pos_prev  = 0.0;
    vel_prev  = 0.0;
    acc_prev  = 0.0;
    pos_buff   = 0.0;
    vel_buff  = 0.0;
    acc_buff  = 0.0;
    pos_new  = 0.0;
    vel_new  = 0.0;
    acc_new  = 0.0;

    via_point = 0.0;
    compesated_des_pos = 0.0;
    compesated_des_vel = 0.0;
    compesated_des_pos_o = 0.0;
}

bool Traj_Planning::Traj_running(double d_pos)
{
        //cal pos, vel, acc
        pos_curr = d_pos;
        vel_curr = pos_curr - pos_old;
        acc_curr = vel_curr - vel_old;

        pos_old = pos_curr;
        vel_old = vel_curr;

        //data buffer save
        if(glo_cnt < delay_for_traj)
        {
            pos_buf.push_back(pos_curr);
            vel_buf.push_back(vel_curr);
            acc_buf.push_back(acc_curr);
        }
        //start operation
        else if(glo_cnt == delay_for_traj)
            Func_Trajectory();
        else
        {
            Func_Trajectory();
            pos_prev = pos_buf.at(glo_cnt- delay_for_traj -buf_cnt);
            vel_prev = vel_buf.at(glo_cnt- delay_for_traj -buf_cnt);
            acc_prev = acc_buf.at(glo_cnt- delay_for_traj -buf_cnt);
        }
    return true;
}

bool Traj_Planning::Func_Trajectory()
{
    static double a[6] = {0.0,};
    double delay_ = 0.01;

    if (cnt == delay_for_traj)
    {
        a[0] = pos_prev;
        a[1] = vel_prev;
        a[2] = acc_prev / 2;
        a[3] = (20*pos_curr - 20*pos_prev - (8*vel_curr + 12*vel_prev)*delay_for_traj - (3*acc_prev - acc_curr)*pow(delay_for_traj,2)) / (2*pow(delay_for_traj,3));
        a[4] = (-30*pos_curr + 30*pos_prev + (14*vel_curr + 16*vel_prev)*delay_for_traj + (3*acc_prev - 2*acc_curr)*pow(delay_for_traj,2)) / (2*pow(delay_for_traj,4));
        a[5] = (12*pos_curr - 12*pos_prev - (6*vel_curr + 6*vel_prev)*delay_for_traj - (acc_prev - acc_curr)*pow(delay_for_traj,2)) / (2*pow(delay_for_traj,5));
        via_point = pos_curr;
        on_traj = true;
        cnt = 1;
       
        pos_buf.clear();
        vel_buf.clear();
        acc_buf.clear();
    }

    if (glo_cnt > delay_for_traj && on_traj == true)
    {
        pos_new = a[0] + a[1]*cnt + a[2]*pow(cnt,2) + a[3]*pow(cnt,3) + a[4]*pow(cnt,4) + a[5]*pow(cnt,5);
        vel_new = pos_new - pos_new_old;
        acc_new = vel_new - vel_new_old;
        cnt = cnt + 1;
        via_point = pos_new;



        pos_buf.push_back(pos_curr);
        vel_buf.push_back(vel_curr);
        acc_buf.push_back(acc_curr);

        if (cnt==delay_for_traj)
            on_traj = false;
    }

    if (glo_cnt > delay_for_traj && on_traj == false)
    {
        pos_new = pos_buf[glo_cnt-delay_for_traj-buf_cnt]; // y_new =  pos(i-delay)
        vel_new = pos_new - pos_new_old;
        acc_new = vel_new - vel_new_old;
        via_point = pos_new;
    }

    pos_new_old = pos_new;
    vel_new_old = vel_new;

    return true;
}


bool Traj_Planning::fifth_order_traj(double tt, double dir)
{
    // static double a[6] = {0.0,};
    // const double T = 0.5;

    // if(dir > 0){    
    //     pos_prev = 0.0;
    //     vel_prev = 0.0;
    //     acc_prev = 0.0;

    //     pos_curr = 1.0;
    //     vel_curr = 0.0;
    //     acc_curr = 0.0;
    // }
    // else if (dir < 0){
    //     pos_prev = 1.0;
    //     vel_prev = 0.0;
    //     acc_prev = 0.0;

    //     pos_curr = 0.0;
    //     vel_curr = 0.0;
    //     acc_curr = 0.0;

    // }

    // a[0] = pos_prev;
    // a[1] = vel_prev;
    // a[2] = acc_prev / 2;
    // a[3] = (20*pos_curr - 20*pos_prev - (8*vel_curr + 12*vel_prev)*T - (3*acc_prev - acc_curr)*pow(T,2)) / (2*pow(T,3));
    // a[4] = (-30*pos_curr + 30*pos_prev + (14*vel_curr + 16*vel_prev)*T + (3*acc_prev - 2*acc_curr)*pow(T,2)) / (2*pow(T,4));
    // a[5] = (12*pos_curr - 12*pos_prev - (6*vel_curr + 6*vel_prev)*T - (acc_prev - acc_curr)*pow(T,2)) / (2*pow(T,5));
    
    // if (dir > 0):
    //     pos_new = a[0] + a[1]*tt + a[2]*pow(tt,2) + a[3]*pow(tt,3) + a[4]*pow(tt,4) + a[5]*pow(tt,5);
   
    // else if (dir < 0):
    //     pos_new = - (a[0] + a[1] * (tt - S_T) + a2 * ((tt - S_T) ** 2) + a3 * (
    //                         (tt - S_T) ** 3) + a4 * (
    //                             (tt - S_T) ** 4) + a5 * ((tt - S_T) ** 5)) + magnitude * (
    //                                  a0 + a1 * T + a2 * (T ** 2) + a3 * (T ** 3) + a4 * (
    //                                      T ** 4) + a5 * (T ** 5))


    return true;
}


    