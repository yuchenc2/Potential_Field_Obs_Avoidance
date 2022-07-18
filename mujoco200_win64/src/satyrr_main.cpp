/*  Copyright © 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/

// #define Trajectory_Planning_On

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include <string>
#include <iostream>
#include <chrono>
#include <vector>
#include "satyrr_controller.hpp"
#include "potential_field.hpp"
#include <fstream>
#include <thread>
#include <cstdio>


#define Hip 1
#define Knee 2
clock_t completion_time_clock;

#define M_PI           3.14159265358979323846
using namespace std::chrono;
using namespace std;
#define min(a,b) a<b?a:b
#define max(a,b) a>b?a:b

// MuJoCo data structures
mjModel *m = NULL; // MuJoCo model
mjData *d = NULL;  // MuJoCo data
mjvCamera cam;     // abstract camera
mjvOption vopt;     // visualization options
mjvScene scn;      // abstract scene
mjrContext con;    // custom GPU context

GLFWwindow *window;
double_t update_rate = 0.001;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;
double wheel_torque = 0.0;
double yaw_damp = 0.0;

//keyboard input
double delta = 0.0;
double keyboard_input_sensitivity_x = 0.1;
double keyboard_input_sensitivity_y = 0.1;
double forward_backward = 0.0;
double left_right = 0.0;
int cnt;
double compensated_des_dx = 0.0;
double compensated_des_dth = 0.0;
double compensated_des_x = 0.0;
double compensated_des_th = 0.0;

double obstacle_position[Num_obstacles][3];
vector<double> sum_obstacle_pos_x;
vector<double> sum_obstacle_pos_y;

double goal_location[3];
bool obstacle_init_flag = false;
double SATYRR_X_offset = 0.0;
double SATYRR_Y_offset = 0.0;
int completed = 0;
int map_choice = 0;


//Class
float_t ctrl_update_freq = 1000;
mjtNum last_update = 0.0;
int torso_Pitch, torso_Roll, torso_Yaw, torso_X, torso_Y, torso_Z, j_hip_l, j_hip_r, j_knee_l, j_knee_r, j_wheel_l, j_wheel_r;

SATYRR_controller SATYRR_Cont;
SATYRR_STATE SATYRR_S;
Traj_Planning Traj_planner;

Potential_Field APF;
ofstream myfile;
bool data_save_flag = true;

// UDP setup
#include <winsock2.h>
#pragma comment(lib,"ws2_32.lib") //Winsock Library
#define SERVER "169.254.215.171"
#define BUFLEN 548	// Max length of buffer
#define PORT_SEND 54004	// The port on which to send data
#define PORT_RECEIVE 54003	// The port on which to receive data
#define ROBOT_DATA_COUNT 11
#define HMI_DATA_COUNT 9


/* UDP variables */
// HMI_Data (receive): CRIO time variable (time_CRIO), human CoM x-postion (xH), human CoM y-postion (yH), human CoM y-velocity (ydH), human CoP y-postion (pyH),
// human previous step SSP period (T_SSP_prev), human previous step DSP period (T_DSP_prev), human walking status (walking_status_H), human CoP x-postion (pxH)
float HMI_Data[HMI_DATA_COUNT] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
// Robot_Data (send): HMI X force (F_HMI_X), MuJoCo time (time_sim), received CRIO time (time_CRIO_rec), robot CoP x-position (pxR), robot sw-leg x-position (sw_x), 
// robot sw-leg z-position (sw_z), robot FSM value (FSM), robot CoM x-position (xR), robot CoM y-position (yR), robot CoM x-position traj (xR_traj), HMI Y force (F_HMI_Y)  
float Robot_Data[ROBOT_DATA_COUNT] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  
float x_COM_HMI = 0.0;
float y_COM_HMI = 0.0;
auto begin_main_receive = std::chrono::high_resolution_clock::now();

// Feedback between human and HMI
double HMI_input_sensitivity_x = 0.1;
double HMI_input_sensitivity_y = 0.2;
//Gains to tune
double human_repulse_x_gain = 10.0;
double human_repulse_y_gain = 10.0;
#define HMI_COM_ACTIVATION 0.008
#define TORQUE_CUTOFF 20
int robot_failed = 0;

void SATYRR_Init(const mjModel* m, mjData* d);




////////////////////////////////////////////function///////////////////////////////////////////////////////////

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void obstacle_control_static(const mjModel *m, mjData *d){
    srand ( time(NULL) );
    vector<int> selected_static_obs;
    string selected_static_obs_name;
    string init_name = "obstacle_";
    string end_name = "_body";
     
    int ran_num = 0;
    int ran_num_old = 0;
    int obs_num = 0;
    const float FLOAT_MIN = 0.0;
    const float FLOAT_MAX = 1.0;
    double rand_loc[7] = {0.0,};

    for(int i=0;i<7;i++){
        ran_num = (rand() %5) + (1+5*i);
        //cout <<  ran_num << "____" << ran_num_old << "\n";
        while(ran_num - ran_num_old == 5) 
            {
                //cout << "while" << "\n";
                ran_num = (rand() %5) + (1+5*i);
                if(ran_num - ran_num_old != 5) break;
            }
        //cout << ran_num <<" \n";
        selected_static_obs.push_back(ran_num);
        ran_num_old = ran_num;  
    }

    for(int i=0;i<7;i++){
        selected_static_obs_name = init_name + to_string(selected_static_obs[i]) + end_name;
        // cout << selected_static_obs_name.c_str() <<" \n";
        m->body_pos[mj_name2id(m, mjOBJ_BODY,selected_static_obs_name.c_str()) * 3 + 0] = 100; // only for x direction
    }

    for(int i=0;i<7;i++){
        rand_loc[i] = FLOAT_MIN + (float)(rand()) / ((float)(RAND_MAX/(FLOAT_MAX - FLOAT_MIN)));
        for(int j=1;j<6;j++){
            obs_num = 5*i + j;
            //cout << j << "__" << i << "__" << obs_num << "\n"; 
            if(obs_num==selected_static_obs[0] || obs_num==selected_static_obs[1] ||
               obs_num==selected_static_obs[2] || obs_num==selected_static_obs[3] ||
               obs_num==selected_static_obs[4] || obs_num==selected_static_obs[5] || obs_num==selected_static_obs[6])
               cout << "pass" << "\n";
            //change the initial location
            else{
                selected_static_obs_name = init_name + to_string(obs_num) + end_name;
                //cout << obs_num << "__" << rand_loc[i] << "__" << selected_static_obs_name << "\n"; 
                m->body_pos[mj_name2id(m, mjOBJ_BODY,selected_static_obs_name.c_str()) * 3 + 0] = rand_loc[i] + m->body_pos[mj_name2id(m, mjOBJ_BODY,selected_static_obs_name.c_str()) * 3 + 0];
            }
        }
        //cout << "\n" << endl;
    }
}

void initalize_environment(const mjModel *m, mjData *d)
{

#ifdef STATIC_MAP
    obstacle_control_static(m,d);
#endif

#if defined DYNAMIC_MAP || defined STATIC_MAP 
    for(int i=0;i<Num_obstacles;i++){
        for(int j=0;j<3;j++){
            obstacle_position[i][j] =  m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i]) * 3 + j];
        }
    }
    for(int i=0; i<Num_obstacles; i++)
    {
        sum_obstacle_pos_x.push_back(obstacle_position[i][0]);
        sum_obstacle_pos_y.push_back(obstacle_position[i][1]);
    }
#endif

    for(int j=0;j<3;j++){
        goal_location[j] = m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body") * 3 + j];
    }

    SATYRR_X_offset =  m->body_pos[mj_name2id(m, mjOBJ_BODY, "torso") * 3];
    SATYRR_Y_offset =   m->body_pos[mj_name2id(m, mjOBJ_BODY, "torso") * 3 + 1];
    obstacle_init_flag = true;

}

void SATYRR_Init(const mjModel* m, mjData* d)
{
    // Convert actuator, sensor, and joint names to ID. The ids will be used in the controller function above
    torso_Pitch = mj_name2id(m, mjOBJ_JOINT, "rotate_pitch");
    torso_Roll = mj_name2id(m, mjOBJ_JOINT, "rotate_roll");
    torso_Yaw = mj_name2id(m, mjOBJ_JOINT, "rotate_yaw");
    torso_X = mj_name2id(m, mjOBJ_JOINT, "move_x");
    torso_Y = mj_name2id(m, mjOBJ_JOINT, "move_y");
    torso_Z = mj_name2id(m, mjOBJ_JOINT, "move_z");
    j_hip_l = mj_name2id(m, mjOBJ_JOINT, "Hip_L");
    j_hip_r = mj_name2id(m, mjOBJ_JOINT, "Hip_R");
    j_knee_l = mj_name2id(m, mjOBJ_JOINT, "Knee_L");
    j_knee_r = mj_name2id(m, mjOBJ_JOINT, "Knee_R");
    j_wheel_l = mj_name2id(m, mjOBJ_JOINT, "Ankle_L");
    j_wheel_r = mj_name2id(m, mjOBJ_JOINT, "Ankle_R");

    //Init position
    d->qpos[m->jnt_qposadr[torso_Z]] = -0.5;  //Initial Height Position of the Robot
    d->qpos[m->jnt_qposadr[torso_X]] = 0.0; 
    d->qpos[m->jnt_qposadr[torso_Y]] = 0.0; 
    d->qpos[m->jnt_qposadr[torso_Pitch]] = 0.0;
    d->qpos[m->jnt_qposadr[torso_Roll]] = 0.0; 
    d->qpos[m->jnt_qposadr[torso_Yaw]] = 0.0; 
    d->qpos[m->jnt_qposadr[j_hip_l]] = SATYRR_S.desHip; 
    d->qpos[m->jnt_qposadr[j_hip_r]] = SATYRR_S.desHip; 
    d->qpos[m->jnt_qposadr[j_knee_l]] = -SATYRR_S.desHip*2; 
    d->qpos[m->jnt_qposadr[j_knee_r]] = -SATYRR_S.desHip*2; 
}

void SATYRR_state_update(const mjModel* m, mjData* d)
{
    //q_vel_hip_left & right
    SATYRR_S.q[0] = d->qvel[m->jnt_dofadr[j_hip_l]];
    SATYRR_S.q[1] = d->qvel[m->jnt_dofadr[j_hip_r]];

    //q_hip_left & right
    SATYRR_S.q[2] = d->qpos[m->jnt_qposadr[j_hip_l]];
    SATYRR_S.q[3] = d->qpos[m->jnt_qposadr[j_hip_r]];

    //q_vel_knee_left & right
    SATYRR_S.q[4] = d->qvel[m->jnt_dofadr[j_knee_l]];
    SATYRR_S.q[5] = d->qvel[m->jnt_dofadr[j_knee_r]];

    //q_knee_left & right
    SATYRR_S.q[6] = d->qpos[m->jnt_qposadr[j_knee_l]];
    SATYRR_S.q[7] = d->qpos[m->jnt_qposadr[j_knee_r]];

    //q_wheel_left & right
    SATYRR_S.q[8] = d->qvel[m->jnt_dofadr[j_wheel_l]];
    SATYRR_S.q[9] = d->qvel[m->jnt_dofadr[j_wheel_r]];

    //q_wheel & right
    SATYRR_S.q[10] = d->qpos[m->jnt_qposadr[j_wheel_l]];
    SATYRR_S.q[11] = d->qpos[m->jnt_qposadr[j_wheel_r]];

    //body state
    SATYRR_S.roll = d->qpos[m->jnt_qposadr[torso_Roll]];
    SATYRR_S.droll = (SATYRR_S.roll - SATYRR_S.roll_old) / 0.001;
    SATYRR_S.roll_old = SATYRR_S.roll;

    SATYRR_S.psi = d->qpos[m->jnt_qposadr[torso_Yaw]]; //-0.06*0.5*(SATYRR_S.q[11] - SATYRR_S.q[10])/SATYRR_S.width_wheel; //
    SATYRR_S.dpsi = (SATYRR_S.psi - SATYRR_S.psi_old) / 0.001;
    SATYRR_S.psi_old = SATYRR_S.psi;

    SATYRR_S.x =  -0.06*0.5*(SATYRR_S.q[10] + SATYRR_S.q[11]); //cos(-SATYRR_S.psi)*d->qpos[m->jnt_qposadr[torso_X]] - sin(-SATYRR_S.psi)*d->qpos[m->jnt_qposadr[torso_Y]]; //-0.06*0.5*(SATYRR_S.q[10] + SATYRR_S.q[11]); //
    SATYRR_S.dx = (SATYRR_S.x - SATYRR_S.x_old) / 0.001;
    SATYRR_S.x_old = SATYRR_S.x;

    SATYRR_S.y = cos(-SATYRR_S.psi)*d->qpos[m->jnt_qposadr[torso_X]] + sin(-SATYRR_S.psi)*d->qpos[m->jnt_qposadr[torso_Y]]; //d->qpos[m->jnt_qposadr[torso_Y]];

    //SATYRR_S.pitch = d->qpos[m->jnt_qposadr[torso_Pitch]];
    SATYRR_S.pitch = cos(-SATYRR_S.psi)*d->qpos[m->jnt_qposadr[torso_Pitch]] + sin(-SATYRR_S.psi)*SATYRR_S.roll;
    SATYRR_S.dpitch = (SATYRR_S.pitch - SATYRR_S.pitch_old) / 0.001;
    SATYRR_S.pitch_old = SATYRR_S.pitch;

    // printf("torso x and y = %f, %f \n",d->qpos[m->jnt_qposadr[torso_X]],d->qpos[m->jnt_qposadr[torso_Y]]);
    // printf("des: %f, %f \n",compensated_des_x , compensated_des_th);
    // printf("state: %f, %f, %f, %f \n",SATYRR_S.x , SATYRR_S.y, SATYRR_S.pitch, SATYRR_S.psi);
    // printf("\n");
}


void keyboard_input(mjData *d)
{
    const double max_speed = 1.35;
    const double max_speed_yaw = 1.5;

// #ifdef KEYBOARD_INPUT
//     delta += update_rate;
//     if(delta > 1)
//        delta = 0;
//     if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS){
//         forward_backward += 0.001;
//         forward_backward = min(forward_backward, max_speed);
//         }
//     else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
//         forward_backward -= 0.001;
//         forward_backward = max(forward_backward, -max_speed);
//         }
//     else
//         forward_backward = 0.0;

//     if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS){
//         left_right += 0.001;
//         left_right = min(left_right, max_speed_yaw);
//         }
//     else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS){
//         left_right -= 0.001;
//         left_right = max(left_right, -max_speed_yaw);
//         }
//     else
//         left_right = 0.0;

//     if(glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS){
//         if (data_save_flag){
//             myfile.close();
//             printf("close file!! \n");
//         }
//     }
// #endif
#ifdef KEYBOARD_INPUT

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS){
        forward_backward += 0.001*0.15;
        forward_backward = min(forward_backward, max_speed);
        }
    else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
        forward_backward -= 0.001*0.15;
        forward_backward = max(forward_backward, -max_speed);
        }
    else{
        if(forward_backward > 0){
            forward_backward -= 0.001*0.15;
            forward_backward = max(forward_backward, 0);
        }
        else if(forward_backward < 0){
            forward_backward += 0.001*0.15;
            forward_backward = min(forward_backward, 0);
        }
        else{
           forward_backward = 0.0;
        }
    }

    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS){
        left_right += 0.001;
        left_right = min(left_right, max_speed_yaw);
        }
    else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS){
        left_right -= 0.001;
        left_right = max(left_right, -max_speed_yaw);
        }
    else{
        if(forward_backward > 0){
            left_right -= 0.001*0.15;
            left_right = max(left_right, 0);
        }
        else if(left_right < 0){
            left_right += 0.001*0.15;
            left_right = min(left_right, 0);
        }
        else{
           left_right = 0.0;
        }
    }
        

    if(glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS){
        if (data_save_flag){
            myfile.close();
            printf("close file!! \n");
        }
    }
#endif
}


void saytrr_controller(const mjModel *m, mjData *d, double des_dx, double d_dyaw, double des_x, double d_yaw)
{
    // Hip controller
    SATYRR_Cont.f_jointContrl(SATYRR_S.q[2], SATYRR_S.q[3], SATYRR_S.q[0], SATYRR_S.q[1], SATYRR_S.desHip, 500 ,5, 500, 5, Hip);

    // Knee controller
    SATYRR_Cont.f_jointContrl(SATYRR_S.q[6], SATYRR_S.q[7], SATYRR_S.q[4], SATYRR_S.q[5], -SATYRR_S.desHip*2, 200 ,2, 200, 2, Knee);

    vector<double> des_state = {des_x, 0.0, des_dx, 0.0};

    vector<double> state_ = {SATYRR_S.x, SATYRR_S.pitch, SATYRR_S.dx, SATYRR_S.dpitch};
    SATYRR_S.getCOM(SATYRR_S.q[2],SATYRR_S.q[3], SATYRR_S.pitch);
    wheel_torque = SATYRR_Cont.f_stabilizationControl(des_state, state_,SATYRR_S.pitch_actual);
       
    vector<double> des_yaw = {d_yaw, 0.0};
    vector<double> curr_yaw = {SATYRR_S.psi, SATYRR_S.dpsi};

    yaw_damp = SATYRR_Cont.f_yawControl(des_yaw, curr_yaw);

    // printf("torq =%f, %f \n",wheel_torque,yaw_damp);
    SATYRR_Cont.applied_torq[2] =  wheel_torque - yaw_damp; // wheel_torque;
    SATYRR_Cont.applied_torq[5] =  wheel_torque + yaw_damp; //wheel_torque;

    //Applied torque
    if (d->time - last_update > 1.0/ctrl_update_freq)
    {
        d->ctrl[0] = -SATYRR_Cont.applied_torq[0];
        d->ctrl[1] = -SATYRR_Cont.applied_torq[1];
        d->ctrl[2] = -SATYRR_Cont.applied_torq[2];
        d->ctrl[3] = -SATYRR_Cont.applied_torq[3];
        d->ctrl[4] = -SATYRR_Cont.applied_torq[4];
        d->ctrl[5] = -SATYRR_Cont.applied_torq[5];
    }
}

void hmi_input(void){
    x_COM_HMI = HMI_Data[1];
    y_COM_HMI = HMI_Data[2];
#ifdef HMI_INPUT
    // piece-wise linear function 
    // For velocity 
    double x_COM_HMI_sign = 0.0;
    double velMax = 1.25; //0.60; // in m/s
    double x_COM_HMI_db = 0.01;
    double x_COM_HMI_max = 0.08;
    double vel_slope = velMax/(x_COM_HMI_max-x_COM_HMI_db); // around 11.5
    // For yaw
    double y_COM_HMI_sign = 0.0;
    double yawMax = 0.8; //0.70; // in m/s
    double y_COM_HMI_db = 0.01;
    double y_COM_HMI_max = 0.125;
    double yaw_slope = yawMax/(y_COM_HMI_max-y_COM_HMI_db); // around 12.4

    // Piece-wise velocity mapping
    if (x_COM_HMI > 0) x_COM_HMI_sign = 1;
    else if (x_COM_HMI < 0) x_COM_HMI_sign = -1;

    if (abs(x_COM_HMI) < x_COM_HMI_db) {
        forward_backward = 0;
    }else if(abs(x_COM_HMI) >= x_COM_HMI_db && abs(x_COM_HMI) < x_COM_HMI_max){
        forward_backward = x_COM_HMI_sign*vel_slope*(abs(x_COM_HMI) - x_COM_HMI_db);
    }else{
        forward_backward = x_COM_HMI_sign*velMax;
    }

    // Piece-wise velocity mapping
    if (y_COM_HMI > 0) y_COM_HMI_sign = 1;
    else if (y_COM_HMI < 0) y_COM_HMI_sign = -1;

    if (abs(y_COM_HMI) < y_COM_HMI_db) {
        left_right = 0;
    }else if(abs(y_COM_HMI) >= y_COM_HMI_db && abs(y_COM_HMI) < y_COM_HMI_max){
        left_right = y_COM_HMI_sign*yaw_slope*(abs(y_COM_HMI) - y_COM_HMI_db);
    }else{
        left_right = y_COM_HMI_sign*yawMax;
    }
#endif
}


// UDP receive
void udp_receive()
{
    
	struct sockaddr_in si_me, si_other;
	
	SOCKET s;
	WSADATA wsa;
    int i;
    int recv_len;
	char buf[BUFLEN];
    int slen = sizeof(si_other);
	
	//Initialise winsock
	printf("\nInitialising Winsock of receive...");
	if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
	{
		printf("Failed. Error Code : %d",WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	printf("Initialised.\n");

	//create a UDP socket
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
	{
		printf("Could not create socket : %d" , WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	printf("Socket created.\n");
	
	// zero out the structure
	memset((char *) &si_me, 0, sizeof(si_me));
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(PORT_RECEIVE);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);
	
	// bind socket to port
	if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
	{
		printf("Bind failed with error code : %d" , WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	puts("Bind done");

    
    // Measuring loop time
    int count_itr_receive = 0;
    bool fflag_receive = false;

	while(1)
	{
        		
		// receive a reply and print it
		// clear the buffer by filling null, it might have previously received data
		memset(buf,'\0', BUFLEN);
		// try to receive some data, this is a blocking call
		if ((recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == -1)
		{
			exit(EXIT_FAILURE);
		}
        
        // printf("RECEIVED DATA: \n");
        int count = 0;
        
		char *token = strtok(buf, ",");
		// Keep printing tokens while one of the
		// delimiters present in str[].
		while (count < HMI_DATA_COUNT)
		{
		    float received_float = strtof(token, NULL);
		    // printf("%f ", received_float);
            HMI_Data[count] = received_float;

		    token = strtok(NULL, ",");
            count++;
		}
        // printf("\n");
        hmi_input();
        
        // count_itr_receive++;
        // auto end_main_receive = std::chrono::high_resolution_clock::now();
        // auto elapsed_main_receive = std::chrono::duration_cast<std::chrono::nanoseconds>(end_main_receive - begin_main_receive);
        // printf("Elapsed time: %d\n", elapsed_main_receive.count() * 1e-9);
        // if(elapsed_main_receive.count() * 1e-9 >= 10.0000000 && fflag_receive == false){
        //     printf("Receive Count: %d\n", count_itr_receive);
        //     fflag_receive = true;
        // }      
	}

	closesocket(s);
	WSACleanup();
}


void obstacle_control_dynamic_init(const mjModel *m, mjData *d){
    // Initialize random number generator.
    srand(time(0)); 
    // Initial midpoint location
    int mid_point = rand() % 2;
    if(mid_point == 0){ // Top
        m->body_pos[mj_name2id(m, mjOBJ_BODY, "mid_location_1_body")*3+1] = 3.0;
        m->body_pos[mj_name2id(m, mjOBJ_BODY, "mid_location_2_body")*3+1] = -3.0;
    }else{ // Bottom
        m->body_pos[mj_name2id(m, mjOBJ_BODY, "mid_location_1_body")*3+1] = -3.0;
        m->body_pos[mj_name2id(m, mjOBJ_BODY, "mid_location_2_body")*3+1] = 3.0;
    }

    // Initial obstacle location
    for(int i = 0; i<Num_obstacles; i++){
        m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+0] = (((double)(rand() % 12))-17.0);
        m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+1] = (((double)(rand() % 7))-3.0);
    }
}




int collision_count = 0;
void contactforce(const mjModel* m, mjData* d)
{
    mjMARKSTACK
        // std::cout << "# contact points: " << d->ncon << std::endl;
    for (int i = 0; i < d->ncon; i++)
    {
        mjContact* cur_contact = &((d->contact)[i]);

        std::string geom1 = mj_id2name(m, mjOBJ_GEOM, cur_contact->geom1);
        std::string geom2 = mj_id2name(m, mjOBJ_GEOM, cur_contact->geom2);
        // std::cout << mj_id2name(m, mjOBJ_GEOM, cur_contact->geom1) << "   contact with     " << mj_id2name(m, mjOBJ_GEOM, cur_contact->geom2) << '\n'; // normal
        // int foot_location = 4; // left foot toe 0, left foot heel 1, right foot toe 2, right foot heel 3

        if (geom1.compare("floor0") != 0 && geom2.compare("floor0") != 0)
        {
            collision_count++;
            printf("collision! Count: %d \n", collision_count);
        } 

    } // for i = 1:ncon
    mjFREESTACK

}


void mycontroller(const mjModel *m, mjData *d)
{
    float x_force = 0.0; // sagital plane
    float y_force = 0.0; // frontal plane
    double sensitivity_x = 0.1;
    double sensitivity_y = 0.1;
    double robot_x = 0.0;
    double robot_y = 0.0;

    robot_x = d->qpos[m->jnt_qposadr[torso_X]] - 19; // Robot's starting position relative to the world frame
    robot_y = d->qpos[m->jnt_qposadr[torso_Y]];


    // Timer for completion time
#if defined DYNAMIC_MAP
    if(robot_x > 7.7808 && completed == 0){
        completion_time_clock = clock() - completion_time_clock;
        printf ("Completion Time: %f second\n",((float)completion_time_clock)/CLOCKS_PER_SEC);
        completed = 1;
    }
#endif  
#if defined STATIC_MAP
    if(robot_x > -1.0 && completed == 0){
        completion_time_clock = clock() - completion_time_clock;
        printf ("Completion Time: %f second\n",((float)completion_time_clock)/CLOCKS_PER_SEC);
        completed = 1;
    }
#endif 
#ifdef PATH_WIDTH_MAP
    if(robot_x > 1.9456 && robot_y < -5.6388 && completed == 0){
        completion_time_clock = clock() - completion_time_clock;
        printf ("Completion Time: %f second\n",((float)completion_time_clock)/CLOCKS_PER_SEC);
        completed = 1;
    }
#endif

    //init position of obstacles
    if (obstacle_init_flag != true)
        initalize_environment(m, d);

    //collision detection
    contactforce(m, d);

    //keyboard input always
    keyboard_input(d);

    //Update robot position
    SATYRR_state_update(m,d);

#ifdef KEYBOARD_INPUT
    sensitivity_x = keyboard_input_sensitivity_x;
    sensitivity_y = keyboard_input_sensitivity_y;
#endif
#ifdef HMI_INPUT
    sensitivity_x = HMI_input_sensitivity_x;
    sensitivity_y = HMI_input_sensitivity_y;
#endif

    
#ifdef CASE1_WITHOUT_FEEDBACK
    APF.fnc_repulsive_force_all(m, robot_x, robot_y, sum_obstacle_pos_x, sum_obstacle_pos_y);
    x_force = 0; // without force to human
    y_force = 0; // without force to human
    compensated_des_dx = sensitivity_x*forward_backward; // without repulsive force for controller
    compensated_des_dth = sensitivity_y*left_right; //without repulsive force for controller
#endif

#ifdef CASE2_FEEDBACK_TO_HUMAN
    //Repulsive force
    APF.fnc_repulsive_force_all(m, robot_x, robot_y, sum_obstacle_pos_x, sum_obstacle_pos_y);
    x_force = human_repulse_x_gain*APF.obs_repul_force_x_human; // with force to human
    y_force = human_repulse_y_gain*APF.obs_repul_force_y_human; // with force to human
    compensated_des_dx = sensitivity_x*forward_backward; // without repulsive force for controller
    compensated_des_dth = sensitivity_y*left_right; //without repulsive force for controller
#endif

#ifdef CASE3_COMPENSATED_CONTROLLER 
    x_force = 0; // without force to human
    y_force = 0; // without force to human
    APF.fnc_repulsive_force_all(m, robot_x, robot_y, sum_obstacle_pos_x, sum_obstacle_pos_y);
    compensated_des_dx = sensitivity_x*forward_backward + APF.obs_repul_force_x_controller; // with repulsive force for controller
    compensated_des_dth = sensitivity_y*left_right + APF.obs_repul_force_y_controller; // with repulsive force for controller
#endif

#ifdef CASE4_COMPENSATED_CONTROLLER_WITH_FEEDBACK_TO_HUMAN    
    APF.fnc_repulsive_force_all(m, robot_x, robot_y, sum_obstacle_pos_x, sum_obstacle_pos_y);
    x_force = human_repulse_x_gain*APF.obs_repul_force_x_human; // with force to human
    y_force = human_repulse_y_gain*APF.obs_repul_force_y_human; // with force to human
    compensated_des_dx = sensitivity_x*forward_backward + APF.obs_repul_force_x_controller; // with repulsive force for controller
    compensated_des_dth = sensitivity_y*left_right + APF.obs_repul_force_y_controller; // with repulsive force for controller
#endif

    compensated_des_x += compensated_des_dx*update_rate;
    compensated_des_th += compensated_des_dth*update_rate;

#ifdef Trajectory_Planning_On
    //Trajectory Planner
    Traj_planner.Traj_running(compensated_des_dx);

    Traj_planner.glo_cnt = Traj_planner.glo_cnt + 1;
    if(Traj_planner.glo_cnt > Traj_planner.delay_for_traj) Traj_planner.buf_cnt = Traj_planner.buf_cnt + 1;

    Traj_planner.compesated_des_pos += Traj_planner.pos_new*update_rate;
    //Traj_planner.compesated_des_vel = (Traj_planner.pos_new - Traj_planner.compesated_des_pos_o) /update_rate;
    //Traj_planner.compesated_des_pos_o = Traj_planner.pos_new;
    
    //robot controller
    saytrr_controller(m, d, Traj_planner.pos_new, compensated_des_dth, Traj_planner.compesated_des_pos, compensated_des_th);
#endif

#ifndef Trajectory_Planning_On
    //robot controller
    saytrr_controller(m, d, compensated_des_dx, compensated_des_dth, compensated_des_x, compensated_des_th);
#endif
    // command force to HMI
    // x_force = 0.0;
    // y_force = 0.0;
    // 

    // Torque cutoff
    if(x_force > TORQUE_CUTOFF){
        x_force = TORQUE_CUTOFF;
    }else if(x_force < -TORQUE_CUTOFF){
        x_force = -TORQUE_CUTOFF;
    }
    if(y_force > TORQUE_CUTOFF){
        y_force = TORQUE_CUTOFF;
    }else if(y_force < -TORQUE_CUTOFF){
        y_force = -TORQUE_CUTOFF;
    }

    if(abs(SATYRR_S.pitch) > 25.0*M_PI/180.0){
        robot_failed = 1;
    }

    // Send to HMI
    if(robot_failed == 0){
        Robot_Data[0] = x_force; 
        Robot_Data[10] = y_force;
    }else{
        Robot_Data[0] = 0; 
        Robot_Data[10] = 0;
    }


    if(cnt % 400 == 0)
    {
        // printf("new pos = %f, old pos = %f \n",Traj_planner.pos_new,compensated_des_x);
        // printf("X: %f, Y: %f \n", robot_x, robot_y);
        // printf("rx: %f, ry: %f \n", SATYRR_S.x + SATYRR_X_offset, SATYRR_S.y + SATYRR_Y_offset);
        // printf("distance_to_wall = %f, rx = %f \n", APF.distance_to_wall, SATYRR_S.x + SATYRR_X_offset);
        printf("x_force: %f, y_force: %f \n",x_force, y_force);
        // printf("state des_x=%f, x=%f, comp_x = %f %f \n",sensitivity*forward_backward, SATYRR_S.x, compensated_des_x, compensated_des_y);
        // printf("attractive force %f, %f \n",APF.attractive_force[0], APF.attractive_force[1]);
        // printf("repulsive force all %f, %f \n",APF.obs_repul_force_x, APF.obs_repul_force_y_controller);
        // printf("repulsive force %f, %f \n", APF.obs_repul_force_x_controller, APF.obs_repul_force_y_controller);
        // printf("comp force %f, %f comp des X %f, %f \n",compensated_des_dx,compensated_des_dth,compensated_des_x,compensated_des_th);
        // printf("distance = %f \n",APF.distance_);
        // printf("repulsive_force_controller_slope_force: %f\n", APF.repulsive_force_controller_slope_force);
        // printf("force_to_human: %f\n", APF.repulsive_force_human_new);
        // printf("con: %f, %f, hum: %f, %f \n", APF.obs_repul_force_x_controller, APF.obs_repul_force_y_controller, x_force, y_force);
       
        
        // printf("error = %f, %f \n",goal_location[0] - (SATYRR_S.x + SATYRR_X_offset), goal_location[1]- (SATYRR_S.y+SATYRR_Y_offset));
        
        // printf("des yaw %f, yaw %f \n",compensated_des_dy, SATYRR_S.y);
        cnt = 0;
    }
    if (data_save_flag){
        if(cnt % 1 == 0 && abs(SATYRR_S.pitch) < 1.54 ){
            myfile << d->time 
            // << "\n" << APF.obs_repul_force_x_controller 
            // << "\n" << APF.obs_repul_force_y_controller 
            // << ", " << APF.obs_repul_force_x_human 
            // << ", " << APF.obs_repul_force_y_human 
            // << ", " << APF.repulsive_force_human_new[0]
            // << ", " << APF.repulsive_force_human_old[0]
            // << ", " << APF.repulsive_force_human_final[0]
            // << ", " << APF.repulsive_force_human_slope_force[0]
            // << ", " << APF.repulsive_force_human_slope_lpf[0]
            << ", " << compensated_des_x
            << ", " << Traj_planner.compesated_des_pos 
            << ", " << SATYRR_S.x  
            // << ", " << SATYRR_S.pitch
            // << ", " << SATYRR_S.pitch_actual
            << ", " << compensated_des_dx
            << ", " << Traj_planner.pos_new
            // << ", " << SATYRR_S.dx 
            // << ", " << SATYRR_S.dpitch
            // << ", " << compensated_des_th 
            // << ", " << SATYRR_S.psi
            // << ", " << SATYRR_S.dpsi
            // << ", " << SATYRR_S.q[10]
            // << ", " << SATYRR_S.q[11]
            // << ", " << wheel_torque
            // << ", " << yaw_damp
            ;
            myfile << "\n";
        } 
    }

    cnt = cnt+1;
}


// main function
int main(int argc, const char **argv)
{
    // activate software
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";
    
#ifdef STATIC_MAP
    m = mj_loadXML("../src/satyyr_static.xml", 0, error, 1000);
#endif
#ifdef DYNAMIC_MAP
    m = mj_loadXML("../src/satyyr_dynamic.xml", 0, error, 1000);
#endif
#ifdef PATH_WIDTH_MAP
    m = mj_loadXML("../src/satyyr_path_width.xml", 0, error, 1000);
#endif
    

    if (!m)
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // SATYRR Init
    SATYRR_Init(m, d);

    // Data logging
    if (data_save_flag)
        myfile.open("../src/data_save.txt",ios::out);

    // controller setup: install control callback
    mjcb_control = mycontroller;

    mjtNum timezero = d->time;
    last_update = timezero-1.0/ctrl_update_freq;


    cam.type = mjCAMERA_TRACKING;
    cam.fixedcamid = mj_name2id(m, mjOBJ_CAMERA, "camera1");
    cam.trackbodyid = mj_name2id(m, mjOBJ_BODY, "torso");
    cam.azimuth = 0;
    cam.elevation = -18;
    cam.distance = 1.2;

    // initialize visualization data structures
    // mjv_defaultCamera(&cam);
    mjv_defaultOption(&vopt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    
    // UDP communication
    // udp_receive
    std::thread udp(udp_receive);
    udp.detach();
    // udp_send setup (client)
    struct sockaddr_in s_other_send; 
    std::string strg;
    int s_send, i_send;
    int slen=sizeof(s_other_send);
	WSADATA wsa;
	//Initialise winsock
	printf("\nInitialising Winsock for send...");
	if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
	{
		printf("Failed. Error Code : %d",WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	printf("Initialised.\n");
	//create socket
    if ( (s_send=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
		printf("socket() failed with error code : %d" , WSAGetLastError());
		exit(EXIT_FAILURE);
    }
	//setup address structure
    memset((char *) &s_other_send, 0, sizeof(s_other_send));
    s_other_send.sin_family = AF_INET;
    s_other_send.sin_port = htons(PORT_SEND);
	s_other_send.sin_addr.S_un.S_addr = inet_addr(SERVER);

    // Timer for path starts
    completion_time_clock = clock();

    //Initialize dynamic obs vel and location
#ifdef DYNAMIC_MAP
    obstacle_control_dynamic_init(m,d);
#endif


    // run main loop, target real-time simulation and 60 fps rendering
    while (!glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0){
            mj_step(m, d);
            // Send the simulated robot's data through UDP
            i_send = 0;     
            while (i_send < ROBOT_DATA_COUNT - 1 ) 
            {
                strg = strg + to_string(Robot_Data[i_send]) + ",";
                i_send = i_send + 1;
            }
            strg = strg + to_string(Robot_Data[i_send]);
            begin_main_receive = std::chrono::high_resolution_clock::now();
            if (sendto(s_send, strg.c_str(), strg.size() + 1, 0 , (struct sockaddr *) &s_other_send, slen) == SOCKET_ERROR)
            {
                printf("sendto() failed with error code : %d" , WSAGetLastError());
                exit(EXIT_FAILURE);
            }
            // cout << "Data Sent: " << strg << endl;
            strg.clear();
        }

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &vopt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    myfile.close();

// terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 1;
}
