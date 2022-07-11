//-----------------------------------//
//  This file is part of MuJoCo.     //
//  Copyright (C) 2016 Roboti LLC.   //
//-----------------------------------//

#include "mujoco.h"
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
#include <ctime>
#include <winsock2.h>
#include "string.h"
#include "GL/glew.h"
#include "glfw3.h"
#include <openvr.h>
#include <../../mujoco200/eigen/Eigen/Dense>

using namespace vr;
using namespace Eigen;

//-------------------------------- Eye Pro Setup ---------------------------------------
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <thread>
#include "./include/SRanipal.h"
#include "./include/SRanipal_Eye.h"
#include "./include/SRanipal_Lip.h"
#include "./include/SRanipal_Enums.h"
#include "./include/SRanipal_NotRelease.h"
#include <iostream>
#pragma comment (lib, "./lib/SRanipal.lib")
using namespace ViveSR;

#define EnableEyeTracking 1
#define DisableEyeTracking 0

//#define UseEyeCallback
//#define UseEyeCallback_v2

bool GetBitMaskValidation(uint64_t mask, ViveSR::anipal::Eye::SingleEyeDataValidity SingleEyeDataType);
std::string CovertErrorCode(int error);

std::thread *t = nullptr;
bool EnableEye = false, EnableEyeV2 = false;
bool EnableLip = false, EnableLipV2 = false;
bool looping = false;
float *gaze;

/* Gains to tune */
// HMI input sensitivity for controller
double HMI_input_sensitivity_x = 0.1; //0.1
double HMI_input_sensitivity_y = 0.2;
double keyboard_input_sensitivity_x = 0.1;
double keyboard_input_sensitivity_y = 0.1;
// Repulsive force back to human
#define HMI_COM_ACTIVATION 0.008
#define TORQUE_CUTOFF_X 25 // 
#define TORQUE_CUTOFF_Y 20 //25
// #define OBS_VEL 0.008 //0.01 = 1m/s, obstacle moving speed


//-------------------------------- Controller Setup -------------------------------------

using namespace std::chrono;
using namespace std;
#define Hip 1
#define Knee 2
#define M_PI           3.14159265358979323846
// #define min(a,b) a<b?a:b
// #define max(a,b) a>b?a:b
double shift_y[11] = {0.0};
int obs_case = 1; // change obs case
//Class
float_t ctrl_update_freq = 1000;
mjtNum last_update = 0.0;
int torso_Pitch, torso_Roll, torso_Yaw, torso_X, torso_Y, torso_Z, j_hip_l, j_hip_r, j_knee_l, j_knee_r, j_wheel_l, j_wheel_r;
SATYRR_controller SATYRR_Cont;
SATYRR_STATE SATYRR_S;
Potential_Field APF;
ofstream myfile;
bool data_save_flag = true;

//----------------------------------- Input Setup ---------------------------------------

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
double forward_backward = 0.0;
double left_right = 0.0;
int cnt;
double compensated_des_dx = 0.0;
double compensated_des_dth = 0.0;
double compensated_des_x = 0.0;
double compensated_des_th = 0.0;

//------------------------------------ Obstacles ----------------------------------------

#define Obs_all 1
#define Obs_closest_one 2
double obstacle_position[Num_obstacles][3];
vector<double> sum_obstacle_pos_x;
vector<double> sum_obstacle_pos_y;
double goal_location[3];
bool obstacle_init_flag = false;
double SATYRR_X_offset = 0.0;
double SATYRR_Y_offset = 0.0;
clock_t completion_time_clock;
int seconds_passed = 0;
int completed = 0;
int map_choice = 0;
int collision_count = 0;

//------------------------------------ UDP Setup ----------------------------------------
#pragma comment(lib,"ws2_32.lib") //Winsock Library
#define SERVER "169.254.159.43" //New labview computer 
// #define SERVER "169.254.205.99"
// #define SERVER "169.254.215.171"
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
int robot_failed = 0;

void SATYRR_Init(const mjModel* m, mjData* d);

//-------------------------------- MuJoCo global data -----------------------------------
// MuJoCo model and data
mjModel* m = 0;
mjData* d = 0;

// MuJoCo visualization
mjvScene scn;
mjvOption vopt;
mjvPerturb pert;
mjrContext con;
GLFWwindow* window;
mjvCamera cam;     // abstract camera
double_t update_rate = 0.001;
double frametime = 0;

int siteID;
double car_steer = 0, car_velocity = 0;
mjtNum eye_direction[3];

//------------------------------------ INPUT Setup ----------------------------------------
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

//------------------------ Initialize Environment and SATYRR ----------------------------
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
    const float FLOAT_MAX = 0.1;
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
            if(collision_count%1==0)
            printf("%d ", collision_count);
        } 

    } // for i = 1:ncon
    mjFREESTACK

}

//------------------------------ Input and Controller -------------------------------------

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
    wheel_torque = SATYRR_Cont.f_stabilizationControl(des_state, state_, SATYRR_S.pitch_actual);
       
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

void keyboard_input(mjData *d)
{
    const double max_speed = 1.35;
    const double max_speed_yaw = 1.5;
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
        left_right += 0.001*0.15;
        left_right = min(left_right, max_speed_yaw);
        }
    else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS){
        left_right -= 0.001*0.15;
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

void hmi_input(void){
    x_COM_HMI = HMI_Data[1];
    y_COM_HMI = HMI_Data[2];
#ifdef HMI_INPUT
    // piece-wise linear function 
    // For velocity 
    double x_COM_HMI_sign = 0.0;
    double velMax = 1.2; //0.60; // in m/s
    double x_COM_HMI_db = 0.01;
    double x_COM_HMI_max = 0.08;
    double vel_slope = velMax/(x_COM_HMI_max-x_COM_HMI_db); // around 11.5
    // For yaw
    double y_COM_HMI_sign = 0.0;
    double yawMax = 0.9; //0.70; // in m/s
    double y_COM_HMI_db = 0.01;
    double y_COM_HMI_max = 0.125;
    double yaw_slope = yawMax/(y_COM_HMI_max-y_COM_HMI_db); // around 12.4
    const double x_sensitivity = 0.5;
    // Piece-wise velocity mapping
    if (x_COM_HMI > 0) x_COM_HMI_sign = 1;
    else if (x_COM_HMI < 0) x_COM_HMI_sign = -1;

    if (abs(x_COM_HMI) < x_COM_HMI_db) {
        //printf("stop or deadzone\n");
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
// #ifdef HMI_INPUT
//     // piece-wise linear function 
//     // For velocity 
//     double x_COM_HMI_sign = 0.0;
//     double velMax = 1.5; //0.60; // in m/s
//     double x_COM_HMI_db = 0.01;
//     double x_COM_HMI_max = 0.08;
//     double vel_slope = velMax/(x_COM_HMI_max-x_COM_HMI_db); // around 11.5
//     // For yaw
//     double y_COM_HMI_sign = 0.0;
//     double yawMax = 0.9; //0.70; // in m/s
//     double y_COM_HMI_db = 0.01;
//     double y_COM_HMI_max = 0.125;
//     double yaw_slope = yawMax/(y_COM_HMI_max-y_COM_HMI_db); // around 12.4
//     const double x_sensitivity = 0.5; //0.15;
//     const double y_sensitivity = 0.5; //0.25;


//     // Piece-wise velocity mapping
//     if (x_COM_HMI > 0) x_COM_HMI_sign = 1;
//     else if (x_COM_HMI < 0) x_COM_HMI_sign = -1;

//     if (abs(x_COM_HMI) < x_COM_HMI_db) {
//         forward_backward = 0;
//     }
//     else if(abs(x_COM_HMI) >= x_COM_HMI_db && abs(x_COM_HMI) < x_COM_HMI_max){  
//         forward_backward = x_COM_HMI_sign*vel_slope*(abs(x_COM_HMI) - x_COM_HMI_db);
//     }
//     else{
//         if(forward_backward > 0){
//             forward_backward -= 0.001*x_sensitivity; //x_sensitivity*0.001*vel_slope*(abs(x_COM_HMI) - x_COM_HMI_db);
//             forward_backward = max(forward_backward, 0);
//         }
//         else if(forward_backward < 0){
//             forward_backward += 0.001*x_sensitivity;// x_sensitivity*0.001*vel_slope*(abs(x_COM_HMI) - x_COM_HMI_db);
//             forward_backward = min(forward_backward, 0);
//         }
//         else{
//         forward_backward = 0.0;
//         }
//     }
    
//     // Piece-wise velocity mapping
//     if (y_COM_HMI > 0) y_COM_HMI_sign = 1;
//     else if (y_COM_HMI < 0) y_COM_HMI_sign = -1;

//     if (abs(y_COM_HMI) < y_COM_HMI_db) {
//         left_right = 0;
//     }
//     else if(abs(y_COM_HMI) >= y_COM_HMI_db && abs(y_COM_HMI) < y_COM_HMI_max){   
//         left_right = y_COM_HMI_sign*yaw_slope*(abs(y_COM_HMI) - y_COM_HMI_db);
//     }
//     else{
//         if(left_right > 0){
//             left_right -= 0.001*y_sensitivity; //y_sensitivity*0.001*yaw_slope*(abs(y_COM_HMI) - y_COM_HMI_db);
//             left_right = max(left_right, 0);
//         }
//         else if(left_right < 0){
//             left_right += 0.001*y_sensitivity; //y_sensitivity*0.001*yaw_slope*(abs(y_COM_HMI) - y_COM_HMI_db);
//             left_right = min(left_right, 0);
//         }
//         else{
//         left_right = 0.0;
//         }
//     }
    
// #endif
}

//----------------------------------- UDP Receive ---------------------------------------
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
	if( ::bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
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
        
        int count = 0;
        
		char *token = strtok(buf, ",");
		// Keep printing tokens while one of the
		// delimiters present in str[].
		while (count < HMI_DATA_COUNT)
		{
		    float received_float = strtof(token, NULL);
            HMI_Data[count] = received_float;

		    token = strtok(NULL, ",");
            count++;
		}
        hmi_input();
	}

	closesocket(s);
	WSACleanup();
}


//-------------------------------- MuJoCo functions -------------------------------------

// load model, init simulation and rendering; return 0 if error, 1 if ok
int initMuJoCo(const char* filename, int width2, int height)
{
    // init GLFW
    if( !glfwInit() )
    {
        printf("Could not initialize GLFW\n");
        return 0;
    }
    glfwWindowHint(GLFW_SAMPLES, 0);
    glfwWindowHint(GLFW_DOUBLEBUFFER, 1);
    glfwWindowHint(GLFW_RESIZABLE, 0);
    window = glfwCreateWindow(width2/4, height/2, "MuJoCo VR", NULL, NULL);
    if( !window )
    {
        printf("Could not create GLFW window\n");
        return 0;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0);
    if( glewInit()!=GLEW_OK )
        return 0;

    // activate
    if( !mj_activate("mjkey.txt") )
        return 0;

    // load and compile
    char error[1000] = "Could not load binary model";
    if( strlen(filename)>4 && !strcmp(filename+strlen(filename)-4, ".mjb") )
        m = mj_loadModel(filename, 0);
    else
        m = mj_loadXML(filename, 0, error, 1000);
    if( !m )
    {
        printf("%s\n", error);
        return 0;
    }

    // make data, run one computation to initialize all fields
    d = mj_makeData(m);
    mj_forward(m, d);

    // set offscreen buffer size to match HMD
    m->vis.global.offwidth = width2;
    m->vis.global.offheight = height;
    m->vis.quality.offsamples = 8;

    // initialize MuJoCo visualization
    mjv_makeScene(m, &scn, 1000);
    mjv_defaultOption(&vopt);
    mjv_defaultPerturb(&pert);
    mjr_defaultContext(&con);
    mjr_makeContext(m, &con, 100);

    // initialize model transform
    scn.enabletransform = 1;
    scn.translate[1] = -0.5;
    scn.translate[2] = -0.5;
    scn.rotate[0] = (float)cos(-0.25*mjPI);
    scn.rotate[1] = (float)sin(-0.25*mjPI);
    scn.scale = 1;

    // stereo mode
    scn.stereo = mjSTEREO_SIDEBYSIDE;

    //Location of camera
    siteID = mj_name2id(m, mjOBJ_SITE, "camera_vr");

    return 1;
}


// all data related to HMD
struct _vHMD_t
{
    // constant properties
    IVRSystem *system;              // opaque pointer returned by VR_Init
    uint32_t width, height;         // recommended image size per eye
    int id;                         // hmd device id
    unsigned int idtex;             // OpenGL texture id for Submit
    float eyeoffset[2][3];          // head-to-eye offsets (assume no rotation)

    // pose in room (raw data)
    float roompos[3];               // position
    float roommat[9];               // orientation matrix
};
typedef struct _vHMD_t vHMD_t;


// vr global variables
vHMD_t hmd;
// vController_t ctl[2];


//-------------------------------- VR Functions -----------------------------------------

// init vr: before MuJoCo init
void v_initPre(void)
{
    int n, i;

    // initialize runtime
    EVRInitError err = VRInitError_None;
    hmd.system = VR_Init(&err, VRApplication_Scene);
    if ( err!=VRInitError_None )
        mju_error_s("Could not init VR runtime: %s", VR_GetVRInitErrorAsEnglishDescription(err));

    // initialize compositor, set to Standing
    if( !VRCompositor() )
    {
        VR_Shutdown();
        mju_error("Could not init Compositor");
    }
    VRCompositor()->SetTrackingSpace(TrackingUniverseStanding);

    // get recommended image size
    hmd.system->GetRecommendedRenderTargetSize(&hmd.width, &hmd.height);

    // check all devices, find hmd and controllers
    hmd.id = k_unTrackedDeviceIndex_Hmd;

    // init HMD pose data
    for( n=0; n<9; n++ )
    {
        hmd.roommat[n] = 0;
        if( n<3 )
            hmd.roompos[n] = 0;
    }
    hmd.roommat[0] = 1;
    hmd.roommat[4] = 1;
    hmd.roommat[8] = 1;

    // get HMD eye-to-head offsets (no rotation)
    for( n=0; n<2; n++ )
    {
        HmdMatrix34_t tmp = hmd.system->GetEyeToHeadTransform((EVREye)n);
        hmd.eyeoffset[n][0] = tmp.m[0][3];
        hmd.eyeoffset[n][1] = tmp.m[1][3];
        hmd.eyeoffset[n][2] = tmp.m[2][3];
    }

}


// init vr: after MuJoCo init
void v_initPost(void)
{
    // set MuJoCo OpenGL frustum to match Vive
    for( int n=0; n<2; n++ )
    {
        // get frustum from vr
        float left, right, top, bottom, znear = 0.05f, zfar = 50.0f;
        hmd.system->GetProjectionRaw((EVREye)n, &left, &right, &top, &bottom);

        // set in MuJoCo
        scn.camera[n].frustum_bottom = -bottom*znear;
        scn.camera[n].frustum_top = -top*znear;
        scn.camera[n].frustum_center = 0.5f*(left + right)*znear;
        scn.camera[n].frustum_near = znear;
        scn.camera[n].frustum_far = zfar;
    }

    // create vr texture
    glActiveTexture(GL_TEXTURE2);
    glGenTextures(1, &hmd.idtex);
    glBindTexture(GL_TEXTURE_2D, hmd.idtex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 2*hmd.width, hmd.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
}


float SmoothData;
float LPF_Beta = 0.005; // 0<ß<1, smaller is slower response
// copy one pose from vr to our format
void v_copyPose(const TrackedDevicePose_t* pose, float* roompos, float* roommat)
{
    // nothing to do if not tracked
    if( !pose->bPoseIsValid )
        return;

    // pointer to data for convenience
    const HmdMatrix34_t* p = &pose->mDeviceToAbsoluteTracking; //in world coordinate

    Matrix3f robot_to_world;
    Matrix3f camera_to_world;
    Matrix3f camera_to_robot;

    //Location of the camera site attached on the car
    double r1 = d->site_xpos[3 * siteID + 0];
    double r2 = d->site_xpos[3 * siteID + 1];
    double r3 = 0.65;

    //Orientation of the camera site on car relative to the world
    robot_to_world(0,0) = d->site_xmat[9 * siteID + 0];
    robot_to_world(0,1) = d->site_xmat[9 * siteID + 1];
    robot_to_world(0,2) = d->site_xmat[9 * siteID + 2];
    robot_to_world(1,0) = d->site_xmat[9 * siteID + 3];
    robot_to_world(1,1) = d->site_xmat[9 * siteID + 4];
    robot_to_world(1,2) = d->site_xmat[9 * siteID + 5];
    robot_to_world(2,0) = d->site_xmat[9 * siteID + 6];
    robot_to_world(2,1) = d->site_xmat[9 * siteID + 7];
    robot_to_world(2,2) = d->site_xmat[9 * siteID + 8];
    // robot_to_world(0,0) = d->site_xmat[9 * siteID + 3];
    // robot_to_world(0,1) = d->site_xmat[9 * siteID + 4];
    // robot_to_world(0,2) = d->site_xmat[9 * siteID + 5];
    // robot_to_world(1,0) = d->site_xmat[9 * siteID + 0];
    // robot_to_world(1,1) = d->site_xmat[9 * siteID + 1];
    // robot_to_world(1,2) = d->site_xmat[9 * siteID + 2];
    // robot_to_world(2,0) = d->site_xmat[9 * siteID + 6];
    // robot_to_world(2,1) = d->site_xmat[9 * siteID + 7];
    // robot_to_world(2,2) = d->site_xmat[9 * siteID + 8];
    robot_to_world = robot_to_world.eval();

    //Orientation matrix to transform camera site to the world frame and VR headset to the world frame

    SmoothData = SmoothData - (LPF_Beta * (SmoothData - SATYRR_S.psi));

    Matrix3f around_x_axis_neg_90, around_x_axis_pos_90;
    around_x_axis_neg_90 << 1, 0, 0, //-90
                            0, 0, 1,
                            0, -1, 0;
    around_x_axis_pos_90 << 1, 0, 0, //90
                            0, 0, -1,
                            0, 1, 0;
    Matrix3f around_y_axis_neg_90, around_y_axis_pos_90, around_y_axis_input;
    around_y_axis_neg_90 << 0, 0, -1, //90
                            0, 1, 0,
                            1, 0, 0;
    around_y_axis_pos_90 << 0, 0, 1, //90
                            0, 1, 0,
                            -1, 0, 0;
                            
    around_y_axis_input << cos(SmoothData), 0, sin(SmoothData), //90
                            0, 1, 0,
                            -sin(SmoothData), 0, cos(SmoothData);
    Matrix3f around_z_axis_neg_90, around_z_axis_pos_90; 
    around_z_axis_neg_90 << 0, 1, 0, //-90
                            -1, 0, 0,
                            0, 0, 1;
    around_z_axis_pos_90 << 0, -1, 0, //90
                            1, 0, 0,
                            0, 0, 1;
 
    //Orientation of the VR headset relative to the world
    camera_to_world(0,0) = p->m[0][0];
    camera_to_world(0,1) = p->m[0][1];
    camera_to_world(0,2) = p->m[0][2];
    camera_to_world(1,0) = p->m[1][0];
    camera_to_world(1,1) = p->m[1][1];
    camera_to_world(1,2) = p->m[1][2];
    camera_to_world(2,0) = p->m[2][0];
    camera_to_world(2,1) = p->m[2][1];
    camera_to_world(2,2) = p->m[2][2];

    //
    //Get camera to robot orientation
    // camera_to_robot = around_y_axis_neg_90*around_y_axis_neg_90*around_z_axis_pos_90*around_z_axis_pos_90*around_x_axis_pos_90*robot_to_world*around_x_axis_pos_90.inverse().eval();
    // around_y_axis*camera_to_world;
    // camera_to_robot = robot_to_world;
    // camera_to_robot = around_y_axis_pos_90*around_y_axis_input*camera_to_world;
    camera_to_robot = around_y_axis_pos_90*around_y_axis_input*camera_to_world;

    //around_y_axis*around_x_axis*robot_to_world*around_x_axis.inverse().eval()*camera_to_world

    // Make camera XYZ positions change relative to the robot's location
    //TODO: roompos = 0 camera does not center at (0, 0, 0)! Why need to add offset -0.5?
    roompos[0] = r1; //add robot location to attached to the robot
    roompos[1] = r3-0.5; //add robot location to attached to the robot  // z direction
    roompos[2] = -r2-0.5; //add robot location to attached to the robot // x direction

    // Make camera's XYZ positions change based on VR headset location
    // roompos[0] = p->m[0][3]; 
    // roompos[1] = p->m[1][3]; 
    // roompos[2] = p->m[2][3]; 

    // Option 1: Make camera orientation change relative to the VR headset orientation
    // roommat[0] = p->m[0][0];
    // roommat[1] = p->m[0][1];
    // roommat[2] = p->m[0][2];
    // roommat[3] = p->m[1][0];
    // roommat[4] = p->m[1][1];
    // roommat[5] = p->m[1][2];
    // roommat[6] = p->m[2][0];
    // roommat[7] = p->m[2][1];
    // roommat[8] = p->m[2][2];   

    // Option 2: Make camera orientation change relative to the car orientation
    roommat[0] = camera_to_robot(0,0);
    roommat[1] = camera_to_robot(0,1);
    roommat[2] = camera_to_robot(0,2);
    roommat[3] = camera_to_robot(1,0); 
    roommat[4] = camera_to_robot(1,1);
    roommat[5] = camera_to_robot(1,2);
    roommat[6] = camera_to_robot(2,0);
    roommat[7] = camera_to_robot(2,1);
    roommat[8] = camera_to_robot(2,2);

    //If eye direction data is present
    if(!(eye_direction[0] == 0 && eye_direction[1] == 0 && eye_direction[2] == 0)){
        
        // Eye direction relative to the camera orientation
        mjtNum mat[9];
        mat[0] = roommat[0];
        mat[1] = roommat[1];
        mat[2] = roommat[2];
        mat[3] = roommat[3];
        mat[4] = roommat[4];
        mat[5] = roommat[5];
        mat[6] = roommat[6];
        mat[7] = roommat[7];
        mat[8] = roommat[8];
        mjtNum res[3];
        eye_direction[0] = eye_direction[0];
        eye_direction[1] = eye_direction[1];
        eye_direction[2] = eye_direction[2];
        mju_mulMatVec(res, mat, eye_direction, 3, 3);

        // Show where the person is looking at by moving a ball around the space
        // m->body_pos[mj_name2id(m, mjOBJ_BODY, "eye_ray_location")*3+0] = roompos[0]-3*res[0];
        // m->body_pos[mj_name2id(m, mjOBJ_BODY, "eye_ray_location")*3+1] = roompos[2]+0.5+3*res[2];
        // m->body_pos[mj_name2id(m, mjOBJ_BODY, "eye_ray_location")*3+2] = roompos[1]+0.5-3*res[1];

        // Eye direction relative to the camera position
        mjtNum rayvector[3];
        rayvector[0] = -res[0];
        rayvector[1] = res[2];
        rayvector[2] = -res[1];        
        

        /*Uncomment code below to show eye-gaze location*/
        // // mj_ray function setup
        // mjtByte flg_static = 1;//if False, we exclude geoms that are children of worldbody.
        // int bodyexclude = mj_name2id(m, mjOBJ_BODY, "eye_ray_location"); //if this is a body ID, we exclude all children geoms of this body. -1 if exlude nothing
        // const mjtByte* geomgroup = NULL; // a vector of booleans of length const.NGROUP which specifies what geom groups (stored in model.geom_group) to enable or disable.  If none, all groups are used
        // int geomid; //return pointed geom id
        // mjtNum eye_position[3];
        // eye_position[0] = r1;
        // eye_position[1] = r2;
        // eye_position[2] = r3;
        // mjtNum distance = mj_ray(m, d, eye_position, rayvector, geomgroup, flg_static, bodyexclude, &geomid);
        // //Using the distance between the eye and the geom, move the ball that indicates where the person is looking at
        // if(distance != -1){
        //     if(geomid != -1){
        //         mjtNum final_res[3];
        //         mju_scl3(final_res, rayvector, distance);
        //         final_res[0] = eye_position[0] + final_res[0];
        //         final_res[1] = eye_position[1] + final_res[1];
        //         final_res[2] = eye_position[2] + final_res[2];
        //         //XYZ position of where the person is looking at
        //         m->body_pos[mj_name2id(m, mjOBJ_BODY, "eye_ray_location")*3+0] = final_res[0];
        //         m->body_pos[mj_name2id(m, mjOBJ_BODY, "eye_ray_location")*3+1] = final_res[1];
        //         m->body_pos[mj_name2id(m, mjOBJ_BODY, "eye_ray_location")*3+2] = final_res[2];
        //         // std::cout  << "XYZLocation = (" << final_res[0] << ","<< final_res[1] <<","<< final_res[2] <<") "<< std::endl;
        //     }
        // }
        
    }
}


// update vr poses and controller states
void v_update(void)
{
    int n, i;
    mjvGeom* g;

    // get new poses
    TrackedDevicePose_t poses[k_unMaxTrackedDeviceCount];
    VRCompositor()->WaitGetPoses(poses, k_unMaxTrackedDeviceCount, NULL, 0 );

    // copy hmd pose
    v_copyPose(poses+hmd.id, hmd.roompos, hmd.roommat); 

    // adjust OpenGL scene cameras to match hmd pose
    for( n=0; n<2; n++ )
    {
        // assign position, apply eye-to-head offset
        for( i=0; i<3; i++ )
            scn.camera[n].pos[i] = hmd.roompos[i] +
                hmd.eyeoffset[n][0]*hmd.roommat[3*i+0] +
                hmd.eyeoffset[n][1]*hmd.roommat[3*i+1] +
                hmd.eyeoffset[n][2]*hmd.roommat[3*i+2];

        // assign forward and up
        // scn.camera[n].forward[0] = hmd.roommat[0];
        // scn.camera[n].forward[1] = hmd.roommat[3];
        // scn.camera[n].forward[2] = hmd.roommat[6];
        // scn.camera[n].up[0] = -hmd.roommat[1];
        // scn.camera[n].up[1] = -hmd.roommat[4];
        // scn.camera[n].up[2] = -hmd.roommat[7];

        
        scn.camera[n].forward[0] = -hmd.roommat[2];
        scn.camera[n].forward[1] = -hmd.roommat[5];
        scn.camera[n].forward[2] = -hmd.roommat[8];
        scn.camera[n].up[0] = hmd.roommat[1];
        scn.camera[n].up[1] = hmd.roommat[4];
        scn.camera[n].up[2] = hmd.roommat[7];
    }
}


// render to vr and window
void v_render(void)
{
    // resolve multi-sample offscreen buffer
    glBindFramebuffer(GL_READ_FRAMEBUFFER, con.offFBO);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, con.offFBO_r);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glBlitFramebuffer(0, 0, 2*hmd.width, hmd.height,
                      0, 0, 2*hmd.width, hmd.height,
                      GL_COLOR_BUFFER_BIT, GL_NEAREST);

    // blit to window, left only, window is half-size
    glBindFramebuffer(GL_READ_FRAMEBUFFER, con.offFBO_r);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glDrawBuffer(con.windowDoublebuffer ? GL_BACK : GL_FRONT);
    glBlitFramebuffer(0, 0, hmd.width, hmd.height,
                      0, 0, hmd.width/2, hmd.height/2,
                      GL_COLOR_BUFFER_BIT, GL_NEAREST);

    // blit to vr texture
    glActiveTexture(GL_TEXTURE2);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, con.offFBO_r);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, hmd.idtex, 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT1);
    glBlitFramebuffer(0, 0, 2*hmd.width, hmd.height,
                      0, 0, 2*hmd.width, hmd.height,
                      GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, 0, 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);

    // submit to vr
    const VRTextureBounds_t boundLeft = {0, 0, 0.5, 1};
    const VRTextureBounds_t boundRight = {0.5, 0, 1, 1};
    // HACK(aray): API_OpenGL replaced with TextureType_OpenGL
    Texture_t vTex = {(void*)hmd.idtex, TextureType_OpenGL, ColorSpace_Gamma};
    VRCompositor()->Submit(Eye_Left, &vTex, &boundLeft);
    VRCompositor()->Submit(Eye_Right, &vTex, &boundRight);

    // swap if window is double-buffered, flush just in case
    if( con.windowDoublebuffer )
        glfwSwapBuffers(window);
    glFlush();
}


//---------------------------- Callback Main Controller  -----------------------------------------
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

    //init position of obstacles
    if (obstacle_init_flag != true)
        initalize_environment(m, d);

    // Collision detection
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

APF.fnc_repulsive_force_all(m, robot_x, robot_y, sum_obstacle_pos_x, sum_obstacle_pos_y);

#ifdef CASE1_WITHOUT_FEEDBACK
    x_force = 0; // without force to human
    y_force = 0; // without force to human
    compensated_des_dx = sensitivity_x*forward_backward; // without repulsive force for controller
    compensated_des_dth = sensitivity_y*left_right; //without repulsive force for controller
#endif
#ifdef CASE2_FEEDBACK_TO_HUMAN
    //Repulsive force
    x_force = APF.obs_repul_force_x_human; // with force to human
    y_force = APF.obs_repul_force_y_human; // with force to human
    compensated_des_dx = sensitivity_x*forward_backward; // without repulsive force for controller
    compensated_des_dth = sensitivity_y*left_right; //without repulsive force for controller
#endif
#ifdef CASE3_COMPENSATED_CONTROLLER 
    x_force = 0; // without force to human
    y_force = 0; // without force to human
    compensated_des_dx = sensitivity_x*forward_backward + APF.obs_repul_force_x_controller; // with repulsive force for controller
    compensated_des_dth = sensitivity_y*left_right + APF.obs_repul_force_y_controller; // with repulsive force for controller
#endif
#ifdef CASE4_COMPENSATED_CONTROLLER_WITH_FEEDBACK_TO_HUMAN    
    x_force = APF.obs_repul_force_x_human; // with force to human
    y_force = APF.obs_repul_force_y_human; // with force to human
    compensated_des_dx = sensitivity_x*forward_backward + APF.obs_repul_force_x_controller; // with repulsive force for controller
    compensated_des_dth = sensitivity_y*left_right + APF.obs_repul_force_y_controller; // with repulsive force for controller
#endif

    compensated_des_x += compensated_des_dx*update_rate;
    compensated_des_th += compensated_des_dth*update_rate;

    //robot controller
    saytrr_controller(m, d, compensated_des_dx, compensated_des_dth, compensated_des_x, compensated_des_th);

    // Torque cutoff
    if(x_force > 0.0){
        x_force = 0.0;
    }else if(x_force < -TORQUE_CUTOFF_X){
        x_force = -TORQUE_CUTOFF_X;
    }
    if(y_force > TORQUE_CUTOFF_Y){
        y_force = TORQUE_CUTOFF_Y;
    }else if(y_force < -TORQUE_CUTOFF_Y){
        y_force = -TORQUE_CUTOFF_Y;
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
    
    if (data_save_flag){
        if(cnt % 10 == 0 && abs(SATYRR_S.pitch) < 1.54 ){
            myfile << d->time 
            << ", " << robot_x 
            << ", " << robot_y 
            << ", " << compensated_des_x
            << ", " << compensated_des_dx 
            << ", " << compensated_des_th //des yaw
            << ", " << SATYRR_S.x 
            << ", " << SATYRR_S.pitch
            << ", " << SATYRR_S.dx 
            << ", " << SATYRR_S.dpitch
            << ", " << SATYRR_S.psi
            << ", " << SATYRR_S.dpsi
            << ", " << SATYRR_S.q[10]
            << ", " << SATYRR_S.q[11]
            << ", " << SATYRR_Cont.applied_torq[2]
            << ", " << SATYRR_Cont.applied_torq[5]
            << ", " << APF.obs_repul_force_x_controller
            << ", " << APF.obs_repul_force_y_controller
            << ", " << x_force // human feedback force
            << ", " << y_force // human feedback force
            << ", " << sensitivity_x*forward_backward
            << ", " << sensitivity_x*left_right
            ;
#ifdef DYNAMIC_MAP
            const char *obstacle_name[11] = {"obstacle_1_body","obstacle_2_body","obstacle_3_body","obstacle_4_body","obstacle_5_body","obstacle_6_body","obstacle_7_body","obstacle_8_body","obstacle_9_body","obstacle_10_body","obstacle_11_body"};
            for(int i = 0; i<11; i++){
            myfile << ", " << m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+0];
            myfile << ", " << m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+1];
            }
#endif
            myfile << "\n";
        } 
    }
    
    if(cnt % 500 == 0)
    {
        //printf("X: %f, Y: %f \n", forward_backward, compensated_des_x);
        // printf("X: %f, Y: %f \n", robot_x, robot_y);
        // printf("rx: %f, ry: %f \n", SATYRR_S.x + SATYRR_X_offset, SATYRR_S.y + SATYRR_Y_offset);
        // printf("distance_to_wall = %f, rx = %f \n", APF.distance_to_wall, SATYRR_S.x + SATYRR_X_offset);
        // printf("x_force: %f, y_force: %f \n", x_force, y_force);
        // printf("state des_x=%f, x=%f, comp_x = %f %f \n",sensitivity*forward_backward, SATYRR_S.x, compensated_des_x, compensated_des_y);
        // printf("attractive force %f, %f \n",APF.attractive_force[0], APF.attractive_force[1]);
        // printf("repulsive force all %f, %f \n",APF.obs_repul_force_x, APF.obs_repul_force_y_controller);
        // printf("con: %f, %f, hum: %f, %f \n", APF.obs_repul_force_x_controller, APF.obs_repul_force_y_controller, x_force, y_force);
        // printf("comp force %f, %f comp des X %f, %f \n",compensated_des_dx,compensated_des_dth,compensated_des_x,compensated_des_th);
        // printf("distance = %f \n",APF.distance_);
        // printf("\n");
        // printf("error = %f, %f \n",goal_location[0] - (SATYRR_S.x + SATYRR_X_offset), goal_location[1]- (SATYRR_S.y+SATYRR_Y_offset));
        // printf("yaw %f, yaw %f \n", SATYRR_S.psi, SATYRR_S.y);
        cnt = 0;
    }

    cnt = cnt+1;
}


////////////////////////////////////////////////////Eye Pro functions///////////////////////////////////////////////////////

//See Eye pro documentation for more info
void streaming() {
    // Eye 
    ViveSR::anipal::Eye::EyeData_v2 eye_data_v2;
	int result = ViveSR::Error::WORK;
	while (looping) {
#ifndef UseEyeCallback_v2
        if (EnableEyeV2) {
            int result = ViveSR::anipal::Eye::GetEyeData_v2(&eye_data_v2); //Using v2
            if (result == ViveSR::Error::WORK) {
                gaze = eye_data_v2.verbose_data.combined.eye_data.gaze_direction_normalized.elem_;
                // printf("[Eye v2] Gaze: %.2f %.2f %.2f\n", gaze[0], gaze[1], gaze[2]);
                eye_direction[0] = gaze[0];
                eye_direction[1] = -gaze[1];
                eye_direction[2] = gaze[2];
            }
        }
#endif
    }
}

#ifdef UseEyeCallback_v2
void TestEyeCallback_v2(ViveSR::anipal::Eye::EyeData_v2 const &eye_data) {
    const float *gaze = eye_data.verbose_data.left.gaze_direction_normalized.elem_;
    printf("[Eye callback v2] Gaze: %.2f %.2f %.2f\n", gaze[0], gaze[1], gaze[2]);
    bool needCalibration = false;
    int error = ViveSR::anipal::Eye::IsUserNeedCalibration(&needCalibration);
    if (needCalibration) {
        printf("[Eye callback v2] Need to do calibration\n");
    }
    else {
        printf("[Eye callback v2] Don't need to do calibration\n");
    }
    const float wide_left = eye_data.expression_data.left.eye_wide;
    const float wide_right = eye_data.expression_data.right.eye_wide;
    printf("[Eye callback v2] Wide: %.2f %.2f\n", wide_left, wide_right);
}
#endif

//Initialize Eye Pro engines
void EyeActivate() {
	char str = 0;
	int error, id = NULL;   
    
    printf("Initializing Eye engines......\n");
	ViveSR::anipal::Release(ViveSR::anipal::Eye::ANIPAL_TYPE_EYE);
	ViveSR::anipal::Release(ViveSR::anipal::Lip::ANIPAL_TYPE_LIP);
    ViveSR::anipal::Release(ViveSR::anipal::Eye::ANIPAL_TYPE_EYE_V2);
    ViveSR::anipal::Release(ViveSR::anipal::Lip::ANIPAL_TYPE_LIP_V2);
    printf("Successfully release old anipal engines.\n");
    error = ViveSR::anipal::Initial(ViveSR::anipal::Eye::ANIPAL_TYPE_EYE_V2, NULL);
    if (error == ViveSR::Error::WORK) {
        EnableEyeV2 = true;
        printf("Successfully initialize version2 Eye engine.\n");
#ifdef UseEyeCallback_v2
        ViveSR::anipal::Eye::RegisterEyeDataCallback_v2(TestEyeCallback_v2);
#endif
    }
    else printf("Fail to initialize version2 Eye engine. please refer the code %d %s.\n", error, CovertErrorCode(error).c_str());

    if (t == nullptr) {
        t = new std::thread(streaming);
        if(t)   looping = true;
    }
}

//Error code for initialization
std::string CovertErrorCode(int error) {
    std::string result = "";
    switch (error) {
    case(RUNTIME_NOT_FOUND):     result = "RUNTIME_NOT_FOUND"; break;
    case(NOT_INITIAL):           result = "NOT_INITIAL"; break;
    case(FAILED):                result = "FAILED"; break;
    case(WORK):                  result = "WORK"; break;
    case(INVALID_INPUT):         result = "INVALID_INPUT"; break;
    case(FILE_NOT_FOUND):        result = "FILE_NOT_FOUND"; break;
    case(DATA_NOT_FOUND):        result = "DATA_NOT_FOUND"; break;
    case(UNDEFINED):             result = "UNDEFINED"; break;
    case(INITIAL_FAILED):        result = "INITIAL_FAILED"; break;
    case(NOT_IMPLEMENTED):       result = "NOT_IMPLEMENTED"; break;
    case(NULL_POINTER):          result = "NULL_POINTER"; break;
    case(OVER_MAX_LENGTH):       result = "OVER_MAX_LENGTH"; break;
    case(FILE_INVALID):          result = "FILE_INVALID"; break;
    case(UNINSTALL_STEAM):       result = "UNINSTALL_STEAM"; break;
    case(MEMCPY_FAIL):           result = "MEMCPY_FAIL"; break;
    case(NOT_MATCH):             result = "NOT_MATCH"; break;
    case(NODE_NOT_EXIST):        result = "NODE_NOT_EXIST"; break;
    case(UNKONW_MODULE):         result = "UNKONW_MODULE"; break;
    case(MODULE_FULL):           result = "MODULE_FULL"; break;
    case(UNKNOW_TYPE):           result = "UNKNOW_TYPE"; break;
    case(INVALID_MODULE):        result = "INVALID_MODULE"; break;
    case(INVALID_TYPE):          result = "INVALID_TYPE"; break;
    case(MEMORY_NOT_ENOUGH):     result = "MEMORY_NOT_ENOUGH"; break;
    case(BUSY):                  result = "BUSY"; break;
    case(NOT_SUPPORTED):         result = "NOT_SUPPORTED"; break;
    case(INVALID_VALUE):         result = "INVALID_VALUE"; break;
    case(COMING_SOON):           result = "COMING_SOON"; break;
    case(INVALID_CHANGE):        result = "INVALID_CHANGE"; break;
    case(TIMEOUT):               result = "TIMEOUT"; break;
    case(DEVICE_NOT_FOUND):      result = "DEVICE_NOT_FOUND"; break;
    case(INVALID_DEVICE):        result = "INVALID_DEVICE"; break;
    case(NOT_AUTHORIZED):        result = "NOT_AUTHORIZED"; break;
    case(ALREADY):               result = "ALREADY"; break;
    case(INTERNAL):              result = "INTERNAL"; break;
    case(CONNECTION_FAILED):     result = "CONNECTION_FAILED"; break;
    case(ALLOCATION_FAILED):     result = "ALLOCATION_FAILED"; break;
    case(OPERATION_FAILED):      result = "OPERATION_FAILED"; break;
    case(NOT_AVAILABLE):         result = "NOT_AVAILABLE"; break;
    case(CALLBACK_IN_PROGRESS):  result = "CALLBACK_IN_PROGRESS"; break;
    case(SERVICE_NOT_FOUND):     result = "SERVICE_NOT_FOUND"; break;
    case(DISABLED_BY_USER):      result = "DISABLED_BY_USER"; break;
    case(EULA_NOT_ACCEPT):       result = "EULA_NOT_ACCEPT"; break;
    case(RUNTIME_NO_RESPONSE):   result = "RUNTIME_NO_RESPONSE"; break;
    case(OPENCL_NOT_SUPPORT):    result = "OPENCL_NOT_SUPPORT"; break;
    case(NOT_SUPPORT_EYE_TRACKING): result = "NOT_SUPPORT_EYE_TRACKING"; break;
    case(LIP_NOT_SUPPORT):       result = "LIP_NOT_SUPPORT"; break;
    default:
        result = "No such error code";
    }
    return result;
}



///////////////////////////////////////////////////////main function ///////////////////////////////////////////////////////

int main(int argc, const char** argv)
{
    char filename[100];
    // load and compile model
    
#ifdef STATIC_MAP
    strcpy(filename, "../model/satyyr_static.xml");
#endif
#ifdef DYNAMIC_MAP
    strcpy(filename, "../model/satyyr_dynamic.xml");
#endif
    
    // pre-initialize vr
    v_initPre();
    
    //Initialize the Pro Eye system
    EyeActivate();

    // initialize MuJoCo, with image size from vr
    if( !initMuJoCo(filename, (int)(2*hmd.width), (int)hmd.height) )
        return 0;

    // post-initialize vr
    v_initPost();
    
    // SATYRR Init
    SATYRR_Init(m, d);



    // Data logging
    if (data_save_flag)
        myfile.open("../../mujoco-htcvive/src/data/" + to_string(method) + "_" + to_string(map) + "_" + to_string(trial) + ".txt",ios::out);

    // Robot car controller setup: install control callback
    mjcb_control = mycontroller;

    // Initialize time for Satyyr's controller
    mjtNum timezero = d->time;
    last_update = timezero-1.0/ctrl_update_freq;
    
    // Camera tracking the robot
    // cam.type = mjCAMERA_TRACKING;
    // cam.fixedcamid = mj_name2id(m, mjOBJ_CAMERA, "camera1");
    // cam.trackbodyid = mj_name2id(m, mjOBJ_BODY, "torso");
    // cam.azimuth = 0;
    // cam.elevation = -18;
    // cam.distance = 1.2;

    //keyboard input installed
    glfwSetKeyCallback(window, keyboard);

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

    // main loop
    double lasttm = glfwGetTime(), FPS = 10;
    frametime = d->time;
    while( !glfwWindowShouldClose(window) )
    {
        if(robot_failed != 1){
            // render new frame if it is time, or if simulation was reset
            if( (d->time-frametime)>1.0/FPS || d->time<frametime )
            {
                // create abstract scene
                mjv_updateScene(m, d, &vopt, NULL, NULL, mjCAT_ALL, &scn);

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

                // update vr poses and controller states
                v_update();

                // render in offscreen buffer
                mjrRect viewFull = {0, 0, 2*(int)hmd.width, (int)hmd.height};
                mjr_setBuffer(mjFB_OFFSCREEN, &con);
                mjr_render(viewFull, &scn, &con);

                // show FPS (window only, hmd clips it)
                FPS = 0.9*FPS + 0.1/(glfwGetTime() - lasttm);
                lasttm = glfwGetTime();
                // char fpsinfo[20];
                // sprintf(fpsinfo, "FPS %.0f", FPS);
                // mjr_overlay(mjFONT_BIG, mjGRID_BOTTOMLEFT, viewFull, fpsinfo, NULL, &con);

                // render to vr and window
                v_render();

                // save simulation time
                frametime = d->time;
            }

            // simulate
            mj_step(m, d);

            // update GUI
            glfwPollEvents();
        }
    }

    return 1;
}








