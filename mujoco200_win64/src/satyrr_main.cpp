/*  Copyright © 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/


#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
//#include "utility.hpp"
#include "satyrr_controller.hpp"
#include <ctime>
#include <cstdio>
#include <iostream>
#include <vector>
#include <math.h>

// #include "Quat.h"
// #include <Eigen>

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// custom header
// mujoco_mouse_keyboard mj_mk;
// mujoco_mouse_keyboard *pC_MMK;
// pC_MMK = &mj_mk;

#define Hip 1
#define Knee 2
# define M_PI           3.14159265358979323846

using namespace std;

void SATYRR_Init(const mjModel* m, mjData* d);

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
double wheel_torque = 0.0;
double yaw_set = 0.0;
double yaw_damp = 0.0;


float_t ctrl_update_freq = 1000;
mjtNum last_update = 0.0;
int torso_Pitch, torso_Roll, torso_Yaw, torso_X, torso_Z, j_hip_l, j_hip_r, j_knee_l, j_knee_r, j_wheel_l, j_wheel_r;

//class
SATYRR_controller SATYRR_Cont;
SATYRR_STATE SATYRR_S;
//mujoco_mouse_keyboard mj_MMK;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation

    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
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
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// geom locations
void obstacleLocations(const mjModel* m, mjData* d){
    printf("\n");
    printf("Obstacle 1: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body")*3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body")*3+1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body")*3+2]);
    printf("Obstacle 2: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body")*3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body")*3+1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body")*3+2]);
    printf("Obstacle 3: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body")*3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body")*3+1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body")*3+2]);
    printf("Obstacle 4: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body")*3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body")*3+1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body")*3+2]);
    printf("Obstacle 5: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body")*3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body")*3+1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body")*3+2]);
    printf("Start Location: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "start_location_body")*3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "start_location_body")*3+1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "start_location_body")*3+2]);
    printf("End Location: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body")*3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body")*3+1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body")*3+2]);
    printf("Wall 1: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall1_body")*3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall1_body")*3+1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall1_body")*3+2]);
    printf("Wall 2: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall2_body")*3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall2_body")*3+1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall2_body")*3+2]);
    printf("Wall 3: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall3_body")*3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall3_body")*3+1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall3_body")*3+2]);
    printf("Wall 4: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall4_body")*3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall4_body")*3+1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall4_body")*3+2]);
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

    //q_vel_wheel_left & right
    SATYRR_S.q[8] = d->qvel[m->jnt_dofadr[j_wheel_l]];
    SATYRR_S.q[9] = d->qvel[m->jnt_dofadr[j_wheel_r]];

    //q_hip_wheel & right
    SATYRR_S.q[10] = d->qpos[m->jnt_qposadr[j_wheel_l]];
    SATYRR_S.q[11] = d->qpos[m->jnt_qposadr[j_wheel_r]];

    //body state
    SATYRR_S.x = d->qpos[m->jnt_qposadr[torso_X]]; //-SATYRR_r*0.5*(SATYRR_S.q[10] + SATYRR_S.q[11]);//
    SATYRR_S.dx = (SATYRR_S.x - SATYRR_S.x_old) / (1/ctrl_update_freq);
    SATYRR_S.x_old = SATYRR_S.x;

    SATYRR_S.pitch = d->qpos[m->jnt_qposadr[torso_Pitch]];
    SATYRR_S.dpitch = (SATYRR_S.pitch - SATYRR_S.pitch_old) / (1/ctrl_update_freq);
    SATYRR_S.pitch_old = SATYRR_S.pitch;

    SATYRR_S.psi = d->qpos[m->jnt_qposadr[torso_Yaw]];
    SATYRR_S.dpsi = (SATYRR_S.psi - SATYRR_S.psi_old) / (1/ctrl_update_freq);
    SATYRR_S.psi_old = SATYRR_S.psi;
}

void mycontroller(const mjModel* m, mjData* d)
{
    //state update
    SATYRR_state_update(m,d);

    // Hip controller
    SATYRR_Cont.f_jointContrl(SATYRR_S.q[2], SATYRR_S.q[3], SATYRR_S.q[0], SATYRR_S.q[1], SATYRR_S.desHip, 500 ,5, 500, 5, Hip);
    //printf("Hip: : (%f, %f, %f, %f, %f) \n", SATYRR_S.desHip, SATYRR_S.q[0], SATYRR_S.q[1], SATYRR_S.q[2], SATYRR_S.q[3]);

    // Knee controller
    SATYRR_Cont.f_jointContrl(SATYRR_S.q[6], SATYRR_S.q[7], SATYRR_S.q[4], SATYRR_S.q[5], -SATYRR_S.desHip*2, 200 ,2, 200, 2, Knee);
    //printf("Knee  : (%f, %f, %f, %f) \n", SATYRR_S.q[6], SATYRR_S.q[7], SATYRR_S.q[4], SATYRR_S.q[5]);

    //printf("STATE X, PITCH = %f, %f \n", SATYRR_S.x, SATYRR_S.angle);
    vector<double> des_state = {0.0, 0.0, 0.0, 0.0};

    //printf("x = %f, pitch = %f, wheel: %f, %f \n",SATYRR_S.x, SATYRR_S.pitch * 180 / M_PI, SATYRR_S.q[8], SATYRR_S.q[9]);
    vector<double> state_ = {SATYRR_S.x, SATYRR_S.pitch, SATYRR_S.dx, SATYRR_S.dpitch};
    wheel_torque = SATYRR_Cont.f_stabilizationControl(des_state, state_);


    vector<double> des_yaw = {yaw_set, 0.0};
    vector<double> yaw_ = {SATYRR_S.psi, SATYRR_S.dpsi};

    yaw_damp = SATYRR_Cont.f_yawControl(des_yaw, yaw_);


    // printf("x = %f, pitch = %f, torq=%f \n",SATYRR_S.x, SATYRR_S.pitch*180/M_PI, wheel_torque);
    // printf("wheel torq = %f, wheel_l = %f, wheel_r = %f \n",SATYRR_Cont.wheel_torque, SATYRR_S.q[10] , SATYRR_S.q[11]);



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

void SATYRR_Init(const mjModel* m, mjData* d)
{
    // Convert actuator, sensor, and joint names to ID. The ids will be used in the controller function above
    torso_Pitch = mj_name2id(m, mjOBJ_JOINT, "rotate_pitch");
    torso_Roll = mj_name2id(m, mjOBJ_JOINT, "rotate_roll");
    torso_Yaw = mj_name2id(m, mjOBJ_JOINT, "rotate_yaw");
    torso_X = mj_name2id(m, mjOBJ_JOINT, "move_x");
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
    d->qpos[m->jnt_qposadr[torso_Pitch]] = 0; 
    d->qpos[m->jnt_qposadr[j_hip_l]] = SATYRR_S.desHip; 
    d->qpos[m->jnt_qposadr[j_hip_r]] = SATYRR_S.desHip; 
    d->qpos[m->jnt_qposadr[j_knee_l]] = -SATYRR_S.desHip*2; 
    d->qpos[m->jnt_qposadr[j_knee_r]] = -SATYRR_S.desHip*2; 
}


// main function
int main(int argc, const char** argv)
{
    // activate software
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";
    // m = mj_loadXML("../src/map1.xml", 0, error, 1000);
    m = mj_loadXML("../src/data/satyrr_wholebody.xml", 0, error, 1000);

    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // SATYRR Init
    SATYRR_Init(m, d);
    
    // controller setup: install control callback
    mjcb_control = mycontroller;

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjtNum timezero = d->time;
    double_t update_rate = 0.001;
    last_update = timezero-1.0/ctrl_update_freq;

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
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

    

    // run main loop, target real-time simulation and 60 fps rendering
    while( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
            mj_step(m, d);

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}
