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
#include <string>
#include <iostream>
#include <chrono>
#include <vector>
#include "satyrr_controller.hpp"
#include "potential_field.hpp"

#define Hip 1
#define Knee 2
# define M_PI           3.14159265358979323846
using namespace std::chrono;
using namespace std;

// MuJoCo data structures
mjModel *m = NULL; // MuJoCo model
mjData *d = NULL;  // MuJoCo data
mjvCamera cam;     // abstract camera
mjvOption opt;     // visualization options
mjvScene scn;      // abstract scene
mjrContext con;    // custom GPU context

GLFWwindow *window;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;
double wheel_torque = 0.0;
double yaw_damp = 0.0;

//keyboard input
double sensitivity = 0.1;
double forward_backward = 0.0;
double left_right = 0.0;
int cnt;
double compensated_des_x = 0.0;
double compensated_des_y = 0.0;
//Obstacles
#define Num_obstacles 10 
double obstacle_position[Num_obstacles][3];
vector<double> sum_obstacle_pos_x;
vector<double> sum_obstacle_pos_y;

double goal_location[3];
bool obstacle_init_flag = false;
double SATYRR_X_offset = -19;

//class
float_t ctrl_update_freq = 1000;
mjtNum last_update = 0.0;
int torso_Pitch, torso_Roll, torso_Yaw, torso_X, torso_Z, j_hip_l, j_hip_r, j_knee_l, j_knee_r, j_wheel_l, j_wheel_r;

SATYRR_controller SATYRR_Cont;
SATYRR_STATE SATYRR_S;
Potential_Field APF;

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

// geom locations
// void obstacleLocations(const mjModel *m, mjData *d)
// {
//     printf("\n");
//     printf("Obstacle 1: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body") * 3 + 2]);
//     printf("Obstacle 2: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body") * 3 + 2]);
//     printf("Obstacle 3: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body") * 3 + 2]);
//     printf("Obstacle 4: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body") * 3 + 2]);
//     printf("Obstacle 5: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body") * 3 + 2]);
//     printf("Start Location: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "start_location_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "start_location_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "start_location_body") * 3 + 2]);
//     printf("End Location: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body") * 3 + 2]);
//     printf("Wall 1: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall1_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall1_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall1_body") * 3 + 2]);
//     printf("Wall 2: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall2_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall2_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall2_body") * 3 + 2]);
//     printf("Wall 3: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall3_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall3_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall3_body") * 3 + 2]);
//     printf("Wall 4: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall4_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall4_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall4_body") * 3 + 2]);
// }


void initalize_environment(const mjModel *m, mjData *d)
{
    const char *obstacle_name[Num_obstacles] = {"obstacle_1_body","obstacle_2_body","obstacle_3_body","obstacle_4_body","obstacle_5_body"
                                               ,"obstacle_6_body","obstacle_7_body","obstacle_8_body","obstacle_9_body","obstacle_10_body"};
    for(int i=0;i<Num_obstacles;i++){
        for(int j=0;j<3;j++){
        obstacle_position[i][j] =  m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i]) * 3 + j];
        }
    }
    for(int j=0;j<3;j++)
        goal_location[j] = m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body") * 3 + j];

    printf("SATYRR START : (%f, %f) \n",SATYRR_S.x + SATYRR_X_offset, SATYRR_S.y);
    printf("Obstacle 1 to Torso: (%f, %f, %f) \n", obstacle_position[0][0], obstacle_position[0][1], obstacle_position[0][2] );
    printf("Obstacle 2 to Torso: (%f, %f, %f) \n", obstacle_position[1][0], obstacle_position[1][1], obstacle_position[1][2] );
    printf("Obstacle 3 to Torso: (%f, %f, %f) \n", obstacle_position[2][0], obstacle_position[2][1], obstacle_position[2][2] );
    printf("Obstacle 4 to Torso: (%f, %f, %f) \n", obstacle_position[3][0], obstacle_position[3][1], obstacle_position[3][2] );
    printf("Obstacle 5 to Torso: (%f, %f, %f) \n", obstacle_position[4][0], obstacle_position[4][1], obstacle_position[4][2] );
    printf("Obstacle 6 to Torso: (%f, %f, %f) \n", obstacle_position[5][0], obstacle_position[5][1], obstacle_position[5][2] );
    printf("Obstacle 7 to Torso: (%f, %f, %f) \n", obstacle_position[6][0], obstacle_position[6][1], obstacle_position[6][2] );
    printf("Obstacle 8 to Torso: (%f, %f, %f) \n", obstacle_position[7][0], obstacle_position[7][1], obstacle_position[7][2] );
    printf("Obstacle 9 to Torso: (%f, %f, %f) \n", obstacle_position[8][0], obstacle_position[8][1], obstacle_position[8][2] );
    printf("Obstacle 10 to Torso: (%f, %f, %f) \n", obstacle_position[9][0], obstacle_position[9][1], obstacle_position[9][2] );
    printf("\n");
    for(int i=0; i<Num_obstacles; i++)
    {
        sum_obstacle_pos_x.push_back(obstacle_position[i][0]);
        sum_obstacle_pos_y.push_back(obstacle_position[i][1]);
    }

    printf("Obstacle 1 to Torso: (%f, %f) \n", sum_obstacle_pos_x[0], sum_obstacle_pos_y[0]);
    printf("Obstacle 2 to Torso: (%f, %f) \n", sum_obstacle_pos_x[1], sum_obstacle_pos_y[1]);
    printf("Obstacle 3 to Torso: (%f, %f) \n", sum_obstacle_pos_x[2], sum_obstacle_pos_y[2] );
    printf("Obstacle 4 to Torso: (%f, %f) \n", sum_obstacle_pos_x[3], sum_obstacle_pos_y[3] );
    printf("Obstacle 5 to Torso: (%f, %f) \n", sum_obstacle_pos_x[4], sum_obstacle_pos_y[4]);
    printf("Obstacle 6 to Torso: (%f, %f) \n", sum_obstacle_pos_x[5], sum_obstacle_pos_y[5] );
    printf("Obstacle 7 to Torso: (%f, %f) \n", sum_obstacle_pos_x[6], sum_obstacle_pos_y[6]);
    printf("Obstacle 8 to Torso: (%f, %f) \n", sum_obstacle_pos_x[7], sum_obstacle_pos_y[7]);
    printf("Obstacle 9 to Torso: (%f, %f) \n", sum_obstacle_pos_x[8], sum_obstacle_pos_y[8]);
    printf("Obstacle 10 to Torso: (%f, %f) \n", sum_obstacle_pos_x[9], sum_obstacle_pos_y[9] );
    obstacle_init_flag = true;

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
    d->qpos[m->jnt_qposadr[torso_Roll]] = 0;  
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

    SATYRR_S.y = d->qpos[m->jnt_qposadr[torso_Yaw]];

    SATYRR_S.pitch = d->qpos[m->jnt_qposadr[torso_Pitch]];
    SATYRR_S.dpitch = (SATYRR_S.pitch - SATYRR_S.pitch_old) / (1/ctrl_update_freq);
    SATYRR_S.pitch_old = SATYRR_S.pitch;

    SATYRR_S.psi = d->qpos[m->jnt_qposadr[torso_Yaw]];
    SATYRR_S.dpsi = (SATYRR_S.psi - SATYRR_S.psi_old) / (1/ctrl_update_freq);
    SATYRR_S.psi_old = SATYRR_S.psi;
}


void keyboard_input()
{
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        forward_backward += 0.01;
    else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        forward_backward -= 0.01;
    else
        forward_backward += 0;

    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        left_right += 0.001;
    
    else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        left_right -= 0.001;
    else
        left_right +=0;
}

void saytrr_controller(const mjModel *m, mjData *d, double des_x, double d_yaw)
{
    // Hip controller
    SATYRR_Cont.f_jointContrl(SATYRR_S.q[2], SATYRR_S.q[3], SATYRR_S.q[0], SATYRR_S.q[1], SATYRR_S.desHip, 500 ,5, 500, 5, Hip);

    // Knee controller
    SATYRR_Cont.f_jointContrl(SATYRR_S.q[6], SATYRR_S.q[7], SATYRR_S.q[4], SATYRR_S.q[5], -SATYRR_S.desHip*2, 200 ,2, 200, 2, Knee);

    // SATURATION
    if (des_x > 5)
        des_x = 5;
    else if (des_x < -5)
        des_x = -5;

    vector<double> des_state = {0.0, 0.0, des_x, 0.0};

    vector<double> state_ = {SATYRR_S.x, SATYRR_S.pitch, SATYRR_S.dx, SATYRR_S.dpitch};
    wheel_torque = SATYRR_Cont.f_stabilizationControl(des_state, state_);
       
    vector<double> des_yaw = {d_yaw, 0.0};
    vector<double> curr_yaw = {SATYRR_S.psi, SATYRR_S.dpsi};

    yaw_damp = SATYRR_Cont.f_yawControl(des_yaw, curr_yaw);

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

void mycontroller(const mjModel *m, mjData *d)
{
    //init position of obstacles
    if (obstacle_init_flag != true)
        initalize_environment(m, d);

    //keyboard input always
    keyboard_input();

    //Update robot position
    SATYRR_state_update(m,d);

    //Calculate Distance
    APF.fnc_cal_distance(SATYRR_S.x + SATYRR_X_offset, SATYRR_S.y, goal_location[0], goal_location[1]);
    //Attractive force

    APF.fnc_attractive_force(SATYRR_S.x + SATYRR_X_offset, SATYRR_S.y, goal_location[0], goal_location[1]);

    //Find closet obstacle
    APF.fnc_closest_obstacle(SATYRR_S.x + SATYRR_X_offset, SATYRR_S.y, sum_obstacle_pos_x, sum_obstacle_pos_y, Num_obstacles);

    //Repulsive force
    APF.fnc_repulsive_force(APF.closest_obs_dist, SATYRR_S.x, SATYRR_S.y, APF.closest_obs_pos[0], APF.closest_obs_pos[1]);


    compensated_des_x = sensitivity*forward_backward + APF.attractive_force[0] + APF.repulsive_force[0];
    compensated_des_y = sensitivity*left_right + APF.attractive_force[1] + APF.repulsive_force[1];
    //robot controller
    saytrr_controller(m, d, compensated_des_x, compensated_des_y);

    if(cnt % 100 == 0)
    {
        printf("state des_x=%f, x=%f, comp_x = %f %f \n",sensitivity*forward_backward, SATYRR_S.x, compensated_des_x, compensated_des_y);
        printf("attractive force %f, %f \n",APF.attractive_force[0], APF.attractive_force[1]);
        printf("repulsive force %f, %f \n",APF.repulsive_force[0], APF.repulsive_force[1]);
        printf("\n");
        //printf("robot x y =%f, %f \n",SATYRR_S.x + SATYRR_X_offset, SATYRR_S.y);
        //printf("distance = %f \n",APF.distance_);
        cnt = 0;
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
    m = mj_loadXML("../src/satyyr.xml", 0, error, 1000);

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

    // controller setup: install control callback
    mjcb_control = mycontroller;

    mjtNum timezero = d->time;
    double_t update_rate = 0.001;
    last_update = timezero-1.0/ctrl_update_freq;


    cam.type = mjCAMERA_TRACKING;
    cam.fixedcamid = mj_name2id(m, mjOBJ_CAMERA, "camera1");
    cam.trackbodyid = mj_name2id(m, mjOBJ_BODY, "torso");
    cam.azimuth = 180;
    cam.elevation = -18;
    cam.distance = 1.2;

    // initialize visualization data structures
    // mjv_defaultCamera(&cam);
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
    while (!glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0)
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

    // free visualization storage
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
