/*  Copyright � 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/


#include "../../../include/mujoco.h"
#include "../../../include/glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
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
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}


// geom locations
void obstacleLocations(const mjModel* m, mjData* d) {
    printf("\n");
    printf("Obstacle 1: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body") * 3 + 2]);
    printf("Obstacle 2: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body") * 3 + 2]);
    printf("Obstacle 3: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body") * 3 + 2]);
    printf("Obstacle 4: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body") * 3 + 2]);
    printf("Obstacle 5: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body") * 3 + 2]);
    printf("Start Location: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "start_location_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "start_location_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "start_location_body") * 3 + 2]);
    printf("End Location: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body") * 3 + 2]);
    printf("Wall 1: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall1_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall1_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall1_body") * 3 + 2]);
    printf("Wall 2: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall2_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall2_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall2_body") * 3 + 2]);
    printf("Wall 3: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall3_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall3_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall3_body") * 3 + 2]);
    printf("Wall 4: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall4_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall4_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall4_body") * 3 + 2]);
}


void mycontroller(const mjModel* m, mjData* d)
{
    //obstacleLocations(m, d);
}

// main function
int main(int argc, const char** argv)
{
    // activate software
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";
    m = mj_loadXML("../src/map1.xml", 0, error, 1000);

    if (!m)
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");


    // controller setup: install control callback
    mjcb_control = mycontroller;

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

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
        mjrRect viewport = { 0, 0, 0, 0 };
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
