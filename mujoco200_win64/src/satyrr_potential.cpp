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
#include <thread>
using namespace std::chrono;
using namespace std;

// UDP setup
#include<winsock2.h>
#pragma comment(lib,"ws2_32.lib") //Winsock Library
#define SERVER "169.254.205.99"
#define BUFLEN 548	// Max length of buffer
#define PORT_SEND 54004	// The port on which to send data
#define PORT_RECEIVE 54003	// The port on which to receive data
#define ROBOT_DATA_COUNT 11
#define HMI_DATA_COUNT 9

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

//keyboard input
float forward_backward = 0.0;
float left_right = 0.0;

#define Num_obstacles 5 
double obstacle_position[Num_obstacles][3];
bool obstacle_init_flag = false;

double robot_state[2][3];


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
void obstacleLocations(const mjModel *m, mjData *d)
{
    // printf("\n");
    // printf("Obstacle 1: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body") * 3 + 2]);
    // printf("Obstacle 2: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body") * 3 + 2]);
    // printf("Obstacle 3: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body") * 3 + 2]);
    // printf("Obstacle 4: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body") * 3 + 2]);
    // printf("Obstacle 5: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body") * 3 + 2]);
    // printf("Start Location: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "start_location_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "start_location_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "start_location_body") * 3 + 2]);
    // printf("End Location: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body") * 3 + 2]);
    // printf("Wall 1: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall1_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall1_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall1_body") * 3 + 2]);
    // printf("Wall 2: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall2_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall2_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall2_body") * 3 + 2]);
    // printf("Wall 3: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall3_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall3_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall3_body") * 3 + 2]);
    // printf("Wall 4: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall4_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall4_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "wall4_body") * 3 + 2]);
}

void potentialFieldVector(const mjModel *m, mjData *d){
    // printf("Obstacle 1 to Torso: (%f, %f, %f) \n", 
    // m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body") * 3] - m->body_pos[mj_name2id(m, mjOBJ_BODY, "torso") * 3], 
    // m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body") * 3 + 1] - m->body_pos[mj_name2id(m, mjOBJ_BODY, "torso") * 3 + 1], 
    // m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body") * 3 + 2] - m->body_pos[mj_name2id(m, mjOBJ_BODY, "torso") * 3 + 2]);
    // printf("Obstacle 2 to Torso: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body") * 3 + 2]);
    // printf("Obstacle 3 to Torso: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body") * 3 + 2]);
    // printf("Obstacle 4 to Torso: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body") * 3 + 2]);
    // printf("Obstacle 5 to Torso: (%f, %f, %f) \n", m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body") * 3], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body") * 3 + 1], m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body") * 3 + 2]);
    
}

void print_HMI_data(void){
    x_COM_HMI = HMI_Data[1];
    y_COM_HMI = HMI_Data[2];
    printf("x_COM_HMI: %f, y_COM_HMI: %f \n", x_COM_HMI, y_COM_HMI);
}

void initalize_environment(const mjModel *m, mjData *d)
{
    const char *obstacle_name[Num_obstacles] = {"obstacle_1_body","obstacle_2_body","obstacle_3_body","obstacle_4_body","obstacle_5_body"};
    for(int i=0;i<5;i++){
        for(int j=0;j<3;j++){
        obstacle_position[i][j] =  m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i]) * 3 + j];
        }
    }
    printf("Obstacle 1 to Torso: (%f, %f, %f) \n", obstacle_position[0][0], obstacle_position[0][1], obstacle_position[0][2] );
    printf("Obstacle 2 to Torso: (%f, %f, %f) \n", obstacle_position[1][0], obstacle_position[1][1], obstacle_position[1][2] );
    printf("Obstacle 3 to Torso: (%f, %f, %f) \n", obstacle_position[2][0], obstacle_position[2][1], obstacle_position[2][2] );
    printf("Obstacle 4 to Torso: (%f, %f, %f) \n", obstacle_position[3][0], obstacle_position[3][1], obstacle_position[3][2] );
    printf("Obstacle 5 to Torso: (%f, %f, %f) \n", obstacle_position[4][0], obstacle_position[4][1], obstacle_position[4][2] );

    obstacle_init_flag = true;
}
// void write_log (const mjModel* m, mjData* d){
//     mju_writeLog("Data",
//                 ("\n Time: " + std::to_string(d->time) +
//                 "\n Satyrr X:" + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "torso") * 3]) +
//                 "\n Satyrr Y:" + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "torso") * 3 + 1]) +
//                 "\n Satyrr Z:" + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "torso") * 3 + 2]) +

//                 "\n Obstacle 1X: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body")*3]) +
//                 "\n Obstacle 1Y: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body")*3 + 1]) +
//                 "\n Obstacle 1Z: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_1_body")*3 + 2]) +
//                 "\n Obstacle 2X " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body")*3]) +
//                 "\n Obstacle 2Y: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body")*3 + 1]) +
//                 "\n Obstacle 2Z: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_2_body")*3 + 2]) +
//                 "\n Obstacle 3X: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body")*3]) +
//                 "\n Obstacle 3Y: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body")*3 + 1]) +
//                 "\n Obstacle 3Z: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_3_body")*3 + 2]) +
//                 "\n Obstacle 4X: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body")*3]) +
//                 "\n Obstacle 4Y: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body")*3 + 1]) +
//                 "\n Obstacle 4Z: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_4_body")*3 + 2]) +
//                 "\n Obstacle 5X: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body")*3]) +
//                 "\n Obstacle 5Y: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body")*3 + 1]) +
//                 "\n Obstacle 5Z: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "obstacle_5_body")*3 + 2]) +

//                 "\n Start X: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "start_location_body")*3]) +
//                 "\n Start Y: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "start_location_body")*3 + 1]) +
//                 "\n Start Z: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "start_location_body")*3 + 2]) +

//                 "\n End X: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body")*3]) +
//                 "\n End Y: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body")*3 + 1]) +
//                 "\n End Z: " + std::to_string(m->body_pos[mj_name2id(m, mjOBJ_BODY, "end_location_body")*3 + 2])
//                 ).c_str()
//                 );
// }

void keyboard_input()
{
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS){
        forward_backward = -1;
    }
    else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
        forward_backward = 1;
    }
    else{
        forward_backward = 0;
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS){
        left_right = -1;
    }
    else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS){
        left_right = 1;
    }
    else{
        left_right = 0;
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


    //Calculate Distance     
    m->body_pos[mj_name2id(m, mjOBJ_BODY, "torso") * 3 + 0] = m->body_pos[mj_name2id(m, mjOBJ_BODY, "torso") * 3 + 0] + 0.001*forward_backward;
    m->body_pos[mj_name2id(m, mjOBJ_BODY, "torso") * 3 + 1] = m->body_pos[mj_name2id(m, mjOBJ_BODY, "torso") * 3 + 1] + 0.001*left_right;
    float x_force = 0.0; // sagital plane
    float y_force = 10.0; // frontal plane
    Robot_Data[0] = x_force;
    Robot_Data[10] = y_force;
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
	printf("\nInitialising Winsock...");
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
        print_HMI_data();
        
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

// main function
int main(int argc, const char **argv)
{
    // activate software
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";
    // m = mj_loadXML("../src/map1.xml", 0, error, 1000);
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

    // controller setup: install control callback
    mjcb_control = mycontroller;

    // initial position
    robot_state[0][0] = m->body_pos[mj_name2id(m, mjOBJ_BODY, "torso") * 3 + 0] = 19;
    robot_state[0][1] = m->body_pos[mj_name2id(m, mjOBJ_BODY, "torso") * 3 + 1] = 0;
    robot_state[0][2] = m->body_pos[mj_name2id(m, mjOBJ_BODY, "torso") * 3 + 2] = 0.3;

    // Get camera to follow the robot
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
	printf("\nInitialising Winsock...");
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
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // Close UDP Client
	closesocket(s_send);
	WSACleanup();

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
