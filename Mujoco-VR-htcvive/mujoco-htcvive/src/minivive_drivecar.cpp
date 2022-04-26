//-----------------------------------//
//  This file is part of MuJoCo.     //
//  Copyright (C) 2016 Roboti LLC.   //
//-----------------------------------//

#include "mujoco.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "GL/glew.h"
#include "glfw3.h"

#include <openvr.h>
#include <iostream>
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
#include <string>
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
double frametime = 0;

int siteID;
double car_steer = 0, car_velocity = 0;
mjtNum eye_direction[3];



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


//-------------------------------- vr functions -----------------------------------------

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
    double r3 = d->site_xpos[3 * siteID + 2];

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
    robot_to_world = robot_to_world.inverse().eval();

    //Orientation matrix to transform camera site to the world frame and VR headset to the world frame
    Matrix3f around_x_axis;
    around_x_axis << 1, 0, 0,
                    0, 0, -1,
                    0, 1, 0;
 
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

    //Get camera to robot orientation
    camera_to_robot = around_x_axis*robot_to_world*around_x_axis.inverse().eval()*camera_to_world;



    // Make camera XYZ positions change relative to the robot's location
    //TODO: roompos = 0 camera does not center at (0, 0, 0)! Why need to add offset -0.5?
    roompos[0] = r1; //add robot location to attached to the robot
    roompos[1] = r3-0.5; //add robot location to attached to the robot  // z direction
    roompos[2] = -r2-0.5; //add robot location to attached to the robot // x direction

    // Make camera's XYZ positions change based on VR headset location
    // roompos[0] = p->m[0][3]; 
    // roompos[1] = p->m[1][3]; 
    // roompos[2] = p->m[2][3]; 

    //Make camera orientation change relative to the VR headset orientation
    roommat[0] = p->m[0][0];
    roommat[1] = p->m[0][1];
    roommat[2] = p->m[0][2];
    roommat[3] = p->m[1][0];
    roommat[4] = p->m[1][1];
    roommat[5] = p->m[1][2];
    roommat[6] = p->m[2][0];
    roommat[7] = p->m[2][1];
    roommat[8] = p->m[2][2];    
    

    //Make camera orientation change relative to the car orientation
    // roommat[0] = camera_to_robot(0,0);
    // roommat[1] = camera_to_robot(0,1);
    // roommat[2] = camera_to_robot(0,2);
    // roommat[3] = camera_to_robot(1,0); 
    // roommat[4] = camera_to_robot(1,1);
    // roommat[5] = camera_to_robot(1,2);
    // roommat[6] = camera_to_robot(2,0);
    // roommat[7] = camera_to_robot(2,1);
    // roommat[8] = camera_to_robot(2,2);

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
        
        // mj_ray function setup
        mjtByte flg_static = 1;//if False, we exclude geoms that are children of worldbody.
        int bodyexclude = mj_name2id(m, mjOBJ_BODY, "eye_ray_location"); //if this is a body ID, we exclude all children geoms of this body. -1 if exlude nothing
        const mjtByte* geomgroup = NULL; // a vector of booleans of length const.NGROUP which specifies what geom groups (stored in model.geom_group) to enable or disable.  If none, all groups are used
        int geomid; //return pointed geom id
        mjtNum eye_position[3];
        eye_position[0] = r1;
        eye_position[1] = r2;
        eye_position[2] = r3;
        mjtNum distance = mj_ray(m, d, eye_position, rayvector, geomgroup, flg_static, bodyexclude, &geomid);

        //Using the distance between the eye and the geom, move the ball that indicates where the person is looking at
        if(distance != -1){
            if(geomid != -1){
                mjtNum final_res[3];
                mju_scl3(final_res, rayvector, distance);
                final_res[0] = eye_position[0] + final_res[0];
                final_res[1] = eye_position[1] + final_res[1];
                final_res[2] = eye_position[2] + final_res[2];
                //XYZ position of where the person is looking at
                m->body_pos[mj_name2id(m, mjOBJ_BODY, "eye_ray_location")*3+0] = final_res[0];
                m->body_pos[mj_name2id(m, mjOBJ_BODY, "eye_ray_location")*3+1] = final_res[1];
                m->body_pos[mj_name2id(m, mjOBJ_BODY, "eye_ray_location")*3+2] = final_res[2];
                std::cout  << "XYZLocation = (" << final_res[0] << ","<< final_res[1] <<","<< final_res[2] <<") "<< std::endl;
            }
        }
        
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


///////////////////////////////////////////////////////Robot Car Controller///////////////////////////////////////////////////////

void mycontroller(const mjModel* m, mjData* d)
{
    int actuator_pos = mj_name2id(m, mjOBJ_ACTUATOR, "buddy_steering_pos");
    int actuator_vel = mj_name2id(m, mjOBJ_ACTUATOR, "buddy_throttle_velocity");

    //Drive the car through keyboard WASD
    if(glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS){
        car_velocity = 0.1;
    }else if(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
        car_velocity = -0.1;
    }else{
        car_velocity = 0;
    }
    if(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS){
        if(car_steer < 3){
            car_steer = car_steer + 0.001;
        }else{
            car_steer = 3;
        }
    }else if(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS){
        if(car_steer > -3){
            car_steer = car_steer - 0.001;
        }else{
            car_steer = -3;
        }
    }else{
        if(car_steer > 0){
            car_steer = car_steer - 0.002;
        }else if(car_steer < 0){
            car_steer = car_steer + 0.002;
        }else{
            car_steer = 0;
        }
    }
    d->ctrl[actuator_vel] = car_velocity*5;
    d->ctrl[actuator_pos] = car_steer;
}


//Alternative way to read keyboard input
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // // do not act on release
    // if( act==GLFW_RELEASE )
    // return;

    // switch( key )
    // {
    // case 'w':                           // previous frame mode
    //     car_velocity = 0.1;
    //     std::cout << "W" << std::endl;
    //     break;
    // case 'a':                          // next frame mode
    //     car_steer = 3;
    //     break;
    // case 's':                           // previous label mode
    //     car_velocity = -0.1;
    //     break;
    // case 'd':                           // next label mode
    //     car_steer = -3;
    //     break;
        

    // default:
    //     car_velocity = 0;
    //     car_steer = 0;
    // }
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

    // get filename from command line or iteractively
    if( argc<2 ){
        strcpy(filename, "../model/one_car.xml");
    }else if( argc==2 ){
        strcpy(filename, argv[1]);
    }

    // pre-initialize vr
    v_initPre();
    
    //Initialize the Pro Eye system
    EyeActivate();

    // initialize MuJoCo, with image size from vr
    if( !initMuJoCo(filename, (int)(2*hmd.width), (int)hmd.height) )
        return 0;

    // post-initialize vr
    v_initPost();

    // Robot car controller setup: install control callback
    mjcb_control = mycontroller;
    
    //keyboard input installed
    glfwSetKeyCallback(window, keyboard);

    // main loop
    double lasttm = glfwGetTime(), FPS = 90;
    frametime = d->time;
    while( !glfwWindowShouldClose(window) )
    {
        // render new frame if it is time, or if simulation was reset
        if( (d->time-frametime)>1.0/FPS || d->time<frametime )
        {
            // create abstract scene
            mjv_updateScene(m, d, &vopt, NULL, NULL, mjCAT_ALL, &scn);

            mj_step(m, d);

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

    return 1;
}








