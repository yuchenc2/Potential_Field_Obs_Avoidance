#ifndef BDH_POTENTIAL_FIELD_HPP_
#define BDH_POTENTIAL_FIELD_HPP_

#include "mujoco.h"
#include <math.h>
#include "string.h"
#include <vector>
#include <ctime>

using namespace std;
#define sgn(v) ( ( (v) < 0 ) ? -1 : ( (v) > 0 ) )
#define M_PI           3.14159265358979323846

/*   Decide cases for feedback  */
// #define CASE1_WITHOUT_FEEDBACK //NOTHING
#define CASE2_FEEDBACK_TO_HUMAN 
// #define CASE3_COMPENSATED_CONTROLLER 
// #define CASE4_COMPENSATED_CONTROLLER_WITH_FEEDBACK_TO_HUMAN

/* Decide control input */
#define KEYBOARD_INPUT 
// #define HMI_INPUT

/* Map Cases */
// #define STATIC_MAP
#define DYNAMIC_MAP


//Obstacles
#ifdef STATIC_MAP
    #define Num_obstacles 26
    static const char *obstacle_name[Num_obstacles] = {"obstacle_1_body","obstacle_2_body","obstacle_3_body","obstacle_4_body","obstacle_5_body"
                                               ,"obstacle_6_body","obstacle_7_body","obstacle_8_body","obstacle_9_body","obstacle_10_body"
                                               ,"obstacle_11_body","obstacle_12_body","obstacle_13_body","obstacle_14_body","obstacle_15_body", "obstacle_16_body"
                                               ,"obstacle_17_body","obstacle_18_body","obstacle_19_body","obstacle_20_body","obstacle_21_body","obstacle_22_body"
                                               ,"obstacle_23_body","obstacle_24_body","obstacle_25_body","obstacle_26_body"};
#endif
#ifdef DYNAMIC_MAP
    #define Num_obstacles 100
    static const char *obstacle_name[Num_obstacles] = {"obstacle_1_body","obstacle_2_body","obstacle_3_body","obstacle_4_body","obstacle_5_body","obstacle_6_body","obstacle_7_body","obstacle_8_body","obstacle_9_body","obstacle_10_body"
                                                        ,"obstacle_11_body","obstacle_12_body","obstacle_13_body","obstacle_14_body","obstacle_15_body","obstacle_16_body","obstacle_17_body","obstacle_18_body","obstacle_19_body","obstacle_20_body"
                                                        ,"obstacle_21_body","obstacle_22_body","obstacle_23_body","obstacle_24_body","obstacle_25_body","obstacle_26_body","obstacle_27_body","obstacle_28_body","obstacle_29_body","obstacle_30_body"
                                                        ,"obstacle_31_body","obstacle_32_body","obstacle_33_body","obstacle_34_body","obstacle_35_body","obstacle_36_body","obstacle_37_body","obstacle_38_body","obstacle_39_body","obstacle_40_body"
                                                        ,"obstacle_41_body","obstacle_42_body","obstacle_43_body","obstacle_44_body","obstacle_45_body","obstacle_46_body","obstacle_47_body","obstacle_48_body","obstacle_49_body","obstacle_50_body"
                                                        ,"obstacle_51_body","obstacle_52_body","obstacle_53_body","obstacle_54_body","obstacle_55_body","obstacle_56_body","obstacle_57_body","obstacle_58_body","obstacle_59_body","obstacle_60_body"
                                                        ,"obstacle_61_body","obstacle_62_body","obstacle_63_body","obstacle_64_body","obstacle_65_body","obstacle_66_body","obstacle_67_body","obstacle_68_body","obstacle_69_body","obstacle_70_body"
                                                        ,"obstacle_71_body","obstacle_72_body","obstacle_73_body","obstacle_74_body","obstacle_75_body","obstacle_76_body","obstacle_77_body","obstacle_78_body","obstacle_79_body","obstacle_80_body"
                                                        ,"obstacle_81_body","obstacle_82_body","obstacle_83_body","obstacle_84_body","obstacle_85_body","obstacle_86_body","obstacle_87_body","obstacle_88_body","obstacle_89_body","obstacle_90_body"
                                                        ,"obstacle_91_body","obstacle_92_body","obstacle_93_body","obstacle_94_body","obstacle_95_body","obstacle_96_body","obstacle_97_body","obstacle_98_body","obstacle_99_body","obstacle_100_body"};
#endif

static clock_t now = clock();
static int delay = 0.01*CLOCKS_PER_SEC;

class Potential_Field
{
    public:
        Potential_Field();
        double attractive_force[2];
        double repulsive_force_controller[2];
        double repulsive_force_human[2];

        double repulsive_force_controller_new[Num_obstacles+4];
        double repulsive_force_controller_old[Num_obstacles+4];
        double repulsive_force_controller_final[Num_obstacles+4]; 
        double repulsive_force_controller_slope_force[Num_obstacles+4];
        double repulsive_force_controller_temp[Num_obstacles+4];
        double repulsive_force_controller_slope_lpf[Num_obstacles+4];
        double repulsive_force_controller_slope_lpf_old[Num_obstacles+4];
        double repulsive_force_controller_slope_lpf_temp[Num_obstacles+4];

        double repulsive_force_human_new[Num_obstacles+4];
        double repulsive_force_human_old[Num_obstacles+4];
        double repulsive_force_human_final[Num_obstacles+4]; 
        double repulsive_force_human_slope_force[Num_obstacles+4];
        double repulsive_force_human_temp[Num_obstacles+4];
        double repulsive_force_human_slope_lpf[Num_obstacles+4];
        double repulsive_force_human_slope_lpf_old[Num_obstacles+4];

        int cnt_for_slope_controller;
        int cnt_for_slope_human;

        double closest_obs_pos[2];
        vector <double> dist_list;
        vector <double> th_list;

        double repulsive_force_all[2];
        double distance_each_obs;
        double thetaO;

        double obs_repul_force_x_human;
        double obs_repul_force_y_human;
        double obs_repul_force_x_controller;
        double obs_repul_force_y_controller;

        double distance_to_wall;  

        double randomVel_x[Num_obstacles];
        double randomVel_y[Num_obstacles];
        double shift_x[Num_obstacles];
        double shift_y[Num_obstacles];

        double fnc_cal_distance_obs(double rx, double ry, double goal_x, double goal_y);  
        bool fnc_repulsive_force_all(const mjModel *m, double rx, double ry, vector<double> ox, vector<double> oy); 
};



#endif