#ifndef BDH_POTENTIAL_FIELD_HPP_
#define BDH_POTENTIAL_FIELD_HPP_

#include "mujoco.h"
#include <math.h>
#include "string.h"
#include <vector>

using namespace std;
#define sgn(v) ( ( (v) < 0 ) ? -1 : ( (v) > 0 ) )
#define M_PI           3.14159265358979323846


class Potential_Field
{
    public:
        Potential_Field();
        

        double attractive_force[2];
        double repulsive_force[2];
        double repulsive_force_human_new;
        double repulsive_force_human_old;
        double repulsive_force_human[2];
        double repulsive_force_controller_new;
        double repulsive_force_controller_old;
        
        double distance_;
        double closest_obs_dist;
        int index_ ;

        double closest_obs_pos[2];
        vector <double> dist_list;
        vector <double> th_list;

        double obs_repul_force_x_human;
        double obs_repul_force_x_controller;
        // double obs_repul_force_y;
        double repulsive_force_all[2];
        double distance_each_obs;
        double thetaO;

        double obs_repul_force_y_human;
        double obs_repul_force_y_controller;
        double repulsive_force_controller_slope_force;
        double repulsive_force_human_slope_force;

        double distance_to_wall;  

        bool fnc_cal_distance(double rx, double ry, double goal_x, double goal_y);
        double fnc_cal_distance_obs(double rx, double ry, double goal_x, double goal_y);  
        bool fnc_attractive_force(double dist, double rx, double ry, double goal_x, double goal_y);
        bool fnc_closest_obstacle(double rx, double ry, vector<double> ox, vector<double> oy, int size); 
        // bool fnc_repulsive_force(double p_star, double rx, double ry, double ox, double oy);
        bool fnc_repulsive_force_all(const mjModel *m, double rx, double ry, vector<double> ox, vector<double> oy, int case_, int map); 
 
};



#endif