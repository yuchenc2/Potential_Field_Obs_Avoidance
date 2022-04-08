#ifndef BDH_POTENTIAL_FIELD_HPP_
#define BDH_POTENTIAL_FIELD_HPP_

#include "mujoco.h"
#include <math.h>
#include "string.h"
#include <vector>

using namespace std;

class Potential_Field
{
    public:
        Potential_Field();
        

        double attractive_force[2];
        double repulsive_force[2];
        double repulsive_force_raw;
        double distance_;
        double closest_obs_dist;
        int index_ ;

        double closest_obs_pos[2];
        vector <double> dist_list;

        bool fnc_cal_distance(double rx, double ry, double goal_x, double goal_y);
        double fnc_cal_distance_obs(double rx, double ry, double goal_x, double goal_y);  
        bool fnc_attractive_force(double rx, double ry, double goal_x, double goal_y);
        bool fnc_closest_obstacle(double rx, double ry, vector<double> ox, vector<double> oy, int size); 
        bool fnc_repulsive_force(double p_star, double rx, double ry, double ox, double oy);
 
};



#endif