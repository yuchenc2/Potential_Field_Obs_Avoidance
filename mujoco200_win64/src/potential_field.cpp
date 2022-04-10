#include "potential_field.hpp"


using namespace std;

Potential_Field::Potential_Field()
{
    for(int i=0;i<2;i++){
        attractive_force[i] = 0.0; //x, y
        repulsive_force[i] = 0.0;
        closest_obs_pos[i] = 0.0;
    }
    repulsive_force_raw = 0.0;
    distance_ = 0.0;
    closest_obs_dist = 38;
    index_ = 0;
}

bool Potential_Field::fnc_cal_distance(double rx, double ry, double goal_x, double goal_y) 
{
    distance_ = sqrt((goal_x-rx)*(goal_x-rx) + (goal_y-ry)*(goal_y-ry));     
    return true;
}

double Potential_Field::fnc_cal_distance_obs(double rx, double ry, double goal_x, double goal_y) 
{
    return sqrt((goal_x-rx)*(goal_x-rx) + (goal_y-ry)*(goal_y-ry));
}

bool Potential_Field::fnc_attractive_force(double rx, double ry, double goal_x, double goal_y) 
{
    const double Kp_x = 0.0001;
    const double Kp_y = 0.000001;
    const double d_star = 2.0;
    const int MAX_FORCE = 50;
    
    // if (distance_ > d_star){
        // attractive_force[0] += (Kp*sin(atan2(goal_x-rx,goal_y-ry)));
        // attractive_force[1] += (Kp*cos(atan2(goal_x-rx,goal_y-ry)));
    attractive_force[0] += Kp_x*((goal_x-rx) / sqrt((goal_x-rx)*(goal_x-rx)));
    attractive_force[1] += Kp_y*((goal_y-ry) / sqrt((goal_y-ry)*(goal_y-ry)));

    // }
    // else{
    //     attractive_force[0] += (Kp_x*(goal_x-rx));
    //     attractive_force[1] += (Kp_y*(goal_y-ry));    
    // }

    //saturation
    if (attractive_force[0] > MAX_FORCE)
        attractive_force[0] = MAX_FORCE;
    else if (attractive_force[0] < -MAX_FORCE)
        attractive_force[0] = -MAX_FORCE;

    if (attractive_force[1] > MAX_FORCE)
        attractive_force[1] = MAX_FORCE;
    else if (attractive_force[1] < -MAX_FORCE)
        attractive_force[1] = -MAX_FORCE;

    return true;
}


bool Potential_Field::fnc_closest_obstacle(double rx, double ry, vector<double> ox, vector<double> oy, int size)
{
    
    double dist_obs_robot = 0.0;
    //closest_obs_dist = fnc_cal_distance_obs(rx, ry, ox[0], ox[1]);
    for (int i=0; i<size; i++){
        dist_obs_robot =  fnc_cal_distance_obs(rx, ry, ox[i], oy[i]);
        // dist_list.push_back(dist_obs_robot);
        if (dist_obs_robot < closest_obs_dist){
            closest_obs_dist = dist_obs_robot;
            index_ = i;
        }
    }

    closest_obs_pos[0] = ox[index_];
    closest_obs_pos[1] = oy[index_];

    // printf("obs pos = (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f) \n",ox[0],ox[1],ox[2],ox[3],
    // ox[4],ox[5],ox[6],ox[7],ox[8],ox[9]);

    // printf("obs = (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f) \n",dist_list[0],dist_list[1],dist_list[2],dist_list[3],
    // dist_list[4],dist_list[5],dist_list[6],dist_list[7],dist_list[8],dist_list[9]);
    // printf("robot x y = %f, %f \n",rx ,ry);
    // printf("closest obs=%d, dis=%f, pos= %f, %f \n",index_+1, closest_obs_dist, closest_obs_pos[0],closest_obs_pos[1]);
    // printf("\n");
    // dist_list.clear();

    return true;
}


bool Potential_Field::fnc_repulsive_force(double p_star, double rx, double ry, double ox, double oy) //only closest one
{
    const double neta = 0.01;
    const int MAX_RP_FORCE = 3;
    const int MAX_RP_FORCE_Y = 3;
    const int p_thres = 3;
    const int sense = 1;

    if (p_star < p_thres){
        repulsive_force_raw = (neta*(1.0/p_star - 1.0/p_thres)) / (p_star*p_star);
        // printf("repu raw = %f \n",repulsive_force_raw);

        repulsive_force[0] += (repulsive_force_raw*sin(atan2(ox-rx, oy-ry)));
        repulsive_force[1] += sense*(repulsive_force_raw*cos(atan2(ox-rx, oy-ry)));
    }
    else{
        repulsive_force[0] += 0.0;
        repulsive_force[1] += 0.0;    
    }

    //saturation
    if (repulsive_force[0] > MAX_RP_FORCE)
        repulsive_force[0] = MAX_RP_FORCE;
    else if (repulsive_force[0] < -MAX_RP_FORCE)
        repulsive_force[0] = -MAX_RP_FORCE;

    if (repulsive_force[1] > MAX_RP_FORCE_Y)
        repulsive_force[1] = MAX_RP_FORCE_Y;
    else if (repulsive_force[1] < -MAX_RP_FORCE_Y)
        repulsive_force[1] = -MAX_RP_FORCE_Y;

    return true;
}