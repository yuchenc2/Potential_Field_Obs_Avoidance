#include "potential_field.hpp"


using namespace std;

Potential_Field::Potential_Field()
{
    for(int i=0;i<2;i++){
        attractive_force[i] = 0.0; //x, y
        repulsive_force[i] = 0.0;
        closest_obs_pos[i] = 0.0;

        repulsive_force_all[i] = 0.0;
    }
    repulsive_force_raw = 0.0;
    distance_ = 0.0;
    closest_obs_dist = 38;
    index_ = 0;

    obs_repul_force_x = 0.0;
    obs_repul_force_y = 0.0;
    distance_each_obs = 0.0;
    thetaO = 0.0;
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

bool Potential_Field::fnc_attractive_force(double dist, double rx, double ry, double goal_x, double goal_y) 
{
    const double Kp = 0.01;
    const double d_star = 2.0;
    const int MAX_FORCE = 1;

    double thetaG = atan2(goal_y-ry, goal_x-rx);

    //from others
    int goalR = 0.01;
    double goalS = 38/5; // The spread of attraction of the goal
    
    if (dist < goalR){
        attractive_force[0] = 0;
        attractive_force[1] = 0;
    }
    else if((goalS+goalR >= dist) && (dist >=goalR))
    {
        attractive_force[0] = Kp*(dist - goalR);
        attractive_force[1] = Kp*thetaG;
    }
    else{
        attractive_force[0] = Kp*goalS;
        attractive_force[1] = Kp*thetaG;
    }

    // else if((goalS+goalR >= dist) && (dist >=goalR))
    // {
    //     attractive_force[0] = Kp_x*(dist - goalR)*cos(thetaG);
    //     attractive_force[1] = Kp_x*(dist - goalR)*sin(thetaG);
    // }
    // else{
    //     attractive_force[0] = Kp_x*goalS*cos(thetaG);
    //     attractive_force[1] = Kp_x*goalS*sin(thetaG);
    // }

    // if (distance_ > d_star){
    //     attractive_force[0] += Kp*cos(thetaG);
    //     attractive_force[1] += Kp*sin(thetaG);
    // }
    // else{
    //     attractive_force[0] += (Kp_x*(goal_x-rx));
    //     attractive_force[1] += (Kp_y*(goal_y-ry));    
    // }
    //saturation
    // if (attractive_force[0] > MAX_FORCE)
    //     attractive_force[0] = MAX_FORCE;
    // else if (attractive_force[0] < -MAX_FORCE)
    //     attractive_force[0] = -MAX_FORCE;

    // if (attractive_force[1] > MAX_FORCE)
    //     attractive_force[1] = MAX_FORCE;
    // else if (attractive_force[1] < -MAX_FORCE)
    //     attractive_force[1] = -MAX_FORCE;

    return true;
}


bool Potential_Field::fnc_closest_obstacle(double rx, double ry, vector<double> ox, vector<double> oy, int size)
{
    
    double dist_obs_robot = 0.0;
    // closest_obs_dist = fnc_cal_distance_obs(rx, ry, ox[0], ox[1]);
    for (int i=0; i<size; i++){
        dist_obs_robot =  fnc_cal_distance_obs(rx, ry, ox[i], oy[i]);
        dist_list.push_back(dist_obs_robot);
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
    dist_list.clear();

    return true;
}


bool Potential_Field::fnc_repulsive_force(double p_star, double rx, double ry, double ox, double oy) //only closest one
{
    const double neta = 0.000001;
    const int MAX_RP_FORCE = 1;
    const double MAX_RP_FORCE_Y = 0.5;
    const int p_thres = 4;

    if (p_star < p_thres){
        repulsive_force_raw = (neta*(1.0/p_star - 1.0/p_thres)) / (p_star*p_star);

        repulsive_force[0] -= repulsive_force_raw*cos(atan2(oy-ry, ox-rx));
        repulsive_force[1] -= repulsive_force_raw*sin(atan2(oy-ry, ox-rx));
        // repulsive_force[0] += repulsive_force_raw * (ox-rx) / p_star;
        // repulsive_force[1] += repulsive_force_raw * (oy-ry) /p_star;
        
    }
    else{
        repulsive_force[0] -= 0.0;
        repulsive_force[1] -= 0.0;    
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

bool Potential_Field::fnc_repulsive_force_all(double rx, double ry, vector<double> ox, vector<double> oy, int size)
{
    // const double neta = 0.000001;
    // const int MAX_RP_FORCE = 1;
    // const double MAX_RP_FORCE_Y = 0.5;
    // const int p_thres = 2;
    // const double sense = 0.001;

    // for (int i=0; i<size; i++){ //Number of obstacles
    //     distance_each_obs =  fnc_cal_distance_obs(rx, ry, ox[i], oy[i]);

    //     if (distance_each_obs < p_thres){
    //         repulsive_force_raw = (neta*(1.0/distance_each_obs - 1.0/p_thres)) / (distance_each_obs*distance_each_obs);
    //         repulsive_force[0] = repulsive_force_raw * (ox[i]-rx) / distance_each_obs;
    //         repulsive_force[1] = repulsive_force_raw * (oy[i]-ry) /distance_each_obs;
    //     }
    //     else{
    //         repulsive_force[0] = 0.0;
    //         repulsive_force[1] = 0.0;    
    //     }

    //     obs_repul_force_x += repulsive_force[0];
    //     obs_repul_force_y += repulsive_force[1];
    // }

    //from others
    const int obsRad = 1;
    const double obsS = 38/5;
    const double inf = 0.001;
    const double beta = 0.001;

    obs_repul_force_x = 0.0;
    obs_repul_force_y = 0.0;

    for (int i=0; i<size; i++){ //Number of obstacles
        distance_each_obs =  fnc_cal_distance_obs(rx, ry, ox[i], oy[i]);
        thetaO = atan2(oy[i]-ry, ox[i]-rx);
    
        // if(distance_each_obs < obsRad)
        // {
        //     repulsive_force[0] = -(sgn(cos(thetaO))) * inf;
        //     repulsive_force[1] = -(sgn(sin(thetaO))) * inf;
        // }
        // else if((distance_each_obs<(obsS + obsRad)) && (distance_each_obs>=obsRad)){
        //     repulsive_force[0] = -beta*(obsS + obsRad - distance_each_obs)*cos(thetaO);
        //     repulsive_force[1] = -beta*(obsS + obsRad - distance_each_obs)*sin(thetaO);
        // }
        if(distance_each_obs < obsRad)
        {
            repulsive_force[0] = - inf;
            repulsive_force[1] = inf*thetaO;
        }
        else if((distance_each_obs<(obsS + obsRad)) && (distance_each_obs>=obsRad)){
            repulsive_force[0] = -beta*(obsS + obsRad - distance_each_obs);
            repulsive_force[1] = beta*thetaO;
        }
        else{
            repulsive_force[0] = 0.0;
            repulsive_force[1] = 0.0;  
        }
        obs_repul_force_x += repulsive_force[0];
        obs_repul_force_y += repulsive_force[1];
    }

    return true;
}
