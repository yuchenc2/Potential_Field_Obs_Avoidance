#include "potential_field.hpp"


using namespace std;

Potential_Field::Potential_Field()
{
    for(int i=0;i<2;i++){
        attractive_force[i] = 0.0; //x, y
        repulsive_force[i] = 0.0;
        repulsive_force_human[i] = 0.0;
        closest_obs_pos[i] = 0.0;

        repulsive_force_all[i] = 0.0;
    }
    repulsive_force_raw = 0.0;
    distance_ = 0.0;
    closest_obs_dist = 38;
    index_ = 0;

    obs_repul_force_x_human = 0.0;
    obs_repul_force_x_controller = 0.0;
    obs_repul_force_y_human = 0.0; //y axis
    obs_repul_force_y_controller = 0.0; // yaw controller
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
    const double Kp = 0.003;
    const double d_star = 2.0;
    const int MAX_FORCE = 1;

    double thetaG = atan2(goal_y-ry, goal_x-rx);

    //from others
    int goalR = 0.01;
    double goalS = 19.0/5.0;
    
    attractive_force[0] = Kp*(dist - goalR);
    attractive_force[1] = Kp*(dist - goalR)*thetaG;

    // if (dist < goalR){
    //     attractive_force[0] = 0;
    //     attractive_force[1] = 0;
    // }
    // else if((goalS+goalR >= dist) && (dist >=goalR))
    // {
    //     attractive_force[0] = Kp*(dist - goalR);
    //     attractive_force[1] = Kp*(dist - goalR)*thetaG;
    // }
    // else{
    //     attractive_force[0] = Kp*goalS;
    //     attractive_force[1] = Kp*thetaG;
    // }

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

bool Potential_Field::fnc_repulsive_force_all(const mjModel *m, double rx, double ry, vector<double> ox, vector<double> oy, int case_, int map)
{
    //from others
    const double obsRad = 0.2;
    // const double obsS = 19.0/5.0;
    const double obsS = 10.0/5.0;
    const double inf = 0.001;
    const double DTR = M_PI/180.0;
    const double p_thres = 0.2;
    const double neta_human = 0.5;
    const double neta_controller = 0.002;
    const double wall_force_activate_distance = 0.5;
    int size = 0;

    obs_repul_force_x_controller = 0.0;
    obs_repul_force_x_human = 0.0;
    obs_repul_force_y_controller = 0.0;
    obs_repul_force_y_human = 0.0;

    if(map == 0){ // get obstacle names
        size = 13;
    }else if(map == 1){
        size = 6;
    }else{ // path width map
        size = 0;
    }

    for (int i=0; i<size; i++){ // Generate repulsive force for the one line maps with obstacles
        if(map == 1){ // if map is dynamic, update obstacle location locally
            const char *obstacle_name[6] = {"obstacle_1_body","obstacle_2_body","obstacle_3_body","obstacle_4_body","obstacle_5_body","obstacle_6_body"};
            ox[i] = m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+0];
            oy[i] = m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+1];
        }
        distance_each_obs =  fnc_cal_distance_obs(rx, ry, ox[i], oy[i]);
        thetaO = atan2(oy[i]-ry, ox[i]-rx);

        //controller
        if(case_ == 1){
            if((distance_each_obs < (obsS + obsRad)) && (distance_each_obs>=obsRad)){
                repulsive_force_raw = (neta_controller* (1.0/distance_each_obs - 1.0/(obsS + obsRad))) / (distance_each_obs*distance_each_obs);
                repulsive_force[0] = -repulsive_force_raw;
                repulsive_force[1] = -repulsive_force_raw*thetaO*3.0;
            }
            else{
                repulsive_force[0] = 0.0;
                repulsive_force[1] = 0.0;
            }

            distance_to_wall = fnc_cal_distance_obs(rx, 0, -20, 0);
            if(distance_to_wall < wall_force_activate_distance){ 
                repulsive_force[0] = repulsive_force[0]+(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall);
            }
            //top wall
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, 1.2192);
            if(distance_to_wall < wall_force_activate_distance){ 
                repulsive_force[1] = repulsive_force[1]+(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //right wall
            distance_to_wall = fnc_cal_distance_obs(rx, 0, 10, 0);
            if(distance_to_wall < wall_force_activate_distance){ 
                repulsive_force[0] = repulsive_force[0]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall);
            }
            //bottom wall
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -1.2192);
            if(distance_to_wall < wall_force_activate_distance){ 
                repulsive_force[1] = repulsive_force[1]+(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            
            // Cutoff for repulsive force
            if(repulsive_force[1] > 360.0 * DTR)
                repulsive_force[1] = repulsive_force[1] - 360.0 * DTR;
            else if(repulsive_force[1] < - 360.0 * DTR)
                repulsive_force[1] = repulsive_force[1] + 360.0 * DTR;
            obs_repul_force_x_controller += repulsive_force[0];
            obs_repul_force_y_controller += repulsive_force[1];
            if(obs_repul_force_y_controller > 360 *M_PI/180){
                obs_repul_force_y_controller = obs_repul_force_y_controller - 360 *M_PI/180;
            }
            else if(obs_repul_force_y_controller < - 360 *M_PI/180){
                obs_repul_force_y_controller = obs_repul_force_y_controller + 360 *M_PI/180;
            }
        }
        //human feedback
        else if(case_ == 0){
            if((distance_each_obs < (obsS + obsRad)) && (distance_each_obs>=obsRad)){
                repulsive_force_raw = (neta_human* (1.0/distance_each_obs - 1.0/(obsS + obsRad))) / (distance_each_obs*distance_each_obs);
                repulsive_force_human[0] = -repulsive_force_raw*cos(thetaO);
                repulsive_force_human[1] = -repulsive_force_raw*sin(thetaO);
            }
            else{
                repulsive_force_human[0] = 0.0;
                repulsive_force_human[1] = 0.0;
            }
            distance_to_wall = fnc_cal_distance_obs(rx, 0, -20, 0);
            if(distance_to_wall < wall_force_activate_distance){ 
                repulsive_force_human[0] = repulsive_force_human[0]+(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall);
            }
            //top wall
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, 1.2192);
            if(distance_to_wall < wall_force_activate_distance){ 
                repulsive_force_human[1] = repulsive_force_human[1]+(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //right wall
            distance_to_wall = fnc_cal_distance_obs(rx, 0, 10, 0);
            if(distance_to_wall < wall_force_activate_distance){ 
                repulsive_force_human[0] = repulsive_force_human[0]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall);
            }
            //bottom wall
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -1.2192);
            if(distance_to_wall < wall_force_activate_distance){ 
                repulsive_force_human[1] = repulsive_force_human[1]+(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }

            obs_repul_force_x_human += repulsive_force_human[0];
            obs_repul_force_y_human += repulsive_force_human[1];
        }
        //both
        else if(case_ == 2){
        }
    }

    if(map == 2){ //Path width map, only has wall repulsive force
        if(case_ == 1){ // feedback to controller
            repulsive_force[0] = 0.0;
            repulsive_force[1] = 0.0;

            //wall 1
            distance_to_wall = fnc_cal_distance_obs(rx, 0, -20, 0);
            if(distance_to_wall < wall_force_activate_distance){ 
                repulsive_force[0] = repulsive_force[0]+(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall);
            }
            //wall 2
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, 1.2192);
            if(distance_to_wall < wall_force_activate_distance && rx < -10.2464){ 
                thetaO = atan2(1.2192-ry, 0);
                repulsive_force[1] = repulsive_force[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 3
            distance_to_wall = fnc_cal_distance_obs(rx, 0, -10.2464, 0);
            if(distance_to_wall < wall_force_activate_distance && ry > -6.096){ 
                thetaO = atan2(-10.2464-rx, 0);
                repulsive_force[1] = repulsive_force[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 4
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -1.2192);
            if(distance_to_wall < wall_force_activate_distance && rx < -12.6848){ 
                thetaO = atan2(-1.2192-ry, 0);
                repulsive_force[1] = repulsive_force[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 5
            distance_to_wall = fnc_cal_distance_obs(rx, 0, -12.6848, 0);
            if(distance_to_wall < wall_force_activate_distance && ry < -1.2192){ 
                thetaO = atan2(-12.6848-rx, 0);
                repulsive_force[1] = repulsive_force[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;             
            }
            //wall 6
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -6.096);
            if(distance_to_wall < wall_force_activate_distance && rx > -10.2464 && rx < -5.3696 ){ 
                thetaO = atan2(-6.096-ry, 0);
                repulsive_force[1] = repulsive_force[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 7
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -7.9248);
            if(distance_to_wall < wall_force_activate_distance && rx > -12.6848 && rx < -3.5408 ){ 
                thetaO = atan2(-7.9248-ry, 0);
                repulsive_force[1] = repulsive_force[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 8
            distance_to_wall = fnc_cal_distance_obs(rx, 0, -5.3696, 0);
            if(distance_to_wall < wall_force_activate_distance && ry > -6.096 && ry < 0.5334){ 
                thetaO = atan2(-5.3696-rx, 0);
                repulsive_force[1] = repulsive_force[1]+(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;             
            }
            //wall 9
            distance_to_wall = fnc_cal_distance_obs(rx, 0, -3.5408, 0);
            if(distance_to_wall < wall_force_activate_distance && ry > -7.9248 && ry < -0.6096){ 
                thetaO = atan2(-3.5408-rx, 0);
                repulsive_force[1] = repulsive_force[1]+(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 10
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -0.6096);
            if(distance_to_wall < wall_force_activate_distance && rx > -3.5408 && rx < 1.9456){ 
                thetaO = atan2(-0.6096-ry, 0);
                repulsive_force[1] = repulsive_force[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 11
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, 0.5334);
            if(distance_to_wall < wall_force_activate_distance && rx > -5.3696 && rx < 3.0886){ 
                thetaO = atan2(0.5334-ry, 0);
                repulsive_force[1] = repulsive_force[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 12
            distance_to_wall = fnc_cal_distance_obs(rx, 0, 3.0886, 0);
            if(distance_to_wall < wall_force_activate_distance && ry > -6.7818 && ry < 0.5334){ 
                thetaO = atan2(3.0886-rx, 0);
                repulsive_force[1] = repulsive_force[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 13
            distance_to_wall = fnc_cal_distance_obs(rx, 0, 1.9456, 0);
            if(distance_to_wall < wall_force_activate_distance && ry > -6.7818 && ry < -0.6096){ 
                thetaO = atan2(1.9456-rx, 0);
                repulsive_force[1] = repulsive_force[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;             
            }
            //wall 14
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -6.7818);
            if(distance_to_wall < wall_force_activate_distance && rx > 1.9456 && rx < 3.0886){ 
                repulsive_force[0] = repulsive_force[0]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall);
            }


            //edge 1
            distance_to_wall = fnc_cal_distance_obs(rx, ry, -12.6848, -1.2192);
            if(distance_to_wall < wall_force_activate_distance){
                repulsive_force_raw = (neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance))/(distance_to_wall*distance_to_wall));
                thetaO = atan2(-1.2192-ry, -12.6848-rx);
                repulsive_force[1] = repulsive_force[1]-repulsive_force_raw*thetaO;
            }

            //edge 2
            distance_to_wall = fnc_cal_distance_obs(rx, ry, -10.2464, -6.096);
            if(distance_to_wall < wall_force_activate_distance){
                repulsive_force_raw = (neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance))/(distance_to_wall*distance_to_wall));
                thetaO = atan2(-6.096-ry, -10.2464-rx);
                repulsive_force[1] = repulsive_force[1]-repulsive_force_raw*thetaO;
            }

            //edge 3
            distance_to_wall = fnc_cal_distance_obs(rx, ry, -5.3696, -6.096);
            if(distance_to_wall < wall_force_activate_distance){
                repulsive_force_raw = (neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance))/(distance_to_wall*distance_to_wall));
                thetaO = atan2(-6.096-ry, -5.3696-rx);
                repulsive_force[1] = repulsive_force[1]-repulsive_force_raw*thetaO;
            }

            //edge 4
            distance_to_wall = fnc_cal_distance_obs(rx, ry, -3.5408, -0.6096);
            if(distance_to_wall < wall_force_activate_distance){
                repulsive_force_raw = (neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance))/(distance_to_wall*distance_to_wall));
                thetaO = atan2(-0.6096-ry, -3.5408-rx);
                repulsive_force[1] = repulsive_force[1]-repulsive_force_raw*thetaO;
            }

            //edge 5
            distance_to_wall = fnc_cal_distance_obs(rx, ry, 1.9456, -0.6096);
            if(distance_to_wall < wall_force_activate_distance){
                repulsive_force_raw = (neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance))/(distance_to_wall*distance_to_wall));
                thetaO = atan2(-0.6096-ry, 1.9456-rx);
                repulsive_force[1] = repulsive_force[1]-repulsive_force_raw*thetaO;
            }
            
            
            // Cutoff for repulsive force
            if(repulsive_force[1] > 360.0 * DTR)
                repulsive_force[1] = repulsive_force[1] - 360.0 * DTR;
            else if(repulsive_force[1] < - 360.0 * DTR)
                repulsive_force[1] = repulsive_force[1] + 360.0 * DTR;
            obs_repul_force_x_controller += repulsive_force[0];
            obs_repul_force_y_controller += repulsive_force[1];
            if(obs_repul_force_y_controller > 360 *M_PI/180){
                obs_repul_force_y_controller = obs_repul_force_y_controller - 360 *M_PI/180;
            }
            else if(obs_repul_force_y_controller < - 360 *M_PI/180){
                obs_repul_force_y_controller = obs_repul_force_y_controller + 360 *M_PI/180;
            }
        } 
        //human feedback
        else if(case_ == 0){
            repulsive_force_human[0] = 0.0;
            repulsive_force_human[1] = 0.0;

            //wall 1
            distance_to_wall = fnc_cal_distance_obs(rx, 0, -20, 0);
            if(distance_to_wall < wall_force_activate_distance){ 
                repulsive_force_human[0] = repulsive_force_human[0]+(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall);
            }
            //wall 2
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, 1.2192);
            if(distance_to_wall < wall_force_activate_distance && rx < -10.2464){ 
                thetaO = atan2(1.2192-ry, 0);
                repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 3
            distance_to_wall = fnc_cal_distance_obs(rx, 0, -10.2464, 0);
            if(distance_to_wall < wall_force_activate_distance && ry > -6.096){ 
                thetaO = atan2(-10.2464-rx, 0);
                repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 4
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -1.2192);
            if(distance_to_wall < wall_force_activate_distance && rx < -12.6848){ 
                thetaO = atan2(-1.2192-ry, 0);
                repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 5
            distance_to_wall = fnc_cal_distance_obs(rx, 0, -12.6848, 0);
            if(distance_to_wall < wall_force_activate_distance && ry < -1.2192){ 
                thetaO = atan2(-12.6848-rx, 0);
                repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;             
            }
            //wall 6
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -6.096);
            if(distance_to_wall < wall_force_activate_distance && rx > -10.2464 && rx < -5.3696 ){ 
                thetaO = atan2(-6.096-ry, 0);
                repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 7
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -7.9248);
            if(distance_to_wall < wall_force_activate_distance && rx > -12.6848 && rx < -3.5408 ){ 
                thetaO = atan2(-7.9248-ry, 0);
                repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 8
            distance_to_wall = fnc_cal_distance_obs(rx, 0, -5.3696, 0);
            if(distance_to_wall < wall_force_activate_distance && ry > -6.096 && ry < 0.5334){ 
                thetaO = atan2(-5.3696-rx, 0);
                repulsive_force_human[1] = repulsive_force_human[1]+(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;             
            }
            //wall 9
            distance_to_wall = fnc_cal_distance_obs(rx, 0, -3.5408, 0);
            if(distance_to_wall < wall_force_activate_distance && ry > -7.9248 && ry < -0.6096){ 
                thetaO = atan2(-3.5408-rx, 0);
                repulsive_force_human[1] = repulsive_force_human[1]+(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 10
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -0.6096);
            if(distance_to_wall < wall_force_activate_distance && rx > -3.5408 && rx < 1.9456){ 
                thetaO = atan2(-0.6096-ry, 0);
                repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 11
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, 0.5334);
            if(distance_to_wall < wall_force_activate_distance && rx > -5.3696 && rx < 3.0886){ 
                thetaO = atan2(0.5334-ry, 0);
                repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 12
            distance_to_wall = fnc_cal_distance_obs(rx, 0, 3.0886, 0);
            if(distance_to_wall < wall_force_activate_distance && ry > -6.7818 && ry < 0.5334){ 
                thetaO = atan2(3.0886-rx, 0);
                repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
            }
            //wall 13
            distance_to_wall = fnc_cal_distance_obs(rx, 0, 1.9456, 0);
            if(distance_to_wall < wall_force_activate_distance && ry > -6.7818 && ry < -0.6096){ 
                thetaO = atan2(1.9456-rx, 0);
                repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;             
            }
            //wall 14
            distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -6.7818);
            if(distance_to_wall < wall_force_activate_distance && rx > 1.9456 && rx < 3.0886){ 
                repulsive_force_human[0] = repulsive_force_human[0]+(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall);
            }

            obs_repul_force_x_human += repulsive_force_human[0];
            obs_repul_force_y_human += repulsive_force_human[1];
        }
    }


    // dist_list.clear();
    // th_list.clear();

    return true;
}
