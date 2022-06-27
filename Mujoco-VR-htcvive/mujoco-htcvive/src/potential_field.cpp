#include "potential_field.hpp"


using namespace std;

Potential_Field::Potential_Field()
{
    for(int i=0;i<2;i++){
        attractive_force[i] = 0.0; //x, y
        repulsive_force_controller[i] = 0.0;
        repulsive_force_human[i] = 0.0;
        closest_obs_pos[i] = 0.0;

        repulsive_force_all[i] = 0.0;
    }
    

    // for(int i=0;i<26;i++){
    for(int i=0;i<26;i++){
        repulsive_force_controller_new[i] = 0.0;
        repulsive_force_controller_old[i] = 0.0;
        repulsive_force_controller_final[i] = 0.0;
        
        repulsive_force_controller_slope_force[i] = 0.0;
        repulsive_force_controller_temp[i] = 0.0;
        repulsive_force_controller_slope_lpf[i] = 0.0;
        repulsive_force_controller_slope_lpf_old[i] = 0.0;

        repulsive_force_human_new[i] = 0.0;
        repulsive_force_human_old[i] = 0.0;
        repulsive_force_human_final[i] = 0.0;
        repulsive_force_human_slope_force[i] = 0.0;
        repulsive_force_human_temp[i] = 0.0;
        repulsive_force_human_slope_lpf[i] = 0.0;
        repulsive_force_human_slope_lpf_old[i]= 0.0; 
    }
    distance_ = 0.0;
    closest_obs_dist = 38;
    index_ = 0;
    alpha = 0.15;

    obs_repul_force_x_human = 0.0;
    obs_repul_force_x_controller = 0.0;
    obs_repul_force_y_human = 0.0; //y axis
    obs_repul_force_y_controller = 0.0; // yaw controller
    

    cnt_for_slope_obs_controller = 0;
    cnt_for_slope_obs_human = 0;
    cnt_for_slope_wall_controller = 0;
    cnt_for_slope_wall_human = 0;
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

bool Potential_Field::fnc_repulsive_force_all(const mjModel *m, double rx, double ry, vector<double> ox, vector<double> oy, int case_, int map)
{
    //from others
    const double obsRad = 0.2; //0.2;
    // const double obsS = 19.0/5.0;
    const double obsS = 3.0; //10.0/5.0;
    const double inf = 0.001;
    const double DTR = M_PI/180.0;
    const double p_thres = 0.2;
    const double neta_human = 500.0; //10.0; //5.0; //2.0; //0.5
    const double neta_controller = 500; //0.002;
    const double wall_force_activate_distance = obsS + obsRad;
    const double max_force_vel_cutoff = 6.0;
    int size = 0;
    const double beta_velocity_controller = 1.0;
    const double beta_velocity_human = 0.1;
    const double wall_force_multiplier = 0.1;
    double wall_force_x_human = 0.0;
    double wall_force_y_human = 0.0;
    double wall_force_x_controller = 0.0;
    double wall_force_y_controller = 0.0;
    double obs_force_x_human = 0.0;
    double obs_force_y_human = 0.0;
    double obs_force_x_controller = 0.0;
    double obs_force_y_controller = 0.0;
    obs_repul_force_x_human = 0.0;
    obs_repul_force_y_controller = 0.0;
    obs_repul_force_y_human = 0.0;



    if(map == 0){ 
        size = 18; // 18 obs + 4 walls
    }else if(map == 1){
        size = 11; // 11 obs + 4 walls
    }else{ // path width map
        size = 0;
    }

    cnt_for_slope_obs_controller++;
    cnt_for_slope_obs_human++;
    cnt_for_slope_wall_controller++;
    cnt_for_slope_wall_human++;

    for (int i=0; i<size; i++){ // Generate repulsive force for the one line maps with obstacles
        if(map == 1){ // if map is dynamic, update obstacle location locally
            const char *obstacle_name[11] = {"obstacle_1_body","obstacle_2_body","obstacle_3_body","obstacle_4_body","obstacle_5_body","obstacle_6_body","obstacle_7_body","obstacle_8_body","obstacle_9_body","obstacle_10_body","obstacle_11_body"};
            ox[i] = m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+0];
            oy[i] = m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+1];
        }

        distance_each_obs =  fnc_cal_distance_obs(rx, ry, ox[i], oy[i]);
        thetaO = atan2(oy[i]-ry, ox[i]-rx);
        if(case_ == 1 || case_ == 2){ //Controller Obstacle Force Calculation
            if(distance_each_obs < (obsS + obsRad)){
               if((cnt_for_slope_obs_controller % 400 == 0)){ //10ms = 0.01s
                    // repulsive_force_controller_new[i] = 10.0/(1.0+exp(distance_each_obs));
                    repulsive_force_controller_new[i] = 7.5/(1.0+exp(2.0*distance_each_obs));
                    repulsive_force_controller_slope_force[i] = beta_velocity_controller*(repulsive_force_controller_new[i]-repulsive_force_controller_old[i])/0.01;
                    repulsive_force_controller_slope_lpf[i] = alpha*repulsive_force_controller_slope_force[i] + (1-alpha)*repulsive_force_controller_slope_lpf_old[i];
                    repulsive_force_controller_slope_lpf_old[i] = repulsive_force_controller_slope_lpf[i];
                    repulsive_force_controller_old[i] = repulsive_force_controller_new[i];
                    cnt_for_slope_obs_controller = 0;
                }
                if(repulsive_force_controller_slope_lpf[i] >= 0.0){ // If approaching the obstacle
                    if(repulsive_force_controller_slope_lpf[i] > max_force_vel_cutoff ){ //max cutoff for force added
                        repulsive_force_controller_slope_lpf[i] = max_force_vel_cutoff ;
                    }
                }else{ // If getting further away, zero out the force   
                    repulsive_force_controller_slope_lpf[i] = 0.0; //-repulsive_force_controller_slope_lpf[i];
                }
                
                repulsive_force_controller_final[i] = repulsive_force_controller_slope_lpf[i];
                repulsive_force_controller[0] = -repulsive_force_controller_final[i];
                repulsive_force_controller[1] = -repulsive_force_controller_final[i]*thetaO;
            }
            else{
                repulsive_force_controller[0] = 0.0;
                repulsive_force_controller[1] = 0.0;
            }
            obs_force_x_controller += repulsive_force_controller[0];
            obs_force_y_controller += repulsive_force_controller[1];
        }
        
        if(case_ == 0 || case_ == 2){ //Human Obstacle Force Calculation
            if(distance_each_obs < (obsS + obsRad)){
                //Modified potential field force
                if((cnt_for_slope_obs_human % 400 == 0)){ //10ms = 0.01s
                    // repulsive_force_human_new[i] = 5.0/(1.0+exp(2.0*distance_each_obs));
                    repulsive_force_human_new[i] = 7.5/(1.0+exp(distance_each_obs));
                    repulsive_force_human_slope_force[i] = beta_velocity_human*(repulsive_force_human_new[i]-repulsive_force_human_old[i])/0.01;
                    repulsive_force_human_slope_lpf[i] = alpha*repulsive_force_human_slope_force[i] + (1-alpha)*repulsive_force_human_slope_lpf_old[i];
                    repulsive_force_human_slope_lpf_old[i] = repulsive_force_human_slope_lpf[i];
                    repulsive_force_human_old[i] = repulsive_force_human_new[i];
                    cnt_for_slope_obs_human = 0;
                }
                if(repulsive_force_human_slope_lpf[i] >= 0.0){ // If approaching the obstacle
                    if(repulsive_force_human_slope_lpf[i] > max_force_vel_cutoff ){ //max cutoff for force added
                        repulsive_force_human_slope_lpf[i] = max_force_vel_cutoff ;
                    }
                }else{ // If getting further away, zero out the force   
                    repulsive_force_human_slope_lpf[i] = 0.0; //-repulsive_force_human_slope_lpf[i];
                }
                
                repulsive_force_human_final[i] = repulsive_force_human_slope_lpf[i];
                repulsive_force_human[0] = -repulsive_force_human_final[i]*cos(thetaO);
                repulsive_force_human[1] = -repulsive_force_human_final[i]*sin(thetaO);
            }else{ 
                repulsive_force_human[0] = 0.0;
                repulsive_force_human[1] = 0.0;
            }
            obs_force_x_human += repulsive_force_human[0];
            obs_force_y_human += repulsive_force_human[1];
        }
    }
    
    if(case_ == 1 || case_ == 2){ //Controller Wall and Final force
        wall_force_x_controller = 0.0;
        wall_force_y_controller = 0.0;

        for (int i=size; i<size+4; i++){
            if(i == size){ //left
                thetaO = 0.0;
                distance_each_obs = fnc_cal_distance_obs(rx, 0, -20, 0);
            }else if(i == size+1){ //top
                thetaO = atan2(1.2192-ry, 0); 
                distance_each_obs = fnc_cal_distance_obs(0, ry, 0, 1.2192);
            }else if(i == size+2){ //right
                thetaO = 0.0;
                distance_each_obs = fnc_cal_distance_obs(rx, 0, 10, 0);
            }else if(i == size+3){ //bottom
                thetaO = atan2(-1.2192-ry, 0);
                distance_each_obs = fnc_cal_distance_obs(0, ry, 0, -1.2192);
            }
            if(distance_each_obs < (obsS + obsRad)){
                if((cnt_for_slope_wall_controller % 400 == 0)){ //10ms = 0.01s
                    repulsive_force_controller_new[i] = 1.0/(1.0+exp(8.0*distance_each_obs));
                    repulsive_force_controller_slope_force[i] = beta_velocity_controller*(repulsive_force_controller_new[i]-repulsive_force_controller_old[i])/0.01;
                    repulsive_force_controller_slope_lpf[i] = alpha*repulsive_force_controller_slope_force[i] + (1-alpha)*repulsive_force_controller_slope_lpf_old[i];
                    repulsive_force_controller_slope_lpf_old[i] = repulsive_force_controller_slope_lpf[i];
                    repulsive_force_controller_old[i] = repulsive_force_controller_new[i];
                    cnt_for_slope_wall_controller = 0;
                }
                if(repulsive_force_controller_slope_lpf[i] >= 0.0){ // If approaching the obstacle
                    if(repulsive_force_controller_slope_lpf[i] > max_force_vel_cutoff ){ //max cutoff for force added
                        repulsive_force_controller_slope_lpf[i] = max_force_vel_cutoff ;
                    }
                }else{ // If getting further away, zero out the force   
                    repulsive_force_controller_slope_lpf[i] = 0.0; //-repulsive_force_controller_slope_lpf[i];
                }
                
                repulsive_force_controller_final[i] = repulsive_force_controller_slope_lpf[i];
                repulsive_force_controller[0] = -repulsive_force_controller_final[i];
                repulsive_force_controller[1] = -repulsive_force_controller_final[i]*thetaO;
            }
            else{
                repulsive_force_controller[0] = 0.0;
                repulsive_force_controller[1] = 0.0;
            }
            wall_force_x_controller += repulsive_force_controller[0];
            wall_force_y_controller += repulsive_force_controller[1];
        }
        obs_repul_force_x_controller = (wall_force_x_controller*2.0 + obs_force_x_controller*1.0)*0.001;
        obs_repul_force_y_controller = (wall_force_y_controller*20.0 + obs_force_y_controller*8.0)*0.001;

        if(obs_repul_force_y_controller > 360 *M_PI/180){
            obs_repul_force_y_controller = obs_repul_force_y_controller - 360 *M_PI/180;
        }
        else if(obs_repul_force_y_controller < - 360 *M_PI/180){
            obs_repul_force_y_controller = obs_repul_force_y_controller + 360 *M_PI/180;
        }
    
        // printf("total_force %f %f, obs_force: %f %f, wall_force %f %f\n", obs_repul_force_x_controller, obs_repul_force_y_controller, obs_force_x_controller, obs_force_y_controller, wall_force_x_controller, wall_force_y_controller);
    }

    if(case_ == 0 || case_ == 2){ //Human wall and final force
        wall_force_x_human = 0.0;
        wall_force_y_human = 0.0;
        for (int i=size; i<size+4; i++){
            if(i == size){ //left
                thetaO = 0.0;
                distance_each_obs = fnc_cal_distance_obs(rx, 0, -20, 0);
            }else if(i == size+1){ //top
                thetaO = atan2(1.2192-ry, 0); 
                distance_each_obs = fnc_cal_distance_obs(0, ry, 0, 1.2192);
            }else if(i == size+2){ //right
                thetaO = 0.0;
                distance_each_obs = fnc_cal_distance_obs(rx, 0, 10, 0);
            }else if(i == size+3){ //bottom
                thetaO = atan2(-1.2192-ry, 0);
                distance_each_obs = fnc_cal_distance_obs(0, ry, 0, -1.2192);
            }
            if(distance_each_obs < (obsS + obsRad)){
                if((cnt_for_slope_wall_human % 400 == 0)){ //10ms = 0.01s
                    repulsive_force_human_new[i] = 1.0/(1.0+exp(8.0*distance_each_obs));
                    repulsive_force_human_slope_force[i] = beta_velocity_human*(repulsive_force_human_new[i]-repulsive_force_human_old[i])/0.01;
                    repulsive_force_human_slope_lpf[i] = alpha*repulsive_force_human_slope_force[i] + (1-alpha)*repulsive_force_human_slope_lpf_old[i];
                    repulsive_force_human_slope_lpf_old[i] = repulsive_force_human_slope_lpf[i];
                    repulsive_force_human_old[i] = repulsive_force_human_new[i];
                    cnt_for_slope_wall_human = 0;
                }
                if(repulsive_force_human_slope_lpf[i] >= 0.0){ // If approaching the obstacle
                    if(repulsive_force_human_slope_lpf[i] > max_force_vel_cutoff ){ //max cutoff for force added
                        repulsive_force_human_slope_lpf[i] = max_force_vel_cutoff;
                    }
                }else{ // If getting further away, zero out the force   
                    repulsive_force_human_slope_lpf[i] = 0.0; //-repulsive_force_human_slope_lpf[i];
                }
                
                repulsive_force_human_final[i] = repulsive_force_human_slope_lpf[i];
                repulsive_force_human[0] = -repulsive_force_human_final[i]*cos(thetaO);
                repulsive_force_human[1] = -repulsive_force_human_final[i]*sin(thetaO);
            }
            else{
                repulsive_force_human[0] = 0.0;
                repulsive_force_human[1] = 0.0;
            }
            wall_force_x_human += repulsive_force_human[0];
            wall_force_y_human += repulsive_force_human[1];
        }
        obs_repul_force_x_human = (wall_force_x_human*6.0 + obs_force_x_human*1.5)*10.0;
        obs_repul_force_y_human = (wall_force_y_human*16.0 + obs_force_y_human*4.0)*10.0;
        // printf("obs_force: %f %f, wall_force %f %f, total_force %f %f\n", obs_force_x_human*5.0, obs_force_y_human*5.0, wall_force_x_human, wall_force_y_human, obs_repul_force_x_human, obs_repul_force_y_human);
        // printf("obs_force: %f %f, wall_force %f %f\n", obs_force_x_human, obs_force_y_human, wall_force_x_human, wall_force_y_human);
    }
    

    // if(map == 2){ //Path width map, only has wall repulsive force
    //     if(case_ == 1){ // feedback to controller
    //         repulsive_force_controller[0] = 0.0;
    //         repulsive_force_controller[1] = 0.0;

    //         //wall 1
    //         distance_to_wall = fnc_cal_distance_obs(rx, 0, -20, 0);
    //         if(distance_to_wall < wall_force_activate_distance){ 
    //             repulsive_force_controller[0] = repulsive_force_controller[0]+(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall);
    //         }
    //         //wall 2
    //         distance_to_wall = fnc_cal_distance_obs(0, ry, 0, 1.2192);
    //         if(distance_to_wall < wall_force_activate_distance && rx < -10.2464){ 
    //             thetaO = atan2(1.2192-ry, 0);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 3
    //         distance_to_wall = fnc_cal_distance_obs(rx, 0, -10.2464, 0);
    //         if(distance_to_wall < wall_force_activate_distance && ry > -6.096){ 
    //             thetaO = atan2(-10.2464-rx, 0);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 4
    //         distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -1.2192);
    //         if(distance_to_wall < wall_force_activate_distance && rx < -12.6848){ 
    //             thetaO = atan2(-1.2192-ry, 0);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 5
    //         distance_to_wall = fnc_cal_distance_obs(rx, 0, -12.6848, 0);
    //         if(distance_to_wall < wall_force_activate_distance && ry < -1.2192){ 
    //             thetaO = atan2(-12.6848-rx, 0);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;             
    //         }
    //         //wall 6
    //         distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -6.096);
    //         if(distance_to_wall < wall_force_activate_distance && rx > -10.2464 && rx < -5.3696 ){ 
    //             thetaO = atan2(-6.096-ry, 0);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 7
    //         distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -7.9248);
    //         if(distance_to_wall < wall_force_activate_distance && rx > -12.6848 && rx < -3.5408 ){ 
    //             thetaO = atan2(-7.9248-ry, 0);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 8
    //         distance_to_wall = fnc_cal_distance_obs(rx, 0, -5.3696, 0);
    //         if(distance_to_wall < wall_force_activate_distance && ry > -6.096 && ry < 0.5334){ 
    //             thetaO = atan2(-5.3696-rx, 0);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]+(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;             
    //         }
    //         //wall 9
    //         distance_to_wall = fnc_cal_distance_obs(rx, 0, -3.5408, 0);
    //         if(distance_to_wall < wall_force_activate_distance && ry > -7.9248 && ry < -0.6096){ 
    //             thetaO = atan2(-3.5408-rx, 0);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]+(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 10
    //         distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -0.6096);
    //         if(distance_to_wall < wall_force_activate_distance && rx > -3.5408 && rx < 1.9456){ 
    //             thetaO = atan2(-0.6096-ry, 0);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 11
    //         distance_to_wall = fnc_cal_distance_obs(0, ry, 0, 0.5334);
    //         if(distance_to_wall < wall_force_activate_distance && rx > -5.3696 && rx < 3.0886){ 
    //             thetaO = atan2(0.5334-ry, 0);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 12
    //         distance_to_wall = fnc_cal_distance_obs(rx, 0, 3.0886, 0);
    //         if(distance_to_wall < wall_force_activate_distance && ry > -6.7818 && ry < 0.5334){ 
    //             thetaO = atan2(3.0886-rx, 0);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 13
    //         distance_to_wall = fnc_cal_distance_obs(rx, 0, 1.9456, 0);
    //         if(distance_to_wall < wall_force_activate_distance && ry > -6.7818 && ry < -0.6096){ 
    //             thetaO = atan2(1.9456-rx, 0);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;             
    //         }
    //         //wall 14
    //         distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -6.7818);
    //         if(distance_to_wall < wall_force_activate_distance && rx > 1.9456 && rx < 3.0886){ 
    //             repulsive_force_controller[0] = repulsive_force_controller[0]-(neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall);
    //         }


    //         //edge 1
    //         distance_to_wall = fnc_cal_distance_obs(rx, ry, -12.6848, -1.2192);
    //         if(distance_to_wall < wall_force_activate_distance){
    //             repulsive_force_raw = (neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance))/(distance_to_wall*distance_to_wall));
    //             thetaO = atan2(-1.2192-ry, -12.6848-rx);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]-repulsive_force_raw*thetaO;
    //         }

    //         //edge 2
    //         distance_to_wall = fnc_cal_distance_obs(rx, ry, -10.2464, -6.096);
    //         if(distance_to_wall < wall_force_activate_distance){
    //             repulsive_force_raw = (neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance))/(distance_to_wall*distance_to_wall));
    //             thetaO = atan2(-6.096-ry, -10.2464-rx);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]-repulsive_force_raw*thetaO;
    //         }

    //         //edge 3
    //         distance_to_wall = fnc_cal_distance_obs(rx, ry, -5.3696, -6.096);
    //         if(distance_to_wall < wall_force_activate_distance){
    //             repulsive_force_raw = (neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance))/(distance_to_wall*distance_to_wall));
    //             thetaO = atan2(-6.096-ry, -5.3696-rx);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]-repulsive_force_raw*thetaO;
    //         }

    //         //edge 4
    //         distance_to_wall = fnc_cal_distance_obs(rx, ry, -3.5408, -0.6096);
    //         if(distance_to_wall < wall_force_activate_distance){
    //             repulsive_force_raw = (neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance))/(distance_to_wall*distance_to_wall));
    //             thetaO = atan2(-0.6096-ry, -3.5408-rx);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]-repulsive_force_raw*thetaO;
    //         }

    //         //edge 5
    //         distance_to_wall = fnc_cal_distance_obs(rx, ry, 1.9456, -0.6096);
    //         if(distance_to_wall < wall_force_activate_distance){
    //             repulsive_force_raw = (neta_controller*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance))/(distance_to_wall*distance_to_wall));
    //             thetaO = atan2(-0.6096-ry, 1.9456-rx);
    //             repulsive_force_controller[1] = repulsive_force_controller[1]-repulsive_force_raw*thetaO;
    //         }
            
            
    //         // Cutoff for repulsive force
    //         if(repulsive_force_controller[1] > 360.0 * DTR)
    //             repulsive_force_controller[1] = repulsive_force_controller[1] - 360.0 * DTR;
    //         else if(repulsive_force_controller[1] < - 360.0 * DTR)
    //             repulsive_force_controller[1] = repulsive_force_controller[1] + 360.0 * DTR;
    //         obs_repul_force_x_controller += repulsive_force_controller[0];
    //         obs_repul_force_y_controller += repulsive_force_controller[1];
    //         if(obs_repul_force_y_controller > 360 *M_PI/180){
    //             obs_repul_force_y_controller = obs_repul_force_y_controller - 360 *M_PI/180;
    //         }
    //         else if(obs_repul_force_y_controller < - 360 *M_PI/180){
    //             obs_repul_force_y_controller = obs_repul_force_y_controller + 360 *M_PI/180;
    //         }
    //     } 
    //     //human feedback
    //     else if(case_ == 0){
    //         repulsive_force_human[0] = 0.0;
    //         repulsive_force_human[1] = 0.0;

    //         //wall 1
    //         distance_to_wall = fnc_cal_distance_obs(rx, 0, -20, 0);
    //         if(distance_to_wall < wall_force_activate_distance){ 
    //             repulsive_force_human[0] = repulsive_force_human[0]+(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall);
    //         }
    //         //wall 2
    //         distance_to_wall = fnc_cal_distance_obs(0, ry, 0, 1.2192);
    //         if(distance_to_wall < wall_force_activate_distance && rx < -10.2464){ 
    //             thetaO = atan2(1.2192-ry, 0);
    //             repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 3
    //         distance_to_wall = fnc_cal_distance_obs(rx, 0, -10.2464, 0);
    //         if(distance_to_wall < wall_force_activate_distance && ry > -6.096){ 
    //             thetaO = atan2(-10.2464-rx, 0);
    //             repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 4
    //         distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -1.2192);
    //         if(distance_to_wall < wall_force_activate_distance && rx < -12.6848){ 
    //             thetaO = atan2(-1.2192-ry, 0);
    //             repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 5
    //         distance_to_wall = fnc_cal_distance_obs(rx, 0, -12.6848, 0);
    //         if(distance_to_wall < wall_force_activate_distance && ry < -1.2192){ 
    //             thetaO = atan2(-12.6848-rx, 0);
    //             repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;             
    //         }
    //         //wall 6
    //         distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -6.096);
    //         if(distance_to_wall < wall_force_activate_distance && rx > -10.2464 && rx < -5.3696 ){ 
    //             thetaO = atan2(-6.096-ry, 0);
    //             repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 7
    //         distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -7.9248);
    //         if(distance_to_wall < wall_force_activate_distance && rx > -12.6848 && rx < -3.5408 ){ 
    //             thetaO = atan2(-7.9248-ry, 0);
    //             repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 8
    //         distance_to_wall = fnc_cal_distance_obs(rx, 0, -5.3696, 0);
    //         if(distance_to_wall < wall_force_activate_distance && ry > -6.096 && ry < 0.5334){ 
    //             thetaO = atan2(-5.3696-rx, 0);
    //             repulsive_force_human[1] = repulsive_force_human[1]+(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;             
    //         }
    //         //wall 9
    //         distance_to_wall = fnc_cal_distance_obs(rx, 0, -3.5408, 0);
    //         if(distance_to_wall < wall_force_activate_distance && ry > -7.9248 && ry < -0.6096){ 
    //             thetaO = atan2(-3.5408-rx, 0);
    //             repulsive_force_human[1] = repulsive_force_human[1]+(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 10
    //         distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -0.6096);
    //         if(distance_to_wall < wall_force_activate_distance && rx > -3.5408 && rx < 1.9456){ 
    //             thetaO = atan2(-0.6096-ry, 0);
    //             repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 11
    //         distance_to_wall = fnc_cal_distance_obs(0, ry, 0, 0.5334);
    //         if(distance_to_wall < wall_force_activate_distance && rx > -5.3696 && rx < 3.0886){ 
    //             thetaO = atan2(0.5334-ry, 0);
    //             repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 12
    //         distance_to_wall = fnc_cal_distance_obs(rx, 0, 3.0886, 0);
    //         if(distance_to_wall < wall_force_activate_distance && ry > -6.7818 && ry < 0.5334){ 
    //             thetaO = atan2(3.0886-rx, 0);
    //             repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;
    //         }
    //         //wall 13
    //         distance_to_wall = fnc_cal_distance_obs(rx, 0, 1.9456, 0);
    //         if(distance_to_wall < wall_force_activate_distance && ry > -6.7818 && ry < -0.6096){ 
    //             thetaO = atan2(1.9456-rx, 0);
    //             repulsive_force_human[1] = repulsive_force_human[1]-(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall)*thetaO;             
    //         }
    //         //wall 14
    //         distance_to_wall = fnc_cal_distance_obs(0, ry, 0, -6.7818);
    //         if(distance_to_wall < wall_force_activate_distance && rx > 1.9456 && rx < 3.0886){ 
    //             repulsive_force_human[0] = repulsive_force_human[0]+(neta_human*(1.0/distance_to_wall - 1.0/(wall_force_activate_distance)))/(distance_to_wall*distance_to_wall);
    //         }

    //         obs_repul_force_x_human += repulsive_force_human[0];
    //         obs_repul_force_y_human += repulsive_force_human[1];
    //     }
    // }


    // dist_list.clear();
    // th_list.clear();

    return true;
}
