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
    
    for(int i=0;i<Num_obstacles+4;i++){
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
        
#ifdef DYNAMIC_MAP
        //dynamic obstacles initial velocity and direction
        if(i < Num_obstacles){
            dynamic_x[i] = 0.0;
            dynamic_y[i] = 0.0;

            while(randomVel_x[i] == 0.0){
            randomVel_x[i] = (((double)(rand() % 4 + 1))/1000.0); // between 0.001 and 0.005
            }
            while(randomVel_y[i] == 0.0){
                randomVel_y[i] = (((double)(rand() % 4 + 1))/1000.0); // between 0.001 and 0.005
            }
            if(rand() % 2 == 0){
                shift_x[i] = randomVel_x[i];
            }else{
                shift_x[i] = -randomVel_x[i];
            }
            if(rand() % 2 == 0){
                shift_y[i] = randomVel_y[i];
            }else{
                shift_y[i] = -randomVel_y[i];
            }
        }
        //printf("shift x, y = %f, %f \n",shift_x[0],shift_y[0]);
#endif
    }

    obs_repul_force_x_human = 0.0;
    obs_repul_force_y_human = 0.0; //y axis
    obs_repul_force_x_controller = 0.0;
    obs_repul_force_y_controller = 0.0; // yaw controller

    cnt_for_slope_controller = 0;
    cnt_for_slope_human = 0;
    distance_each_obs = 0.0;
    thetaO = 0.0;
}

double Potential_Field::fnc_cal_distance_obs(double rx, double ry, double goal_x, double goal_y) 
{
    return sqrt((goal_x-rx)*(goal_x-rx) + (goal_y-ry)*(goal_y-ry));
}

bool Potential_Field::fnc_repulsive_force_all(const mjModel *m, double rx, double ry, vector<double> ox, vector<double> oy, double torso_Yaw)
{
    //from others
    const double obsRad = 0.2; //0.2;
    // const double obsS = 19.0/5.0;
    const double obsS = 2.0; //10.0/5.0;
    const double max_force_vel_cutoff = 6.0;
    const double beta_velocity_controller = 1.0;
    const double beta_velocity_human = 0.1;
    double wall_force_x_human = 0.0;
    double wall_force_y_human = 0.0;
    double wall_force_x_controller = 0.0;
    double wall_force_y_controller = 0.0;
    double obs_force_x_human = 0.0;
    double obs_force_y_human = 0.0;
    double obs_force_x_controller = 0.0;
    double obs_force_y_controller = 0.0;
    double alpha = 0.15;
    double top_wall_y = 0.0;
    double bottom_wall_y = 0.0;
    double right_wall_x = 0.0;
    double left_wall_x = 0.0;

    obs_repul_force_x_human = 0.0;
    obs_repul_force_y_human = 0.0;
    obs_repul_force_x_controller = 0.0;
    obs_repul_force_y_controller = 0.0;

    // Wall force
    wall_force_x_human = 0.0;
    wall_force_y_human = 0.0;
    wall_force_x_controller = 0.0;
    wall_force_y_controller = 0.0;
    
    cnt_for_slope_controller++;
    cnt_for_slope_human++;

    for (int i=0; i<Num_obstacles+4; i++){ // Generate repulsive force for obstacles and four walls
        // if map is dynamic, update obstacle location
#ifdef DYNAMIC_MAP 
        if(clock() - now > delay && i < Num_obstacles){
            if(m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+0] >= -5.2){ // X wall boundary
                shift_x[i] = -shift_x[i];
                shift_y[i] = shift_y[i];
            }else if(m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+0] <= -19.8){
                shift_x[i] = -shift_x[i];
                shift_y[i] = shift_y[i];
            }
            if(m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+1] >= 3.8){ // Y wall boundary
                shift_x[i] = shift_x[i];
                shift_y[i] = -shift_y[i];
            }else if(m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+1] <= -3.8){
                shift_x[i] = shift_x[i];
                shift_y[i] = -shift_y[i];
            }
            m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+0] = m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+0]+shift_x[i];
            m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+1] = m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+1]+shift_y[i];
            dynamic_x[i] = m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+0];
            dynamic_y[i] = m->body_pos[mj_name2id(m, mjOBJ_BODY, obstacle_name[i])*3+1];
            if(i == Num_obstacles-1){
                now = clock();
            }
        }
        top_wall_y = 5.0;
        bottom_wall_y = -5.0;
        right_wall_x = 10.0;
        left_wall_x = -20.0;
#endif

#ifdef STATIC_MAP 
        top_wall_y = 1.2192;
        bottom_wall_y = -1.2192;
        right_wall_x = 10.0;
        left_wall_x = -20.0;
#endif
        

        // Calculate distance
    
        double theta_body = torso_Yaw;
        double v_body_x = 0.0;
        double v_body_y = 0.0;
        // //only consider first spin
        // if(theta_body > 2*M_PI && yaw_count == 0){
        //     theta_body = theta_body - 2*M_PI;
        //     yaw_count++; //0 -> 1
        // }
        // else if(theta_body < -2*M_PI && yaw_count == 0){
        //     theta_body = theta_body + 2*M_PI;
        //     yaw_count--; // 0 -> -1
        // }
        
        // //yaw_count != 0 
        // else if(theta_body > (yaw_count+1)*2*M_PI && yaw_count != 0){ // theta_body > 0
        //     theta_body = theta_body - (yaw_count+1)*2*M_PI;
        //     yaw_count++;
        // }
        // else if(theta_body < -(yaw_count+1)*2*M_PI && yaw_count != 0){
        //     theta_body = theta_body + (yaw_count+1)*2*M_PI;
        //     yaw_count--;
        // }
        // else theta_body = theta_body;

        if(i < Num_obstacles){
#ifdef STATIC_MAP
            // v_body_x = (ox[i]-rx)*cos(theta_body) - (oy[i]-ry)*sin(theta_body);
            // v_body_y = (oy[i]-ry)*cos(theta_body) + (ox[i]-rx)*sin(theta_body);

            v_body_x = (ox[i]-rx)*cos(theta_body) - (oy[i]-ry)*sin(theta_body);
            v_body_y = (oy[i]-ry)*cos(theta_body) + (ox[i]-rx)*sin(theta_body);
            thetaO = atan2(v_body_y, v_body_x);
            distance_each_obs = fnc_cal_distance_obs(rx, ry, ox[i], oy[i]);
#endif
#ifdef DYNAMIC_MAP
            v_body_x = (dynamic_x[i]-rx)*cos(theta_body) - (dynamic_y[i]-ry)*sin(theta_body);
            v_body_y = (dynamic_y[i]-ry)*cos(theta_body) + (dynamic_x[i]-rx)*sin(theta_body);
            thetaO = atan2(v_body_y, v_body_x);
            distance_each_obs = fnc_cal_distance_obs(rx, ry, dynamic_x[i], dynamic_y[i]);
#endif
        }else if(i == Num_obstacles){ //left wall
            thetaO = 0.0;
            distance_each_obs = fnc_cal_distance_obs(rx, 0, left_wall_x, 0);
        }else if(i == Num_obstacles+1){ //top wall
            // v_body_x = (0)*cos(theta_body) - (top_wall_y-ry)*sin(theta_body);
            // v_body_y = (top_wall_y-ry)*cos(theta_body) + (0)*sin(theta_body);
            // thetaO = atan2(v_body_y, v_body_x);
            thetaO = atan2(top_wall_y-ry, 0); 
            distance_each_obs = fnc_cal_distance_obs(0, ry, 0, top_wall_y);
        }else if(i == Num_obstacles+2){ //right wall
            thetaO = 0.0;
            distance_each_obs = fnc_cal_distance_obs(rx, 0, right_wall_x, 0);
        }else if(i == Num_obstacles+3){ //bottom wall
            // v_body_x = (0)*cos(theta_body) - (bottom_wall_y-ry)*sin(theta_body);
            // v_body_y = (bottom_wall_y-ry)*cos(theta_body) + (0)*sin(theta_body);
            // thetaO = atan2(v_body_y, v_body_x);
            thetaO = atan2(bottom_wall_y-ry, 0);
            distance_each_obs = fnc_cal_distance_obs(0, ry, 0, bottom_wall_y);
        }
        // printf("%f, %f, %f, %f, %f \n", rx, ox[i], ry, oy[i], distance_each_obs);

        //controller
#if defined CASE3_COMPENSATED_CONTROLLER || defined CASE4_COMPENSATED_CONTROLLER_WITH_FEEDBACK_TO_HUMAN 
        if(distance_each_obs < (obsS + obsRad)){
            if((cnt_for_slope_controller % 400 == 0)){ //10ms = 0.01s
                // printf("atan2: %f \n", thetaO);
#ifdef DYNAMIC_MAP
                // repulsive_force_controller_new[i] = 4.0/(1.0+exp(3.5*distance_each_obs));
                repulsive_force_controller_new[i] = 2.3/(1.0+exp(6.0*distance_each_obs));
#endif
#ifdef STATIC_MAP
                repulsive_force_controller_new[i] = 1.8/(1.0+exp(8.0*distance_each_obs));
#endif
                repulsive_force_controller_slope_force[i] = beta_velocity_controller*(repulsive_force_controller_new[i]-repulsive_force_controller_old[i])/0.01;
                repulsive_force_controller_slope_lpf[i] = alpha*repulsive_force_controller_slope_force[i] + (1-alpha)*repulsive_force_controller_slope_lpf_old[i];
                repulsive_force_controller_slope_lpf_old[i] = repulsive_force_controller_slope_lpf[i];
                repulsive_force_controller_old[i] = repulsive_force_controller_new[i];
                cnt_for_slope_controller = 0;
            }
            if(repulsive_force_controller_slope_lpf[i] <= 0.0){ // If getting further away, zero out the force   
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
        
        if(i < Num_obstacles){
            obs_force_x_controller += repulsive_force_controller[0];
            obs_force_y_controller += repulsive_force_controller[1];
        }else{
            wall_force_x_controller += repulsive_force_controller[0];
            wall_force_y_controller += repulsive_force_controller[1];
        }        
        
#endif
        //human feedback
#if defined CASE2_FEEDBACK_TO_HUMAN || defined CASE4_COMPENSATED_CONTROLLER_WITH_FEEDBACK_TO_HUMAN
        // Obstacle force
        if(distance_each_obs < (obsS + obsRad)){
            //Modified potential field force
            if((cnt_for_slope_human % 400 == 0)){ //10ms = 0.01s
#ifdef DYNAMIC_MAP
                repulsive_force_human_new[i] = 4.0/(1.0+exp(3.5*distance_each_obs));
#endif
#ifdef STATIC_MAP
                // repulsive_force_human_new[i] = 1.8/(1.0+exp(8.0*distance_each_obs));
                repulsive_force_human_new[i] = 2.7/(1.0+exp(6.0*distance_each_obs));
#endif
                repulsive_force_human_slope_force[i] = beta_velocity_human*(repulsive_force_human_new[i]-repulsive_force_human_old[i])/0.01;
                repulsive_force_human_slope_lpf[i] = alpha*repulsive_force_human_slope_force[i] + (1-alpha)*repulsive_force_human_slope_lpf_old[i];
                repulsive_force_human_slope_lpf_old[i] = repulsive_force_human_slope_lpf[i];
                repulsive_force_human_old[i] = repulsive_force_human_new[i];
                cnt_for_slope_human = 0;
            }
            if(repulsive_force_human_slope_lpf[i] <= 0.0){ // If getting further away, zero out the force   
                repulsive_force_human_slope_lpf[i] = 0.0; //-repulsive_force_human_slope_lpf[i];
            }
            // printf("i = %d, force = %f \n", i, repulsive_force_human_slope_lpf[i]);
            repulsive_force_human_final[i] = repulsive_force_human_slope_lpf[i];
            repulsive_force_human[0] = -repulsive_force_human_final[i]*cos(thetaO);
            repulsive_force_human[1] = -repulsive_force_human_final[i]*sin(thetaO);
        }else{ 
            repulsive_force_human[0] = 0.0;
            repulsive_force_human[1] = 0.0;
        }

        if(i < Num_obstacles){
            obs_force_x_human += repulsive_force_human[0];
            obs_force_y_human += repulsive_force_human[1];
        }else{
            wall_force_x_human += repulsive_force_human[0];
            wall_force_y_human += repulsive_force_human[1];
        }        
#endif
    }

    // Sum wall and obs force + cutoffs
#if defined CASE3_COMPENSATED_CONTROLLER || defined CASE4_COMPENSATED_CONTROLLER_WITH_FEEDBACK_TO_HUMAN 
    #ifdef DYNAMIC_MAP
        // obs_repul_force_x_controller = (wall_force_x_controller*10.0 + obs_force_x_controller*10.0)*0.001;
        obs_repul_force_y_controller = (wall_force_y_controller*70.0 + obs_force_y_controller*200.0)*0.001;
    #endif
    #if defined STATIC_MAP 
        // obs_repul_force_x_controller = (wall_force_x_controller*10.0 + obs_force_x_controller*10.0)*0.001;
        obs_repul_force_y_controller = (wall_force_y_controller*60.0 + obs_force_y_controller*270.0)*0.001;
        obs_repul_force_y_controller = (obs_force_y_controller*270.0)*0.001;
    #endif
    if(obs_repul_force_y_controller > 360 *M_PI/180){
        obs_repul_force_y_controller = obs_repul_force_y_controller - 360 *M_PI/180;
    }
    else if(obs_repul_force_y_controller < - 360 *M_PI/180){
        obs_repul_force_y_controller = obs_repul_force_y_controller + 360 *M_PI/180;
    }
#endif

#if defined CASE2_FEEDBACK_TO_HUMAN
    // obs_repul_force_x_human = (wall_force_x_human*6.0 + obs_force_x_human*1.5)*8.0;
    // obs_repul_force_y_human = (wall_force_y_human*16.0 + obs_force_y_human*4.0)*25.0;
    // obs_repul_force_x_human = (wall_force_x_human*6.0 + obs_force_x_human*6.0)*12.0;
    obs_repul_force_y_human = (wall_force_y_human*7 + obs_force_y_human*16.0)*35.0;

#endif
#if defined CASE4_COMPENSATED_CONTROLLER_WITH_FEEDBACK_TO_HUMAN
    obs_repul_force_y_human = (wall_force_y_human*7 + obs_force_y_human*21.0)*35.0;
#endif
    //printf("obs_force: %f %f, wall_force %f %f, total_force %f %f\n", obs_force_x_controller, obs_force_y_controller, wall_force_x_controller, wall_force_y_controller, obs_repul_force_x_controller, obs_repul_force_y_controller);
    // printf("obs_force: %f %f, wall_force %f %f, total_force %f %f\n", obs_force_x_human, obs_force_y_human, wall_force_x_human, wall_force_y_human, obs_repul_force_x_human, obs_repul_force_y_human);
    //printf("shift x, y = %f, %f \n",shift_x[0],shift_y[0]);
    return true;
}
