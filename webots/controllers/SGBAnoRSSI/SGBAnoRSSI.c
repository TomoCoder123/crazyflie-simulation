/*
 * File:          SGBA_runner.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
 
#include <string.h>
#include <errno.h>
#include <math.h>

//#include "FreeRTOS.h"
//#include "task.h"
//#include "semphr.h"
//#include "log.h"
//#include "param.h"
//#include "system.h"
//rtos stuff
//#include "gradient_bug.h"
//#include "commander.h"
//#include "sensors.h"
//#include "stabilizer_types.h"

//#include "estimator_kalman.h"
//#include "stabilizer.h"

//#include "wallfollowing_multiranger_onboard.h"
//#include "lobe_navigation.h"
//#include "wallfollowing_with_avoid.h"
//#include "com_bug_with_looping.h"
//#include "com_bug_with_looping_and_avoid.h"
//#include "lobe_bug_with_looping.h"
//#include "gradient_bug_with_looping.h"


// #include "oa.h"
// #include "multiranger.h"

// #include "radiolink.h"

// #include "median_filter.h"

// #include "stereoboard.h"
// #include "flowdeck_v1v2.h"
#include <webots/robot.h>

#include "stabilizer_types.h"

#include "../../../controllers/pid_controller.h"

#include "../../../controllers/wallfollowing_multiranger.h"
#include "../../../controllers/wallfollowing_multiranger_onboard.h"
#include "../../../controllers/SGBA.h" 
#include "../../../controllers/wallfollowing_with_avoid.h"
#include <math.h>
#include <stdio.h>
#include<time.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
static bool first_run = true;
static float ref_distance_from_wall = 0;
static float max_speed = 0.5;
uint8_t rssi_threshold = 58;// normal batteries 50/52/53/53 bigger batteries 55/57/59
uint8_t rssi_collision_threshold = 50; // normal batteris 43/45/45/46 bigger batteries 48/50/52
//assuming just sets a number for distance to something and perform an action when at that distance
static bool outbound = true;
bool on_the_ground = true;
float height;
float state_start_time;
static bool taken_off = false;
static float nominal_height = 0.4; //???
static bool keep_flying = false;
point_t pos;

static float wanted_angle = 0;
static int own_id;


setpoint_t setpoint_BG;
float vel_x_cmd, vel_y_cmd, vel_w_cmd;
float current_heading;
float right_range;
float front_range;
float left_range;
float up_range;
float back_range;
float rssi_angle;
int state;
int state_wf;
float up_range_filtered;
uint8_t rssi_beacon_filtered;
bool made_it = false;
uint8_t send_to_number = 0;

static float wraptopi(float number) //just makes radians conventional
{
  if (number > (float)M_PI) {
    return (number - (float)(2 * M_PI));
  } else if (number < (float)(-1 * M_PI)) {
    return (number + (float)(2 * M_PI));
  } else {
    return (number);
  }

}
static void hoversp(setpoint_t *sp, float height)
{
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->mode.z = modeAbs;
  sp->velocity.x = 0.0;
  sp->velocity.y = 0.0;
  sp->position.z = height;
  sp->mode.yaw = modeVelocity;
  sp->attitudeRate.yaw = 0.0;
}
static int get_time(){
 

 #include <sys/time.h>

 struct timeval start;
 gettimeofday(&start, NULL);
//do stuff

 return (start.tv_sec) * 1000000 + start.tv_usec; 
}

static void set_id(){
 
 const char *name = wb_robot_get_name();
 own_id = name[9];
 
}
static int transition(int new_state) //takes in state, sets the time of the state, returns new state) 
{

  //float t = usecTimestamp() / 1e6;

  //state_start_time = t; //global variable referring to the time of the state
  float t =  get_time();
  state_start_time = t;
  
  return new_state;

}
// static void hover(setpoint_t *sp, float height)
// {
  // sp->mode.x = modeVelocity; //arrow??
  // sp->mode.y = modeVelocity;
  // sp->mode.z = modeAbs;
  // sp->velocity.x = 0.0;
  // sp->velocity.y = 0.0;
  // sp->position.z = height;
  // sp->mode.yaw = modeVelocity;
  // sp->attitudeRate.yaw = 0.0;
// }
void init_gradient_bug_loop_controller(float new_ref_distance_from_wall, float max_speed_ref,
                                       float begin_wanted_heading) //same as wall follower init
{
  ref_distance_from_wall = new_ref_distance_from_wall;
  max_speed = max_speed_ref;
  wanted_angle = begin_wanted_heading;
  //init_median_filter_i(&medFiltdiffRssi,101);
  first_run = true;
}
int gradient_bug_loop_controller(float *vel_x, float *vel_y, float *vel_w, int *state_wallfollowing,
                                 float front_range, float left_range, float right_range, float back_range,
                                 float current_heading, float current_pos_x, float current_pos_y, bool priority, bool outbound)
                                 //float *rssi_angle after vel_w
                                 //uint8_t rssi_beacon after current_pos,  then uint8_t rssi_inter, 
                                 //
                                 
                                 //rssi_inter, rssi_angle_inter, rssi_beacon??
                                 //&vel_x_cmd, &vel_y_cmd, &vel_w_cmd, &rssi_angle, &state_wf, front_range,
                                           //  left_range, right_range, back_range, current_heading,
                                             //(float)pos.x, (float)pos.y, rssi_beacon_filtered, rssi_inter_ext, rssi_angle_inter_ext, priority, outbound
{
      int state = 2;
                       
      static int state_wf = 0;
      static float wanted_angle_dir = 0;
      static float pos_x_hit = 0;
      static float pos_y_hit = 0;
      static float pos_x_sample = 0;
      static float pos_y_sample = 0;
      
      static bool overwrite_and_reverse_direction = false;
      static float direction = 1;
      static bool cannot_go_to_goal = false;
      static uint8_t prev_rssi = 150;
      static int diff_rssi = 0;
      static bool rssi_sample_reset = false;
      static float heading_rssi = 0;
      static uint8_t correct_heading_array[8] = {0}; //make heading array
      
      static bool first_time_inbound = true;
      static float wanted_angle_hit = 0;
      
      if (first_run) {

        wanted_angle_dir = wraptopi(current_heading - wanted_angle); // to determine the direction when turning to goal

        overwrite_and_reverse_direction = false;
        state = 2;
       
        float t = get_time();
        state_start_time = t;
        //#endif
        first_run = false;
  }
  


    if (first_time_inbound) { 
        wraptopi(wanted_angle - 3.14f);
        wanted_angle_dir = wraptopi(current_heading - wanted_angle);
        state = transition(2);
        first_time_inbound = false;
  }
  /***********************************************************
   * Handle state transitions
   ***********************************************************/
  if (state == 1){
    if (front_range < ref_distance_from_wall + 0.2f) {
      if (overwrite_and_reverse_direction) {
        direction = -1.0f * direction; //direction of drone
        overwrite_and_reverse_direction = false;
      } else {
        if (left_range < right_range && left_range < 2.0f) {
          direction = -1.0f;
        } else if (left_range > right_range && right_range < 2.0f) {
          direction = 1.0f;

        } else if (left_range > 2.0f && right_range > 2.0f) {
          direction = 1.0f;
        } else {

        }
      }

      pos_x_hit = current_pos_x;
      pos_y_hit = current_pos_y;
      wanted_angle_hit = wanted_angle;

      wallFollowerInit(0.4, 0.5, 3); //make wall follower function

      for (int it = 0; it < 8; it++) { correct_heading_array[it] = 0; }

      state = transition(3); //wall_following

    }
    
    
    }
  
  

  else if (state ==2){
    // check if heading is close to the preferred_angle
    bool goal_check = logicIsCloseTo(wraptopi(current_heading - wanted_angle), 0, 0.1f); //make logic function
    if (front_range < ref_distance_from_wall + 0.2f) {
      cannot_go_to_goal =  true;
      wallFollowerInit(0.4, 0.5, 3);

      state = transition(3); //wall_following

    }
    if (goal_check) {
      state = transition(1); //forward
    }
  
  
  }
  else if (state == 3){
   // if another drone is close and there is no right of way, move out of the way
    if (priority == false) { //&& rssi_inter < rssi_threshold after false
      //if (outbound) {
        //if ((rssi_angle_inter < 0 && wanted_angle < 0) || (rssi_angle_inter > 0 && wanted_angle > 0)) {
          //wanted_angle = -1 * wanted_angle;
          //wanted_angle_dir = wraptopi(current_heading - wanted_angle);
          //state= transition(2);
        //}
      //}
      //if (rssi_inter < rssi_collision_threshold) {
        state = transition(4);

      //}
    }

    // If going forward with wall following and cannot_go_to_goal bool is still on
    //    turn it off!
    if (state_wf == 5 && cannot_go_to_goal) {
      cannot_go_to_goal  = false;
    }



    // Check if the goal is reachable from the current point of view of the agent
    float bearing_to_goal = wraptopi(wanted_angle - current_heading);
    bool goal_check_WF = false;
    if (direction == -1) {
      goal_check_WF = (bearing_to_goal < 0 && bearing_to_goal > -1.57f);
    } else {
      goal_check_WF = (bearing_to_goal > 0 && bearing_to_goal < 1.57f);
    }

    // Check if bug went into a looping while wall following,
    //    if so, then forse the reverse direction predical.
    float rel_x_loop = current_pos_x - pos_x_hit;   //  diff_rssi = (int)prev_rssi - (int)rssi_beacon;
    float rel_y_loop = current_pos_y - pos_y_hit;
    float loop_angle = wraptopi(atan2(rel_y_loop, rel_x_loop));

    //if(outbound)
    //{


    if (fabs(wraptopi(wanted_angle_hit + 3.14f - loop_angle)) < 1.0) {
      overwrite_and_reverse_direction = true;
    } else {
     // TODO: check following line causes deadlocks!!!!
      //overwrite_and_reverse_direction = false; // this didn't really work...
    }
    //}else{
    //  overwrite_and_reverse_direction = false;
    //}

    // if during wallfollowing, agent goes around wall, and heading is close to rssi _angle
    //      got to rotate to goal
    // TODO check if the cannot go to goal still does anything useful...
    if ((state_wf == 6 || state_wf == 8) && goal_check_WF && front_range > ref_distance_from_wall + 0.4f
        && !cannot_go_to_goal) {
      wanted_angle_dir = wraptopi(current_heading - wanted_angle); // to determine the direction when turning to goal
      state = transition(2); //rotate_to_goal
    }

    // If going straight
    //    determine by the gradient of the crazyradio what the approx direction is.
    if (state_wf == 5) {


      // if (!outbound) {
        //Reset sample gathering
        // if (rssi_sample_reset) {
          // pos_x_sample = current_pos_x;
          // pos_y_sample = current_pos_y;
          // rssi_sample_reset = false;
          // prev_rssi = rssi_beacon;
        // }


        //if the crazyflie traveled for 1 meter, than measure if it went into the right path
        // float rel_x_sample = current_pos_x - pos_x_sample;
        // float rel_y_sample = current_pos_y - pos_y_sample;
        // float distance = sqrt(rel_x_sample * rel_x_sample + rel_y_sample * rel_y_sample);
        // if (distance > 1.0f) {
          // rssi_sample_reset = true;
          // heading_rssi = current_heading;
          // int diff_rssi_unf = (int)prev_rssi - (int)rssi_beacon;

          // //rssi already gets filtered at the radio_link.c
          // diff_rssi = diff_rssi_unf; //diff rssi???

          //Estimate the angle to the beacon
          // wanted_angle = fillHeadingArray(correct_heading_array, heading_rssi, diff_rssi, 4); //make fillheading


          //for simulation
          // //printf("wanted_angle %f, heading_rssi %f, diff_rssi, %d \n",wanted_angle,heading_rssi,diff_rssi);
          // //for(int it=0;it<8;it++)printf("%d, ",correct_heading_array[it]);printf("\n");
        // }
      // }

    // } else {
      // rssi_sample_reset = true;
    // }
  
  
  // }
  // else if (state == 4){
  
  // if (rssi_inter >= rssi_collision_threshold) {

      // state = transition(2); //rotate_to_goal
    // }
  
  }
    /***********************************************************
   * Handle state actions
   ***********************************************************/
  float temp_vel_x = 0;
  float temp_vel_y = 0; //xyw??
  float temp_vel_w = 0; //???
  
  if (state == 1){
  
    if (left_range < ref_distance_from_wall) {
      temp_vel_y = -0.2f; //why change y velocity
    }
    if (right_range < ref_distance_from_wall) {
      temp_vel_y = 0.2f;
    }
    temp_vel_x = 0.5;
  
  }
  else if (state ==2){
  
   if (wanted_angle_dir < 0) {
      commandTurn(&temp_vel_w, 0.5); //make turn function
    } else {
      commandTurn(&temp_vel_w, -0.5); 
    }
  
  
  }
  else if (state == 3){
   if (direction == -1) {
      state_wf = wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range, left_range, current_heading, direction); //make wallfollow function
    } else {
      state_wf = wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range, right_range, current_heading, direction); 
    }
  
  
  }
    else if (state == 4){
  float save_distance = 0.7f;
    if (left_range < save_distance) {
      temp_vel_y = temp_vel_y - 0.5f;
    }
    if (right_range < save_distance) {
      temp_vel_y = temp_vel_y + 0.5f;
    }
    if (front_range < save_distance) {
      temp_vel_x = temp_vel_x - 0.5f;
    }
    if (back_range < save_distance) {
      temp_vel_x = temp_vel_x + 0.5f;
    }
  
  }                    
     }
                        
  /***********************************************************
   * Handle state actions
   ***********************************************************/


int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  set_id();
  wb_robot_init();
  
  int timestep = (int)wb_robot_get_basic_time_step();

  // Initialize motors
  WbDeviceTag m1_motor = wb_robot_get_device("m1_motor");
  wb_motor_set_position(m1_motor, INFINITY);
  wb_motor_set_velocity(m1_motor, -1.0);
  WbDeviceTag m2_motor = wb_robot_get_device("m2_motor");
  wb_motor_set_position(m2_motor, INFINITY);
  wb_motor_set_velocity(m2_motor, 1.0);
  WbDeviceTag m3_motor = wb_robot_get_device("m3_motor");
  wb_motor_set_position(m3_motor, INFINITY);
  wb_motor_set_velocity(m3_motor, -1.0);
  WbDeviceTag m4_motor = wb_robot_get_device("m4_motor");
  wb_motor_set_position(m4_motor, INFINITY);
  wb_motor_set_velocity(m4_motor, 1.0);

  // Initialize Sensors
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep); //The timestep argument specifies the sampling period of the sensor and is expressed in milliseconds. Note that the first measurement will be available only after the first sampling period elapsed.
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  wb_keyboard_enable(timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag range_front = wb_robot_get_device("range_front");
  wb_distance_sensor_enable(range_front, timestep);
  WbDeviceTag range_left = wb_robot_get_device("range_left");
  wb_distance_sensor_enable(range_left, timestep);
  WbDeviceTag range_back = wb_robot_get_device("range_back");
  wb_distance_sensor_enable(range_back, timestep);
  WbDeviceTag range_right = wb_robot_get_device("range_right");
  wb_distance_sensor_enable(range_right, timestep);
  
  ActualState_t actualState = {0};
  DesiredState_t desiredState = {0};
  double pastXGlobal =0;
  double pastYGlobal=0;
  double past_time = wb_robot_get_time();
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 2.0)
      break;
  }
  
  GainsPID_t gainsPID;
  gainsPID.kp_att_y = 1;
  gainsPID.kd_att_y = 0.5;
  gainsPID.kp_att_rp =0.5;
  gainsPID.kd_att_rp = 0.1;
  gainsPID.kp_vel_xy = 2;
  gainsPID.kd_vel_xy = 0.5;
  gainsPID.kp_z = 10;
  gainsPID.ki_z = 50;
  gainsPID.kd_z = 5;
  init_pid_attitude_fixed_height_controller();
  desiredState.roll = 0;
  
  desiredState.pitch = 0;
  desiredState.vx = 0;
  desiredState.vy = 0;
  desiredState.yaw_rate = 0;
  desiredState.altitude = 0.7;
  MotorPower_t motorPower;

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    height = estimatorKalmanGetElevation(); //find way to get the elevation
    current_heading = getHeading() * (float)M_PI / 180.0f; //find way to get heading
    const double dt = wb_robot_get_time() - past_time;
    // Get measurements
    
    actualState.roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    actualState.pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    actualState.yaw_rate = wb_gyro_get_values(gyro)[2];
    actualState.altitude = wb_gps_get_values(gps)[2];
    double xGlobal= wb_gps_get_values(gps)[0];
    double vxGlobal = (xGlobal - pastXGlobal)/dt;
    double yGlobal = wb_gps_get_values(gps)[1];
    double vyGlobal = (yGlobal - pastYGlobal)/dt;

    // Get body fixed velocities
    double actualYaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    double cosyaw = cos(actualYaw);
    double sinyaw = sin(actualYaw);
    actualState.vx = vxGlobal * cosyaw + vyGlobal * sinyaw;
    actualState.vy = - vxGlobal * sinyaw + vyGlobal * cosyaw;
    float range_front_value = (float)wb_distance_sensor_get_value(range_front)/1000.0f;
    float range_right_value = (float)wb_distance_sensor_get_value(range_right)/1000.0f;
    
    
    wb_motor_set_velocity(m1_motor, - motorPower.m1);
    wb_motor_set_velocity(m2_motor, motorPower.m2);
    wb_motor_set_velocity(m3_motor, - motorPower.m3);
    wb_motor_set_velocity(m4_motor, motorPower.m4);
    
    past_time = wb_robot_get_time();
    pastXGlobal = xGlobal;
    pastYGlobal = yGlobal;
    
    point_t pos; //???
    estimatorKalmanGetEstimatedPos(&pos);  //find way to get position
    uint8_t rssi_beacon_threshold = 58; //what does threshold do?
    
    
    state = 0;
    //rssi_beacon_filtered = rssi_ext; //??


    // Main flying code
    if (keep_flying) {
      if (taken_off) {
        /*
         * If the flight is given a OK
         *  and the crazyflie has taken off
         *   then perform state machine
         */
        hoversp(&setpoint_BG, nominal_height); //make hover function 
        bool priority = false;
        //if (id_inter_ext > own_id) { //define these id variables
          //priority = true;
        //} else {
          //priority = false;

        //}
        state = gradient_bug_loop_controller(&vel_x_cmd, &vel_y_cmd, &vel_w_cmd,&state_wf, front_range,
                                             left_range, right_range, back_range, current_heading,
                                             (float)pos.x, (float)pos.y, priority, outbound); //already taken off


  }
  else{
      if (own_id == 1) { //make own_id
            init_gradient_bug_loop_controller(0.4, 0.5, -0.8);//hasnt taken off
          } else if (own_id == 2) {
            init_gradient_bug_loop_controller(0.4, 0.5, 0.8);
          } else if (own_id == 3) {
            init_gradient_bug_loop_controller(0.4, 0.5, -2.4);
          } //else if (own_id == 5 || own_id == 9) {
           // init_gradient_bug_loop_controller(0.4, 0.5, 2.4);
         // } else {
        //    init_gradient_bug_loop_controller(0.4, 0.5, 0.8);
         // }
  }
  }
  else{
        /*
         * If the flight is given a not OK
         *  but the crazyflie  has already taken off
         *   then land
         */
        land(&setpoint_BG, 0.2f); //find way to create a land function
        if (height < 0.1f) {
          shut_off_engines(&setpoint_BG); 
          taken_off = false;
        }
        on_the_ground = false;
  
  }
    //commanderSetSetpoint(&setpoint_BG, GRADIENT_BUG_COMMANDER_PRI); //understand this set point stuff
    //radiolinkSendInfoGradientBug(state, rssi_angle, send_to_number); //find way to implement rssi 
  }
  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  
  wb_robot_cleanup();

  return 0;
}
