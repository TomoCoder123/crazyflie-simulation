/*
 * File:          SGBA_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <math.h>
#include <stdio.h>
#include<time.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>

#include "../../../controllers/pid_controller.h"
#include "../../../controllers/wallfollowing_multiranger.h"
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

static void take_off(setpoint_t *sp, float velocity)
{
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->mode.z = modeVelocity;
  sp->velocity.x = 0.0;
  sp->velocity.y = 0.0;
  sp->velocity.z = velocity;
  sp->mode.yaw = modeVelocity;
  sp->attitudeRate.yaw = 0.0;
}

static void land(setpoint_t *sp, float velocity)
{
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->mode.z = modeVelocity;
  sp->velocity.x = 0.0;
  sp->velocity.y = 0.0;
  sp->velocity.z = - velocity;
  sp->mode.yaw = modeVelocity;
  sp->attitudeRate.yaw = 0.0;
}


static void hover(setpoint_t *sp, float height)
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

static void vel_command(setpoint_t *sp, float vel_x, float vel_y, float yaw_rate, float height)
{
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->mode.z = modeAbs;
  sp->velocity.x = vel_x;
  sp->velocity.y = vel_y;
  sp->position.z = height;
  sp->mode.yaw = modeVelocity;
  sp->attitudeRate.yaw = yaw_rate;
  sp->velocity_body = true;

}

static void shut_off_engines(setpoint_t *sp)
{
  sp->mode.x = modeDisable;
  sp->mode.y = modeDisable;
  sp->mode.z = modeDisable;
  sp->mode.yaw = modeDisable;

}

float state_start_time;
uint8_t rssi_threshold = 58;// normal batteries 50/52/53/53 bigger batteries 55/57/59
uint8_t rssi_collision_threshold = 50; // normal batteris 43/45/45/46 bigger batteries 48/50/52
//assuming just sets a number for distance to something and perform an action when at that distance

//static variables only used for initialization
static bool first_run = true;
static float ref_distance_from_wall = 0;
static float max_speed = 0.5;

#define deg2rad(angleDegrees) (angleDegrees * (float)M_PI / 180.0f)

// Converts radians to degrees.
#define rad2deg(angleRadians) (angleRadians * 180.0f / (float)M_PI)

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
 static int getTime(){
 

 #include <sys/time.h>

 struct timeval start;
 gettimeofday(&start, NULL);
//do stuff

 return (start.tv_sec) * 1000000 + start.tv_usec; 
}
 static int transition(int new_state) //takes in state, sets the time of the state, returns new state) 
{

  float t = getTime(); //need to replace usecTimestamp with a time unit

  state_start_time = t; //global variable referring to the time of the state

  return new_state;

}

static bool logicIsCloseTo(float real_value, float checked_value, float margin)  //logicIsCloseTo(wraptopi(current_heading - wanted_angle), 0, 0.1f); checks if it is close to whatever goal
{
  if (real_value > checked_value - margin && real_value < checked_value + margin) {
    return true;
  } else {
    return false;
  }
}
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
static void commandTurn(float *vel_w, float max_rate) //does this even do anyhting
{
  *vel_w = max_rate;
}
uint8_t maxValue(uint8_t myArray[], int size)
{
  /* enforce the contract */
  //assert(myArray && size);
  int i;
  uint8_t maxValue = myArray[0];

  for (i = 1; i < size; ++i) {
    if (myArray[i] > maxValue) {
      maxValue = myArray[i];
    }
  }
  return maxValue;
}






static float fillHeadingArray(uint8_t *correct_heading_array, float rssi_heading, int diff_rssi, int max_meters)
{

	//Heading array of action choices
  static float heading_array[8] = { -135.0f, -90.0f, -45.0f, 0.0f, 45.0f, 90.0f, 135.0f, 180.0f}; //these are the various actions that the robot can go in.
  float rssi_heading_deg = rad2deg(rssi_heading); //converts the rssi angle to radians

  for (int it = 0; it < 8; it++) {

	  // Fill array based on heading and rssi heading
    if ((rssi_heading_deg >= heading_array[it] - 22.5f && rssi_heading_deg < heading_array[it] + 22.5f && it != 7) || ( //what's special about 7
          it == 7 && (rssi_heading_deg >= heading_array[it] - 22.5f || rssi_heading_deg < -135.0f - 22.5f))) {
      uint8_t temp_value_forward = correct_heading_array[it];
      uint8_t temp_value_backward = correct_heading_array[(it + 4) % 8];

      // if gradient is good, increment the array corresponding to the current heading and decrement the exact opposite
      if (diff_rssi > 0) { //differential? is it if you are going the right way, then the headings change accoridngly to continue going in the right way?
        correct_heading_array[it] = temp_value_forward + 1; //(uint8_t)abs(diff_rssi);
        if (temp_value_backward > 0) {
          correct_heading_array[(it + 4) % 8] = temp_value_backward - 1;  //(uint8_t)abs(diff_rssi);
        }
        // if gradient is bad, decrement the array corresponding to the current heading and increment the exact opposite

      } else if (diff_rssi < 0) {
        if (temp_value_forward > 0) {
          correct_heading_array[it] = temp_value_forward - 1;  //(uint8_t)abs(diff_rssi);
        }
        correct_heading_array[(it + 4) % 8] = temp_value_backward + 1; //(uint8_t)abs(diff_rssi); //why do the modulus operation?
      }

    }
  }

// degrading function
  //    If one of the arrays goes over maximum amount of points (meters), then decrement all values
  if (maxValue(correct_heading_array, 8) > max_meters) {
    for (int it = 0; it < 8; it++) {
      if (correct_heading_array[it] > 0) {
        correct_heading_array[it] = correct_heading_array[it] - 1;
      }
    }
  }


  // Calculate heading where the beacon might be
  int count = 0;
  float y_part = 0, x_part = 0;

  for (int it = 0; it < 8; it++) {
    if (correct_heading_array[it] > 0) {
      x_part += (float)correct_heading_array[it] * (float)cos(heading_array[it] * (float)M_PI / 180.0f);
      y_part += (float)correct_heading_array[it] * (float)sin(heading_array[it] * (float)M_PI / 180.0f);

      //sum += heading_array[it];
      count = count + correct_heading_array[it];

    }
  }
  float wanted_angle_return = 0;
  if (count != 0) {
    wanted_angle_return = atan2(y_part / (float)count, x_part / (float)count);
  }


  return wanted_angle_return;

}





static float wanted_angle = 0;
void init_SGBA_controller(float new_ref_distance_from_wall, float max_speed_ref,
                                       float begin_wanted_heading)
{
  ref_distance_from_wall = new_ref_distance_from_wall;
  max_speed = max_speed_ref;
  wanted_angle = begin_wanted_heading;
  first_run = true;
}
int SGBA_controller(float *vel_x, float *vel_y, float *vel_w, float *rssi_angle, int *state_wallfollowing,
                                 float front_range, float left_range, float right_range, float back_range,
                                 float current_heading, float current_pos_x, float current_pos_y, uint8_t rssi_beacon,
                                 uint8_t rssi_inter, float rssi_angle_inter, bool priority, bool outbound)
{

  // Initalize static variables
  static int state = 2;
  //static float previous_heading = 0;
  static int state_wf = 0;
  static float wanted_angle_dir = 0;
  static float pos_x_hit = 0;
  static float pos_y_hit = 0;
  static float pos_x_sample = 0;
  static float pos_y_sample = 0;
  //static float pos_x_move = 0;
  //static float pos_y_move = 0;
  static bool overwrite_and_reverse_direction = false;
  static float direction = 1;
  static bool cannot_go_to_goal = false;
  static uint8_t prev_rssi = 150;
  static int diff_rssi = 0;
  static bool rssi_sample_reset = false;
  static float heading_rssi = 0;
  static uint8_t correct_heading_array[8] = {0};

  static bool first_time_inbound = true;
  static float wanted_angle_hit = 0;



  // if it is reinitialized
  if (first_run) {

    wanted_angle_dir = wraptopi(current_heading - wanted_angle); // to determine the direction when turning to goal

    overwrite_and_reverse_direction = false;
    state = 2;
    printf("hello");
    float t = getTime(); //what's t purpose
    state_start_time = t;
    first_run = false;
  }

  if (first_time_inbound) { //seems like there are multiple travels
    wraptopi(wanted_angle - 3.14f);
    wanted_angle_dir = wraptopi(current_heading - wanted_angle);
    state = transition(2);
    first_time_inbound = false;
  }

  /***********************************************************
   * State definitions
   ***********************************************************/
  // 1 = forward
  // 2 = rotate_to_goal //what is defined as goal
  // 3 = wall_following
  // 4 = move out of way

  /***********************************************************
   * Handle state transitions
   ***********************************************************/

  if (state == 1) {     //FORWARD
    if (front_range < ref_distance_from_wall + 0.2f) { // range_front_value = wb_distance_sensor_get_value(range_front));

// if looping is detected, reverse direction (only on outbound)
      if (overwrite_and_reverse_direction) {
        direction = -1.0f * direction;
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

      wallFollowerInit(0.4, 0.5, 3); //wallfolower funciton

      for (int it = 0; it < 8; it++) { correct_heading_array[it] = 0; }

      state = transition(3); //wall_following

    }
  } else if (state == 2) { //ROTATE_TO_GOAL
    // check if heading is close to the preferred_angle
    bool goal_check = logicIsCloseTo(wraptopi(current_heading - wanted_angle), 0, 0.1f);
    if (front_range < ref_distance_from_wall + 0.2f) {
      cannot_go_to_goal =  true;
      wallFollowerInit(0.4, 0.5, 3);//wall_follower_init(0.4, 0.5, 3);

      state = transition(3); //wall_following

    }
    if (goal_check) {
      state = transition(1); //forward
    }
  } else if (state == 3) {      //WALL_FOLLOWING

    // if another drone is close and there is no right of way, move out of the way
    if (priority == false && rssi_inter < rssi_threshold) { //rssi inter (between drones?)
      if (outbound) {
        if ((rssi_angle_inter < 0 && wanted_angle < 0) || (rssi_angle_inter > 0 && wanted_angle > 0)) {
          wanted_angle = -1 * wanted_angle;
          wanted_angle_dir = wraptopi(current_heading - wanted_angle);
          //state= transition(2);
        }
      }
      if (rssi_inter < rssi_collision_threshold) { //collision detection
        state = transition(4);

      }
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
      goal_check_WF = (bearing_to_goal < 0 && bearing_to_goal > -1.5f);
    } else {
      goal_check_WF = (bearing_to_goal > 0 && bearing_to_goal < 1.5f);
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
    }

    // if during wallfollowing, agent goes around wall, and heading is close to rssi _angle
    //      got to rotate to goal
    if ((state_wf == 6 || state_wf == 8) && goal_check_WF && front_range > ref_distance_from_wall + 0.4f
        && !cannot_go_to_goal) {
      wanted_angle_dir = wraptopi(current_heading - wanted_angle); // to determine the direction when turning to goal
      state = transition(2); //rotate_to_goal
    }

    // If going straight
    //    determine by the gradient of the crazyradio what the approx direction is.
    if (state_wf == 5) {


      if (!outbound) {
        // Reset sample gathering
        if (rssi_sample_reset) {
          pos_x_sample = current_pos_x;
          pos_y_sample = current_pos_y;
          rssi_sample_reset = false;
          prev_rssi = rssi_beacon;
        }


        // if the crazyflie traveled for 1 meter, than measure if it went into the right path
        float rel_x_sample = current_pos_x - pos_x_sample;
        float rel_y_sample = current_pos_y - pos_y_sample;
        float distance = sqrt(rel_x_sample * rel_x_sample + rel_y_sample * rel_y_sample);
        if (distance > 1.0f) {
          rssi_sample_reset = true;
          heading_rssi = current_heading;
          int diff_rssi_unf = (int)prev_rssi - (int)rssi_beacon;

          //rssi already gets filtered at the radio_link.c
          diff_rssi = diff_rssi_unf;

          // Estimate the angle to the beacon
          wanted_angle = fillHeadingArray(correct_heading_array, heading_rssi, diff_rssi, 5);
        }
      }

    } else {
      rssi_sample_reset = true;
    }
  } else if (state == 4) {    //MOVE_OUT_OF_WAY
    // once the drone has gone by, rotate to goal
    if (rssi_inter >= rssi_collision_threshold) {

      state = transition(2); //rotate_to_goal
    }

  }



  /***********************************************************
   * Handle state actions
   ***********************************************************/

  float temp_vel_x = 0;
  float temp_vel_y = 0;
  float temp_vel_w = 0;

  if (state == 1) {        //FORWARD
    // stop moving if there is another drone in the way
    // forward max speed
    if (left_range < ref_distance_from_wall) {
      temp_vel_y = -0.2f;
    }
    if (right_range < ref_distance_from_wall) {
      temp_vel_y = 0.2f;
    }
    temp_vel_x = 0.5;
    //}

  } else  if (state == 2) {  //ROTATE_TO_GOAL (what is goal)
    // rotate to goal, determined on the sign
    if (wanted_angle_dir < 0) {
      commandTurn(&temp_vel_w, 0.5);
    } else {
      commandTurn(&temp_vel_w, -0.5);
    }


  } else  if (state == 3) {       //WALL_FOLLOWING
    //Get the values from the wallfollowing
    if (direction == -1) {
      state_wf = wallFollower(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range, left_range, current_heading, direction);
    } else {
      state_wf = wallFollower(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range, right_range, current_heading, direction);
    }
  } else if (state == 4) {      //MOVE_AWAY

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


  *rssi_angle = wanted_angle;
  *state_wallfollowing = state_wf;

  *vel_x = temp_vel_x;
  *vel_y = temp_vel_y;
  *vel_w = temp_vel_w;

  return state;
}
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
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
  init_SGBA_controller(0.4, 0.5, -0.8);

  while (wb_robot_step(TIME_STEP) != -1) {
     if (wb_robot_get_time() > 2.0)
      break;
  };
  ActualState_t actualState = {0};
  DesiredState_t desiredState = {0};
  double pastXGlobal =0;
  double pastYGlobal=0;
  double past_time = wb_robot_get_time();
  
  GainsPID_t gainsPID;
  gainsPID.kp_att_y = 1; //?????
  gainsPID.kd_att_y = 0.5;
  gainsPID.kp_att_rp =0.5;
  gainsPID.kd_att_rp = 0.1;
  gainsPID.kp_vel_xy = 2;
  gainsPID.kd_vel_xy = 0.5;
  gainsPID.kp_z = 10;
  gainsPID.ki_z = 50;
  gainsPID.kd_z = 5;
  
  init_pid_attitude_fixed_height_controller(); //pid_controller()
  MotorPower_t motorPower; 
  wallFollowerInit(0.5, 0.3, 0); //wallfollowing_multiranger
  /* Enter your cleanup code here */
  while (wb_robot_step(timestep) != -1) {

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

    // Initialize values
    desiredState.roll = 0;
    desiredState.pitch = 0;
    desiredState.vx = 0;
    desiredState.vy = 0;
    desiredState.yaw_rate = 0;
    desiredState.altitude = 0.7;

    float forwardDesired = 0;
    float sidewaysDesired = 0;
    float yawDesired = 0; 

    // Control altitude
 
    // Example how to get sensor data
    // range_front_value = wb_distance_sensor_get_value(range_front));
    // const unsigned char *image = wb_camera_get_image(camera);


    float range_front_value = (float)wb_distance_sensor_get_value(range_front)/1000.0f;
    float range_right_value = (float)wb_distance_sensor_get_value(range_right)/1000.0f;

    CommandVel_t cmdVel; //wallfollowing multiranger()

    wallFollower(&cmdVel, range_front_value, range_right_value, actualYaw, 1, wb_robot_get_time());
    

    desiredState.yaw_rate = cmdVel.cmdAngW;

    // PID velocity controller with fixed height
    desiredState.vy = cmdVel.cmdVelY;
    desiredState.vx = cmdVel.cmdVelX;
    pid_velocity_fixed_height_controller(actualState, &desiredState,
    gainsPID, dt, &motorPower);

    // PID attitude controller with fixed height
    /*desiredState.roll = sidewaysDesired;
    desiredState.pitch = forwardDesired;
     pid_attitude_fixed_height_controller(actualState, &desiredState,
    gainsPID, dt, &motorPower);*/
    
    // Setting motorspeed
    wb_motor_set_velocity(m1_motor, - motorPower.m1);
    wb_motor_set_velocity(m2_motor, motorPower.m2);
    wb_motor_set_velocity(m3_motor, - motorPower.m3);
    wb_motor_set_velocity(m4_motor, motorPower.m4);
    
    // Save past time for next time step
    past_time = wb_robot_get_time();
    pastXGlobal = xGlobal;
    pastYGlobal = yGlobal;


  };
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
