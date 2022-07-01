/*
 * Copyright 1996-2021 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Description:   Robot execution code for genetic algorithm

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#define NUM_WHEELS 4
#define NUM_LEGS 4
#define NUM_SENSORS 4
#define GENOTYPE_SIZE (NUM_SENSORS * NUM_WHEELS)
#define SPEED_UNIT 0.00628

// sensor to wheels multiplication matrix
// each each sensor has a weight for each wheel
double matrix[NUM_SENSORS][NUM_WHEELS];

WbDeviceTag sensors[NUM_LEGS];     // proximity sensors
WbDeviceTag motors[NUM_LEGS];     // proximity sensors

WbDeviceTag receiver;                 // for receiving genes from Supervisor
WbDeviceTag left_motor, right_motor;  // motors

// 
double alpha[] = {0.0, 1.0, 2.4, 0.0};
double beta[] = {2.0, 5.05, 3.25, 3.5};

// initial velocities
double velocity[] = {0.0, 0.0, 0.0, 0.0};

// check if a new set of genes was sent by the Supervisor
// in this case start using these new genes immediately
void check_for_new_genes() {
  if (wb_receiver_get_queue_length(receiver) > 0) {
    // check that the number of genes received match what is expected
    assert(wb_receiver_get_data_size(receiver) == GENOTYPE_SIZE * sizeof(double));

    // copy new genes directly in the sensor/actuator matrix
    // we don't use any specific mapping nor left/right symmetry
    // it's the GA's responsability to find a functional mapping
    memcpy(matrix, wb_receiver_get_data(receiver), GENOTYPE_SIZE * sizeof(double));

    // prepare for receiving next genes packet
    wb_receiver_next_packet(receiver);
  }
}

static double clip_value(double value, double min_max) {
  if (value > min_max)
    return min_max;
  else if (value < -min_max)
    return -min_max;

  return value;
}

void sense_compute_and_actuate() {
  // read sensor values
  
  for (int i = 0; i < NUM_SENSORS; i++){
    double sensor_value = wb_position_sensor_get_value(sensors[i]);
    
    if(velocity[i]>=0){
      if((0.7-sensor_value) < 0.1){
        double currentVelocity = -1.0;
        wb_motor_set_velocity(motors[i], currentVelocity);
        velocity[i] = currentVelocity;
      }
      else{
        double currentVelocity = beta[i]*(0.7 - sensor_value)+alpha[i];
        wb_motor_set_velocity(motors[i], currentVelocity);
        velocity[i] = currentVelocity;
      }
    }
    
    else if(velocity[i]<0){
      if((sensor_value+0.7)<0.1){
        double currentVelocity = 1.0;
        wb_motor_set_velocity(motors[i], currentVelocity);
        velocity[i] = currentVelocity;
      }
      else{
        double currentVelocity = beta[i]*(-0.7 - sensor_value)-alpha[i];
        wb_motor_set_velocity(motors[i], currentVelocity);
        velocity[i] = currentVelocity;
      }
    }
  }
}

int main(int argc, const char *argv[]) {

  wb_robot_init();  // initialize Webots
  
  //printf("Controller of dog is running\n");

  // find simulation step in milliseconds (WorldInfo.basicTimeStep)
  int time_step = wb_robot_get_basic_time_step();

  // find and enable proximity sensors
  char name[32];
  int i;
  for (i = 0; i < NUM_SENSORS; i++) {
    sprintf(name, "leg_%d", i+1);
    motors[i] = wb_robot_get_device(name);
    
    sprintf(name, "pos_leg_%d", i+1);
    sensors[i] = wb_robot_get_device(name);
    wb_position_sensor_enable(sensors[i], time_step);
  }

  // find and enable receiver
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  
  for (i = 0; i < NUM_LEGS; i++) {
      wb_motor_set_position(motors[i], INFINITY);
      wb_motor_set_velocity(motors[i], 0.0);
  }
  
  // initialize matrix to zero, hence the robot
  // wheels will initially be stopped
  memset(matrix, 0.0, sizeof(matrix));

  // run until simulation is restarted
  while (wb_robot_step(time_step) != -1) {

    //check_for_new_genes();*/
    sense_compute_and_actuate();
  }

  wb_robot_cleanup();  // cleanup Webots
  return 0;            // ignored
}
