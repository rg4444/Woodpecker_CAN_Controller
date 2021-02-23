
/** 
 ** steering_pid.c - Steering angle PID controller
 **/

#include <stdio.h>
#include <sys/types.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <locale.h>
#include <time.h>
#include <pthread.h>
#include <ifaddrs.h>

#include "oscc.h"

#include "commander.h"
#include "timer.h"
#include "oscc_pid.h"
#include "steering_pid.h"


// Steering PID controller
pid_s steering_pid;

struct timeval t_steering;

int steering_running = 1;

pthread_t tid_steering_angle_loop;

float steering_angle_target = 0.0;

extern int system_mode;
extern double curr_angle;

extern act_s act;           /* Actuator signal state */

void set_steering_angle(double target_angle) {
    steering_angle_target = target_angle;
}

void* steering_angle_loop(void *arg) {
    unsigned long t;
    double control;
    
    printf("steering_angle_loop() starting... ");

    timer_start(&t_steering);

    while (steering_running) {
        t = timer_stop(&t_steering);
        
        if (t > T_STEERING) {
            timer_start(&t_steering);

            if (system_mode==0)         // if disabled mode
            {
                pid_zeroize( &steering_pid, STEERING_PID_WINDUP_GUARD );
                steering_angle_target = curr_angle;
            } 
            else
            {
				
//			    control = -pid_update(
//                    &steering_pid,
//                    steering_angle_target,
//                    curr_angle,
//                    T_STEERING ) + STEERING_PID_OUTPUT_V0;

					
			control = -pid_update(
                    &steering_pid,
                    steering_angle_target,
                    curr_angle,
                    T_STEERING );
				if (control > STEERING_PID_IDLE_MAX) 
					control = control + STEERING_PID_OUTPUT_V0;
				else if (control < -STEERING_PID_IDLE_MAX) 
					control = control - STEERING_PID_OUTPUT_V0;

					
                // check bounds of control
                if (control < STEERING_PID_OUTPUT_MIN)
                    control = STEERING_PID_OUTPUT_MIN;
                else if (control > STEERING_PID_OUTPUT_MAX)
                    control = STEERING_PID_OUTPUT_MAX;
				
               
				// send to actuator
                act.steering = calc_exponential_average(
                    act.steering,
                    control,
                    STEERING_FILTER_FACTOR);
					
		//		printf("Steering PID output: %f\t%f\t%f\t%f\t%f\t \n",control,steering_pid,curr_angle,steering_angle_target,act.steering); 
            }

        }
        usleep(1000);
    }

   printf("steering_angle_loop() stopped. ");
   return 0;
}


int steering_angle_loop_start() {

    pid_zeroize( &steering_pid, STEERING_PID_WINDUP_GUARD );
    steering_pid.proportional_gain = STEERING_PID_PROPORTIONAL_GAIN;
    steering_pid.integral_gain = STEERING_PID_INTEGRAL_GAIN;
    steering_pid.derivative_gain = STEERING_PID_DERIVATIVE_GAIN;

    steering_angle_target = 0.0;
    
    int rc = pthread_create(&tid_steering_angle_loop, NULL, &steering_angle_loop, NULL);
    if (rc != 0) 
    {
        printf("\nCan't create steering_angle_loop() thread :[%s]", strerror(rc));
    }
    return rc;
}


int steering_angle_loop_stop() {
    steering_running = 0;
    return 0;
}


void steering_angle_loop_wait() {
    pthread_join(tid_steering_angle_loop, NULL);
}

