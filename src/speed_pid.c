
/** 
 ** speed_pid.c - Speed PID controller
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
#include "speed_pid.h"


// Speed PID controller
pid_s speed_pid;

struct timeval t_speed;

int speed_running = 1;

pthread_t tid_speed_loop;

extern int system_mode;
extern double curr_speed;
extern act_s udpdrive_act;  /* Actuator signal state from udp */


extern struct timeval frame_timer;

extern act_s act_ctrl;

float speed_target = 0.0;


void set_speed(double target_speed) {
    act_ctrl.speed = target_speed;
    act_ctrl.speed_active = 1;
}


void* speed_loop(void *arg) {
    unsigned long t,t1;
    float control;
    
    printf("speed_loop() starting... ");

    timer_start(&t_speed);

//    float thr1 = 0.364727;
//    float thr2= 0.556871;
    float thr1 = 0.435367;
        float thr2= 0.780389;
           long unsigned T = 200000;

    while (speed_running) {
        t = timer_stop(&t_speed);
        
        if (t > T_SPEED) {

            timer_start(&t_speed);

            t1 = timer_stop(&frame_timer);

            if (system_mode==1 && act_ctrl.speed_active==1) 
            {

                if (curr_speed > act_ctrl.speed + SPEED_PRECISION)
                {
                    act_ctrl.braking = 0.0;
                    act_ctrl.throttle = 0.0;
                    printf("====== slow\n");
                } else {
                    if (curr_speed < act_ctrl.speed - SPEED_PRECISION)
                    {
                        act_ctrl.braking = 0.0;
                        act_ctrl.throttle = 1.0;
                    printf("====== fast   %f     %f\n",curr_speed,act_ctrl.speed - SPEED_PRECISION);
                    } else {
                    printf("====== hold\n");
                        act_ctrl.braking = 0.0;
                        if ((t1/T)%2==1)
                        {
                            act_ctrl.throttle = thr1;
                        } else {
                            act_ctrl.throttle = thr2;
                        }
                    }
                }

            }

        }
        usleep(1000);
    }

   printf("speed_loop() stopped. ");
   return 0;
}


int speed_loop_start() {
    int rc = pthread_create(&tid_speed_loop, NULL, &speed_loop, NULL);
    if (rc != 0) 
    {
        printf("\nCan't create speed_loop() thread :[%s]", strerror(rc));
    }
    return rc;
}


int speed_loop_stop() {
    speed_running = 0;
    return 0;
}


void speed_loop_wait() {
    pthread_join(tid_speed_loop, NULL);
}



