
/** 
 ** udp_api.c - UDP API for commander
 **/

#include <stdio.h>
#include <sys/types.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <locale.h>
#include <time.h>
#include <pthread.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <net/ethernet.h>  //For ether_header
#include <netinet/in.h>
#include <netinet/udp.h>   //Provides declarations for udp header
#include <netinet/ip.h>    //Provides declarations for ip header
#include <netinet/if_ether.h>  //For ETH_P_ALL
#include <math.h>

#include "timer.h"
#include "commander.h"
#include "udp_api.h"


struct timeval t_udp_api;

int udp_api_running = 1;

pthread_t tid_udp_api_timer_loop;
pthread_t tid_udp_api_listener_loop;

act_s udpdrive_act;  /* Actuator signal state from udp */

extern double curr_speed;

void* udp_api_listener_loop(void *arg) {
   printf("UDP listener starting... ");
   udp_server();
   printf("UDP listener stopped. \n");
   return 0;
}


double steering_column_to_wheel_angle(double angle, double v) {
    if (fabs(v)<=60.0) {
 //       return angle*0.043169;
        return angle*0.0636942;
    } else {
        return angle*0.03195;
    }
}

double steering_wheel_to_column_angle(double angle, double v) {
    if (fabs(v)<=60.0) {
//        return angle/0.043169;
		return angle/0.0636942;
    } else {
        return angle/0.03195;
    }
}


void* udp_api_timer_loop(void *arg) {
    unsigned long t;
    
    printf("udp_api_timer_loop() starting... ");

    timer_start(&t_udp_api);

    while (udp_api_running) {
        t = timer_stop(&t_udp_api);
        
        if (1==2 && t > T_UDP_API) {
          timer_start(&t_udp_api);

          // Do something once per T_UDP_API ms
          udpdrive_act.active   = 0;
          udpdrive_act.throttle = 0.0;
          udpdrive_act.steering = 0.0;
          udpdrive_act.braking   = 0.0;
          udpdrive_act.speed_active   = 0;
          udpdrive_act.speed   = 0.0;

          //printf("UDP timer %lu\n", t);

        }
        usleep(10000);
    }

   printf("udp_api_timer_loop() stopped. ");
   return 0;
}


int udp_api_start() {
    int rc;

    rc = pthread_create(&tid_udp_api_timer_loop, NULL, &udp_api_timer_loop, NULL);
    if (rc != 0) 
    {
        printf("\nCan't create udp_api_timer_loop() thread :[%s]", strerror(rc));
    }

    rc = pthread_create(&tid_udp_api_listener_loop, NULL, &udp_api_listener_loop, NULL);
    if (rc != 0) 
    {
        printf("\nCan't create udp_api_listener_loop() thread :[%s]", strerror(rc));
    }

    return 0;
}

