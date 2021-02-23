#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include <fcntl.h>


#include "oscc.h"
#include "commander.h"
#include "can_protocols/steering_can_protocol.h"
#include "steering_pid.h"
#include "speed_pid.h"
#include "gps.h"
#include "udp_api.h"

#include "bumble_grabber.hpp"

#include "timer.h"


#define COMMANDER_UPDATE_INTERVAL_MICRO (50000)
#define SLEEP_TICK_INTERVAL_MICRO (1000)

int m_test_loop_start(void);


struct timeval frame_timer; //extern in commander.c


static int error_thrown = OSCC_OK;

static unsigned long long get_timestamp_micro( )
{
    struct timeval time;

    gettimeofday( &time, NULL );

    return ( time.tv_usec );
}

static unsigned long long get_elapsed_time( unsigned long long timestamp )
{
    unsigned long long now = get_timestamp_micro( );
    unsigned long long elapsed_time = now - timestamp;

    return elapsed_time;
}

void signal_handler( int signal_number )
{
    if ( signal_number == SIGINT )
    {
        error_thrown = OSCC_ERROR;
        destroy_camera();
    }
}




int main( int argc, char **argv )
{
    oscc_result_t ret = OSCC_OK;
    unsigned long long update_timestamp = get_timestamp_micro();
    unsigned long long elapsed_time = 0;


    int channel;

    errno = 0;

    if ( argc != 2 || ( channel = atoi( argv[1] ), errno ) != 0 )
    {
        printf( "usage %s channel\n", argv[0] );
        exit( 1 );
    }

    struct sigaction sig;
    sig.sa_handler = signal_handler;
    sigaction( SIGINT, &sig, NULL );

    ret = commander_init( channel );

    timer_start(&frame_timer);

    if ( ret == OSCC_OK )
    {

        // Steering angle control loop init and start
        steering_angle_loop_start();

        // Speed control loop init and start
        //speed_loop_start();

        // GPS loop start
        gps_loop_start();
        
        // UDP API for commander
        udp_api_start();

        // Wait a bit
        usleep( 500000 );

        // Menu
        printf( "\n\n\nH E L P :\n" );
        printf( "=========\n" );
        printf( "    START - Enable Controls, set Joytick mode (1)\n" );
        printf( "        Controls:\n" );
        printf( "           - LEFT TRIGGER-Brake\n" );
        printf( "           - RIGHT TRIGGER-Throttle\n" );
        printf( "           - LEFT STICK-Steering\n" );
    //    printf( "    X - UDP drive mode (2)\n" );
        printf( "    BACK - Disable Controls, set manual mode (0)\n" );
    //   printf( "* Caution: to use X command, Controls must be enabled (START pressed before)\n");




        // Main loop
        while ( ret == OSCC_OK && error_thrown == OSCC_OK )
        {
            elapsed_time = get_elapsed_time( update_timestamp );

            if ( elapsed_time > COMMANDER_UPDATE_INTERVAL_MICRO )
            {
                update_timestamp = get_timestamp_micro();
                ret = check_for_controller_update( );
            }

            // Delay 1 ms to avoid loading the CPU
            (void) usleep( SLEEP_TICK_INTERVAL_MICRO );
        }
        commander_close( channel );
   
    }


    return 0;
}


