#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_joystick.h>
#include <SDL2/SDL_gamecontroller.h>
#include <sys/time.h>
#include <time.h>
#include <linux/can.h>

#include "oscc.h"
#include "vehicles.h"

#include "can_protocols/brake_can_protocol.h"
#include "can_protocols/steering_can_protocol.h"
#include "can_protocols/throttle_can_protocol.h"
#include "can_protocols/fault_can_protocol.h"

#include "commander.h"
#include "joystick.h"
#include "steering_pid.h"

#include "bumble_grabber.hpp"

#include "speed_pid.h"

#include "timer.h"
#include "gps.h"
#include "udp_api.h"


#define CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define JOYSTICK_AXIS_THROTTLE (SDL_CONTROLLER_AXIS_TRIGGERRIGHT)
#define JOYSTICK_AXIS_BRAKE (SDL_CONTROLLER_AXIS_TRIGGERLEFT)
#define JOYSTICK_AXIS_STEER (SDL_CONTROLLER_AXIS_LEFTX)
#define JOYSTICK_BUTTON_ENABLE_CONTROLS (SDL_CONTROLLER_BUTTON_START)
#define JOYSTICK_BUTTON_DISABLE_CONTROLS (SDL_CONTROLLER_BUTTON_BACK)

#define JOYSTICK_BUTTON_ENABLE_UDPDRIVE (SDL_CONTROLLER_BUTTON_X)

#define BRAKES_ENABLED_MIN (0.05)
#define JOYSTICK_DELAY_INTERVAL (50000)
#define COMMANDER_ENABLED ( 1 )
#define COMMANDER_DISABLED ( 0 )

#define DELAY_TIME 80000

static int commander_enabled = COMMANDER_DISABLED;

static bool control_enabled = false;

int system_mode = 0;    /* Control method: 0 - manual, 1 - by joystick, 2 - by udp */

double curr_angle;          // Steering column angle (in degrees)
double curr_angle_wheels;   // Front wheels angle (in degrees)
double curr_speed;          // Speed (in km/h)
double acceleration;        // Acceleration (in km/h^2)

uint8_t obd_4b0[8];

extern struct timeval frame_timer;
static unsigned long time_esplased;

act_s act;        /* Actuator signal state */
act_s act_ctrl;   /* Actuator control signal state */

extern float steering_angle_target;

struct timeval t_start;

extern act_s udpdrive_act;  /* Actuator signal state from udp */

FILE *m_gps_log;

extern char gpstime[50];
extern unsigned long gps_ts;
extern double gps_lat;
extern double gps_lon;
extern double gps_alt;
extern double gps_vf;

char *filename;
static unsigned long frame_num = 0;

static int get_normalized_position( unsigned long axis_index, double * const normalized_position );
static int check_trigger_positions( );
static int commander_disable_controls( );
static int commander_enable_controls( );
static int get_button( unsigned long button, unsigned int* const state );
static int command_brakes( );
static int command_throttle( );
static int command_steering( );
static void brake_callback(oscc_brake_report_s *report);
static void throttle_callback(oscc_throttle_report_s *report);
static void steering_callback(oscc_steering_report_s *report);
static void fault_callback(oscc_fault_report_s *report);
static void obd_callback(struct can_frame *frame);


#define CANMAX  0xFFFF


typedef struct
{
    uint8_t on;
    uint8_t d[8];
} can_line_s;

can_line_s c[CANMAX];

void cls() {
  const char* CLEAR_SCREE_ANSI = "\e[1;1H\e[2J";
  write(STDOUT_FILENO,CLEAR_SCREE_ANSI,12);
}


double normalized_to_steer_angle(double normalized_position) {
    if (normalized_position > 1.0) {
        return -STEERING_SENSITIVENESS;
    } else if (normalized_position < -1.0) {
        return STEERING_SENSITIVENESS;
    } else {
        return -STEERING_SENSITIVENESS * normalized_position;
    }
}

double steer_angle_to_normalized(double angle) {
    if (angle > STEERING_SENSITIVENESS) {
        return 1.0;
    } else if (angle < -STEERING_SENSITIVENESS) {
        return -1.0;
    } else {
        return angle / STEERING_SENSITIVENESS;
    }
}


void print_canlines() {
    int i;
    cls();
    printf("\n");
    for (i=0;i<CANMAX;i++)
    {
        if (c[i].on==1)
        {
            printf("%04x    ", i );
            printf("%02x ", c[i].d[0] );
            printf("%02x ", c[i].d[1] );
            printf("%02x ", c[i].d[2] );
            printf("%02x ", c[i].d[3] );
            printf("%02x ", c[i].d[4] );
            printf("%02x ", c[i].d[5] );
            printf("%02x ", c[i].d[6] );
            printf("%02x ", c[i].d[7] );
            printf("\n");
        }
    }
}


int commander_init( int channel )
{
    int return_code = OSCC_ERROR;

    int i;
    for (i=0;i<CANMAX;i++) {
        c[i].on = 0;
        c[i].d[0] = 0;
        c[i].d[1] = 0;
        c[i].d[2] = 0;
        c[i].d[3] = 0;
        c[i].d[4] = 0;
        c[i].d[5] = 0;
        c[i].d[6] = 0;
        c[i].d[7] = 0;
    }


    m_gps_log = fopen("m_gps_log.csv", "w");

    act_ctrl.steering_source = 0;
    act_ctrl.throttle_source = 0;
    act_ctrl.braking_source = 0;

    udpdrive_act.active = 0;
    curr_angle = 0.0;
    curr_angle_wheels = 0.0;
    curr_speed = 0.0;

    #ifdef USE_BB_CAMERA
        create_camera();
    #endif


    if ( commander_enabled == COMMANDER_DISABLED )
    {
        commander_enabled = COMMANDER_ENABLED;

        #if defined(USE_OSCC)
            return_code = oscc_open( channel );
        #else
            return_code = OSCC_OK;
        #endif

        if ( return_code != OSCC_ERROR )
        {

            // register callback handlers
            #if defined(USE_OSCC)
                oscc_subscribe_to_obd_messages(obd_callback);
                oscc_subscribe_to_brake_reports(brake_callback);
                oscc_subscribe_to_steering_reports(steering_callback);
                oscc_subscribe_to_throttle_reports(throttle_callback);
                oscc_subscribe_to_fault_reports(fault_callback);
            #endif

            #if defined(USE_OSCC) || defined(USE_OSCC_JOYSTICK_ONLY)

                return_code = joystick_init( );

                printf( "Waiting for joystick controls to zero\n" );

                while ( return_code != OSCC_ERROR )
                {
                    return_code = check_trigger_positions( );

                    if ( return_code == OSCC_WARNING )
                    {
                        (void) usleep( JOYSTICK_DELAY_INTERVAL );
                    }
                    else if ( return_code == OSCC_ERROR )
                    {
                        printf( "Failed to wait for joystick to zero the control values\n" );
                    }
                    else
                    {
                        printf( "Joystick controls successfully initialized\n" );
                        break;
                    }
                }

            #else
                return_code = OSCC_OK;

            #endif

        }
    }

    return ( return_code );
}

void commander_close( int channel )
{
    if ( commander_enabled == COMMANDER_ENABLED )
    {
        commander_disable_controls( );

        oscc_disable( );

        oscc_close( channel );

        joystick_close( );

        commander_enabled = COMMANDER_DISABLED;
    }
}

int check_for_controller_update( )
{

    int return_code = OSCC_OK;

    #if defined(USE_OSCC) || defined(USE_OSCC_JOYSTICK_ONLY)

        static unsigned int disable_button_previous = 0;
        unsigned int disable_button_current = 0;

        return_code = joystick_update( );

        if ( return_code == OSCC_OK )
        {
            return_code = get_button( JOYSTICK_BUTTON_DISABLE_CONTROLS,
                                      &disable_button_current );
        }

        if ( return_code == OSCC_OK )
        {
            if ( (disable_button_previous != 1)
                && (disable_button_current != 0 ) )
            {
                return_code = commander_disable_controls( );
                system_mode = 0;
                curr_angle = 0.0;
                udpdrive_act.active = 0;
                act.braking = 0.0;
                act.throttle = 0.0;
                act.steering = 0.0;
                act_ctrl.speed_active = 0;
                act_ctrl.active = 0;
                act_ctrl.braking = 0.0;
                act_ctrl.throttle = 0.0;
                act_ctrl.steering = 0.0;
                act_ctrl.speed = 0.0;
                act_ctrl.steering_source = 0;
                act_ctrl.throttle_source = 0;
                act_ctrl.braking_source = 0;
//Uzlabojumi
            udpdrive_act.steering = 0.0;
            udpdrive_act.throttle = 0.0;
            udpdrive_act.braking  = 0.0;

            udpdrive_act.steering_source = 0;
            udpdrive_act.throttle_source = 0;
            udpdrive_act.braking_source  = 0;
//Uzlabojumi            
            }

            disable_button_previous = disable_button_current;
        }

        static unsigned int enable_button_previous = 0;
        unsigned int enable_button_current = 0;

        if ( return_code == OSCC_OK )
        {
                return_code = get_button( JOYSTICK_BUTTON_ENABLE_CONTROLS,
                                          &enable_button_current );

                if ( return_code == OSCC_OK )
                {
                    if ( (enable_button_previous != 1)
                        && (enable_button_current != 0 ) )
                    {
                        return_code = commander_enable_controls( );
                        system_mode = 1;
                        curr_angle = 0.0;
                        udpdrive_act.active = 0;
                        act.braking = 0.0;
                        act.throttle = 0.0;
                        act.steering = 0.0;
                        act_ctrl.speed_active = 0;
                        act_ctrl.active = 0;
                        act_ctrl.braking = 0.0;
                        act_ctrl.throttle = 0.0;
                        act_ctrl.steering = 0.0;
                        act_ctrl.speed = 0.0;
                        act_ctrl.steering_source = 0;
                        act_ctrl.throttle_source = 0;
                        act_ctrl.braking_source = 0;
 //Uzlabojumi
            udpdrive_act.steering = 0.0;
            udpdrive_act.throttle = 0.0;
            udpdrive_act.braking  = 0.0;

            udpdrive_act.steering_source = 0;
            udpdrive_act.throttle_source = 0;
            udpdrive_act.braking_source  = 0;
//Uzlabojumi   
                        //set_speed(40.0);
                    }

                    enable_button_previous = enable_button_current;
                }
        }

    #endif


    if ( control_enabled )
    {

        return_code = command_brakes( );

    if ( return_code == OSCC_OK )
        {
            return_code = command_throttle( );
        }

    if ( return_code == OSCC_OK )
        {
            return_code = command_steering( );
        }

    if ( return_code == OSCC_OK )
        {
           
		   fprintf(m_gps_log, "%d\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n", system_mode,udpdrive_act.steering,curr_angle,steering_angle_target,curr_angle_wheels,act_ctrl.throttle_source,act_ctrl.braking_source,act_ctrl.steering_source,act_ctrl.throttle,act_ctrl.braking,act_ctrl.steering, act.throttle,act.braking,act.steering);

//            printf("%d\t%s\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%s\t%f\t%f\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n", system_mode,gpstime,gps_lon,gps_lat,gps_alt,curr_speed,curr_angle,steering_angle_target,curr_angle_wheels,acceleration,filename, lon_to_m(gps_lon), lat_to_m(gps_lat), act_ctrl.source, act_ctrl.throttle, act_ctrl.braking, act_ctrl.steering, act.throttle, act.braking, act.steering);

            printf("%d\t%f\t%f\t%f\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n", system_mode,curr_angle,steering_angle_target,curr_angle_wheels,act_ctrl.throttle_source,act_ctrl.braking_source,act_ctrl.steering_source,act_ctrl.throttle,act_ctrl.braking,act_ctrl.steering, act.throttle,act.braking,act.steering);

            usleep(DELAY_TIME);
        }


    } else {


        time_esplased = timer_stop(&frame_timer);

        if(time_esplased >= DELAY_TIME){

            #ifdef USE_BB_CAMERA
                filename =  save_frame_png(frame_num);
            #else
                filename = "no_image";
            #endif

			fprintf(m_gps_log, "%d\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n", system_mode,udpdrive_act.steering,curr_angle,steering_angle_target,curr_angle_wheels,act_ctrl.throttle_source,act_ctrl.braking_source,act_ctrl.steering_source,act_ctrl.throttle,act_ctrl.braking,act_ctrl.steering, act.throttle,act.braking,act.steering);

//            printf("%d\t%s\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%s\t%f\t%f\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n", system_mode,gpstime,gps_lon,gps_lat,gps_alt,curr_speed,curr_angle,steering_angle_target,curr_angle_wheels,acceleration,filename, lon_to_m(gps_lon), lat_to_m(gps_lat), act_ctrl.source, act_ctrl.throttle, act_ctrl.braking, act_ctrl.steering, act.throttle, act.braking, act.steering);

            printf("%d\t%f\t%f\t%f\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n", system_mode,curr_angle,steering_angle_target,curr_angle_wheels,act_ctrl.throttle_source,act_ctrl.braking_source,act_ctrl.steering_source,act_ctrl.throttle,act_ctrl.braking,act_ctrl.steering, act.throttle,act.braking,act.steering);

            frame_num++;
            timer_start(&frame_timer);

        }
    }

    return return_code;

}

//
static int get_normalized_position( unsigned long axis_index, double * const normalized_position )
{
    int return_code = OSCC_ERROR;

    int axis_position = 0;

    return_code = joystick_get_axis( axis_index, &axis_position );

    if ( return_code == OSCC_OK )
    {
        if ( axis_index == JOYSTICK_AXIS_STEER )
        {
            ( *normalized_position ) = CONSTRAIN(
            ((double) axis_position) / INT16_MAX,
            -1.0,
            1.0);
        }
        else
        {
            ( *normalized_position ) = CONSTRAIN(
            ((double) axis_position) / INT16_MAX,
            0.0,
            1.0);
        }
    }

    return ( return_code );

}

static int check_trigger_positions( )
{
    int return_code = OSCC_ERROR;

    return_code = joystick_update( );


    double normalized_brake_position = 0;

    if ( return_code == OSCC_OK )
    {
        return_code = get_normalized_position( JOYSTICK_AXIS_BRAKE, &normalized_brake_position );
    }


    double normalized_throttle_position = 0;

    if ( return_code == OSCC_OK )
    {
        return_code = get_normalized_position( JOYSTICK_AXIS_THROTTLE, &normalized_throttle_position );
    }


    if ( return_code == OSCC_OK )
    {
        if ( ( normalized_throttle_position > 0.0 )
             || ( normalized_brake_position > 0.0 ) )
        {
            return_code = OSCC_WARNING;
        }
    }

    return return_code;
}

static int commander_disable_controls( )
{
    int return_code = OSCC_ERROR;

    if ( (commander_enabled == COMMANDER_ENABLED)
        && (control_enabled == true) )
    {
        printf( "Disable controls\n" );

        system_mode = 0;
        return_code = oscc_disable();

        if ( return_code == OSCC_OK )
        {
            control_enabled = false;
        }
    }
    else
    {
        return_code = OSCC_OK;
    }

    return return_code;
}

static int commander_enable_controls( )
{
    int return_code = OSCC_ERROR;

    if ( (commander_enabled == COMMANDER_ENABLED)
        && (control_enabled == false) )
    {
        printf( "Enable controls\n" );

        return_code = oscc_enable();

        if ( return_code == OSCC_OK )
        {
            control_enabled = true;
        }
    }
    else
    {
        return_code = OSCC_OK;
    }

    return ( return_code );
}

static int get_button( unsigned long button, unsigned int* const state )
{
    int return_code = OSCC_ERROR;

    if ( state != NULL )
    {
        unsigned int button_state;

        return_code = joystick_get_button( button, &button_state );

        if ( ( return_code == OSCC_OK ) &&
             ( button_state == JOYSTICK_BUTTON_STATE_PRESSED ) )
        {
            ( *state ) = 1;
        }
        else
        {
            ( *state ) = 0;
        }
    }

    return ( return_code );
}

// Since the OSCC API requires a normalized value, we will read in and
// normalize a value from the game pad, using that as our requested brake position.
static int command_brakes( )
{
    int return_code = OSCC_ERROR;

 //   double normalized_throttle_position = 0;
    double normalized_brake_position = 0;

    if ( commander_enabled == COMMANDER_ENABLED && control_enabled == true )
    {

        return_code = get_normalized_position( JOYSTICK_AXIS_BRAKE, &normalized_brake_position );

    //Papildinajums
         if (normalized_brake_position > 0.0 && udpdrive_act.braking_source==2 ) 
         {
          udpdrive_act.throttle_source = 0;
          udpdrive_act.throttle = 0.0;
          udpdrive_act.braking_source = 0;
          udpdrive_act.braking = 0.0;
          if (system_mode==2) {
                system_mode = 1;
            }
         } 
    //Papildinajums   

           // Owerwrite if received by UDP API
        if (normalized_brake_position==0.0 && udpdrive_act.braking_source==2) 
            {
                normalized_brake_position = udpdrive_act.braking;
            }


        int fix = 0;
        if (fix && normalized_brake_position>0) 
        {
            normalized_brake_position = 0.5;
        }

         act_ctrl.braking  = normalized_brake_position;

  //   printf("Act_ctrol_braking %f \n", act_ctrl.braking);    

        if ( return_code == OSCC_OK && act_ctrl.braking >= 0.0 )
        {
            act.braking = calc_exponential_average(
                act.braking,
                act_ctrl.braking,
                BRAKE_FILTER_FACTOR );

            #if defined(USE_OSCC)
                return_code = oscc_publish_brake_position(act.braking);
            #endif
        }

    }

    return ( return_code );
}

// For the throttle command, we want to send a normalized position based on the
// throttle position trigger. We also don't want to send throttle commands if
// we are currently braking.
static int command_throttle( )
{
    int return_code = OSCC_ERROR;

    double normalized_throttle_position = 0;
    double normalized_brake_position = 0;

    if ( commander_enabled == COMMANDER_ENABLED && control_enabled == true )
    {

        return_code = get_normalized_position( JOYSTICK_AXIS_THROTTLE, &normalized_throttle_position );
    

    //Papildinajums
         if (normalized_throttle_position > 0.0 && udpdrive_act.throttle_source==2) 
         {
          udpdrive_act.throttle_source = 0;
          udpdrive_act.throttle = 0.0;
          udpdrive_act.braking_source = 0;
          udpdrive_act.braking = 0.0;
          if (system_mode==2) {
                system_mode = 1;
            }
         } 
    //Papildinajums   

        // Owerwrite if received by UDP API
        if (normalized_throttle_position==0.0 && udpdrive_act.throttle_source==2) 
			{
				normalized_throttle_position = udpdrive_act.throttle;
			}

        int fix = 0;
        if (fix && normalized_throttle_position>0) 
        {
            normalized_throttle_position = 0.1;
        }

        if ( return_code == OSCC_OK && normalized_throttle_position >= 0.0 )
        {

            return_code = get_normalized_position( JOYSTICK_AXIS_BRAKE, &normalized_brake_position );

            if ( normalized_brake_position >= BRAKES_ENABLED_MIN )
            {
                normalized_throttle_position = 0.0;
            }
        }

		
        act_ctrl.throttle = normalized_throttle_position;
        act_ctrl.braking  = normalized_brake_position;

        if ( return_code == OSCC_OK && normalized_throttle_position >= 0.0 )
        {
            act.throttle = calc_exponential_average(
                act.throttle,
                act_ctrl.throttle,
                THROTTLE_FILTER_FACTOR );

            #if defined(USE_OSCC)
                return_code = oscc_publish_throttle_position(act.throttle);
            #endif
        }

    }

    return ( return_code );
}

// To send the steering command, we first get the normalized axis position from
// the game controller. Since the car will fault if it detects too much discontinuity
// between spoofed output signals, we use an exponential average filter to smooth
// our output.
static int command_steering( )
{
    int return_code = OSCC_ERROR;

    if ( commander_enabled == COMMANDER_ENABLED && control_enabled == true )
    {
        double normalized_position = 0;

        return_code = get_normalized_position( JOYSTICK_AXIS_STEER, &normalized_position );

        // Owerwrite if received by UDP API
        if (udpdrive_act.steering_source==2) {
            normalized_position = udpdrive_act.steering;
        }

        int fix = 0;
        float angle = 10.0;
        float val = steer_angle_to_normalized(steering_wheel_to_column_angle(angle,curr_speed));
        if (fix && normalized_position>0) {
            normalized_position = val;
        } else if (fix && normalized_position<0) {
            normalized_position = -val;
        } else if (fix) {
            normalized_position = 0.0;
        }

        act_ctrl.steering = normalized_position;

        if( return_code == OSCC_OK )
        {
            // Send target value to steering PID controller
            set_steering_angle(normalized_to_steer_angle(act_ctrl.steering));

            // send to actuator value computed by steering PID controller
            #if defined(USE_OSCC)
                return_code = oscc_publish_steering_torque(act.steering);
            #endif

        }

    }

    return ( return_code );
}

/*
 * These callback functions just check the reports for operator overrides. The
 * firmware modules should have disabled themselves, but we will send the
 * command again just to be safe.
 *
 */
static void throttle_callback(oscc_throttle_report_s *report)
{
    if ( report->operator_override )
    {
        commander_disable_controls();

        printf("Override: Throttle\n");
    }
    else
    {
        //printf("Throttle report received: %d\n",report->enabled);
    }
}

static void steering_callback(oscc_steering_report_s *report)
{
    if ( report->operator_override )
    {
        commander_disable_controls();

        printf("Override: Steering\n");
    }
    else
    {
        //printf("Steering report received: %02x  %02x  %02x\n",report->reserved[0],report->reserved[1],report->reserved[2]);
    }
}

static void brake_callback(oscc_brake_report_s * report)
{
    if ( report->operator_override )
    {
        commander_disable_controls();

        printf("Override: Brake\n");
    }
    else
    {
        //printf("Brake report received: %d\n",report->enabled);
    }
}

static void fault_callback(oscc_fault_report_s *report)
{
    printf("Fault: ");

    if ( report->fault_origin_id == FAULT_ORIGIN_BRAKE )
    {
        commander_disable_controls();
        printf("Brake\n");
    }
    else if ( report->fault_origin_id == FAULT_ORIGIN_STEERING )
    {
        //commander_disable_controls();
        printf("Steering\n");
    }
    else if ( report->fault_origin_id == FAULT_ORIGIN_THROTTLE )
    {
        commander_disable_controls();
        printf("Throttle\n");
    }
}


// To cast specific OBD messages, you need to know the structure of the
// data fields and the CAN_ID.
static void obd_callback(struct can_frame *frame)
{
    uint8_t *b = frame->data;
    c[frame->can_id].on = 1;
    c[frame->can_id].d[0] = b[0];
    c[frame->can_id].d[1] = b[1];
    c[frame->can_id].d[2] = b[2];
    c[frame->can_id].d[3] = b[3];
    c[frame->can_id].d[4] = b[4];
    c[frame->can_id].d[5] = b[5];
    c[frame->can_id].d[6] = b[6];
    c[frame->can_id].d[7] = b[7];

    if ( frame->can_id == KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID )
    {
        kia_soul_obd_steering_wheel_angle_data_s * steering_data = (kia_soul_obd_steering_wheel_angle_data_s*) frame->data;

        curr_angle = steering_data->steering_wheel_angle * KIA_SOUL_OBD_STEERING_ANGLE_SCALAR;

        curr_angle_wheels = steering_column_to_wheel_angle(curr_angle,curr_speed);

    }
    else if ( frame->can_id == KIA_SOUL_OBD_WHEEL_SPEED_CAN_ID )
    {
        #ifndef SPEED_FROM_GPS
            kia_soul_obd_wheel_speed_data_s * speed_data = (kia_soul_obd_wheel_speed_data_s*) frame->data;
            curr_speed = (speed_data->wheel_speed_rear_left + speed_data->wheel_speed_rear_right)/2.0 * KIA_SOUL_OBD_WHEEL_SPEED_SCALAR;
        #endif

        obd_4b0[0] = b[0];
        obd_4b0[1] = b[1];
        obd_4b0[2] = b[2];
        obd_4b0[3] = b[3];
        obd_4b0[4] = b[4];
        obd_4b0[5] = b[5];
        obd_4b0[6] = b[6];
        obd_4b0[7] = b[7];
    }
    else if ( frame->can_id == KIA_SOUL_OBD_BRAKE_PRESSURE_CAN_ID )
    {

    }

}

double calc_exponential_average( double average,
                                 double setpoint,
                                 double factor )
{
    double exponential_average =
        ( setpoint * factor ) + ( ( 1.0 - factor ) * average );

    return ( exponential_average );
}
