/** 
 ** steering_pid.h - Steering angle PID controller
 **/

/*
 * @brief Proportional gain of the PID controller.
 *
 */
//#define STEERING_PID_PROPORTIONAL_GAIN (0.003) //( 0.03 )
//#define STEERING_PID_PROPORTIONAL_GAIN (0.008/2) 
  #define STEERING_PID_PROPORTIONAL_GAIN (0.001)
/*
 * @brief Integral gain of the PID controller.
 *
 */
//#define STEERING_PID_INTEGRAL_GAIN (0.0) // ( 0.01 )
//#define STEERING_PID_INTEGRAL_GAIN (0.02/3) //(0.04/3) 
  #define STEERING_PID_INTEGRAL_GAIN (0.01)

/*
 * @brief Derivative gain of the PID controller.
 *
 */
//#define STEERING_PID_DERIVATIVE_GAIN (0.0) //( 0.0 )
  #define STEERING_PID_DERIVATIVE_GAIN (0.02)

/*
 * @brief Windup guard of the PID controller.
 *
 */
#define STEERING_PID_WINDUP_GUARD ( 10 )

/*
 * @brief Minimum output value of PID to be within a valid range.
 *
 */
//#define STEERING_PID_OUTPUT_MIN ( -1.0 )
#define STEERING_PID_OUTPUT_MIN ( -0.5 )

/*
 * @brief Maximum output value of PID to be within a valid range.
 *
 */
//#define STEERING_PID_OUTPUT_MAX ( 1.0 )
#define STEERING_PID_OUTPUT_MAX ( 0.5 )


/*
 * @brief Sensitiveness of steer.
 *
 */
#define STEERING_SENSITIVENESS ( 500.0 )   // max abs degrees for steering column



/*
 * @brief PID controller delay.
 *
 */
#define T_STEERING          10000

/*
 * @brief Maximum steering torque value at which the steer does not move (when driving).
 *
 */
//#define STEERING_PID_OUTPUT_V0 ( 0.05 )
#define STEERING_PID_OUTPUT_V0 ( 0.05 )


#define STEERING_PID_IDLE_MAX (STEERING_PID_OUTPUT_V0 / 50.0 ) 


void set_steering_angle(double target_angle);

int steering_angle_loop_start(void);

int steering_angle_loop_stop(void);

void steering_angle_loop_wait(void);

