/** 
 ** speed_pid.h - Speed PID controller
 **/

/*
 * @brief Proportional gain of the PID controller.
 *
 */
#define SPEED_PID_PROPORTIONAL_GAIN (0.01) //( 0.03 )

/*
 * @brief Integral gain of the PID controller.
 *
 */
#define SPEED_PID_INTEGRAL_GAIN (0.01) // ( 0.01 )

/*
 * @brief Derivative gain of the PID controller.
 *
 */
#define SPEED_PID_DERIVATIVE_GAIN (0.0) //( 0.0 )

/*
 * @brief Windup guard of the PID controller.
 *
 */
#define SPEED_PID_WINDUP_GUARD ( 10 )

/*
 * @brief Minimum output value of PID to be within a valid range.
 *
 */
#define SPEED_PID_OUTPUT_MIN ( 0.0 )

/*
 * @brief Maximum output value of PID to be within a valid range.
 *
 */
#define SPEED_PID_OUTPUT_MAX ( 1.0 )


/*
 * @brief Sensitiveness of steer.
 *
 */
#define SPEED_SENSITIVENESS ( 50.0 )   // max speed

/*
 * @brief Precision of speed
 *
 */
#define SPEED_PRECISION ( 1.0 )   // max speed


/*
 * @brief PID controller delay.
 *
 */
#define T_SPEED          10000


void set_speed(double target_speed);

int speed_loop_start(void);

int speed_loop_stop(void);

void speed_loop_wait(void);

