/**
 * @file oscc_pid.c
 *
 */


#include "oscc_pid.h"


void pid_zeroize( pid_s* pid, float integral_windup_guard )
{
    // set prev and integrated error to zero
    pid->prev_input = 0;
    pid->int_error = 0;
    pid->prev_steering_angle = 0;
    pid->windup_guard = integral_windup_guard;
}


float pid_update( pid_s* pid, float setpoint, float input, float dt )
{
    float diff;
    float p_term;
    float i_term;
    float d_term;

    float curr_error = setpoint - input;

    static int count = 0;

    // integration with windup guarding
    pid->int_error += (curr_error * dt);

    count++;

    if (pid->int_error < -(pid->windup_guard))
    {
        pid->int_error = -(pid->windup_guard);
    }
    else if (pid->int_error > pid->windup_guard)
    {
        pid->int_error = pid->windup_guard;
    }

    // differentiation
    diff = ((input - pid->prev_input) / dt);

    // scaling
    p_term = (pid->proportional_gain * curr_error);
    i_term = (pid->integral_gain     * pid->int_error);
    d_term = (pid->derivative_gain   * diff);

    // summation of terms
    pid->control = p_term + i_term - d_term;

    // save current error as previous error for next iteration
    pid->prev_input = input;

    return pid->control;
}

