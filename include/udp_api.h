/** 
 ** udp_api.h - UDP API for commander
 **/

#ifndef UDP_API_H
#define UDP_API_H

#define BUFMAX 1024

/*
 * @brief Loop delay.
 *
 */
#define T_UDP_API      1500000

/*
 * @brief Commands
 *
 */
#define STEERING      1
#define THROTTLE      2
#define BRAKING       3
#define SPEED         4
#define OFF           0


#define STEERING_VAL_MAX     31.85		// max abs front wheel angle in degrees
#define BRAKING_VAL_MAX      5.0		// max abs braking acceleration in m/s^2 
#define SPEED_VAL_MAX        40.0		// max speed in km/h

#define UDP_API_PORT      8050


double steering_column_to_wheel_angle(double angle, double v);
double steering_wheel_to_column_angle(double angle, double v);

int udp_api_start(void);
int udp_server(void);

#endif /* UDP_API_H */
