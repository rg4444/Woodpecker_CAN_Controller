/**
 * @file gps.h
 * @brief Oxts RTK GPS Interface.
 *
 */


#ifndef GPS_H
#define GPS_H

double lat_to_m(double x);
double lon_to_m(double x);


int gps_loop_start(void);


#endif /* GPS_H */
