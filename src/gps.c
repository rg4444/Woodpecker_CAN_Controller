#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <netinet/in.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/udp.h>   //Provides declarations for udp header
#include <netinet/ip.h>    //Provides declarations for ip header
#include <netinet/if_ether.h>  //For ETH_P_ALL
#include <net/ethernet.h>  //For ether_header
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <assert.h>
#include <math.h>

#include "commander.h"
#include "timer.h"
#include "gps.h"


#define GPS_ETH_DEVICE      "eth0"

#define CAR_WHEELBASE       2.570 // m (KIA Soul PS)


pthread_t tid_gps_loop;

double gps_lat = 0.0;
double gps_lon = 0.0;
double gps_alt = 0.0;
double gps_vf = 0.0;
unsigned long gps_ts = 0;
char gpstime[50];
unsigned long gps_ts_prev;
double curr_speed_prev;

extern struct timeval t_start;

extern double curr_speed;
extern double acceleration;


double m_to_lat(double x) {
    return x * 0.000008990459;
}
double m_to_lon(double x) {
    return x * 0.000013947824;
}
double lat_to_m(double x) {
    return x * 111229.0262377;
}
double lon_to_m(double x) {
    return x * 71695.771326;
}
double r2d(double x) {
    return x * 57.2958;
}
double d2r(double x) {
    return x / 57.2958;
}
double distance(double x1, double y1, double x2, double y2) {
    double dx = x1-x2;
    double dy = y1-y2;
    return sqrt(dx*dx + dy*dy);
}
double tri_area(double a, double b, double c, double d, double e, double f) {
    return 0.5 * ((a-e)*(d-f) - (c-e)*(b-f));
}
double radius(double x1, double y1, double x2, double y2, double x3, double y3) {
    double area = tri_area(x1,y1,x2,y2,x3,y3);
    double radius;
    if (fabs(area)<0.00000000001 && area<0)
        radius = -999999999999.0;
    else if (fabs(area)<0.00000000001 && area>=0)
        radius = 999999999999.0;
    else
        radius = distance(x1,y1,x2,y2) * distance(x1,y1,x3,y3) * distance(x2,y2,x3,y3) / 4.0 / area;
    return radius;
}
double wheel_angle(double radius, double wheelbase) {
    double angle;
    if (fabs(radius)==999999999999.0)
        angle = 0.0;
    else {
        angle = r2d(atan(radius/wheelbase));
        if (angle<0)
            angle = -angle-90;
        else if (angle>0)
            angle = 90-angle;
        else
            angle = 0.0;
    }
    return angle;
}


#ifdef USE_GPS


    #include "erl_interface.h"
    #include "ei.h"

    #define GPS_SERVER          "10.13.137.176"
    #define GPS_PORT            7201

    int send_udp_binary(char *pmessage, int length, char *addr, int port)
    {
        struct sockaddr_in si_other;
        int s, slen=sizeof(si_other);

        if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
        {
            perror("socket()");
            exit(1);
        }

        memset((char *) &si_other, 0, sizeof(si_other));
        si_other.sin_family = AF_INET;
        si_other.sin_port = htons(port);

        if (inet_aton(addr , &si_other.sin_addr) == 0)
        {
            fprintf(stderr, "inet_aton() failed\n");
            exit(1);
        }

            //send the message
            if (sendto(s, pmessage, length, 0, (struct sockaddr *) &si_other, slen)==-1)
            {
                perror("sendto()");
                exit(1);
            } 

        close(s);
        return 0;
    }

    char udp_buf1[1000];
    int send_gps_message(
        double gpstime, 
        char *gpstime_text,
        double lat, 
        double lon, 
        double alt, 
        double north_acc, 
        double east_acc, 
        double alt_acc, 
        double vf, 
        double vl, 
        double ax, 
        double ay, 
        double az,
        double heading0, 
        double pitch, 
        double roll,
        double heading_acc, 
        double pitch_acc, 
        double roll_acc   
        ) 
    {
        static int p = 0;

        // compute heading 
        static double x0=0.0,y0=0.0,x1=0.0,y1=0.0,x,y,heading=0.0,prev_heading=0.0,dx,dy;

        x = lon * 71695.771326;
        y = lat * 111229.0262377;
        dx = x-x0;
        dy = y-y0;

        if (vf*3.6>=5.0) {  // >= 5 km/h
            heading = 90 - atan2(y-y0,x-x0) * 57.2958;
        } else {
            heading = prev_heading;
        }

        if (p % 2 == 0) {
            printf("%f  %f     %f  %f     %f\n",x,y,x0,y0,heading);
        }
        p++;

        if (sqrt(dy*dy + dx*dx) > 1.0) {  // 1m
            x0 = x1;
            y0 = y1;
            x1 = x;
            y1 = y;
        }

        prev_heading = heading;

        int i = 0;
        udp_buf1[i++] = 131;
        ei_encode_tuple_header(udp_buf1, &i, 20);
        ei_encode_atom(udp_buf1, &i, "gps");
        ei_encode_double(udp_buf1, &i, gpstime);
        ei_encode_string(udp_buf1, &i, gpstime_text);
        ei_encode_double(udp_buf1, &i, lat);
        ei_encode_double(udp_buf1, &i, lon);
        ei_encode_double(udp_buf1, &i, alt);
        ei_encode_double(udp_buf1, &i, north_acc);
        ei_encode_double(udp_buf1, &i, east_acc);
        ei_encode_double(udp_buf1, &i, alt_acc);
        ei_encode_double(udp_buf1, &i, vf);
        ei_encode_double(udp_buf1, &i, vl);
        ei_encode_double(udp_buf1, &i, ax);
        ei_encode_double(udp_buf1, &i, ay);
        ei_encode_double(udp_buf1, &i, az);
        ei_encode_double(udp_buf1, &i, heading);
        ei_encode_double(udp_buf1, &i, pitch);
        ei_encode_double(udp_buf1, &i, roll);
        ei_encode_double(udp_buf1, &i, heading_acc);
        ei_encode_double(udp_buf1, &i, pitch_acc);
        ei_encode_double(udp_buf1, &i, roll_acc);
        send_udp_binary(udp_buf1, i, GPS_SERVER, GPS_PORT);
        return 0;
    }


    #include "NComRxC.h"
     
    // Prototypes for some helper functions.
    //static void report(const NComRxC *nrx);
    //static void print(FILE *fp, const NComRxC *nrx);

    NComRxC *nrx;         // NComRxC object


    void* gps_loop(void *arg) {
     
        int saddr_size, size;
        struct sockaddr saddr;

        int i;
        unsigned char c;
     
        unsigned char *buffer = (unsigned char *) malloc(65536);

        int sock_raw = socket( AF_PACKET , SOCK_RAW , htons(ETH_P_ALL)) ;
        setsockopt(sock_raw , SOL_SOCKET , SO_BINDTODEVICE , GPS_ETH_DEVICE , strlen(GPS_ETH_DEVICE)+1 );

        // Create NCom decoder and check
    	nrx = NComCreateNComRxC();
    	if(nrx == NULL)
    	{
    	//Print the error with proper message
            perror("Error: Unable to create NCom decoder.\n");
            return 0;
    	}

        if(sock_raw < 0)
        {
            //Print the error with proper message
            perror("Socket Error");
            return 0;
        }

        printf("gps_loop() starting... ");

        while(1)
        {
            saddr_size = sizeof saddr;
            //Receive a packet
            size = recvfrom(sock_raw, buffer, 65536, 0, &saddr, (socklen_t*)&saddr_size);
            if(size < 0 )
            {
                printf("Recvfrom error , failed to get packets\n");
                return 0;
            }
            
            gps_ts_prev = gps_ts;
            gps_ts = timer_stop(&t_start);    // Get local timestamp of gps data
    	    
            //Now process the packet
            //Get the IP Header part of this packet , excluding the ethernet header
            struct iphdr *iph = (struct iphdr*)(buffer + sizeof(struct ethhdr));
            unsigned short iphdrlen = iph->ihl*4;
            struct udphdr *udph = (struct udphdr*)(buffer + iphdrlen  + sizeof(struct ethhdr));
            int dport;

            double     gps2machine, mMachineTime;
            time_t     t1;
            struct tm *td;
            int        ms;
            

    	    switch (iph->protocol) //Check the Protocol and do accordingly...
            {
                case 17: //UDP Protocol
                    dport = ntohs(udph->dest);

                    if (dport==3000) {

                    	//while((c = fgetc(fpin)) != EOF)

                    	for(i = 0; i < 8192; i++)
            			{
            				c = buffer[i];

            				// Decode the data
            				if(NComNewChar(nrx, (unsigned char) c) == COM_NEW_UPDATE)
            				{
            					// For regular updates then output to main output file, otherwise,
            					// for falling edge input triggers then output to trigger file.
            					switch(nrx->mOutputPacketType)
            					{
            						case OUTPUT_PACKET_REGULAR:

                                        if (nrx->mIsTimeValid)
                                        {
                                            // Convert GPS seconds (from 1980-01-06 00:00:00) to machine seconds (from 1970-01-01 00:00:00). It is
                                            // very likely the machine will adjust for leap seconds, hence the correct GPS UTC difference is
                                            // applied. If the local machine time does not start from 1970-01-01 00:00:00 then the value of
                                            // gps2machine below needs to change.
                                            gps2machine  = 315964800.0;
                                            mMachineTime = nrx->mTime + gps2machine + nrx->mTimeUtcOffset;

                                            // Compute local time
                                            t1 = (time_t) floor(mMachineTime);
                                            td = localtime(&t1);
                                            ms = floor(0.5 + (mMachineTime - t1) * 1000.0);
                                            if(ms < 0) ms = 0; else if(ms > 999) ms = 999;

                                            // Print: GPS time, local date, time zone.
                                            sprintf( gpstime, "%04d-%02d-%02d %02d:%02d:%02d.%03d,",
                                                    1900+td->tm_year, 1+td->tm_mon, td->tm_mday, td->tm_hour, td->tm_min, td->tm_sec, ms);
                                        } else {
                                            gpstime[0] = 0;
                                        }

                                        // constraints
                                        if (nrx->mLat<50 || nrx->mLat>60) nrx->mLat = 0;
                                        if (nrx->mLon<20 || nrx->mLon>30) nrx->mLon = 0;
                                        if (nrx->mAlt<0.01 && nrx->mAlt>-0.01) nrx->mAlt = 0;
                                        if (nrx->mVf<0.01 && nrx->mVf>-0.01) nrx->mVf = 0;
                                        if (nrx->mVl<0.01 && nrx->mVl>-0.01) nrx->mVl = 0;
                                        if (nrx->mHeading<0.01 && nrx->mHeading>-0.01) nrx->mHeading = 0;

                                        gps_lat = nrx->mLat;
                                        gps_lon = nrx->mLon;
                                        gps_alt = nrx->mAlt;
                                        gps_vf  = nrx->mVf;

                                        #ifdef SPEED_FROM_GPS
                                            curr_speed_prev = curr_speed;
                                            curr_speed = gps_vf * 3.6;
                                            acceleration = (curr_speed-curr_speed_prev) / (gps_ts-gps_ts_prev);
                                        #endif

                                        send_gps_message(
                                            mMachineTime,
                                            gpstime,
                                            nrx->mLat,
                                            nrx->mLon,
                                            nrx->mAlt,
                                            nrx->mNorthAcc,
                                            nrx->mEastAcc,
                                            nrx->mAltAcc,
                                            nrx->mVf,
                                            nrx->mVl,
                                            round(nrx->mFiltAx * 10000.0) / 10000.0,
                                            round(nrx->mFiltAy * 10000.0) / 10000.0,
                                            round(nrx->mFiltAz * 10000.0) / 10000.0,
                                            nrx->mHeading,
                                            nrx->mPitch,
                                            nrx->mRoll,
                                            nrx->mHeadingAcc,
                                            nrx->mPitchAcc,
                                            nrx->mRollAcc
                                        );

            							break;
            						
                                    default : 
            							break;
            						
                                    break;
            					}
            				}
            			}

                    }

                    break;
            }
        }
        close(sock_raw);

        return 0;
    }

#endif


int gps_loop_start() {    
    int rc = 0;

    #ifdef USE_GPS
        rc = pthread_create(&tid_gps_loop, NULL, &gps_loop, NULL);
        if (rc != 0) 
        {
            printf("\nCan't create m_cnncollect_loop() thread :[%s]", strerror(rc));
        }
    #endif
    
    return rc;
}


