/** 
 ** udp_serverlib.c - UDP server functions
 **/

#define PLATFORM_WINDOWS  1
#define PLATFORM_MAC      2
#define PLATFORM_LINUX    3

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "commander.h"
#include "udp_api.h"


#if defined(_WIN32)

    #define PLATFORM PLATFORM_WINDOWS

#elif defined(__APPLE__)

    #define PLATFORM PLATFORM_MAC

#else

    #define PLATFORM PLATFORM_LINUX

#endif

#if PLATFORM == PLATFORM_WINDOWS

    #include <winsock2.h>

#elif PLATFORM == PLATFORM_MAC || PLATFORM == PLATFORM_LINUX

    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <fcntl.h>
    #include <arpa/inet.h>

#endif

#if PLATFORM == PLATFORM_WINDOWS

    #pragma comment(lib,"ws2_32.lib")

#endif


#if PLATFORM == PLATFORM_MAC || PLATFORM == PLATFORM_LINUX
    void macLinuxUDPLoop(int, struct sockaddr*, socklen_t);
    void macLinuxUDPServer();
#endif

#if PLATFORM == PLATFORM_WINDOWS
    void windowsUDPLoop(SOCKET, sockaddr*, size_t);
    int windowsUDPServer();
#endif


extern act_s udpdrive_act;  /* Actuator signal state from udp */

uint8_t cmd,val_h,val_l;
int8_t sign;
double steering_wheel_angle,steering,throttle,braking,speed;

extern double curr_speed;
extern int system_mode;    /* Control method: 0 - manual, 1 - by joystick, 2 - by udp */


int process_message(void *_buf, size_t n) {

    const char *buf = (const char *)_buf;
    cmd = ((uint8_t*)buf)[0];

    // Process received UDP message

    if ( (cmd==STEERING) || 
         (cmd==THROTTLE) || 
         (cmd==BRAKING) || 
         (cmd==SPEED) || 
         (cmd==OFF)) 
   {
         if (cmd==STEERING) 
			{
					    sign = ((int8_t*)buf)[1];
						val_h = ((uint8_t*)buf)[2];
						val_l = ((uint8_t*)buf)[3];
						
						printf("Steering UDP recieved ||| %d %d %d %d |||||||| \n", cmd, sign, val_h, val_l);
						
						steering_wheel_angle = ((float)val_h*256+val_l) / 100.0;

						if (sign==1) 
						{
							steering_wheel_angle = -steering_wheel_angle;
						}

						if (steering_wheel_angle > STEERING_VAL_MAX) {
							steering_wheel_angle = STEERING_VAL_MAX;
						} else if (steering_wheel_angle < -STEERING_VAL_MAX) {
							steering_wheel_angle = -STEERING_VAL_MAX;
						}

						steering = steer_angle_to_normalized(steering_wheel_to_column_angle(steering_wheel_angle,curr_speed));

						if (steering < -1.0)
							steering = -1.0;
						else if (steering > 1.0)
							steering = 1.0;

						udpdrive_act.steering = steering;
						udpdrive_act.steering_source = 2;	
			}
        
			else if (cmd==THROTTLE) {
				val_h = ((uint8_t*)buf)[1];
				val_l = ((uint8_t*)buf)[2];
			
				throttle = ((float)val_h*256+val_l) / 1000.0;
				
				if (throttle < 0.0)
					throttle = 0.0;
				else if (throttle > 1.0)
					throttle = 1.0;

				udpdrive_act.throttle        = throttle;
				udpdrive_act.throttle_source = 2;
				
				printf("Acceleration UDP recieved ||| %d %d %d Throttle value = %d|||||||| \n", cmd, val_h, val_l, throttle);			
			}
		
			else if (cmd==BRAKING) {
				val_h = ((uint8_t*)buf)[1];
				val_l = ((uint8_t*)buf)[2];
	 
				printf("Braking UDP recieved ||| %d %d %d |||||||| \n", cmd, val_h, val_l);
	 
				braking = ((float)val_h*256+val_l) / 1000.0;

				if (braking < 0.0)
					braking = 0.0;
				else if (braking > 1.0)
					braking = 1.0;

				udpdrive_act.braking        = braking;
				udpdrive_act.braking_source = 2;
			}
		
			else if (cmd==SPEED) {
				val_h = ((uint8_t*)buf)[1];
				val_l = ((uint8_t*)buf)[2];
	 
				printf("Speed UDP recieved ||| %d %d %d |||||||| \n", cmd, val_h, val_l);
	 
				speed = ((float)val_h*256+val_l) / 100.0;

				if (speed < 0.0)
					speed = 0.0;
				else if (speed > SPEED_VAL_MAX)
					speed = SPEED_VAL_MAX;

			}

        if (cmd != OFF) 
			{   udpdrive_act.active = 1;
				if (system_mode==1) 
				{
					system_mode = 2;
				}
			 }
		else {
			printf("OFF UDP recieved ||| %d |||||||| \n", cmd);		
            udpdrive_act.active   = 0;
            udpdrive_act.steering = 0.0;
            udpdrive_act.throttle = 0.0;
            udpdrive_act.braking  = 0.0;

            udpdrive_act.steering_source = 0;
            udpdrive_act.throttle_source = 0;
            udpdrive_act.braking_source  = 0;

            if (system_mode==2) {
                system_mode = 1;
            }
        }

   //     printf("UDP ||| steering:%f   throttle:%f   braking:%f\n", udpdrive_act.steering, udpdrive_act.throttle, udpdrive_act.braking);

    }

    return EXIT_SUCCESS;
}


int udp_server()
{
    #if PLATFORM == PLATFORM_WINDOWS
        windowsUDPServer();
    #else
        macLinuxUDPServer();
    #endif
    return EXIT_SUCCESS;
}

#if PLATFORM == PLATFORM_MAC || PLATFORM == PLATFORM_LINUX

void macLinuxUDPServer()
{
    int socketFileDescriptor;
    struct sockaddr_in serverAddr;
    struct sockaddr_in clientAddr;
    int r;

    socketFileDescriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    bzero(&serverAddr, sizeof(serverAddr));

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    printf("port: %d ", UDP_API_PORT);
    serverAddr.sin_port = htons(UDP_API_PORT);
    r = bind(socketFileDescriptor, (struct sockaddr*)& serverAddr, sizeof(serverAddr));
    if (r<0) {
        printf("failed. (already started)\n");
        exit(0);
    }

    printf(" started.\n");
    macLinuxUDPLoop(socketFileDescriptor, (struct sockaddr*)& clientAddr, sizeof(clientAddr));
}

void macLinuxUDPLoop(int sockFd, struct sockaddr* cliaddr, socklen_t clilen)
{
    int bytesRead;
    socklen_t len;
    char msg[BUFMAX] = {0};

    for(;;)
    {
        len = clilen;
        bzero(&msg, sizeof(msg));
        bytesRead = recvfrom(sockFd, msg, BUFMAX, 0, cliaddr, &len);
        process_message(msg, bytesRead);
    }
}

#endif

#if PLATFORM == PLATFORM_WINDOWS

int windowsUDPServer()
{
    SOCKET sock;
    sockaddr_in serverAddr;
    sockaddr_in clientAddr;
    WSADATA wsaDat;

    int r;

    int wsaError = WSAStartup( MAKEWORD(2,2), &wsaDat );

    if(!wsaError)
    {
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

        ZeroMemory(&serverAddr, sizeof(serverAddr));

        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

        printf("port: %d ", UDP_API_PORT);
        serverAddr.sin_port = htons(UDP_API_PORT);
        r = bind(sock, (struct sockaddr*)& serverAddr, sizeof(serverAddr));
        if (r<0) {
            printf("failed. (already started)\n");
            exit(0);
        }

        printf(" started.\n");
        windowsUDPLoop(sock, (struct sockaddr*)& clientAddr, sizeof(clientAddr));
    }
    else
    {
        return EXIT_FAILURE;
    }
}

void windowsUDPLoop(SOCKET sock, sockaddr* cliaddr, size_t clilen)
{
    int bytesRead;
    int len;
    char msg[BUFMAX] = {0};

    //printf("Waiting for datagrams on 127.0.0.1:%d\n", UDP_API_PORT);

    for(;;)
    {
        len = clilen;
        ZeroMemory(&msg, sizeof(msg));
        bytesRead = recvfrom(sock, msg, BUFMAX, 0, cliaddr, &len);
        process_message(msg, bytesRead);
    }

}

#endif



