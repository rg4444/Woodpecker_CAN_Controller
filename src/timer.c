#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>


void timer_start(struct timeval *t) {
    gettimeofday(t, NULL);
}

unsigned long timer_stop(struct timeval *t) {
    static struct timeval t2;
    gettimeofday(&t2, NULL);
    return t2.tv_sec * 1000000 + t2.tv_usec 
         - t->tv_sec * 1000000 - t->tv_usec;
}


