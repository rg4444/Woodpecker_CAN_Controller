#ifndef _BUMBLE_GRABBER_HPP_
#define _BUMBLE_GRABBER_HPP_


#ifdef __cplusplus
    extern "C" {
#endif

        
bool create_camera(void);

char* save_frame_png(unsigned int num);

bool destroy_camera(void);

#ifdef __cplusplus
    }
#endif

#endif
