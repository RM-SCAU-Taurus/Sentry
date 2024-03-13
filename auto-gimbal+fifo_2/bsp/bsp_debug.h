#ifndef __BSP_DEBUG_H__
#define __BSP_DEBUG_H__

/*-----------------------------chassis_debug---------------------------*/
#define RC_CH4_SCALE_DEBUG    40//12   
#define RC_CH3_SCALE_DEBUG    40//12

typedef enum
{
    eDEBUG_STOP = 0,
    eDEBUG_SHOOT = 1,
    eDEBUG_FLY  = 2,
    eDEBUG_VISION = 3,
} debug_mode_e;

typedef struct
{
    debug_mode_e mode;

} debug_t;

extern debug_t debug;
void debug_init(void);
void debug_task(void const *argu);

#endif
