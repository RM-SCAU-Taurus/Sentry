#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

#ifdef  __SHOOT_TASK_GLOBALS
#define __SHOOT_TASK_EXT
#else
#define __SHOOT_TASK_EXT extern
#endif

#include "stm32f4xx_hal.h"
#include "bsp_FricMotor.h"
#include "cmsis_os.h"
#include "modeswitch_task.h"
/* 红外激光 */
#define LASER_UP		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET)
#define LASER_DOWN	    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET)

typedef void (*Fric_callback)();
typedef void (*Trigger_callback)();

/* 拨盘闭环控制变量结构 */
typedef struct
{
    /* position ecd loop */
		float trigger_hz_ref;
    float trigger_hz_fdb;
    float trigger_hz_error;
	
    float trigger_ecd_ref;
    float trigger_ecd_fdb;
    float trigger_ecd_error;
    /* speed loop */
    float trigger_spd_ref;
    float trigger_spd_fdb;
} trigger_pid_t;

/* 摩擦轮模式 */
typedef enum
{
    FIRC_MODE_STOP = 0,
    FIRC_MODE_RUN = 1
} shoot_firc_mode_e;
/* 摩擦轮保护模式 */
typedef enum
{
	FRIC_PROTECT 	= 0,
	FRIC_UNPROTECT 	= 1,
	FRIC_SLOW_TO_PROTECT = 2
} fric_protect_mode_e;
/*摩擦轮pid相关数据*/
typedef struct 
{
	float fric_ecd_ref;
	float fric_ecd_fdb;
	float fric_ecd_err;
	
	float fric_spd_ref;
	float fric_spd_fdb;
	float fric_spd_err;
}fric_pid_t;



/* 拨盘模式 */
typedef enum
{
    STIR_MODE_PROTECT,
    STIR_MODE_STOP,
    STIR_MODE_SINGLE,
    STIR_MODE_SERIES
} shoot_stir_mode_e;

/* 弹舱模式 */
typedef enum
{
    HOUSE_MODE_OPEN = 0,
    HOUSE_MODE_CLOSE = 1,
    HOUSE_MODE_PROTECT = 2
} shoot_house_mode_e;

/* 枪管数据结构 */
typedef struct
{
    trigger_pid_t pid;
    int16_t     current;            //发射器电流值
    float       heat;               //当前热量
    int32_t     heat_remain;        //剩余热量
    uint16_t    heat_max;			//最大热量	裁判系统读出
    uint16_t    cooling_rate;	    //冷却速率  裁判系统读出
    uint32_t    shoot_period;       //射击周期
} barrel_param_t;

/* 发射器控制中心结构 */
typedef struct
{
    shoot_firc_mode_e   firc_mode;
	fric_protect_mode_e	fric_protect_mode;	//摩擦轮保护模式
    shoot_stir_mode_e   stir_mode;
    shoot_house_mode_e  house_mode;
    barrel_param_t      barrel;
    uint16_t            shoot_speed;		//射速
    float               trigger_period;	//拨盘拨出一颗子弹的周期，体现射频
    uint8_t             shoot_speed_vision;//发给视觉的射速档位
} shoot_t;

typedef struct {
    uint32_t 	protect_cnt;
    uint8_t 	protect_flag;
    uint8_t 	init_flag;
	float 		set_speed;			//设置目标转速
	fric_pid_t	Fric_Pid_Set[2];	//摩擦轮pid参数
} fric_t;


/*简单工厂*/
typedef struct{
	ctrl_mode_e mode;
	Fric_callback Fric_action;
	Trigger_callback  Trigger_action;
	struct mode_t *next;
}shoot_class_parent_t;


typedef struct{
		shoot_class_parent_t base;
}shoot_class_child_t;




extern fric_t fric;
void shoot_task(void const *argu);
void shoot_init(void);

__SHOOT_TASK_EXT shoot_t shoot;    //发射器控制中心结构
extern int flag;

#endif
