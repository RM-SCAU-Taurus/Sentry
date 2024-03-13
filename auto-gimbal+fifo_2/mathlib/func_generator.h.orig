#ifndef __FUNC_GENERATOR_H__
#define __FUNC_GENERATOR_H__

#ifdef  __FUNC_GENERATOR_GLOBALS__
    #define __FUNC_GENERATOR_EXT
#else
    #define __FUNC_GENERATOR_EXT extern
#endif

#include "stdint.h"
#include "stdlib.h"
#include "math.h"
#include "math_calcu.h"


/*------------------------------------正弦信号交直流量生成器-----------------------------------*/
typedef struct
{
    uint16_t Td;    //信号生成周期
    uint32_t time;  //当前运行时间

    float max;      //信号最大幅值
    float min;      //信号最小幅值

    float dc;       //直流量

    float T;        //正弦信号的周期
    float A;        //正弦信号的振幅
    float phi;      //正弦信号的初相

    float out;      //输出记录
} FGT_sin_t;


void FGT_sin_init(
    FGT_sin_t*  sin,
    uint16_t    Td,
    float       max,
    float       min,
    float       dc,
    float       T,
    float       A,
    float       phi
);

void FGT_sin_reinit(
    FGT_sin_t*  sin,
    float       max,
    float       min,
    float       dc,
    float       T,
    float       A
);

float FGT_sin_cal(FGT_sin_t* sin);

/*------------------------------------方波信号交直流量生成器-----------------------------------*/
typedef struct
{
    uint16_t Td;
    uint32_t time;

    float max;
    float min;

    float dc;

    float Th;   //高电平时间
    float Tl;   //低电平时间
    float high; //高电平幅值
    float low;  //低电平幅值

    float out;
} FGT_sqr_t;


void FGT_sqr_init(
    FGT_sqr_t* sqr,

    uint16_t Td,

    float max,
    float min,

    float dc,

    float Th,
    float Tl,
    float high,
    float low
);

float FGT_sqr_cal(FGT_sqr_t* sqr);


/*------------------------------------角波信号交直流量生成器-----------------------------------*/
typedef struct
{
    uint16_t Td;
    uint32_t time;

    float max;
    float min;

    float dc;

    float T1;   //周期始到第一个拐点的时间
    float T2;   //第一二个拐点间的时间
    float high;
    float low;
    float out;
} FGT_agl_t;


void FGT_agl_init(
    FGT_agl_t* agl,

    uint16_t Td,
    float max,
    float min,

    float dc,

    float T1,
    float T2,
    float high,
    float low
);

float FGT_agl_cal(FGT_agl_t* agl);


/*------------------------------------符号函数-----------------------------------*/
typedef struct
{
    uint16_t Td;
    uint32_t time;

    float T1;   //周期始到第一个拐点的时间
    float T2;   //第一二个拐点间的时间
    float T3;   //第二个拐点到周期末的时间

    float out;
} FGT_npz_t;

void FGT_npz_init(
    FGT_npz_t* npz,
    uint16_t Td,
    float T1,
    float T2,
    float T3
);

float FGT_npz_cal(FGT_npz_t* npz);


/*------------------------------------噪声（随机数）-----------------------------------*/
/*使能硬件RNG时可用*/
//HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *hrng, uint32_t *random32bit);
int RNG_Get_RandomRange(int min,int max);


/*------------------------------------高斯噪声-----------------------------------*/
float GaussGenerate(float mu, float sigma_f);
void Gauss(float gs[], int lengh, float mu, float sigma_f);


/*------------------------------------一般函数信号发生器-----------------------------------*/
typedef struct _FGT_f_t
{
    float (*f)(float x);
    float time;
    float T;
    float Td;
    float max;
    float min;
    float out;
} FGT_f_t;

float FGT_f_generator(FGT_f_t* pf);

/*------------------------------------变量声明与总初始化-----------------------------------*/
void FGT_init(void);

__FUNC_GENERATOR_EXT FGT_sin_t test_sin;
__FUNC_GENERATOR_EXT FGT_sin_t test_cos;
__FUNC_GENERATOR_EXT FGT_sqr_t test_sqr;
__FUNC_GENERATOR_EXT FGT_agl_t test_agl;
__FUNC_GENERATOR_EXT FGT_npz_t test_npz_101;
__FUNC_GENERATOR_EXT FGT_npz_t test_npz_10;
__FUNC_GENERATOR_EXT FGT_f_t test_f;

__FUNC_GENERATOR_EXT FGT_sqr_t test_pit;
__FUNC_GENERATOR_EXT FGT_sin_t chassis_spin_sin;


#endif

