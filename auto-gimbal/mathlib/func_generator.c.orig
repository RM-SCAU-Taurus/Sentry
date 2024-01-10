/**
  *************************************************************************
  * @brief  函数信号发生器
  * @author ZZJ
  * @date   2021/2/10
  * @note   1. 可离散产生函数信号：
  *                 正弦，方波，三角波（或锯齿波）信号，符号信号
  *         2. 时间都是以ms为单位
  *         3. 初始化时的变量顺序为：
  *             生成器指针，生成周期，最大限幅，最小限幅，
  *             直流量，（周期相关量），其他交流特征量
  *         4. 可产生随机噪声和高斯噪声，其中前者需要开启硬件RNG
  *************************************************************************
  */
#define __FUNC_GENERATOR_GLOBALS__
#include "func_generator.h"


/*------------------------------------正弦信号交直流量生成器-----------------------------------*/
/**
  * @Brief  正弦信号发生器初始化
  * @note   一个参数决定周期
  */
void FGT_sin_init(
    FGT_sin_t*  sin,
    uint16_t    Td,
    float       max,
    float       min,
    float       dc,
    float       T,
    float       A,
    float       phi
)
{
    sin->Td = Td;
    sin->time = 0;

    sin->max = max;
    sin->min = min;

    sin->dc = dc;

    sin->A = A;
    sin->T = T;
    sin->phi = phi;

    sin->out = 0;
}

/**
  * @Brief  正弦信号发生器重新初始化参数
  * @note   只用于发生器运行过程中改变部分参数
  */
void FGT_sin_reinit(
    FGT_sin_t*  sin,
    float       max,
    float       min,
    float       dc,
    float       T,
    float       A
)
{
    sin->max = max;
    sin->min = min;

    sin->dc = dc;

    sin->A = A;
    sin->T = T;
}


/**
  * @Brief  产生正弦信号
  */
float FGT_sin_cal(FGT_sin_t* sin)
{
    float ac  = 0;
    float res = 0;

    /*计算交流量*/
    ac = (sin->A) * sinf( (2*pi / sin->T) * (sin->time) + sin->phi );

    /*计算交直流量*/
    res = ac + sin->dc;
    Output_Limit(res, sin->max, sin->min);

    /*迭代时间（先输出再计算，使从0开始）*/
    sin->time = fmodf(sin->time+sin->Td, sin->T);

    /*迭代输出记录*/
    sin->out = res;

    /*输出角波交直流量*/
    return sin->out;
}


/*------------------------------------方波信号交直流量生成器-----------------------------------*/
/**
  * @Brief  方波信号发生器初始化
  * @note   两个参数决定周期
  */
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
)
{
    sqr->Td   = Td;
    sqr->time = 0;

    sqr->max = max;
    sqr->min = min;

    sqr->dc  = dc;

    sqr->high = high;
    sqr->low  = low;
    sqr->Th   = Th;
    sqr->Tl   = Tl;

    sqr->out  = 0;
}


/**
  * @Brief  产生方波信号
  */
float FGT_sqr_cal(FGT_sqr_t* sqr)
{
    float ac  = 0;
    float res = 0;

    /*计算周期*/
    float T   = sqr->Th + sqr->Tl;

    /*计算交流量*/
    if( sqr->time > sqr->Th )
        ac = sqr->low;
    else
        ac = sqr->high;

    /*计算交直流量*/
    res = ac + sqr->dc;
    Output_Limit(res, sqr->max, sqr->min);

    /*迭代时间（先输出再计算，使从0开始）*/
    sqr->time = fmodf(sqr->time+sqr->Td, T);

    /*迭代输出记录*/
    sqr->out = res;

    /*返回方波交直流量*/
    return sqr->out;
}


/*------------------------------------角波信号交直流量生成器-----------------------------------*/
/**
  * @Brief  角波信号发生器初始化
  * @note   三个参数决定周期
  */
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
)
{
    agl->Td   = Td;
    agl->time = 0;

    agl->max = max;
    agl->min = min;

    agl->dc  = dc;

    agl->T1 = T1;
    agl->T2 = T2;

    agl->high = high;
    agl->low  = low;

    agl->out = 0;
}


/**
  * @Brief  产生角波信号
  */
float FGT_agl_cal(FGT_agl_t* agl)
{
    float ac  = 0;
    float res = 0;

    /*计算周期*/
    float T   = agl->T1 + agl->T2;

    /*确定区间,计算相对时间，计算交流量*/
    if( agl->time < agl->T1 )  //在第一个拐点之前
    {
        ac = ( (agl->low-agl->high)/agl->T1 ) * agl->time + agl->high;
    }
    else if( agl->time >= agl->T1 )  //在两个拐点之间
    {
        ac = ( (agl->high-agl->low)/agl->T2 ) * ( agl->time - agl->T1 ) + agl->low;
    }

    /*计算交直流量*/
    res = ac + agl->dc;
    Output_Limit(res, agl->max, agl->min);

    /*迭代时间*/
    agl->time = fmodf(agl->time+agl->Td,T);

    /*迭代输出记录*/
    agl->out = res;

    /*输出角波交直流量*/
    return agl->out;
}


/*------------------------------------符号函数-----------------------------------*/
/**
  * @Brief  符号生成器初始化
  */
void FGT_npz_init(
    FGT_npz_t* npz,
    uint16_t Td,
    float T1,
    float T2,
    float T3
)
{
    npz->Td   = Td;
    npz->time = 0;

    npz->T1 = T1;
    npz->T2 = T2;
    npz->T3 = T3;

    npz->out = 0;
}


/**
  * @Brief  产生符号信号
  * @func   依次产生T1时间的1，T2时间的0，T3时间的-1信号
  */
float FGT_npz_cal(FGT_npz_t* npz)
{
    /*计算周期*/
    float T   = npz->T1 + npz->T2 + npz->T3;

    /*确定输出*/
    if( npz->time <= npz->T1 )  //在第一个拐点之前
        npz->out = 1;
    else if( (npz->T1 < npz->time) && (npz->time < npz->T1 + npz->T2) )  //在两个拐点之间
        npz->out = 0;
    else if( (npz->T1 + npz->T2 <= npz->time) && (npz->time <= T) )  //在第二个拐点之后
        npz->out = -1;

    /*迭代时间（先输出再计算，使从0开始）*/
    npz->time = fmodf(npz->time+npz->Td,T);

    /*输出符号*/
    return npz->out;
}

/*------------------------------------噪声（随机数）-----------------------------------*/
/**
  * @Brief  读取当前生成的32位无符号随机数
  */
//HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *hrng, uint32_t *random32bit);


/**
  * @Brief  生成[min,max]范围的随机数
  */
int RNG_Get_RandomRange(int min,int max)
{
    uint32_t temp_32_rng;
//    HAL_RNG_GenerateRandomNumber(&hrng,&temp_32_rng);
    return temp_32_rng%(max-min+1) +min;
}


/*------------------------------------高斯噪声-----------------------------------*/
/**
  * @Brief  产生一个高斯噪声点
  */
float GaussGenerate(float mu, float sigma_f)
{
    float t1,t2,a,r;
    float x;
    /*产生两个均匀分布的0~1的随机序列*/
    t1 = (float)rand()/(RAND_MAX);
    t2 = (float)rand()/(RAND_MAX);
    /*极坐标的两个随机变量分布序列*/
    a = 2*pi*t1;            //a是极坐标的角度：变成了0~2*pi的均匀分布
    r = sqrt(-2*log2f(t2));   //r是极坐标的距离：变成自然对数开根号的一种分布
    /*用极坐标(a,r)转换成笛卡尔坐标(x,y)，这就是产生的高斯白噪声*/
    x = r*cos(a);

    return mu+sigma_f*x;
}


/**
  * @Brief  产生一个高斯噪声数组
  */
void Gauss(float gs[], int lengh, float mu, float sigma_f)
{
    for(int i=0; i<lengh; i++)
        gs[i]=GaussGenerate(mu,sigma_f);
}


/*------------------------------------一般函数信号发生器-----------------------------------*/
/**
  * @Brief  一般函数信号发生器
  * @param  函数结构体指针
  * @note   主要函数调用周期与结构体的周期，微分量之间的对应关系
  */
float FGT_f_generator(FGT_f_t* pf)
{
    float temp_out;
    /*迭代时间（先输出再计算，使从0开始）*/
    pf->time = fmodf(pf->time+pf->Td,pf->T);

    /*计算输出 限幅*/
    temp_out = pf->f(pf->time);
    Output_Limit(temp_out, pf->max, pf-> min);
    pf->out = temp_out;

    /*返回输出*/
    return pf->out;
}



/*------------------------------------总初始化-----------------------------------*/
#include "control_def.h"

static float f_test(float x)
{
    return 0.1f * x;
}


/*
    若执行周期 10ms，T = 6000， Td = 1，意味着
    计算过程将执行 （6000 /1.）* 10 = 60000ms = 1min
    最后输出前经过限幅
*/
FGT_f_t test_f =
{
    .f = f_test,
    .time = 0,
    .T = 6000,
    .Td = 10,
    .max = 7000,
    .min = 0,
    .out = 0
};


void FGT_init(void)
{
    
    /* 一般波形信号发生器 */
    FGT_sin_init(&test_sin, 1, 3670, 2400, 3035, 800,        600,0);
    FGT_sin_init(&test_cos, 1, 300, 0, 150, 4000,        100, pi/2);
    FGT_sqr_init(&test_sqr, 1, 3500, 2500, 0, 1000, 1000,    3500,2500);
    FGT_agl_init(&test_agl, 5, 500, -500, 200, 100,200,     100, -50);

    FGT_npz_init(&test_npz_101, 5, 3000, 1000, 3000);
    FGT_npz_init(&test_npz_10, 5, 3000, 1000, 0);
    
    /* 特殊功能信号发生器 */
    //FGT_sin_init(&chassis_spin_sin, 2, 3670, 2400, 3035, 800,        600,0);
    FGT_sqr_init(&test_pit, 1, (GIMBAL_PIT_MAX-150), (GIMBAL_PIT_MIN+150), 0, 800, 800,  (GIMBAL_PIT_MAX-150),(GIMBAL_PIT_MIN+150));   
}

//FGT_sin_t test_s = 
//{
//    .Td = 1,
//    .time = 0,
//    .max = 1000,
//    .min = -1000,
//    .dc = 0,
//    .T = 100,
//    .A = 50,
//    .phi = 0,
//    .out = 0
//};



