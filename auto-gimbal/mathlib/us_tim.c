#include "us_tim.h"

prv_ust_t prv_ust;

static UST_TYPE ust_tim_get(void) {
    return UST_HTIM.Instance->CNT;
}

static void ust_tim_minus(ust_t* ust) {
    if (ust->now_cnt == ust->last_cnt) {  /* 没有溢出 */
        ust->dt = (ust->now_tim - ust->last_tim) * 1.0e-3;
    } else if (ust->now_cnt > ust->last_cnt) {  /* 发生溢出 */
        ust->dt = 1.0e-3 * UST_PRECISION * 
        ((ust->now_cnt - ust->last_cnt) * (UST_PERIOD + 1) + (ust->now_tim - ust->last_tim));
    }
}

void ust_tim_start(void) {
    UST_HTIM.Instance->ARR = (UST_TYPE)UST_PERIOD;
    HAL_TIM_Base_Start_IT(&UST_HTIM);
}

void ust_tim_end(void) {
    HAL_TIM_Base_Stop_IT(&UST_HTIM);
}

float ust_period_test(ust_t* ust) {
    ust->now_tim = ust_tim_get();
    ust->now_cnt = prv_ust.overflow_cnt;
    
    ust_tim_minus(ust);
    
    ust->last_tim = ust->now_tim;
    ust->last_cnt = ust->now_cnt;
    
    return ust->dt;
}

void ust_interval_test_start(ust_t* ust) {
    ust->last_tim = ust_tim_get();
    ust->last_cnt = prv_ust.overflow_cnt;
    ust->interval_start_flag = 1;
}

float ust_interval_test_end(ust_t* ust) {
    if (ust->interval_start_flag) {
        ust->now_tim = ust_tim_get();
        ust->now_cnt = prv_ust.overflow_cnt;
        
        ust_tim_minus(ust);
        ust->interval_start_flag = 0;
    } else {
        ust->dt = -1;
    }
    return ust->dt;
}

void ust_delay(UST_TYPE us) {
    if (us > UST_PERIOD - ust_tim_get()) {
        prv_ust.predict_overflow_cnt++;
    }
    
    while (
        ust_tim_get() >= prv_ust.predict_tim && 
        prv_ust.predict_overflow_cnt >= prv_ust.predict_overflow_cnt
    );
}
