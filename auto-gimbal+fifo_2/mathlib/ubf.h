/**
 * @file ubf.h
 * @brief 实时更新环形缓冲器
 * @note 使用方法参考 .c文件下方
 * 
 * @author ZZJ
 * @version 1.0 2022-02-25 首次完成基本功能：缓存区的创建、压栈、弹出、删除。
 *          1.1 2022-03-04 增加清空缓存区的功能。
 *          1.2 2022-03-14 增加将缓存区中数据转存到给定数组中的功能。
 *          1.3 2022-03-24 增加实时更新下的均值滤波、线性加权逐差法求数据差分和线性加权预测的功能；
 *                         线性加权预测功能尚未完善；
 *                         解决了缓存区删除功能内存泄露与清空功能未初始化写指针索引的BUG。
 * @copyright Copyright (c) Taurus 2022
 * 
 */
#ifndef __UPDATE_BUFFER_H__
#define __UPDATE_BUFFER_H__

#include "stdint.h"

/**
 * @brief 缓冲器指针变量类型
 */
typedef struct _ubf_t* ubf_t;

/**
 * @brief 创建缓冲器
 * 
 * @param num 数据单元个数
 * @param size 数据单元占字节数 
 * @return ubf_t 缓存器指针
 */
ubf_t ubf_create(uint32_t num, uint32_t size);

/**
 * @brief 删除缓存器，释放内存
 * 
 * @param ubf 待删除缓存器指针的指针
 */
void ubf_delete(ubf_t* ubf);

/**
 * @brief 投喂数据
 * 
 * @param ubf 缓存器指针
 * @param pdata 数据指针
 * @return uint8_t 
 */
void ubf_push(ubf_t ubf, const void* pdata);

/**
 * @brief 获取 k 时刻的数据
 * 
 * @param ubf 缓存器指针
 * @param k 时刻索引，k∈[0, num-1]
 * @return void* 返回数据指针
 */
void* ubf_pop(ubf_t ubf, uint32_t k);

/**
 * @brief 获取缓存器中当前数据数量
 * 
 * @param ubf 缓存器指针
 * @return uint32_t 缓存器中当前数据数量
 */
uint32_t ubf_get_stock(ubf_t ubf);

/**
 * @brief 清空缓存区
 * 
 * @param ubf 缓存器指针
 */
void ubf_clear(ubf_t ubf);

/**
 * @brief   取出缓存区若干数据，按照数据从新到旧排列成数组返回
 * @notice  注意传进函数里的数组指针一定要提前声明好！
 *
 * @param   ubf 缓存器指针
 * @param   pdata 返回数组首地址（用户创建）
 * @param   k 时刻索引，k∈[0, num-1]
 * @param   num 所需数据个数
 * @return  uint32_t 实际返回数据个数
 */
uint32_t ubf_pop_into_array_new2old(ubf_t ubf, void* pdata, uint32_t k, uint32_t num);

/**
 * @brief 取出缓存区若干数据，按照数据从旧到新排列成数组返回
 * @notice  注意传进函数里的数组指针一定要提前声明好！
 *
 * @param ubf 缓存器指针
 * @param pdata 返回数组首地址（用户创建）
 * @param k 时刻索引，k∈[0, num-1]
 * @param num 所需数据个数
 * @return uint32_t 实际返回数据个数
 */
uint32_t ubf_pop_into_array_old2new(ubf_t ubf, void* pdata, uint32_t k, uint32_t num);

/**
 * @brief 实时数据更新并进行平均滤波
 * 
 * @param ubf   缓存器指针
 * @param pdata 最新数据指针
 * @param num   平均分母
 * @return float 
 */
float ubf_avg_filter(ubf_t ubf, float* pdata, uint32_t num);

/**
 * @brief 线性加权滤波器（高阶低通滤波器）
 * 
 * @param ubf      缓存器指针
 * @param pdata    最新数据指针
 * @param pweight  权重数组首地址
 * @param num      数据量
 * @return float 
 */
float ubf_lin_wei_filter(ubf_t ubf, float* pdata, float* pweight, uint32_t num);

/**
 * @brief          线性加权逐差法
 * 
 * @param ubf      缓存器指针
 * @param pdata    最新数据指针
 * @param pweight  权重数组首地址 
 * @param num      数据量
 * @return float 
 */
float ubf_lin_wei_gradual_deduction(ubf_t ubf, float* pdata, float* pweight, uint32_t num);

/**
 * @brief          数据线性加权预测
 * 
 * @param ubf      缓存器指针
 * @param pdata    最新数据指针
 * @param pweight  权重数组首地址 
 * @param num      数据量
 * @param k        从前第k时刻开始补帧
 * @param flag     是否开启预测
 *                      0：关闭预测，存入当前数据；
 *                      1：打开预测，修改当前数据为预测值
 * @return float   当前数据或预测结果
 */
float ubf_lin_wei_predict(ubf_t ubf, float* pdata, float* pweight, uint32_t num, uint8_t k, uint8_t flag);

#endif
