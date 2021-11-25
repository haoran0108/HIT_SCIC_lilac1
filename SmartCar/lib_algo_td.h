/**
 * Copyright 2018 - 2021 HITSIC
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/**
 * @file    :   lib_algo_td.h
 * @author  :   FYX
 * @version :   v1.0
 *
 * @date        v1.0      2021.04.18
 */

#ifndef _LIB_ALGO_TD_H_
#define _LIB_ALGO_TD_H_

#include "math.h"

typedef struct td
{
    float x1;   ///< 一阶跟随量
    float x2;   ///< 二阶跟随量
    float fh;   ///< fhan输出值
    float *v;   ///< 观测目标
    float h;    ///< 跟踪速度因子（观测周期）
    float r;    ///< 滤波因子（加速度）
}td_t;          ///< 跟踪微分器 Tracking Differentiator

void TD_Update(td_t *_td);

void TD_Clear(td_t *_td);

#endif /* _LIB_ALGO_TD_H_ */
