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
 * @file    :   lib_algo_td.c
 * @author  :   FYX
 * @version :   v1.0
 *
 * @date        v1.0      2021.04.18
 */

#include "lib_algo_td.h"

float sign(float x)
{
    return x < 0 ? -1.0f : 1.0f;
}

float fhan(float x1, float x2, float r, float h)
{
    float d = r * h * h;
    float a0 = h * x2;
    float y = x1 + a0;
    float a1 = sqrt(d * (d + 8 * fabs(y)));
    float a2 = a0 + sign(y) * (a1 - d) * 0.5f;
    float sy = (sign(y + d) - sign(y - d)) * 0.5f;
    float a = (a0 + y - a2) * sy + a2;
    float sa = (sign(a + d) - sign(a - d)) * 0.5f;
    return -r * (a / d - sign(a)) * sa - r * sign(a);
}

void TD_Update(td_t *_td)
{
    _td->fh = fhan(_td->x1-*(_td->v), _td->x2, _td->r, _td->h);
    _td->x1 = _td->x1 + _td->h * _td->x2;
    _td->x2 = _td->x2 + _td->h * _td->fh;
}

void TD_Clear(td_t *_td)
{
    _td->fh = 0;
    _td->x1 = 0;
    _td->x2 = 0;
}

