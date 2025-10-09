//
// Created by 20852 on 2025/10/5.
//

#ifndef STARTM3508_OMNIWHEEL_H
#define STARTM3508_OMNIWHEEL_H

#pragma once
#include <array>

class OmniWheel {
public:
    // 构造函数：传入车长 L 和车宽 W
    OmniWheel(float length, float width, float radius = 1.0f);

    // 计算四个轮子的速度（前左、前右、后左、后右）
    // 输入: Vx 前后速度, Vy 左右速度, ω 角速度(rad/s)
    std::array<float, 4> OmniWheel_Calculate(float Vx, float Vy, float omega);

private:
    float L;   // 车长
    float W;   // 车宽
    float r;   // 轮半径
};

#ifdef __cplusplus
extern "C" {
#endif

    void omniwheel_calculate(float Vx, float Vy, float omega);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_OMNIWHEEL_H