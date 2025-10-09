//
// Created by 20852 on 2025/10/5.
//

#include "../Inc/omniwheel.h"
#include <cmath>

OmniWheel::OmniWheel(float length, float width, float radius)
    : L(length), W(width), r(radius) {}

std::array<float, 4> OmniWheel::OmniWheel_Calculate(float Vx, float Vy, float omega) {
    float R = L + W;
    std::array<float, 4> WheelSpeeds;

    WheelSpeeds[0] = ( Vx * std::cos(M_PI_4) - Vy * std::sin(M_PI_4) + R * omega ) / r;  // 1号轮 左前
    WheelSpeeds[1] = ( Vx * std::cos(M_PI_4) - Vy * std::sin(M_PI_4) - R * omega ) / r;  // 4号轮 右前
    WheelSpeeds[2] = ( Vx * std::cos(M_PI_4) + Vy * std::sin(M_PI_4) + R * omega ) / r;  // 3号轮 左后
    WheelSpeeds[3] = ( Vx * std::cos(M_PI_4) + Vy * std::sin(M_PI_4) - R * omega ) / r;  // 2号轮 右后

    return WheelSpeeds;
}

extern "C" {
    static OmniWheel omni(0.435f, 0.435f, 0.075f);

    void omniwheel_calculate(float Vx, float Vy, float omega) {
        omni.OmniWheel_Calculate(Vx, Vy, omega);
    }
}
