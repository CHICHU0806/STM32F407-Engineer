//
// Created by 20852 on 2025/11/27.
//

#include "ImuTempControl.h"
#include "tim.h"

ImuTempControl::ImuTempControl(TIM_HandleTypeDef* htim, uint32_t channel)
        : htim_(htim), channel_(channel) {}

void ImuTempControl::init() {
    HAL_TIM_PWM_Start(htim_, channel_);
}

void ImuTempControl::update(float targetTemp, float currentTemp, float dt) {
    uint32_t duty = temp_pid_calculate(targetTemp,currentTemp, dt);

    auto compare = static_cast<uint32_t>(htim_->Init.Period * duty);

    __HAL_TIM_SET_COMPARE(htim_, channel_, compare);
}

extern "C" {
    static ImuTempControl imuTemp(&htim10, TIM_CHANNEL_1);

    void ImuTempControl_Init() {
        imuTemp.init();
    }

    void ImuTempControl_Update(float targetTemp, float currentTemp, float dt) {
        imuTemp.update(targetTemp, currentTemp, dt);
    }
}