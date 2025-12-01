//
// Created by 20852 on 2025/11/27.
//

#ifndef STARTM3508_IMUTEMPCONTROL_H
#define STARTM3508_IMUTEMPCONTROL_H

#include "temp_pid.h"
#include "main.h"

#ifdef __cplusplus
class ImuTempControl {
public:
    ImuTempControl(TIM_HandleTypeDef* htim, uint32_t channel);

    void init();
    void update(float targetTemp, float currentTemp, float dt);

private:
    TIM_HandleTypeDef* htim_;
    uint32_t channel_;

    float targetTemp_ = 0.0f;
};
#endif  // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif

    void ImuTempControl_Init();
    void ImuTempControl_Update(float targetTemp, float currentTemp, float dt);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_IMUTEMPCONTROL_H