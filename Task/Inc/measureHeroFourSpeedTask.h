//
// Created by 20852 on 2025/9/25.
//

#ifndef STARTM3508_MEASURESPEEDTASK_H
#define STARTM3508_MEASURESPEEDTASK_H

#pragma once
#include "../TaskBase.h"

#ifdef __cplusplus
class HeroMeasureSpeedTask : public TaskBase {
public:
    void run() override;   // 继承并实现 run()
};

#endif

#ifdef __cplusplus
extern "C" {
#endif

    void MeasureHeroFourSpeedTask_Init();

#ifdef __cplusplus
}
#endif


#endif //STARTM3508_MEASURESPEEDTASK_H