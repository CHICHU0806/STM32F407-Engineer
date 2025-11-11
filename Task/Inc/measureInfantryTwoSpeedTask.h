//
// Created by 20852 on 2025/11/11.
//

#ifndef STARTM3508_MEASUREINFANTRYTWOSPEEDTASK_H
#define STARTM3508_MEASUREINFANTRYTWOSPEEDTASK_H


#pragma once
#include "../TaskBase.h"

#ifdef __cplusplus
class MeasureInfantryTwoSpeedTask : public TaskBase {
public:
    void run() override;   // 继承并实现 run()
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void MeasureInfantryTwoSpeedTask_Init();

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_MEASUREINFANTRYTWOSPEEDTASK_H