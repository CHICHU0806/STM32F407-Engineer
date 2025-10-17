//
// Created by 20852 on 2025/10/17.
//

#ifndef STARTM3508_SWERVETASK_H
#define STARTM3508_SWERVETASK_H

#pragma once
#include "../TaskBase.h"

#ifdef __cplusplus
class SwerveTask : public TaskBase {
public:
    void run() override;   // 继承并实现 run()
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void SwerveTask_Init();

#ifdef __cplusplus
}
#endif


#endif //STARTM3508_SWERVETASK_H