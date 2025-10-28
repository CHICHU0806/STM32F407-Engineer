//
// Created by 20852 on 2025/10/28.
//

#ifndef STARTM3508_TARGETCARTASK_H
#define STARTM3508_TARGETCARTASK_H

#pragma once
#include "../TaskBase.h"

#ifdef __cplusplus
class TargetCarTask : public TaskBase {
public:
    void run() override;   // 继承并实现 run()
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void TargetCarTask_Init();

#ifdef __cplusplus
}
#endif


#endif //STARTM3508_TARGETCARTASK_H