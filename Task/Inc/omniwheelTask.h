//
// Created by 20852 on 2025/10/5.
//

#ifndef STARTM3508_OMNIWHEELTASK_H
#define STARTM3508_OMNIWHEELTASK_H

#pragma once
#include "../TaskBase.h"

#ifdef __cplusplus
class OmniWheelTask : public TaskBase{
public:
    void run() override;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void OmniWheelTask_Init();

#ifdef __cplusplus
}
#endif


#endif //STARTM3508_OMNIWHEELTASK_H