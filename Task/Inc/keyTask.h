//
// Created by 20852 on 2025/10/15.
//

#ifndef STARTM3508_KEYTASK_H
#define STARTM3508_KEYTASK_H

#pragma once
#include "../TaskBase.h"
#include "main.h"

#ifdef __cplusplus
class KeyTask : public TaskBase {
public:
    void run() override;   // 继承并实现 run()
};

#endif

#ifdef __cplusplus
extern "C" {
#endif

    void KeyTask_Init();

#ifdef __cplusplus
}
#endif


#endif //STARTM3508_KEYTASK_H