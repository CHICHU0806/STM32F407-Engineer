//
// Created by 20852 on 2025/10/30.
//

#include "../Inc/masterBoardTask.h"
#include "bsp_can.h"
#include "dbus.h"

void MasterBoardTask::run() {
    for (;;) {
        //bsp_can1_sendmotorcmd(dbus.ch[1],dbus.ch[0],dbus.ch[2],0);
        bsp_can2_sendremotecontrolcmd(dbus.ch[3],dbus.ch[2],dbus.ch[4]);
        osDelay(10);
    }
}

extern "C" {
    static MasterBoardTask master_board_task;

    void MasterBoardTask_Init() {
        master_board_task.start((char*)"MasterBoardTask", 256, osPriorityNormal);
    }

}

