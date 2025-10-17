//
// Created by 20852 on 2025/10/17.
//

#ifndef STARTM3508_MESSAGE_BUS_H
#define STARTM3508_MESSAGE_BUS_H

#pragma once
#ifdef __cplusplus
#include <functional>
#include <vector>
#include <cstdint>

// -----------------------------
// 1. 消息类型定义
// -----------------------------
enum class MsgType : uint16_t {
    None = 0,
    MotorCtrl = 0x0101,
    LedCtrl   = 0x0102,
    DebugInfo = 0x0201,
    // ... 可以扩展更多类型
};

// -----------------------------
// 2. 通用消息结构体
// -----------------------------
struct Message {
    MsgType type;
    void* payload;    // 指向数据结构体，如 MotorCommand*
};

// -----------------------------
// 3. 订阅者类型
// -----------------------------
using MsgCallback = std::function<void(const Message&)>;

// -----------------------------
// 4. 发布/订阅中心类
// -----------------------------
class MessageBus {
public:
    static MessageBus& instance() {
        static MessageBus bus;
        return bus;
    }

    // 注册订阅者
    void subscribe(MsgType type, MsgCallback cb) {
        subscribers_.push_back({ type, cb });
    }

    // 发布消息
    void publish(const Message& msg) {
        for (auto& sub : subscribers_) {
            if (sub.type == msg.type && sub.cb) {
                sub.cb(msg);
            }
        }
    }

private:
    struct Subscriber {
        MsgType type;
        MsgCallback cb;
    };

    std::vector<Subscriber> subscribers_;
    MessageBus() = default;
};
#endif

#endif //STARTM3508_MESSAGE_BUS_H