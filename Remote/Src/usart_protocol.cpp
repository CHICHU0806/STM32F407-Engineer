//
// Created by 20852 on 2025/10/30.
//

#include "../Inc/usart_protocol.h"
#include "string.h"

UartProtocol::UartProtocol(FrameCallback cb)
    : recv_buf_(), cb_(cb), frame_type_(0), data_len_(0), data_index_(0)
{
    memset(recv_buf_, 0, sizeof(recv_buf_));
}

void UartProtocol::input(uint8_t byte)
{
    switch (state_)
    {
        case WAIT_HEAD1:
            if (byte == FRAME_HEAD_1) state_ = WAIT_HEAD2;
            break;

        case WAIT_HEAD2:
            if (byte == FRAME_HEAD_2) state_ = WAIT_TYPE;
            else state_ = WAIT_HEAD1;
            break;

        case WAIT_TYPE:
            frame_type_ = byte;
            state_ = WAIT_LEN;
            break;

        case WAIT_LEN:
            data_len_ = byte;
            if (data_len_ > FRAME_MAX_LEN - 5) { // 防止越界
                state_ = WAIT_HEAD1;
                break;
            }
            data_index_ = 0;
            state_ = (data_len_ == 0) ? WAIT_CHECKSUM : WAIT_PAYLOAD;
            break;

        case WAIT_PAYLOAD:
            recv_buf_[data_index_++] = byte;
            if (data_index_ >= data_len_) state_ = WAIT_CHECKSUM;
            break;

        case WAIT_CHECKSUM: {
            uint8_t sum = frame_type_;
            for (uint8_t i = 0; i < data_len_; ++i) sum += recv_buf_[i];
            sum &= 0xFF;

            if (sum == byte && cb_) {
                cb_(frame_type_, recv_buf_, data_len_);
            }
            state_ = WAIT_HEAD1;
            break;
        }
    }
}

uint8_t UartProtocol::calcChecksum(const uint8_t* data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; ++i) sum += data[i];
    return sum;
}

uint8_t UartProtocol::buildFrame(uint8_t* out_buf, uint8_t frame_type, const uint8_t* payload, uint8_t len)
{
    // 检查
    if (out_buf == nullptr || (len > FRAME_MAX_LEN - 5)) {
        return 0; // 无效帧
    }

    out_buf[0] = FRAME_HEAD_1;
    out_buf[1] = FRAME_HEAD_2;
    out_buf[2] = frame_type;
    out_buf[3] = len;

    memcpy(&out_buf[4], payload, len);

    uint8_t checksum = frame_type;
    for (uint8_t i = 0; i < len; ++i) checksum += payload[i];
    checksum &= 0xFF;

    out_buf[4 + len] = checksum;
    return 5 + len; // 返回完整帧长度
}