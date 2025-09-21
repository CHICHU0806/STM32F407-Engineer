//
// Created by 20852 on 2025/9/21.
//

#ifndef STARTM3508_MAGICTOOLS_H
#define STARTM3508_MAGICTOOLS_H

#pragma once

#ifdef __cplusplus
template <typename T>
inline T clamp(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}
#endif

#endif //STARTM3508_MAGICTOOLS_H