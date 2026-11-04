/**
 * @file    app_jump.h
 * @brief   从 Bootloader 跳到 APP 区（0x08010000）。
 */

#ifndef __APP_JUMP_H
#define __APP_JUMP_H

#include <stdint.h>

#define APP_BASE   0x08010000u
#define APP_END    0x08100000u

/* 尝试跳转到 APP。只有真的跳不出去才会返回，返回值说明原因：
 *   -1 向量表里的栈顶指针不像样
 *   -2 复位向量不在 APP 区内（没烧过 APP，或者固件坏了） */
int app_jump_try(void);

#endif /* __APP_JUMP_H */
