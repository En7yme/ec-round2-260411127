#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <unistd.h> // 提供 _write 的函数原型
#include <sys/stat.h> // 提供文件状态结构体定义


extern UART_HandleTypeDef huart2;

/**
 * @brief 重定向 printf 的底层输出函数
 * @param file 文件描述符（1表示标准输出，2表示标准错误）
 * @param ptr 指向要发送的字符数组
 * @param len 要发送的字符长度
 * @return 实际发送的字节数
 */
int _write(int file, char *ptr, int len) {
    // 调用 HAL 库的串口阻塞发送函数
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}