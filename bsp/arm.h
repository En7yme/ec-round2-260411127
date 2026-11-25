#ifndef ARM_H
#define ARM_H
#include "main.h"
#include <stdint.h>

// 0号：360度底座控制
void Base_Init(void);
void Base_MoveAngle(int16_t angle, float max_speed_offset);
void Base_Update(float p);

// 1、2、3号：180度舵机控制
void Arm_Init(void);
void Arm_SetTarget(int16_t q1_shoulder, int16_t q2_elbow, int16_t q3_gripper);
void Arm_Update(void); // 放在 5ms 定时器里插补
float shder_and_el_to_Servo(float angle);//1,2号舵机的角度映射
uint8_t Arm_IsMoving(void);

// 夹爪特殊的连杆映射
float Gripper_To_Servo(float gripper_angle); // 0~900度 映射到舵机角度

#endif