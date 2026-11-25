#include "arm.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define GRIPPER_PULSE_MIN 1200.0f //经实测夹爪舵机脉宽1200-->0度  1350-->90度
#define GRIPPER_PULSE_MAX 1350.0f
#define GRIPPER_ANG_MIN 0.0f
#define GRIPPER_ANG_MAX 900.0f

#define SERVO_TO_PULSE_MIN 500.0f
#define SERVO_TO_PULSE_MAX 2500.0f
#define SERVO_TO_PULSE_MID 1500.0f
#define SERVO_TO_PULSE_K 1.111f
#define SERVO_ANG_MIN -900.0f
#define SERVO_ANG_MAX 900.0f



extern TIM_HandleTypeDef htim4;

float global_p=0.0f;
// ==========================================
// 第一部分：底座（0号 360度舵机）单独封装
// ==========================================
// 底座（360度舵机）变量
typedef struct {
    int16_t target_angle;
    int16_t current_angle; // 软件积分的开环角度（估算值）
    float start_pulse;
    float max_speed_offset; // 比如 50 表示最大速度时脉宽偏移 50*5=250us
    uint8_t is_moving;
} Base360_t;

Base360_t base360;

void Base_Init(void) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1500); // 上电停止
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    base360.current_angle = 0;
    base360.is_moving = 0;
} 

// 启动底座转动 (angle是相对当前的角度，max_speed_offset控制最大速度 1~100)
void Base_MoveAngle(int16_t angle, float max_speed_offset) {
    base360.target_angle = angle; 
    base360.max_speed_offset = max_speed_offset;
    if(angle != 0) base360.is_moving = 1;
}

//p 是总进度 0.0~1.0
void Base_Update(float p) {
    if (!base360.is_moving) return;

    // 如果运动结束
    if (p >= 1.0f) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1500); // 刹停
        base360.is_moving = 0;
        return;
    }

    // 1. 计算速度因子 v_f (即 6p(1-p))，保证它在 0~1.5 之间变化
    float v_f = 6.0f * p * (1.0f - p);

    // 2. 决定转动方向
    float dir = (base360.target_angle > 0) ? 1.0f : -1.0f;
    // 如果你的底座目标角极小，可能就是原地不动

    // 3. 计算当前脉宽偏移
    // 因为 360 舵机 1500 是停，1500 + 偏置是转。我们用 v_f 缩放最大速度
    float offset = base360.max_speed_offset * v_f * dir;
    
    // 防止死区（如果偏移太小，舵机不转，直接滤掉）
    if (fabs(offset) < 2.0f) offset = 0.0f;

    // 4. 写入寄存器
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint16_t)(1500.0f + offset));
}

// ==========================================
// 第二部分：肩、肘、夹爪（1~3号 180度舵机）
// ==========================================
typedef struct {
    float start_pulse;
    float target_pulse;
    float current_pulse;
    
    float total_diff; //脉宽总差值
    uint16_t total_steps;//总步数
    uint16_t current_step;

    uint8_t pwm_ch;//通道
} Joint180_t;

Joint180_t arm180[3]; // 索引 0=肩(Ch2), 1=肘(Ch3), 2=夹爪(Ch4)


void Arm_Init(void) {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);//肩
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);//肘
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);//夹爪
    
    // 初始化肩、肘      
    for(int i=0; i<2; i++) {
        if(i==0) arm180[i].pwm_ch = TIM_CHANNEL_2;
        else arm180[i].pwm_ch = TIM_CHANNEL_3;
        arm180[i].current_pulse = SERVO_TO_PULSE_MID;
    }
    
    // 初始化夹爪 
    arm180[2].pwm_ch = TIM_CHANNEL_4;
    arm180[2].current_pulse = GRIPPER_PULSE_MIN;
}

float shder_and_el_to_Servo(float angle){
    // 软限位钳位
    if(angle < SERVO_ANG_MIN) angle = SERVO_ANG_MIN;
    if(angle > SERVO_ANG_MAX) angle = SERVO_ANG_MAX;
    
    return (angle*SERVO_TO_PULSE_K + SERVO_TO_PULSE_MID);
}

// 连杆映射：夹爪开合(0~900) -> 舵机脉宽(1200~1350)
float Gripper_To_Servo(float gripper_angle) {
    // 软限位钳位
    if(gripper_angle < GRIPPER_ANG_MIN) gripper_angle = GRIPPER_ANG_MIN;
    if(gripper_angle > GRIPPER_ANG_MAX) gripper_angle = GRIPPER_ANG_MAX;
    
    return (gripper_angle/GRIPPER_ANG_MAX*(GRIPPER_PULSE_MAX-GRIPPER_PULSE_MIN) + GRIPPER_PULSE_MIN);
}

// 自动模式下发目标角度  软限位钳位之后就把角度映射成脉宽，便于armupdata()进行插补和速度规划
void Arm_SetTarget(int16_t q1_shoulder, int16_t q2_elbow, int16_t q3_gripper) {
    int16_t targets[3] = {q1_shoulder, q2_elbow, q3_gripper};
    
    // 1. 计算各个轴的目标脉宽
    for(int i=0; i<3; i++) {
        
        // 记录起始脉宽
        arm180[i].start_pulse = arm180[i].current_pulse;
        
        // 角度转目标脉宽
        if(i == 2) {
            arm180[i].target_pulse = Gripper_To_Servo((float)targets[i]);
        } else {
            arm180[i].target_pulse = shder_and_el_to_Servo((float)targets[i]);
        }
        
        // 计算脉宽差值
        arm180[i].total_diff = arm180[i].target_pulse - arm180[i].start_pulse;
    }
    
    // 2. 找出所有轴里，脉宽跨度最大的那个（用于决定总步数）
    float max_pulse_diff = 0.0f;
    for(int i=0; i<3; i++) {
        float abs_diff = fabs(arm180[i].total_diff);
        if(abs_diff > max_pulse_diff) max_pulse_diff = abs_diff;
    }
    
    // 3. 核心算法：保证每拍步进 <= 2us，动态算出总步数
    // 如果跨度是 1.5us，至少也走 1 步。
    uint16_t calculated_steps = (uint16_t)(max_pulse_diff / 2.0f);
    if(calculated_steps < 1) calculated_steps = 1;
    
    // 4. 将步数同步给所有轴（保证多轴同起同停）
    for(int i=0; i<3; i++) {
        arm180[i].total_steps = calculated_steps;
        arm180[i].current_step = 0;  // 重新起步
    }
}

// 在 5ms 定时器中调用，执行插补与PWM输出
void Arm_Update(void) {
    //同步global_p
    if (arm180[0].current_step < arm180[0].total_steps) {
        global_p = (float)arm180[0].current_step / (float)arm180[0].total_steps;
    } else {
        global_p = 1.0f;
    }
    //1-3号舵机
    for(int i=0; i<3; i++) {
        // 如果还在运动过程中
        if (arm180[i].current_step < arm180[i].total_steps) {
            
            // 1. 算出当前的进度百分比 (0.0 ~ 1.0)
            float p = (float)arm180[i].current_step / (float)arm180[i].total_steps;
            
            // 2. S 曲线速度因子 f (3次多项式)
            // 开头 f 接近 0，中间 f 变大，结尾 f 接近 1（注意：这里是位置插值的进度因子）
            float f = (3.0f * p * p) - (2.0f * p * p * p); //3次平滑曲线
            
            // 3. 根据进度因子算出当前的绝对脉宽
            // 这样写比累加更精准，不会产生累积误差
            arm180[i].current_pulse = arm180[i].start_pulse + (arm180[i].total_diff * f);
            
            arm180[i].current_step++;
            
        } else {
            // 运动结束，锁死在目标位置
            arm180[i].current_pulse = arm180[i].target_pulse;
        }
        
        // 4. 输出到硬件寄存器
        // 考虑到定时器 1us 的分辨率，把浮点脉宽强转为整数
        uint16_t output_pulse = (uint16_t)(arm180[i].current_pulse+0.5f);
        
        // 安全限幅（防止意外烧舵机）
        if(output_pulse < 500) output_pulse = 500;
        if(output_pulse > 2500) output_pulse = 2500;
        
        __HAL_TIM_SET_COMPARE(&htim4, arm180[i].pwm_ch, output_pulse);
    }

    //驱动0号舵机（底座）
    Base_Update(global_p);
}

