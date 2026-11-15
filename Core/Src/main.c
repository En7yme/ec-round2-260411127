/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "stm32f1xx_hal_adc.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include <stdint.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_ACC 1000.0f  //最大加速度
#define ROCKER_to_SPEED 300.0f //摇杆映射到速度的参数
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t f_1ms=0;//1ms标志位
volatile uint16_t tick=0;//毫秒计数器

//定义舵机结构体
typedef struct{
  int16_t target_ang;//单位为0.1度
  int16_t init_ang;
  int16_t current_ang;
  int16_t max_ang;
  int16_t min_ang;

}engine;

engine arm={0,0,0,900,-900};


//定义速度
float current_speed=0.0f;
float target_speed=0.0f;
//保留精度的current_ang
float current_ang_f=0.0f;
// float max_acc=500.0f;//最大加速度
uint16_t Angle_To_Pulse(int16_t angle) {
    float pulse = 1500.0f + (angle * 1.111f); //脉宽角度映射
    if (pulse < 500) pulse = 500;//防溢出
    if (pulse > 2500) pulse = 2500;
    return (uint16_t)pulse;
}

void Set_Servo_Angle(int16_t angle) {
    uint16_t pulse = Angle_To_Pulse(angle);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulse);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
1  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  arm.current_ang=arm.init_ang;//
  current_ang_f=(float)arm.init_ang;
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4, Angle_To_Pulse(arm.init_ang));
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);//开启pwm

  HAL_TIM_Base_Start_IT(&htim3);//时基

  HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t adc_raw = HAL_ADC_GetValue(&hadc1); //后台任务，获取ADC值
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(f_1ms){
      f_1ms=0;

      //1.每1ms执行的任务


      tick++;
      //2.分频任务1
      if(tick%5==0){//插补 插补周期为200Hz 即5ms一次
        int16_t joy=(int16_t)adc_raw-2048;//读摇杆 
        //死区处理
        if(abs(joy)<150){
          target_speed=0.0f;
        }
        else{
          //摇杆映射到速度
          target_speed = (joy/2048.0f)*ROCKER_to_SPEED;
        }
        
        //速度规划
        float dt=0.005f;//5ms
        float step_acc=MAX_ACC*dt;

        if(current_speed<target_speed){
          current_speed+=step_acc;
          if(current_speed>target_speed){
            current_speed=target_speed;
          }
        }
        else if(current_speed>target_speed){
          current_speed-=step_acc;
          if(current_speed<target_speed){
            current_speed=target_speed;
          }
        }
        
        if (target_speed == 0.0f && abs(current_speed) < 0.1f) {//摇杆中位时的防抖：直接输出角度,不让current_ang_f微调
            current_speed = 0.0f;
            // 不执行累加，直接输出当前的整型角度
            Set_Servo_Angle(arm.current_ang); 
        }
        else{
          //计算位置增量
          float step_ang=current_speed*dt;
          arm.target_ang=current_ang_f+=step_ang;

          //软限位钳位
          if (current_ang_f > arm.max_ang) {
              current_ang_f = arm.max_ang;
              current_speed = 0.0f; // 撞到限位，强制速度归零
          } else if (current_ang_f < arm.min_ang) {
              current_ang_f = arm.min_ang;
              current_speed = 0.0f;
          }

          arm.current_ang=(int16_t)current_ang_f;
          arm.target_ang=arm.current_ang;

          //输出PWM
          Set_Servo_Angle(arm.current_ang);
        }
      } 

      //3.分频任务2
      if(tick%20==0){//每20ms状态上报
        char msg[50];
        // 打印原始 ADC 值，以及目标速度
        sprintf(msg, "ADC: %lu, Speed: %.2f, Angle: %d\r\n", adc_raw, target_speed, arm.current_ang);
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim==&htim3){
    f_1ms=1;//1ms时
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
