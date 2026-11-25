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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "arm.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
volatile uint8_t f_1ms=0;//1ms的控制节拍
volatile uint16_t tick=0;//计时器
uint8_t key_state=1;//用于反转led的按键的状态 1=高电平 未按下

//声明四个状态变量
void State_auto();//自动模式
void State_manual();//手动模式
void State_return();//回零模式
void State_fault();//故障模式

//定义状态机结构体
typedef struct{        
  void (*f)(void);
} StateMachine;

//状态机的枚举
typedef enum{
  STATE_ANTO=0,
  STATE_MANUAL,
  STATE_RETURN,
  STATE_FAULT
} Sys_State;

//状态机数组
StateMachine state_tab[4]={
  {State_auto},
  {State_manual},
  {State_return},
  {State_fault}
};
Sys_State current_sys_state=STATE_RETURN;//定义当前系统状态 初始为回零状态
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  Base_Init();
  Arm_Init();
  HAL_TIM_Base_Start_IT(&htim3);//开启时基
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //===
    if(f_1ms){
      tick++;
      f_1ms=0;

      if(tick%1000==0){//LED以1Hz闪烁
        HAL_GPIO_TogglePin(CHECK_GPIO_Port, CHECK_Pin);
      }
      //每1ms要执行的-------------------

      

      //-------------------------------
      
      //200Hz插补周期-------------------
      if(tick%5==0){
      //------------测试运行的代码
      //   Arm_Update();

      //   static uint8_t test_step = 0;
      //   static uint16_t step_timer = 0;
      //   step_timer += 5;

      //   // --- 动作 0：全部回中 (持续 2 秒) ---
      //   if(test_step == 0) {
      //       if(step_timer >= 2000) {
      //           // 1~3号走 S 曲线，底座转动（假设要往正方向转）
      //           Arm_SetTarget(450, -450, 0); 
                
      //           // 调用底座：相对转 0度，最大速度偏移给 80（大概 80*5 = 400us）
      //           // 如果你希望底座只在这个动作转 1 圈，可以通过校准得到角度值
      //           // 但我们现在只是演示，让它在这个 1.2 秒的动作里转一下
      //           Base_MoveAngle(360, 80); 
                
      //           test_step = 1;
      //           step_timer = 0;
      //       }
      //   }
      //   // --- 动作 1：大角度展开 (持续 2 秒) ---
      //   else if(test_step == 1) {
      //       if(step_timer >= 2000) {
      //           Arm_SetTarget(0, 0, 900);    
      //           Base_MoveAngle(-360, 80); // 反转回来
                
      //           test_step = 2;
      //           step_timer = 0;
      //       }
      //   }
      } 
      //----------------------------------------------

      //50Hz状态上报--------------------
      if(tick%20==0){

        //按键翻转LED
        uint8_t current_key=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
        if(current_key==GPIO_PIN_RESET && key_state==GPIO_PIN_SET){//下降沿
          HAL_GPIO_TogglePin(CHECK_GPIO_Port, CHECK_Pin);
        }
        key_state=current_key;
      }//------------------------------
    
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){//1ms节拍中断
  if(htim==&htim3){
    f_1ms=1;//把1ms标志位置1
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
