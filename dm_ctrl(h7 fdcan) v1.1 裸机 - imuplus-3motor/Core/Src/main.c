/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_fdcan.h"
#include "dm_motor_ctrl.h"
#include <math.h> 
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

uint8_t rx_buffer[1];       // 接收缓冲
uint8_t data_buffer[11];    // 协议包缓冲
uint8_t rx_cnt = 0;         // 计数

// --- 物理量定义 ---
volatile float acc[3];        // 原始加速度 (含重力)
volatile float angle[3];      // 角度 (Roll, Pitch, Yaw)
volatile float linear_acc[3]; // 纯运动加速度 (去除重力)

// 计数器
volatile uint32_t update_counter = 0;

// --- 多电机管理数组 ---
// 下标0->Motor1(X), 下标1->Motor2(Y), 下标2->Motor3(Z)
volatile float Motor_Zero_Offset[4] = {0.0f}; 
volatile uint8_t Motor_Is_Zeroed[4] = {0};    
volatile float Motor_Show_Pos[4]    = {0.0f}; 

// --- 参数设置 (数组化，方便三轴独立调试) ---
// 建议调试顺序：先调X，把Y/Z的KP设为0；然后调Y...
double Kp_Vib[3] = {0, -0.1, 0}; // 三轴抑振力度0.1 0.1 0.1
double Kv_damp[3] = {0.0f,0.005f,0.0f};//三轴抑振速度阻尼

// 软限位范围 (虚拟墙开始介入的位置)
const float SOFT_LIMIT_RAD[3] = {1.0f, 1.5f, 1.0f}; 

// 虚拟墙参数
const float WALL_K_SPRING[3]  = {0.05f, 0.05f, 0.05f};//{0.05f, 0.05f, 0.05f}  
const float WALL_K_DAMP[3]    = {0.01f, 0.05f, 0.01f};//{0.01f, 0.01f, 0.01f}

// 物理最大力矩保护
const float MAX_TORQUE = 1.0f;   

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
  * @brief  FDCAN FIFO0 接收中断回调函数
  * @note   这里简单粗暴地根据 CAN ID 判断是哪个电机
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            int motor_idx = -1;

            // --- 简单判断 ID (假设反馈ID是 0x11, 0x12, 0x13) ---
            // 注意：这里取决于电机反馈的 ID 是 Master ID 还是 CAN ID
            // 既然你之前单轴用的是 0x11 (mst_id)，这里我按 0x11, 0x12, 0x13 来写
            if      (RxHeader.Identifier == 0x11) motor_idx = Motor1; // 0
            else if (RxHeader.Identifier == 0x12) motor_idx = Motor2; // 1
            else if (RxHeader.Identifier == 0x13) motor_idx = Motor3; // 2

            // 如果匹配到了有效的电机
            if (motor_idx >= 0)
            {
                dm_motor_fbdata(&motor[motor_idx], RxData); // 解析数据

                // --- 自动记录上电零点 (每个电机独立记录) ---
                if (Motor_Is_Zeroed[motor_idx] == 0)
                {
                    Motor_Zero_Offset[motor_idx] = motor[motor_idx].para.pos; 
                    Motor_Is_Zeroed[motor_idx] = 1; 
                }
                
                motor[motor_idx].tmp.read_flag = 1; 
            }
        }
    }
}

// 错误处理回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart7, rx_buffer, 1);
    }
}

// 定时器回调（这里暂时不用发指令，指令移到主循环统一发）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
    if (htim->Instance == TIM3) {
        // 空置，主循环控制
    }
}

// 串口接收中断回调（保持原样，计算 Acc）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7) 
    {   
        uint8_t received_byte = rx_buffer[0];

        if (rx_cnt == 0 && received_byte != 0x55) {
            HAL_UART_Receive_IT(&huart7, rx_buffer, 1);
            return;
        }

        data_buffer[rx_cnt++] = received_byte;

        if (rx_cnt >= 11) {
            rx_cnt = 0;
            uint8_t sum = 0;
            for (int i = 0; i < 10; i++) sum += data_buffer[i];

            if (sum == data_buffer[10]) 
            {
                if (data_buffer[1] == 0x51) 
                {
                    short ax_raw = (short)((data_buffer[3] << 8) | data_buffer[2]);
                    short ay_raw = (short)((data_buffer[5] << 8) | data_buffer[4]);
                    short az_raw = (short)((data_buffer[7] << 8) | data_buffer[6]);
                    acc[0] = (float)ax_raw / 32768.0f * 16.0f;
                    acc[1] = (float)ay_raw / 32768.0f * 16.0f;
                    acc[2] = (float)az_raw / 32768.0f * 16.0f;
                }
                else if (data_buffer[1] == 0x53) 
                {
                    short roll_raw  = (short)((data_buffer[3] << 8) | data_buffer[2]);
                    short pitch_raw = (short)((data_buffer[5] << 8) | data_buffer[4]);
                    short yaw_raw   = (short)((data_buffer[7] << 8) | data_buffer[6]);
                    angle[0] = (float)roll_raw / 32768.0f * 180.0f; 
                    angle[1] = (float)pitch_raw / 32768.0f * 180.0f; 
                    angle[2] = (float)yaw_raw / 32768.0f * 180.0f; 
                    
                    // 去重力计算
                    float rad_roll  = angle[0] * 0.0174533f;
                    float rad_pitch = angle[1] * 0.0174533f;
                    
                    float g_x = -sinf(rad_pitch);
                    float g_y = cosf(rad_pitch) * sinf(rad_roll);
                    float g_z = cosf(rad_pitch) * cosf(rad_roll);
                    
                    linear_acc[0] = acc[0] - g_x;
                    linear_acc[1] = acc[1] - g_y;
                    linear_acc[2] = acc[2] - g_z;
                    
                    update_counter++;
                }
            }
        }
        HAL_UART_Receive_IT(&huart7, rx_buffer, 1);
    }
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_UART7_Init();
  MX_USART10_UART_Init();
  
  /* USER CODE BEGIN 2 */
    
    // --- 1. 电源配置 ---
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE(); 
    GPIO_InitStruct.Pin = GPIO_PIN_15; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    power(1);        
    power_2(1);
    power_5v(1);
    HAL_Delay(1000); 
      
    // --- 2. 传感器配置 ---
    uint8_t cfg_200hz[] = {0xFF, 0xAA, 0x03, 0x0B, 0x00};
    HAL_UART_Transmit(&huart7, cfg_200hz, 5, 100);
    HAL_Delay(100); 

    // --- 3. 电机初始化 (3个电机) ---
    bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M); 
    bsp_can_init();
    dm_motor_init(); // 初始化结构体
    HAL_Delay(10);
    
    // 开启中断
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    // 依次使能三个电机
    dm_motor_enable(&hfdcan1, &motor[Motor1]); HAL_Delay(100);
    dm_motor_enable(&hfdcan1, &motor[Motor2]); HAL_Delay(100);
    dm_motor_enable(&hfdcan1, &motor[Motor3]); HAL_Delay(100);
    
    HAL_TIM_Base_Start_IT(&htim3); 

    // --- 4. 传感器启动 ---
    HAL_UART_Transmit(&huart10, (uint8_t*)"System Start: 3-Axis Control\r\n", 30, 100);
    
    __HAL_UART_CLEAR_OREFLAG(&huart7);
    HAL_UART_Receive_IT(&huart7, rx_buffer, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // ============================================
    // === 循环处理 3 个轴 (i=0:X, i=1:Y, i=2:Z) ===
    // ============================================
    for (int i = 0; i < 3; i++)
    {
        Motor_Show_Pos[i] = motor[i].para.pos - Motor_Zero_Offset[i];
        
        float cur_pos = Motor_Show_Pos[i];
        float cur_vel = motor[i].para.vel;
        float tor_final = 0.0f;

        float tor_vib = Kp_Vib[i] * linear_acc[i];
        float tor_damp = -Kv_damp[i] * cur_vel;
        
        // --- C. 虚拟墙力矩 ---
        float tor_wall = 0.0f;
        if (cur_pos > SOFT_LIMIT_RAD[i]) 
        {
            float depth = cur_pos - SOFT_LIMIT_RAD[i];
            tor_wall = -1.0f * (WALL_K_SPRING[i] * depth + WALL_K_DAMP[i] * cur_vel);
        }
        else if (cur_pos < -SOFT_LIMIT_RAD[i]) 
        {
            float depth = cur_pos - (-SOFT_LIMIT_RAD[i]); 
            tor_wall = -1.0f * (WALL_K_SPRING[i] * depth + WALL_K_DAMP[i] * cur_vel);
        }
        
        // --- D. 合成 ---
        tor_final = tor_vib + tor_damp + tor_wall ;
        
        // --- E. 限幅 ---
        if (tor_final > MAX_TORQUE)  tor_final = MAX_TORQUE;
        if (tor_final < -MAX_TORQUE) tor_final = -MAX_TORQUE;
        // --- F. 发送 ---
        motor[i].ctrl.tor_set = tor_final;
        
        // 只有读到数据了才发，或者无条件发(推荐无条件发，保证响应)
        dm_motor_ctrl_send(&hfdcan1, &motor[i]);
        

        
        motor[i].tmp.read_flag = 0;
    }
            // 每个电机发完歇 1ms，防止 CAN 拥堵
        HAL_Delay(1); 
    // --- 打印逻辑 (略微降频，只打印 Motor1 和 Acc 作参考，防止刷屏太快) ---
    if (update_counter >= 10) 
    {
        update_counter = 0;
        char print_buf[128];
sprintf(print_buf, 
        "=== 三轴完整数据 ===\r\n"
        "X轴: Pos=%.3f | Zero=%.3f | Show=%.3f | Tor=%.3f | Acc=%.3f\r\n"
        "Y轴: Pos=%.3f | Zero=%.3f | Show=%.3f | Tor=%.3f | Acc=%.3f\r\n"
        "Z轴: Pos=%.3f | Zero=%.3f | Show=%.3f | Tor=%.3f | Acc=%.3f\r\n"
        "=====================\r\n",
        // X轴
        motor[Motor1].para.pos, Motor_Zero_Offset[Motor1], Motor_Show_Pos[Motor1],
        motor[Motor1].ctrl.tor_set, linear_acc[0],
        // Y轴
        motor[Motor2].para.pos, Motor_Zero_Offset[Motor2], Motor_Show_Pos[Motor2],
        motor[Motor2].ctrl.tor_set, linear_acc[1],
        // Z轴
        motor[Motor3].para.pos, Motor_Zero_Offset[Motor3], Motor_Show_Pos[Motor3],
        motor[Motor3].ctrl.tor_set, linear_acc[2]);
        
        HAL_UART_Transmit(&huart10, (uint8_t*)print_buf, strlen(print_buf), 50);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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