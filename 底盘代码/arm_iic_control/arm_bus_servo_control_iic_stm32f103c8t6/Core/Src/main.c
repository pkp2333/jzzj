/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bus_servo_control.h"  // 假设包含 bus_servo_read、bus_servo_control 等函数声明
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint16_t set_position;  // 目标位置
    uint16_t time;          // 到达时间（ms）
    uint16_t real_position; // 实际位置
} Servo;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 8          // 串口接收缓冲区大小
#define READ_INTERVAL 20          // 高频读取间隔（50ms）
#define IIC_SLAVE                 // 使能 I2C 从机功能
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// 舵机位置数组（[0]对应UART2总线，[1]对应UART3总线；索引1-6为舵机ID）
Servo servo_positions[2][7] = {
    {{0,0,0}, {2000,0,0}, {2000,0,0}, {2000,0,0}, {2000,0,0}, {1486,0,0}, {3100,0,0}},  // UART2总线（索引0未使用）
    {{0,0,0}, {2000,0,0}, {2000,0,0}, {2000,0,0}, {2000,0,0}, {1486,0,0}, {3100,0,0}}   // UART3总线（索引0未使用）
};

uint8_t torque_enabled = 1;         // 扭矩使能标志
uint8_t action_group_running = 0;   // 动作组运行标志

// UART2 接收相关变量
uint8_t u2_rx_buffer[RX_BUFFER_SIZE];
uint8_t my_u2_rx_buffer[RX_BUFFER_SIZE];
uint8_t u2_rx_index = 0;

// UART3 接收相关变量
uint8_t u3_rx_buffer[RX_BUFFER_SIZE];
uint8_t my_u3_rx_buffer[RX_BUFFER_SIZE];
uint8_t u3_rx_index = 0;

// I2C 从机相关变量
static uint8_t current_i2c_address = 0;      // 当前匹配的I2C地址（0x10或0x11）
static uint8_t current_bus_servo_index = 0;  // 当前总线索引（0=UART2，1=UART3）
static uint8_t rx_ram[256];                  // I2C 接收缓冲区
static uint8_t tx_ram[256];                  // I2C 发送缓冲区
uint8_t rx_offset = 0;                       // 接收缓冲区偏移
uint8_t tx_offset = 0;                       // 发送缓冲区偏移
static uint8_t rx_count = 0;

// 舵机位置更新时间戳（防旧数据覆盖）
uint32_t servo2_last_update[7] = {0};  // UART2总线（索引1-6对应舵机ID）
uint32_t servo3_last_update[7] = {0};  // UART3总线（索引1-6对应舵机ID）


uint8_t huart2_torque_need_enabled = 0;
uint8_t huart2_torque_need_disable = 0;

uint8_t huart3_torque_need_enabled = 0;
uint8_t huart3_torque_need_disable = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/************************** UART2 中断回调（处理舵机位置接收） **************************/
void USER_UART2_rx_callback(void)
{
    // 接收当前字节到缓冲区
    u2_rx_buffer[u2_rx_index] = huart2.Instance->DR;
    u2_rx_index++;

    // 检查是否接收到有效头部（0xFF, 0xF5）
    if (u2_rx_index >= 2 && u2_rx_buffer[0] == 0xFF && u2_rx_buffer[1] == 0xF5)
    {
        // 完整数据包（8字节）
        if (u2_rx_index >= 8)
        {
            // 复制到临时缓冲区
            memcpy(my_u2_rx_buffer, u2_rx_buffer, 8);
            uint8_t id = my_u2_rx_buffer[2];  // 舵机ID（1-6）
            uint16_t raw_position = (my_u2_rx_buffer[5] << 8) | my_u2_rx_buffer[6];  // 解析位置值
            uint32_t current_time = HAL_GetTick();  // 获取当前时间戳

            // 数据有效性校验（0-4096）
            if (raw_position <= 4096)
            {
                // 仅当新数据时间戳更新时才覆盖（防旧数据）
                if (current_time > servo2_last_update[id])
                {
                    servo_positions[0][id].real_position = raw_position;
                    servo2_last_update[id] = current_time;  // 更新时间戳
                }
            }
            else
            {
                // 无效数据：重置缓冲区并重新读取该舵机
                u2_rx_index = 0;
                memset(u2_rx_buffer, 0, RX_BUFFER_SIZE);
                memset(my_u2_rx_buffer, 0, RX_BUFFER_SIZE);
                bus_servo_read(id, huart2);  // 重新发送读取命令
                return;
            }

            u2_rx_index = 0;  // 重置索引接收下一组数据
        }
    }
    // 缓冲区溢出处理（超过8字节未匹配头部）
    else if (u2_rx_index >= RX_BUFFER_SIZE)
    {
        u2_rx_index = 0;
        memset(u2_rx_buffer, 0, RX_BUFFER_SIZE);
    }

    // 继续接收下一个字节（防越界）
    if (u2_rx_index < RX_BUFFER_SIZE)
    {
        HAL_UART_Receive_IT(&huart2, &u2_rx_buffer[u2_rx_index], 1);
    }
    else
    {
        u2_rx_index = 0;
        HAL_UART_Receive_IT(&huart2, u2_rx_buffer, 1);
    }
}

/************************** UART3 中断回调（逻辑同UART2） **************************/
void USER_UART3_rx_callback(void)
{
    u3_rx_buffer[u3_rx_index] = huart3.Instance->DR;
    u3_rx_index++;

    if (u3_rx_index >= 2 && u3_rx_buffer[0] == 0xFF && u3_rx_buffer[1] == 0xF5)
    {
        if (u3_rx_index >= 8)
        {
            memcpy(my_u3_rx_buffer, u3_rx_buffer, 8);
            uint8_t id = my_u3_rx_buffer[2];
            uint16_t raw_position = (my_u3_rx_buffer[5] << 8) | my_u3_rx_buffer[6];
            uint32_t current_time = HAL_GetTick();

            if (raw_position <= 4096)
            {
                if (current_time > servo3_last_update[id])
                {
                    servo_positions[1][id].real_position = raw_position;
                    servo3_last_update[id] = current_time;
                }
            }
            else
            {
                u3_rx_index = 0;
                memset(u3_rx_buffer, 0, RX_BUFFER_SIZE);
                memset(my_u3_rx_buffer, 0, RX_BUFFER_SIZE);
                bus_servo_read(id, huart3);  // 重新读取该舵机
                return;
            }

            u3_rx_index = 0;
        }
    }
    else if (u3_rx_index >= RX_BUFFER_SIZE)
    {
        u3_rx_index = 0;
        memset(u3_rx_buffer, 0, RX_BUFFER_SIZE);
    }

    if (u3_rx_index < RX_BUFFER_SIZE)
    {
        HAL_UART_Receive_IT(&huart3, &u3_rx_buffer[u3_rx_index], 1);
    }
    else
    {
        u3_rx_index = 0;
        HAL_UART_Receive_IT(&huart3, u3_rx_buffer, 1);
    }
}

/************************** I2C 从机命令处理函数 **************************/
#ifdef IIC_SLAVE
void Process_I2C_Command(void)
{
    uint8_t current_register = rx_ram[0];  // 当前操作寄存器地址
    UART_HandleTypeDef *current_uart;      // 当前操作的UART句柄指针

    // 根据I2C地址选择对应的UART总线
    switch (current_i2c_address)
    {
        case 0x10:  // 地址0x10对应UART2总线
            current_uart = &huart2;
            current_bus_servo_index = 0;
            break;
        case 0x11:  // 地址0x11对应UART3总线
            current_uart = &huart3;
            current_bus_servo_index = 1;
            break;
        default:
            return;  // 无效地址，直接返回
    }

    // 处理不同寄存器命令
    switch (current_register)
    {
        // 控制单个舵机（0x11-0x16对应舵机ID1-6）
        case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16:
        {
            uint8_t servo_id = current_register - 0x11 + 1;  // 舵机ID（1-6）
            uint16_t set_pos = (rx_ram[1] << 8) | rx_ram[2];  // 目标位置
            uint16_t time = (rx_ram[3] << 8) | rx_ram[4];     // 到达时间
            servo_positions[current_bus_servo_index][servo_id].set_position = set_pos;
            servo_positions[current_bus_servo_index][servo_id].time = time;
            //bus_servo_control(servo_id, set_pos, time, *current_uart);  // 发送控制命令
            break;
        }

        // 控制所有舵机（0x17）
        case 0x17:
        {
            uint16_t set_pos = (rx_ram[1] << 8) | rx_ram[2];
            uint16_t time = (rx_ram[3] << 8) | rx_ram[4];
            for (uint8_t i = 1; i <= 6; i++)
            {
                servo_positions[current_bus_servo_index][i].set_position = set_pos;
                servo_positions[current_bus_servo_index][i].time = time;
            }
            //bus_servo_control(0xFE, set_pos, time, *current_uart);  // 广播控制所有舵机
            break;
        }

        // 控制指定舵机（0x19）
        case 0x19:
        {
            uint8_t servo_id = rx_ram[1];
            uint16_t set_pos = (rx_ram[2] << 8) | rx_ram[3];
            uint16_t time = (rx_ram[4] << 8) | rx_ram[5];
            servo_positions[current_bus_servo_index][servo_id].set_position = set_pos;
            servo_positions[current_bus_servo_index][servo_id].time = time;
            //bus_servo_control(servo_id, set_pos, time, *current_uart);
            break;
        }

        // 扭矩开关（0x1A）
        case 0x1A:
        {
            uint8_t enable = rx_ram[1];
            torque_enabled = enable;
			if (current_uart->Instance == USART2)
			{
				if (enable == 1)
				{
					huart2_torque_need_enabled = 1;
				}
				else if (enable == 0)
				{
					huart2_torque_need_disable = 1;
				}
			}
			
			if (current_uart->Instance == USART3)
			{
				if (enable == 1)
				{
					huart3_torque_need_enabled = 1;
				}
				else if (enable == 0)
				{
					huart3_torque_need_disable = 1;
				}
			}
			
            //bus_servo_Unloading(0xFE, enable, *current_uart);  // 广播控制所有舵机扭矩
            break;
        }

        // 读取单个舵机位置（0x31-0x36对应舵机ID1-6）
        case 0x31: case 0x32: case 0x33: case 0x34: case 0x35: case 0x36:
        {
            uint8_t servo_id = current_register - 0x31 + 1;
            tx_ram[1] = (servo_positions[current_bus_servo_index][servo_id].real_position >> 8) & 0xFF;  // 高8位
            tx_ram[2] = servo_positions[current_bus_servo_index][servo_id].real_position & 0xFF;         // 低8位
            tx_offset = 0;  // 重置发送偏移
            break;
        }

        // 读取指定舵机位置（0x37）
        case 0x37:
        {
            uint8_t servo_id = rx_ram[1];
            tx_ram[1] = (servo_positions[current_bus_servo_index][servo_id].real_position >> 8) & 0xFF;
            tx_ram[2] = servo_positions[current_bus_servo_index][servo_id].real_position & 0xFF;
            tx_offset = 0;
            break;
        }

        // 设置6路舵机时间（0x1E）
        case 0x1E:
        {
            uint16_t time = (rx_ram[1] << 8) | rx_ram[2];
            for (uint8_t i = 1; i <= 6; i++)
            {
                servo_positions[current_bus_servo_index][i].time = time;
            }
            break;
        }

        // 控制6路舵机（0x1D）
        case 0x1D:
        {
            for (uint8_t i = 1; i <= 6; i++)
            {
                uint16_t set_pos = (rx_ram[(i-1)*2 + 1] << 8) | rx_ram[(i-1)*2 + 2];
                servo_positions[current_bus_servo_index][i].set_position = set_pos;
                //bus_servo_control(i, set_pos, servo_positions[current_bus_servo_index][i].time, *current_uart);
            }
            break;
        }

        // 读取6路舵机位置（0x2D）
        case 0x2D:
        {
            for (uint8_t i = 1; i <= 6; i++)
            {
                tx_ram[(i-1)*2 + 1] = (servo_positions[current_bus_servo_index][i].real_position >> 8) & 0xFF;
                tx_ram[(i-1)*2 + 2] = servo_positions[current_bus_servo_index][i].real_position & 0xFF;
            }
            tx_offset = 0;
            break;
        }

        default:
            break;  // 未处理的寄存器，忽略
    }
}

/************************** I2C 从机监听完成回调 **************************/
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (rx_count > 0)
    {
        Process_I2C_Command();
    }
    rx_count = 0;
    rx_offset = 0;
    tx_offset = 0;
    (void)HAL_I2C_EnableListen_IT(hi2c);
}

/************************** I2C 地址匹配回调 **************************/
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    uint8_t received_addr = AddrMatchCode >> 1;  // 获取7位I2C地址（去掉R/W位）

    if (received_addr == ARM_I2C_ADDR_UART2)
    {
        current_i2c_address = ARM_I2C_ADDR_UART2;
    }
    else if (received_addr == ARM_I2C_ADDR_UART3)
    {
        current_i2c_address = ARM_I2C_ADDR_UART3;
    }
    else
    {
        return;  // 无效地址，直接返回
    }

    // 根据传输方向（主机发送/接收）启动对应操作
    if (TransferDirection == I2C_DIRECTION_TRANSMIT)
    {
        rx_offset = 0;
        rx_count = 0;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, &rx_ram[rx_offset], 1, I2C_NEXT_FRAME);
    }
    else
    {
        // 主机接收，从机发送：开始发送tx_ram[0]的数据
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &tx_ram[tx_offset], 1, I2C_NEXT_FRAME);
    }
}

/************************** I2C 从机接收完成回调 **************************/
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (rx_offset < sizeof(rx_ram) - 1)
    {
        rx_offset++;
        rx_count++;
    }
    // 继续接收下一个字节
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, &rx_ram[rx_offset], 1, I2C_NEXT_FRAME);
}

/************************** I2C 从机发送完成回调 **************************/
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (tx_offset < sizeof(tx_ram) - 1)
    {
        tx_offset++;  // 发送偏移递增
    }
    // 继续发送下一个字节
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &tx_ram[tx_offset], 1, I2C_NEXT_FRAME);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    rx_count = 0;
    rx_offset = 0;
    tx_offset = 0;

    /*
     * For an address probe, the master normally sends only the address and
     * then stops. HAL reports that as ACK failure and calls
     * HAL_I2C_ListenCpltCallback() after this callback. Let that callback
     * restart listen mode, otherwise the listen state is changed twice.
     */
    if ((hi2c->ErrorCode & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF)
    {
        return;
    }

    /* Recover from bus, overrun, or arbitration errors immediately. */
    (void)HAL_I2C_DisableListen_IT(hi2c);
    (void)HAL_I2C_EnableListen_IT(hi2c);
}
#endif  // IIC_SLAVE

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
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  (void)HAL_I2C_EnableListen_IT(&hi2c1);
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  BusServo_Init();  // 初始化舵机驱动（假设已实现）

  /* USER CODE BEGIN 2 */
  // 开启串口中断接收（UART2和UART3）
  HAL_UART_Receive_IT(&huart2, u2_rx_buffer, 1);
  HAL_UART_Receive_IT(&huart3, u3_rx_buffer, 1);

  // 初始加载扭矩（所有舵机）
  bus_servo_Unloading(0xFE, 0, huart2);
  bus_servo_Unloading(0xFE, 0, huart3);

  // 初始读取所有舵机位置（UART2和UART3）
  for (uint8_t i = 1; i <= 6; i++)
  {
      bus_servo_read(i, huart2);
      bus_servo_read(i, huart3);
      HAL_Delay(5);  // 防止命令冲突
  }

   // 初始加载扭矩（所有舵机）
  bus_servo_Unloading(0xFE, 0, huart2);
  bus_servo_Unloading(0xFE, 0, huart3);
  
	uint32_t last_read_time = 0;       // 上次读取时间戳
	uint8_t current_servo_id = 1;     // 当前读取的舵机ID（1-6循环）
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* USER CODE END WHILE */
	  if (huart2_torque_need_enabled == 1)
	  {
		bus_servo_Unloading(0xFE, 1, huart2);  // 广播控制所有舵机扭矩
		  huart2_torque_need_enabled = 0;
	  }
	  else if (huart2_torque_need_disable == 1)
	  {
		bus_servo_Unloading(0xFE, 0, huart2);  // 广播控制所有舵机扭矩
		  huart2_torque_need_disable = 0;
	  }
	  
	  
	  if (huart3_torque_need_enabled == 1)
	  {
		bus_servo_Unloading(0xFE, 1, huart3);  // 广播控制所有舵机扭矩
		  huart3_torque_need_enabled = 0;
	  }
	  else if (huart3_torque_need_disable == 1)
	  {
		bus_servo_Unloading(0xFE, 0, huart3);  // 广播控制所有舵机扭矩
		  huart3_torque_need_disable = 0;
	  }
	  
	  
	  
	  for (uint8_t i= 0;i< 2;i++)
	  {
		for(uint8_t j = 1;j<7;j++)
		  {
			  if (i ==0)
			  {
					bus_servo_control(j, servo_positions[i][j].set_position, servo_positions[i][j].time, huart2);
					HAL_Delay(5);
			  }
			  if (i == 1)
			  {
					bus_servo_control(j, servo_positions[i][j].set_position, servo_positions[i][j].time, huart3);
					HAL_Delay(5);
			  }
			  //HAL_Delay(5);
			
		  }
	  }
	// 初始读取所有舵机位置（UART2和UART3）
  for (uint8_t i = 1; i <= 6; i++)
  {
      bus_servo_read(i, huart2);
      HAL_Delay(5);  // 防止命令冲突
  }
  for(uint8_t i = 1; i<= 6; i++)
  {
	bus_servo_read(i, huart3);
    HAL_Delay(5);  // 防止命令冲突
  }
    /* USER CODE BEGIN 3 */
/* USER CODE BEGIN 3 */
	  
/* USER CODE END 3 */
    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
    // 错误处理（可添加LED闪烁、串口打印等）
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  // 断言失败处理（可添加串口打印）
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
