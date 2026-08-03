/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F4 car controller with text and 0x42 binary protocols
  ******************************************************************************
  * Binary protocol: 0x42 header + address + total length + payload + checksum
  * Car control: ADDR=1, total length=10 (header/address/length/speed/servo/checksum)
  * Heartbeat: ADDR=0, total length=4
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define USB_FRAME_HEAD     0x42
#define USB_FRAME_LENMIN   4
#define USB_FRAME_LENMAX   12
#define USB_ADDR_CARCTRL   1
#define USB_ADDR_BUZZER    4
#define USB_ADDR_HEART     0

typedef struct
{
    uint8_t recvStart;
    uint8_t index;
    uint8_t rxBuf[USB_FRAME_LENMAX];
}SerialFrame_t;

// Byte conversion helpers for 32-bit and 16-bit values
typedef union{
    uint8_t buff[4];
    float   f32;
    int32_t i32;
}Bit32Union_t;

typedef union{
    uint8_t buff[2];
    uint16_t u16;
    int16_t  i16;
}Bit16Union_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_SPEED_PERCENT 100
#define PWM_TIM8_PERIOD   1600
#define PWM_TIM1_PERIOD   20000
#define SERVO_MIN_PULSE   500
#define SERVO_MAX_PULSE   2500
#define UART_TX_DELAY     100
#define UART_RX_QUEUE_SIZE 128

#define STEER_INPUT_MIN  -50
#define STEER_INPUT_MAX   50
#define SERVO_ANG_LEFT_LIMIT  140
#define SERVO_ANG_RIGHT_LIMIT 40
#define SERVO_ANG_MID         90
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Text M/T command buffer
uint8_t rx_buffer[32];
uint8_t rx_index = 0;
uint8_t rx_data;
volatile uint8_t uart_rx_queue[UART_RX_QUEUE_SIZE];
volatile uint16_t uart_rx_head = 0;
volatile uint16_t uart_rx_tail = 0;
volatile uint8_t uart_rx_overflow = 0;

// Binary frame parser state
SerialFrame_t serialFrame;

// Current car state
int16_t current_speed = 0;
int16_t current_steer_input = 0;
uint16_t current_angle = 90;
uint16_t servoPwmSet = 1500;

// Motor driver pins
#define LEFT_MOTOR_IN1_PIN  GPIO_PIN_11
#define LEFT_MOTOR_IN2_PIN  GPIO_PIN_12
#define LEFT_MOTOR_IN1_PORT GPIOA
#define LEFT_MOTOR_IN2_PORT GPIOA

#define RIGHT_MOTOR_IN1_PIN  GPIO_PIN_8
#define RIGHT_MOTOR_IN2_PIN  GPIO_PIN_9
#define RIGHT_MOTOR_IN1_PORT GPIOC
#define RIGHT_MOTOR_IN2_PORT GPIOC

// PWM channels
#define LEFT_PWM_CHANNEL   TIM_CHANNEL_1
#define RIGHT_PWM_CHANNEL  TIM_CHANNEL_2
#define SERVO_PWM_CHANNEL  TIM_CHANNEL_1

// Startup message
uint8_t init_ok_msg[] = "\r\n==== Car Init OK ====\r\nText Cmd:Mxx/Txx\r\nBinary Frame Head=0x42\r\nTest Frame:42 01 0A 00 00 00 00 DC 05 00\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void set_motor_speed(int16_t speed_percent);
void set_servo_steer(int16_t steer_val);
void set_servo_by_pwm(uint16_t pwmUs);
void process_command(char *cmd);
void SerialFrameParse(uint8_t byte);
void FrameDataHandle(void);
void process_received_byte(uint8_t byte);
void reset_protocol_parsers(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_motor_speed(int16_t speed_percent)
{
    if (speed_percent > MAX_SPEED_PERCENT) speed_percent = MAX_SPEED_PERCENT;
    if (speed_percent < -MAX_SPEED_PERCENT) speed_percent = -MAX_SPEED_PERCENT;
    uint16_t duty = (uint16_t)(abs(speed_percent) * (PWM_TIM8_PERIOD - 1) / MAX_SPEED_PERCENT);

    if (speed_percent > 0)
    {
        HAL_GPIO_WritePin(LEFT_MOTOR_IN1_PORT, LEFT_MOTOR_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEFT_MOTOR_IN2_PORT, LEFT_MOTOR_IN2_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RIGHT_MOTOR_IN1_PORT, RIGHT_MOTOR_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RIGHT_MOTOR_IN2_PORT, RIGHT_MOTOR_IN2_PIN, GPIO_PIN_RESET);
    }
    else if (speed_percent < 0)
    {
        HAL_GPIO_WritePin(LEFT_MOTOR_IN1_PORT, LEFT_MOTOR_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEFT_MOTOR_IN2_PORT, LEFT_MOTOR_IN2_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RIGHT_MOTOR_IN1_PORT, RIGHT_MOTOR_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RIGHT_MOTOR_IN2_PORT, RIGHT_MOTOR_IN2_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(LEFT_MOTOR_IN1_PORT, LEFT_MOTOR_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEFT_MOTOR_IN2_PORT, LEFT_MOTOR_IN2_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RIGHT_MOTOR_IN1_PORT, RIGHT_MOTOR_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RIGHT_MOTOR_IN2_PORT, RIGHT_MOTOR_IN2_PIN, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim8, LEFT_PWM_CHANNEL, duty);
    __HAL_TIM_SET_COMPARE(&htim8, RIGHT_PWM_CHANNEL, duty);
    current_speed = speed_percent;
}

void set_servo_steer(int16_t steer_val)
{
    if (steer_val < STEER_INPUT_MIN) steer_val = STEER_INPUT_MIN;
    if (steer_val > STEER_INPUT_MAX) steer_val = STEER_INPUT_MAX;
    int32_t delta_input = steer_val - STEER_INPUT_MIN;
    int32_t total_input_range = STEER_INPUT_MAX - STEER_INPUT_MIN;
    int32_t total_angle_range = SERVO_ANG_LEFT_LIMIT - SERVO_ANG_RIGHT_LIMIT;
    uint16_t angle = SERVO_ANG_LEFT_LIMIT - (delta_input * total_angle_range) / total_input_range;
    if (angle > SERVO_ANG_LEFT_LIMIT) angle = SERVO_ANG_LEFT_LIMIT;
    if (angle < SERVO_ANG_RIGHT_LIMIT) angle = SERVO_ANG_RIGHT_LIMIT;

    uint16_t pulse = SERVO_MIN_PULSE + (uint16_t)((uint32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180);
    if (pulse > PWM_TIM1_PERIOD) pulse = PWM_TIM1_PERIOD;
    __HAL_TIM_SET_COMPARE(&htim1, SERVO_PWM_CHANNEL, pulse);
    current_steer_input = steer_val;
    current_angle = angle;
    servoPwmSet = pulse;
}

void set_servo_by_pwm(uint16_t pwmUs)
{
    if(pwmUs < SERVO_MIN_PULSE) pwmUs = SERVO_MIN_PULSE;
    if(pwmUs > SERVO_MAX_PULSE) pwmUs = SERVO_MAX_PULSE;
    servoPwmSet = pwmUs;
    __HAL_TIM_SET_COMPARE(&htim1, SERVO_PWM_CHANNEL, pwmUs);
    current_angle = (uint16_t)((uint32_t)(pwmUs - SERVO_MIN_PULSE) * 180 / (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
}

void process_command(char *cmd)
{
    while (*cmd == ' ') cmd++;
    if (*cmd == '\0') return;
    char echo_buf[128] = {0};

    if (cmd[0] == 'M' || cmd[0] == 'm')
    {
        int16_t speed = atoi(cmd + 1);
        set_motor_speed(speed);
        uint16_t duty = abs(current_speed)*(PWM_TIM8_PERIOD-1)/100;
        sprintf(echo_buf, "[TEXT OK] Speed=%d%% PWM=%d\r\n", current_speed, duty);
        HAL_UART_Transmit(&huart1, (uint8_t *)echo_buf, strlen(echo_buf), UART_TX_DELAY);
    }
    else if (cmd[0] == 'T' || cmd[0] == 't')
    {
        int16_t steer_in = atoi(cmd + 1);
        set_servo_steer(steer_in);
        sprintf(echo_buf, "[TEXT OK] SteerIn=%d Angle=%d PWM=%dus\r\n", current_steer_input, current_angle, servoPwmSet);
        HAL_UART_Transmit(&huart1, (uint8_t *)echo_buf, strlen(echo_buf), UART_TX_DELAY);
    }
    else
    {
        strcpy(echo_buf, "[ERR Cmd Use Mxx/Txx\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)echo_buf, strlen(echo_buf), UART_TX_DELAY);
    }
}

/**
 * @brief Binary frame parser; rxBuf[2] is the total frame length
 */
void reset_protocol_parsers(void)
{
    rx_index = 0;
    serialFrame.recvStart = 0;
    serialFrame.index = 0;
    memset(serialFrame.rxBuf, 0, sizeof(serialFrame.rxBuf));
}

void process_received_byte(uint8_t byte)
{
    if(serialFrame.recvStart || byte == USB_FRAME_HEAD)
    {
        SerialFrameParse(byte);
        return;
    }

    if(rx_index >= sizeof(rx_buffer) - 1)
    {
        rx_index = 0;
    }

    if(byte == '\r' || byte == '\n')
    {
        if(rx_index > 0)
        {
            rx_buffer[rx_index] = 0;
            process_command((char *)rx_buffer);
        }
        rx_index = 0;
    }
    else
    {
        rx_buffer[rx_index++] = byte;
    }
}
void SerialFrameParse(uint8_t byte)
{
    // Debug output intentionally disabled
    // char debugBuf[80];
    // sprintf(debugBuf, "[DBG Byte:0x%02X]\r\n", byte);
    // HAL_UART_Transmit(&huart1, (uint8_t *)debugBuf, strlen(debugBuf), UART_TX_DELAY);

    uint8_t totalFrameLen;
    if(!serialFrame.recvStart)
    {
        if(byte == USB_FRAME_HEAD)
        {
            serialFrame.recvStart = 1;
            serialFrame.index = 0;
            serialFrame.rxBuf[serialFrame.index++] = byte;
        }
        return;
    }

    if(serialFrame.index >= USB_FRAME_LENMAX)
    {
        serialFrame.recvStart = 0;
        serialFrame.index = 0;
        memset(serialFrame.rxBuf,0,USB_FRAME_LENMAX);
        return;
    }

    serialFrame.rxBuf[serialFrame.index++] = byte;

    if(serialFrame.index >= 3)
    {
        totalFrameLen = serialFrame.rxBuf[2];
        if(totalFrameLen > USB_FRAME_LENMAX || totalFrameLen < USB_FRAME_LENMIN)
        {
            serialFrame.recvStart = 0;
            serialFrame.index = 0;
            memset(serialFrame.rxBuf,0,USB_FRAME_LENMAX);
            return;
        }
        if(serialFrame.index >= totalFrameLen)
        {
            uint8_t checkSum = 0;
            for(uint8_t i=0; i < totalFrameLen-1; i++)
                checkSum += serialFrame.rxBuf[i];
            if(checkSum == serialFrame.rxBuf[totalFrameLen-1])
            {
                FrameDataHandle();
            }
            else
            {
                // Report checksum errors
                char debugBuf[80];
                sprintf(debugBuf,"[ERR CheckSum Calc:0x%02X Rx:0x%02X]\r\n",checkSum,serialFrame.rxBuf[totalFrameLen-1]);
                HAL_UART_Transmit(&huart1, (uint8_t *)debugBuf, strlen(debugBuf), UART_TX_DELAY);
            }
            serialFrame.recvStart = 0;
            serialFrame.index = 0;
            memset(serialFrame.rxBuf,0,USB_FRAME_LENMAX);
        }
    }
}
/**
 * @brief Handle one validated binary frame
 */
void FrameDataHandle(void)
{
    uint8_t addr = serialFrame.rxBuf[1];
    uint8_t frameLen = serialFrame.rxBuf[2];
    char txBuf[128] = {0};
    Bit32Union_t speedUnion;
    Bit16Union_t servoUnion;
    float speedMps;
    int16_t motorPercent;

    switch(addr)
    {
        case USB_ADDR_CARCTRL:
            if(frameLen != 10)
            {
                sprintf(txBuf, "[BIN ERR] Car frame length=%u, expected=10\r\n", frameLen);
                HAL_UART_Transmit(&huart1, (uint8_t *)txBuf, strlen(txBuf), UART_TX_DELAY);
                break;
            }
            // Payload starts at index 3: 4-byte speed and 2-byte servo value
            memcpy(speedUnion.buff, &serialFrame.rxBuf[3], 4);
            memcpy(servoUnion.buff, &serialFrame.rxBuf[7], 2);
            speedMps = speedUnion.f32;
            if(speedMps > 1.0f) speedMps = 1.0f;
            if(speedMps < -1.0f) speedMps = -1.0f;
            motorPercent = (int16_t)(speedMps * 100.0f);
            set_motor_speed(motorPercent);
            set_servo_by_pwm(servoUnion.u16);
            sprintf(txBuf,"[BIN OK] Spd=%.2fm/s Motor=%d%% Servo=%dus Angle=%d deg\r\n",
                speedMps,motorPercent,servoUnion.u16,current_angle);
            HAL_UART_Transmit(&huart1, (uint8_t *)txBuf, strlen(txBuf), UART_TX_DELAY);
            break;
        case USB_ADDR_HEART:
            if(frameLen != 4)
            {
                sprintf(txBuf, "[BIN ERR] Heart frame length=%u, expected=4\r\n", frameLen);
                HAL_UART_Transmit(&huart1, (uint8_t *)txBuf, strlen(txBuf), UART_TX_DELAY);
                break;
            }
            strcpy(txBuf,"[BIN Heart Recv]\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)txBuf, strlen(txBuf), UART_TX_DELAY);
            break;
        default:
            sprintf(txBuf,"[BIN Unknown ADDR:%d]\r\n",addr);
            HAL_UART_Transmit(&huart1, (uint8_t *)txBuf, strlen(txBuf), UART_TX_DELAY);
            break;
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
  memset(&serialFrame,0,sizeof(SerialFrame_t));
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
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, SERVO_PWM_CHANNEL);
  HAL_TIM_PWM_Start(&htim8, LEFT_PWM_CHANNEL);
  HAL_TIM_PWM_Start(&htim8, RIGHT_PWM_CHANNEL);
  set_motor_speed(0);
  set_servo_by_pwm(1500);
  HAL_UART_Transmit(&huart1, init_ok_msg, sizeof(init_ok_msg)-1, UART_TX_DELAY);
  // Start one-byte interrupt-driven reception
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

/* USER CODE BEGIN 3 */
    if(uart_rx_overflow)
    {
      __disable_irq();
      uart_rx_tail = uart_rx_head;
      uart_rx_overflow = 0;
      __enable_irq();
      reset_protocol_parsers();
      const char overflowMsg[] = "[ERR UART RX overflow]\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t *)overflowMsg, sizeof(overflowMsg) - 1, UART_TX_DELAY);
    }
    while(uart_rx_tail != uart_rx_head)
    {
      uint8_t byte = uart_rx_queue[uart_rx_tail];
      uart_rx_tail = (uint16_t)((uart_rx_tail + 1U) % UART_RX_QUEUE_SIZE);
      process_received_byte(byte);
    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1599;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    uint16_t nextHead = (uint16_t)((uart_rx_head + 1U) % UART_RX_QUEUE_SIZE);
    if(nextHead != uart_rx_tail)
    {
      uart_rx_queue[uart_rx_head] = rx_data;
      uart_rx_head = nextHead;
    }
    else uart_rx_overflow = 1;
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
