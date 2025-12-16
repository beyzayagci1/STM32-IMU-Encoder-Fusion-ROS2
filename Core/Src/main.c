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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

// ROS 2 Mesajları
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/quaternion.h>
#include <nav_msgs/msg/odometry.h>

// --- AYARLAR ---
#define WHEEL_RADIUS 0.033f  // 3.3 cm
#define WHEEL_BASE   0.20f   // 20 cm
#define PPR_LEFT     80.0f   // KY-040 (20x4)
#define PPR_RIGHT    1600.0f // E38S6G5 (400x4)
#define BNO055_ADDR  (0x28 << 1)
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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
// --- DONANIM HANDLERLARI ---
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2; // Sol Enkoder (32-bit)
extern TIM_HandleTypeDef htim3; // Sağ Enkoder (16-bit) - PWM DEĞİL ARTIK!

// --- MICRO-ROS NESNELERİ ---
rcl_publisher_t imu_pub;
rcl_publisher_t heading_pub;
rcl_publisher_t odom_pub;
rcl_publisher_t speed_pub; // Debug için

sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Quaternion heading_msg;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Float32 speed_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// --- ROBOT DURUMU (ODOMETRİ) ---
float robot_x = 0.0f;
float robot_y = 0.0f;

// Enkoder Takip
uint32_t prev_enc_L = 0;
uint32_t prev_enc_R = 0;
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// BNO055'i NDOF (Füzyon) Moduna Alma
void BNO055_Init_NDOF() {
    uint8_t cmd[2];
    // Config Moduna Geç
    cmd[0] = 0x3D; cmd[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, BNO055_ADDR, cmd, 2, 100);
    HAL_Delay(30);

    // NDOF Modunu Seç (Sensör içi füzyon)
    cmd[0] = 0x3D; cmd[1] = 0x0C;
    HAL_I2C_Master_Transmit(&hi2c1, BNO055_ADDR, cmd, 2, 100);
    HAL_Delay(30);
}

// Quaternion Verisini Okuma
void BNO055_ReadQuaternion(float* q) {
    uint8_t reg = 0x20; // Quaternion Data Başlangıç Register'ı
    uint8_t data[8];

    HAL_I2C_Master_Transmit(&hi2c1, BNO055_ADDR, &reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, BNO055_ADDR, data, 8, 100);

    int16_t w = (data[1] << 8) | data[0];
    int16_t x = (data[3] << 8) | data[2];
    int16_t y = (data[5] << 8) | data[4];
    int16_t z = (data[7] << 8) | data[6];

    // 1 Quaternion = 2^14 LSB (Datasheet bilgisi)
    const float scale = 1.0f / 16384.0f;
    q[0] = w * scale;
    q[1] = x * scale;
    q[2] = y * scale;
    q[3] = z * scale;
}

// Quaternion'dan Yaw (Z ekseni açısı) Hesaplama
float GetYaw(float w, float x, float y, float z) {
    return atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
	  /* USER CODE BEGIN TIM2_Init 0 */
	  __HAL_RCC_TIM2_CLK_ENABLE(); // ✅ Clock'u aktifleştir
  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_SCK_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(SPI1_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */

  // 1. Transport Ayarı
  rmw_uros_set_custom_transport(
    true, NULL,
    cubemx_transport_open, cubemx_transport_close,
    cubemx_transport_write, cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Allocator Hatasi!\n");
  }

  // Ajanı Bekle
  while(rmw_uros_ping_agent(100, 1) != RMW_RET_OK) { osDelay(100); }

  // 2. Micro-ROS Başlat
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "stm32_fusion_node", "", &support);

  // --- PUBLISHER TANIMLARI ---
  // A) IMU Verisi
  rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "raw_imu");
  // B) Heading (Yönelim)
  rclc_publisher_init_default(&heading_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion), "fused_heading");
  // C) Robot State (Odometri)
  rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "robot_state");
  // D) Debug Hız
  rclc_publisher_init_default(&speed_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel_speed");

  // String alanları için statik bufferlar tanımla
  static char imu_frame[] = "imu_link";
  static char odom_frame[] = "odom";
  static char base_frame[] = "base_link";

  // IMU mesajı için frame_id
  imu_msg.header.frame_id.data = imu_frame;
  imu_msg.header.frame_id.size = strlen(imu_frame);
  imu_msg.header.frame_id.capacity = strlen(imu_frame) + 1;

  // Odometry mesajı için frame_id'ler
  odom_msg.header.frame_id.data = odom_frame;
  odom_msg.header.frame_id.size = strlen(odom_frame);
  odom_msg.header.frame_id.capacity = strlen(odom_frame) + 1;

  odom_msg.child_frame_id.data = base_frame;
  odom_msg.child_frame_id.size = strlen(base_frame);
  odom_msg.child_frame_id.capacity = strlen(base_frame) + 1;

  // --- 3. DONANIMI BAŞLAT ---
  BNO055_Init_NDOF(); // IMU Başlat

  // Enkoderleri Başlat (PWM YOK! Sadece Encoder Mode)
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Sol (KY-040)
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Sağ (E38S6G5)

  // Sayaçları Ortala (Taşma yönetimi için)
  __HAL_TIM_SET_COUNTER(&htim2, 2000000000); // TIM2 (32-bit)
  prev_enc_L = 2000000000;

  __HAL_TIM_SET_COUNTER(&htim3, 30000);      // TIM3 (16-bit)
  prev_enc_R = 30000;

  float q[4]; // Quaternion [w, x, y, z]
  uint32_t last_time = HAL_GetTick();

  // --- SONSUZ DÖNGÜ ---
  for(;;)
  {
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time) / 1000.0f;
    if (dt < 0.01f) { osDelay(1); continue; }
    last_time = now;

    // --- 1. ENKODER OKUMA ---
    uint32_t curr_L = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t curr_R = __HAL_TIM_GET_COUNTER(&htim3);

    // Sol (32-bit): Direkt fark
    int32_t diff_L = (int32_t)(curr_L - prev_enc_L);

    // Sağ (16-bit): Taşmayı yönetmek için önce 16-bite cast et
    int16_t diff_R_16 = (int16_t)(curr_R - prev_enc_R);
    int32_t diff_R = (int32_t)diff_R_16;

    prev_enc_L = curr_L;
    prev_enc_R = curr_R;

    // --- 2. KİNEMATİK (HIZ HESABI) ---
    // Her tekerleğin gittiği mesafe (metre)
    float dist_L = (diff_L / PPR_LEFT) * (2 * M_PI * WHEEL_RADIUS);
    float dist_R = (diff_R / PPR_RIGHT) * (2 * M_PI * WHEEL_RADIUS);

    // Ortalama Lineer Hız
    float v = (dist_L + dist_R) / 2.0f / dt;

    // Hızı yayınla (Debug)
    speed_msg.data = v;
    rcl_publish(&speed_pub, &speed_msg, NULL);

    // --- 3. IMU VE FÜZYON ---
    BNO055_ReadQuaternion(q);
    float yaw = GetYaw(q[0], q[1], q[2], q[3]);

    // Odometri Güncelle: Konum = Eski + (Hız * Yön)
    robot_x += v * dt * cosf(yaw);
    robot_y += v * dt * sinf(yaw);

    // --- 4. YAYINLAMA ---

    // A) raw_imu
    imu_msg.header.stamp.sec = now / 1000;
    imu_msg.header.stamp.nanosec = (now % 1000) * 1000000;
    imu_msg.orientation.w = q[0];
    imu_msg.orientation.x = q[1];
    imu_msg.orientation.y = q[2];
    imu_msg.orientation.z = q[3];

    rcl_ret_t ret1 = rcl_publish(&imu_pub, &imu_msg, NULL);

    // B) fused_heading
    heading_msg = imu_msg.orientation;
    rcl_ret_t ret2 = rcl_publish(&heading_pub, &heading_msg, NULL);

    // C) robot_state - ÖNEMLİ: Her yayınlamadan önce frame_id'leri yeniden ata
    odom_msg.header.stamp.sec = now / 1000;
    odom_msg.header.stamp.nanosec = (now % 1000) * 1000000;

    // Frame ID'leri tekrar ata (kritik!)
    odom_msg.header.frame_id.data = odom_frame;
    odom_msg.header.frame_id.size = strlen(odom_frame);

    odom_msg.child_frame_id.data = base_frame;
    odom_msg.child_frame_id.size = strlen(base_frame);

    // Pose bilgisi
    odom_msg.pose.pose.position.x = robot_x;
    odom_msg.pose.pose.position.y = robot_y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.w = q[0];
    odom_msg.pose.pose.orientation.x = q[1];
    odom_msg.pose.pose.orientation.y = q[2];
    odom_msg.pose.pose.orientation.z = q[3];

    // Twist bilgisi
    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0; // İsterseniz jiroskop verisini ekleyin

    rcl_ret_t ret3 = rcl_publish(&odom_pub, &odom_msg, NULL);

    // Hata kontrolü (debug için)
    if(ret3 != RCL_RET_OK) {
        // LED yakarak veya başka şekilde hata gösterin
        HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET); // Kırmızı LED
    } else {
        HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);
    }

    osDelay(10);
  }
  /* USER CODE END 5 */


}

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
