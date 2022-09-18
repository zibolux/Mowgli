/**
  ******************************************************************************
  * @file    main.c
  * @author  Georg Swoboda <cn@warp.at>
  * @brief   main / bootup and initialization, motor control routines, usb init
  ******************************************************************************
  * Version 1.0 
  * 
  * compile with -DBOARD_YARDFORCE500 to enable the YF500 GForce pinout
  * 
  * ROS integration howto taken from here: https://github.com/Itamare4/ROS_stm32f1_rosserial_USB_VCP (Itamar Eliakim)
  * 
  ******************************************************************************
  */

#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_adc.h"
#include "main.h"
// stm32 custom
#include "board.h"
#include "panel.h"
#include "emergency.h"
#include "blademotor.h"
#include "drivemotor.h"
#include "ultrasonic_sensor.h"
#include "soft_i2c.h"
#include "spiflash.h"
#include "i2c.h"
#include "imu/imu.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "nbt.h"

// ros
#include "cpp_main.h"
#include "ringbuffer.h"


static void WATCHDOG_vInit(void);
static void WATCHDOG_Refresh(void);

static nbt_t main_chargecontroller_nbt;
static nbt_t main_statusled_nbt;
static nbt_t main_emergency_nbt;
static nbt_t main_ultrasonicsensor_nbt;
static nbt_t main_blademotor_nbt;
static nbt_t main_drivemotor_nbt;
static nbt_t main_wdg_nbt;




// MASTER rx buffering
static uint8_t master_rcvd_data;
volatile uint8_t  master_rx_buf[32];
volatile uint32_t master_rx_buf_idx = 0;
volatile uint8_t  master_rx_buf_crc = 0;
volatile uint8_t  master_rx_LENGTH = 0;
volatile uint8_t  master_rx_CRC = 0;
volatile uint8_t  master_rx_STATUS = RX_WAIT;
// MASTER tx buffering
volatile uint8_t  master_tx_busy = 0;
static uint8_t master_tx_buffer_len;
static char master_tx_buffer[255];

#define ADC_NUMBER_MEASURE 80
typedef struct 
{
    uint16_t current_raw;
    uint16_t charge_voltage_raw;
    uint16_t battery_voltage_raw;
    uint16_t chargerInput_voltage_raw;
}__attribute__((__packed__))ADC_Data_t;


volatile ADC_Data_t adc_buffer[ADC_NUMBER_MEASURE]={0};

typedef enum{
    CHARGER_STATE_IDLE,
    CHARGER_STATE_CONNECTED,
    CHARGER_STATE_CHARGING_CC,
    CHARGER_STATE_CHARGING_CV,
    CHARGER_STATE_END_CHARGING,
} CHARGER_STATE_e;

/* union to store a float in the U16 backup register */
union FtoU{
  float_t  f;
  uint16_t u[2];
};

int blade_motor = 0;

static uint8_t panel_rcvd_data;
volatile float batteryVoltage,current,chargerVoltage,chargerInputVoltage;

union FtoU ampere_acc;
union FtoU charge_current_offset;
// exported via rostopics
float_t SOC = 0;
float_t battery_voltage;
float_t charge_voltage;
float_t charge_current;
uint16_t chargecontrol_pwm_val = MIN_CHARGE_PWM;
uint8_t  chargecontrol_is_charging = 0;

UART_HandleTypeDef MASTER_USART_Handler; // UART  Handle

// SPI3 FLASH
SPI_HandleTypeDef SPI3_Handle;

// Drive Motors DMA

DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_adc;

ADC_HandleTypeDef ADC_Handle;
TIM_HandleTypeDef TIM1_Handle;  // PWM Charge Controller
TIM_HandleTypeDef TIM2_Handle;  // Time Base for ADC
TIM_HandleTypeDef TIM3_Handle;  // PWM Beeper
TIM_HandleTypeDef TIM4_Handle;  // PWM Buzzer

IWDG_HandleTypeDef IwdgHandle = {0};
WWDG_HandleTypeDef WwdgHandle = {0};

RTC_HandleTypeDef hrtc = {0};

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
   // do nothing here
}

/*
 * called when DMA transfer completes
 * update <xxxx>_tx_busy to let XXXX_Transmit function now when the DMA buffer is free 
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{   
    if (huart->Instance == MASTER_USART_INSTANCE)
    {
        if (__HAL_USART_GET_FLAG(&MASTER_USART_Handler, USART_FLAG_TC))        
        {
            master_tx_busy = 0;
        }
    }
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
   // do nothing here
}

/*
 * Master UART receive ISR
 * DriveMotors UART receive ISR
 * PANEL UART receive ISR
 */ 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{           
       if (huart->Instance == MASTER_USART_INSTANCE)
       {
            ULTRASONICSENSOR_ReceiceIT();       
       }
       else if(huart->Instance == BLADEMOTOR_USART_INSTANCE)
       {
            BLADEMOTOR_ReceiceIT();
       }
       else if (huart->Instance == DRIVEMOTORS_USART_INSTANCE)
       {
        DRIVEMOTOR_ReceiceIT();
       }       
       else if(huart->Instance == PANEL_USART_INSTANCE)
       {
           PANEL_Handle_Received_Data(panel_rcvd_data);
           HAL_UART_Receive_IT(&PANEL_USART_Handler, &panel_rcvd_data, 1);   // rearm interrupt               
       }
}

int main(void)
{        
   HAL_Init();
    SystemClock_Config();

    __HAL_RCC_AFIO_CLK_ENABLE();    
    __HAL_RCC_PWR_CLK_ENABLE();

    MX_DMA_Init();
    
    MASTER_USART_Init();

    DB_TRACE("\r\n");
    DB_TRACE("    __  ___                    ___\r\n");
    DB_TRACE("   /  |/  /___ _      ______ _/ (_)\r\n");
    DB_TRACE("  / /|_/ / __ \\ | /| / / __ `/ / / \r\n");
    DB_TRACE(" / /  / / /_/ / |/ |/ / /_/ / / /  \r\n");
    DB_TRACE("/_/  /_/\\____/|__/|__/\\__, /_/_/   \r\n");
    DB_TRACE("                     /____/        \r\n");
    DB_TRACE("\r\n\r\n");
    DB_TRACE(" * Master USART (debug) initialized\r\n");
    LED_Init();
    DB_TRACE(" * LED initialized\r\n");
    TIM2_Init();
    ADC1_Init();    
    DB_TRACE(" * ADC1 initialized\r\n");  
    TIM3_Init();
    HAL_TIM_PWM_Start(&TIM3_Handle, TIM_CHANNEL_4);    
    DB_TRACE(" * Timer3 (Beeper) initialized\r\n"); 
    TF4_Init();
    DB_TRACE(" * 24V switched on\r\n");
    RAIN_Sensor_Init();
    DB_TRACE(" * RAIN Sensor enable\r\n");
    HALLSTOP_Sensor_Init();
    DB_TRACE(" * HALL Sensor enabled\r\n");
    
    if (SPIFLASH_TestDevice())
    {        
        SPIFLASH_Config();
        SPIFLASH_IncBootCounter();
    }
    else
    {
         DB_TRACE(" * SPIFLASH: unable to locate SPI Flash\r\n");    
    }    
    DB_TRACE(" * SPIFLASH initialized\r\n");

    I2C_Init();
    DB_TRACE(" * Hard I2C initialized\r\n");
    if (I2C_Acclerometer_TestDevice())
    {
         I2C_Accelerometer_Setup();          
    }
    else
    {
        chirp(3);        
        DB_TRACE(" * WARNING: initalization of onboard accelerometer for tilt protection failed !\r\n");
    }    
    DB_TRACE(" * Accelerometer (onboard/tilt safety) initialized\r\n");    
    //SW_I2C_Init();
    DB_TRACE(" * Soft I2C (J18) initialized\r\n");
    DB_TRACE(" * Testing supported IMUs:\r\n");
    //IMU_TestDevice();
    //IMU_Init();
    //IMU_Calibrate();
    Emergency_Init();
    DB_TRACE(" * Emergency sensors initialized\r\n");  
    TIM1_Init();   
    DB_TRACE(" * Timer1 (Charge PWM) initialized\r\n");    
    MX_USB_DEVICE_Init();
    DB_TRACE(" * USB CDC initialized\r\n");
    PANEL_Init();
    DB_TRACE(" * Panel initialized\r\n");
    
    // Charge CH1/CH1N PWM Timer
    HAL_TIM_PWM_Start(&TIM1_Handle, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&TIM1_Handle, TIM_CHANNEL_1);
    DB_TRACE(" * Charge Controler PWM Timers initialized\r\n");
    
    // Init Drive Motors and Blade Motor
    #ifdef DRIVEMOTORS_USART_ENABLED
        DRIVEMOTOR_Init();
        DB_TRACE(" * Drive Motors USART initialized\r\n");        
    #endif
    #ifdef BLADEMOTOR_USART_ENABLED
        BLADEMOTOR_Init();
    #endif
    

   // HAL_UART_Receive_IT(&MASTER_USART_Handler, &master_rcvd_data, 1);
    DB_TRACE(" * Master Interrupt enabled\r\n");
    
    HAL_UART_Receive_IT(&PANEL_USART_Handler, &panel_rcvd_data, 1);   // rearm interrupt               
    DB_TRACE(" * Panel Interrupt enabled\r\n");

    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, 0);
    HAL_GPIO_WritePin(TF4_GPIO_PORT, TF4_PIN, 1);  

    TIM4_Init();
    HAL_TIM_PWM_Start(&TIM4_Handle, TIM_CHANNEL_3); 
    
    // Initialize Main Timers
    NBT_init(&main_chargecontroller_nbt, 10);
    NBT_init(&main_statusled_nbt, 1000);
    NBT_init(&main_emergency_nbt, 10);
    NBT_init(&main_ultrasonicsensor_nbt, 50);
    NBT_init(&main_blademotor_nbt, 100);
    NBT_init(&main_drivemotor_nbt, 10);
    NBT_init(&main_wdg_nbt,10);

    DB_TRACE(" * NBT Main timers initialized\r\n");   

 #ifdef I_DONT_NEED_MY_FINGERS
    DB_TRACE("\r\n");
    DB_TRACE("=========================================================\r\n");
    DB_TRACE(" EMERGENCY/SAFETY FEATURES ARE DISABLED IN board.h ! \r\n");
    DB_TRACE("=========================================================\r\n");
    DB_TRACE("\r\n");
 #endif
    // Initialize ROS
    init_ROS();
    DB_TRACE(" * ROS serial node initialized\r\n");     
    DB_TRACE("\r\n >>> entering main loop ...\r\n\r\n"); 
    // <chirp><chirp> means we are in the main loop 
    chirp(2);    
    while (1)
    {        
      chatter_handler();
      motors_handler();    
      panel_handler();
      spinOnce();                       
      broadcast_handler(); 

      DRIVEMOTOR_App_Rx();

      if (NBT_handler(&main_chargecontroller_nbt))
	    {            
			  ChargeController();
	    }
        if (NBT_handler(&main_statusled_nbt))
	    {            
			  StatusLEDUpdate();          
                       
            // DB_TRACE("master_rx_STATUS: %d  drivemotors_rx_buf_idx: %d  cnt_usart2_overrun: %x\r\n", master_rx_STATUS, drivemotors_rx_buf_idx, cnt_usart2_overrun);           
	    }
        if (NBT_handler(&main_ultrasonicsensor_nbt))
	    {            
			  ULTRASONICSENSOR_App();                   
	    }

      if (NBT_handler(&main_wdg_nbt))
	    {            
			  WATCHDOG_Refresh();                   
	    }

        if (NBT_handler(&main_drivemotor_nbt))
	    {            
        DRIVEMOTOR_App_10ms();      
	    }
      
        if (NBT_handler(&main_blademotor_nbt))
	    {            
			  BLADEMOTOR_App();       
        DB_TRACE(" Charge Voltage: %2.2fV, Battery Voltage: %2.2fV, Current: %fA \r\n", charge_voltage, battery_voltage,charge_current);
        DB_TRACE(" A acc: %fA, SOC: %2.2f \r\n", ampere_acc.f, SOC);     

	    }
        
#ifndef I_DONT_NEED_MY_FINGERS
		if (NBT_handler(&main_emergency_nbt))
		{
			EmergencyController();
		}
#endif        
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // we never get here ...
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

/**
 * @brief Init the Master Serial Port  - this what connects to the upstream controller
 * @retval None
 */
void MASTER_USART_Init()
{
    // enable port and usart clocks
    MASTER_USART_GPIO_CLK_ENABLE();
    MASTER_USART_USART_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    // RX
    GPIO_InitStruct.Pin = MASTER_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(MASTER_USART_RX_PORT, &GPIO_InitStruct);

    // TX
    GPIO_InitStruct.Pin = MASTER_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    //GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(MASTER_USART_TX_PORT, &GPIO_InitStruct);


    MASTER_USART_Handler.Instance = MASTER_USART_INSTANCE;     // USART1 (DEV)
    MASTER_USART_Handler.Init.BaudRate = 115200;               // Baud rate
    MASTER_USART_Handler.Init.WordLength = UART_WORDLENGTH_8B; // The word is  8  Bit format
    MASTER_USART_Handler.Init.StopBits = USART_STOPBITS_1;     // A stop bit
    MASTER_USART_Handler.Init.Parity = UART_PARITY_NONE;       // No parity bit
    MASTER_USART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No hardware flow control
    MASTER_USART_Handler.Init.Mode = USART_MODE_TX_RX;         // Transceiver mode

    HAL_UART_Init(&MASTER_USART_Handler); // HAL_UART_Init() Will enable  UART1

    /* UART4 DMA Init */
    /* UART4_RX Init */    
    hdma_uart4_rx.Instance = DMA2_Channel3;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_NORMAL;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&MASTER_USART_Handler,hdmarx,hdma_uart4_rx);
    
    /* UART4 DMA Init */
    /* UART4_TX Init */
    hdma_uart4_tx.Instance = DMA2_Channel5;
    hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_tx.Init.Mode = DMA_NORMAL;
    hdma_uart4_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_uart4_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&MASTER_USART_Handler,hdmatx,hdma_uart4_tx);

    // enable IRQ
    HAL_NVIC_SetPriority(MASTER_USART_IRQ, 0, 0);
	HAL_NVIC_EnableIRQ(MASTER_USART_IRQ);     

    __HAL_UART_ENABLE_IT(&MASTER_USART_Handler, UART_IT_TC);
}

/**
 * @brief Init LED
 * @retval None
 */
void LED_Init()
{
    LED_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief Poll RAIN Sensor
 * @retval 1 if rain is detected, 0 if no rain
 */
int RAIN_Sense(void)
{
  return(!HAL_GPIO_ReadPin(RAIN_SENSOR_PORT, RAIN_SENSOR_PIN)); // pullup, active low
}

/**
 * @brief Init RAIN Sensor (PE2) Input
 * @retval None
 */
void RAIN_Sensor_Init()
{
    RAIN_SENSOR_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = RAIN_SENSOR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(RAIN_SENSOR_PORT, &GPIO_InitStruct);
}

/**
 * @brief Init HALL STOP Sensor (PD2&3) Inputs
 * @retval None
 */
void HALLSTOP_Sensor_Init()
{
     HALLSTOP_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(HALLSTOP_PORT, &GPIO_InitStruct);
}

/**
 * @brief Poll HALLSTOP Sensor
 * @retval 1 if right, 2 if left, 3 if both, 0 if no stop sensor trigger
 */
int HALLSTOP_Sense(void)
{
  return(!HAL_GPIO_ReadPin(HALLSTOP_PORT, GPIO_PIN_3))<<1 | (!HAL_GPIO_ReadPin(HALLSTOP_PORT, GPIO_PIN_2)); // pullup, active low
}

/**
 * @brief Init TF4 (24V Power Switch)
 * @retval None
 */
void TF4_Init()
{
    TF4_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = TF4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(TF4_GPIO_PORT, &GPIO_InitStruct);
}


/**
 * @brief SPI3 Bus (onboard FLASH)
 * @retval None
 */
void SPI3_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Disable JTAG only to free PA15, PB3* and PB4. SWD remains active
    MODIFY_REG(AFIO->MAPR, AFIO_MAPR_SWJ_CFG, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

    __HAL_RCC_SPI3_CLK_ENABLE();
    FLASH_SPI_CLK_ENABLE();
    FLASH_SPICS_CLK_ENABLE();

    GPIO_InitStruct.Pin = FLASH_CLK_PIN | FLASH_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FLASH_SPI_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = FLASH_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(FLASH_SPI_PORT, &GPIO_InitStruct);
  
    GPIO_InitStruct.Pin = FLASH_nCS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FLASH_SPICS_PORT, &GPIO_InitStruct);
 
    // now init HAL SPI3_Handle
    SPI3_Handle.Instance = SPI3;
    SPI3_Handle.Init.Mode = SPI_MODE_MASTER;
    SPI3_Handle.Init.Direction = SPI_DIRECTION_2LINES;
    SPI3_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
    SPI3_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    SPI3_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    SPI3_Handle.Init.NSS = SPI_NSS_HARD_OUTPUT;
    SPI3_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    SPI3_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPI3_Handle.Init.TIMode = SPI_TIMODE_DISABLE;
    SPI3_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    SPI3_Handle.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&SPI3_Handle) != HAL_OK)
    {
        Error_Handler();
    }    
}

/**
 * @brief Deinit SPI3 Bus (onboard FLASH)
 * @retval None
 */
void SPI3_DeInit()
{        
    __HAL_RCC_SPI3_CLK_DISABLE();
    
    HAL_GPIO_DeInit(FLASH_SPI_PORT,FLASH_CLK_PIN | FLASH_MISO_PIN | FLASH_nCS_PIN);
    HAL_GPIO_DeInit(FLASH_SPICS_PORT,FLASH_nCS_PIN);
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
        DB_TRACE("Error Handler reached, oops\r\n");
        chirp(1);        
    }
    /* USER CODE END Error_Handler_Debug */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB | RCC_PERIPHCLK_RTC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

}

/*
 * VREF+ = 3.3v
 */
void ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /**ADC1 GPIO Configuration
    PA1     ------> Charge Current
    PA2     ------> Charge Voltage
    PA3     ------> Battery Voltage
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    ADC_Handle.Instance = ADC1;
    ADC_Handle.Init.ScanConvMode = ADC_SCAN_ENABLE;
    ADC_Handle.Init.ContinuousConvMode = DISABLE;
    ADC_Handle.Init.DiscontinuousConvMode = DISABLE;
    ADC_Handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC2;
    ADC_Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    ADC_Handle.Init.NbrOfConversion = 4;
    if (HAL_ADC_Init(&ADC_Handle) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_1; // PA1 Charge Current
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&ADC_Handle, &sConfig) != HAL_OK)
    {
       Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_2; // PA2 Charge Voltage
    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&ADC_Handle, &sConfig) != HAL_OK)
    {
       Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_3; // PA3 Battery
    sConfig.Rank = ADC_REGULAR_RANK_3;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&ADC_Handle, &sConfig) != HAL_OK)
    {
       Error_Handler();
    }

    
    sConfig.Channel = ADC_CHANNEL_7; // PA7 Charger Input voltage
    sConfig.Rank = ADC_REGULAR_RANK_4;
    sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    if (HAL_ADC_ConfigChannel(&ADC_Handle, &sConfig) != HAL_OK)
    {
       Error_Handler();
    }

    /* ADC DMA Init */
    /* ADC Init */
    hdma_adc.Instance = DMA1_Channel1;
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc.Init.Mode = DMA_CIRCULAR;
    hdma_adc.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc) != HAL_OK)
    {
      Error_Handler();
    }

   __HAL_LINKDMA(&ADC_Handle,DMA_Handle,hdma_adc);

  // calibrate  - important for accuracy !
  HAL_ADCEx_Calibration_Start(&ADC_Handle); 
  memset(&adc_buffer[0],0,ADC_NUMBER_MEASURE*sizeof(ADC_Data_t));
  HAL_ADC_Start_DMA(&ADC_Handle,(uint32_t*)&adc_buffer[0],320);
  HAL_TIM_OC_Start(&TIM2_Handle,TIM_CHANNEL_2);

    /* USER CODE BEGIN RTC_MspInit 0 */
  __HAL_RCC_PWR_CLK_ENABLE();
  /* USER CODE END RTC_MspInit 0 */
  /* Enable BKP CLK enable for backup registers */
  __HAL_RCC_BKP_CLK_ENABLE();
  /* Peripheral clock enable */
  __HAL_RCC_RTC_ENABLE();
  /* USER CODE BEGIN RTC_MspInit 1 */
  HAL_PWR_EnableBkUpAccess();

  /* not sure if needed  TODO remove*/
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  ampere_acc.u[0] = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
  ampere_acc.u[1] = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);

  charge_current_offset.u[0] = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
  charge_current_offset.u[1] = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);
}

 
/**
* @brief RTC MSP Initialization
* This function configures the hardware resources used in this example
* @param hrtc: RTC handle pointer
* @retval None
*/
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */
  __HAL_RCC_PWR_CLK_ENABLE();
  /* USER CODE END RTC_MspInit 0 */
  /* Enable BKP CLK enable for backup registers */
  __HAL_RCC_BKP_CLK_ENABLE();
  /* Peripheral clock enable */
  __HAL_RCC_RTC_ENABLE();
  /* USER CODE BEGIN RTC_MspInit 1 */
  HAL_PWR_EnableBkUpAccess();
  /* USER CODE END RTC_MspInit 1 */
  }

}

/**
* @brief RTC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hrtc: RTC handle pointer
* @retval None
*/
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
 void TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM1_Handle.Instance = TIM1;
  TIM1_Handle.Init.Prescaler = 0;
  TIM1_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM1_Handle.Init.Period = 1400;
  TIM1_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM1_Handle.Init.RepetitionCounter = 0;
  TIM1_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&TIM1_Handle) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM1_Handle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM1_Handle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 120;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&TIM1_Handle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime = 40;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&TIM1_Handle, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

  GPIO_InitTypeDef GPIO_InitStruct = {0};  
  CHARGE_GPIO_CLK_ENABLE();
  /** TIM1 GPIO Configuration
  PA7 or PE8     -----> TIM1_CH1N
  PA8 oe PE9    ------> TIM1_CH1
  */
  GPIO_InitStruct.Pin = CHARGE_LOWSIDE_PIN|CHARGE_HIGHSIDE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CHARGE_GPIO_PORT, &GPIO_InitStruct);
  #ifdef BOARD_BLUEPILL
    __HAL_AFIO_REMAP_TIM1_PARTIAL();        // to use PA7/8 it is a partial remap
  #endif
  #ifdef BOARD_YARDFORCE500 
     __HAL_AFIO_REMAP_TIM1_ENABLE();        // to use PE8/9 it is a full remap
  #endif  
}

/**
  * @brief TIM2 Initialization Function
  *
  * Used to start ADC every 250µs
  * 
  * @param None
  * @retval None
  */
void TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  __HAL_RCC_TIM2_CLK_ENABLE();

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM2_Handle.Instance = TIM2;
  TIM2_Handle.Init.Prescaler = 18-1; // 72Mhz -> 4Mhz
  TIM2_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM2_Handle.Init.Period = 250-1; /*4khz*/
  TIM2_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM2_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&TIM2_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM2_Handle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM2_Handle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 125;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&TIM2_Handle, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  *
  * Beeper is on PB1 (PWM)
  * 
  * @param None
  * @retval None
  */
void TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  __HAL_RCC_TIM3_CLK_ENABLE();

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM3_Handle.Instance = TIM3;
  TIM3_Handle.Init.Prescaler = 36000; // 72Mhz -> 2khz
  TIM3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM3_Handle.Init.Period = 50;
  TIM3_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM3_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&TIM3_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM3_Handle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&TIM3_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM3_Handle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&TIM3_Handle, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**TIM3 GPIO Configuration
  PB1     ------> TIM3_CH4
  */
  GPIO_InitTypeDef GPIO_InitStruct = {0};  
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief TIM4 Initialization Function
  *
  * Buzzer is on PD14 (PWM)
  * 
  * @param None
  * @retval None
  */
void TIM4_Init(void)
{
  __HAL_RCC_TIM4_CLK_ENABLE();

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  TIM4_Handle.Instance = TIM4;
  TIM4_Handle.Init.Prescaler = 36000; // 72Mhz -> 2khz
  TIM4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM4_Handle.Init.Period = 50;
  TIM4_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM4_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&TIM4_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM4_Handle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&TIM4_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM4_Handle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&TIM4_Handle, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_AFIO_REMAP_TIM4_ENABLE();        // to use PD14 it is a full remap

  __HAL_RCC_GPIOD_CLK_ENABLE();
  /**TIM4 GPIO Configuration
  PD14    ------> TIM4_CH3
  */
  GPIO_InitTypeDef GPIO_InitStruct = {0};  
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);  

}


/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

   /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
/* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);

}


/*
 * manages the charge voltage, and charge, lowbat LED
 * improvementt need to be done to avoid sparks when connected charger and disconnected 
 * todo PID current measure
 * needs to be called frequently
 */
void ChargeController(void)
{                        
  static CHARGER_STATE_e charger_state = CHARGER_STATE_IDLE;
  static uint32_t timestamp = 0;
  charge_voltage =  chargerVoltage;                    
  battery_voltage = batteryVoltage;
  charge_current = current - charge_current_offset.f;

  //DB_TRACE(" charger_state : %d  V : %f \r\n", charger_state,chargerInputVoltage);

  /*charger disconnected force idle state*/
  if((chargerInputVoltage < MIN_DOCKED_VOLTAGE) ){
    charger_state = CHARGER_STATE_IDLE;
  }
    
    switch (charger_state)
    {
    case CHARGER_STATE_CONNECTED:
        
        /* when connected the 3.3v and 5v is provided by the charger so we get the real biais of the current measure */
        chargecontrol_pwm_val = 0;

        /* wait 100ms to read current */
        if( (HAL_GetTick() - timestamp) > 100){
            charge_current_offset.f = current;
          // Writes a data in a RTC Backup data Register 3&4
          HAL_PWR_EnableBkUpAccess();
          HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, charge_current_offset.u[0]);    
          HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, charge_current_offset.u[1]);   
          HAL_PWR_DisableBkUpAccess(); 
            HAL_GPIO_WritePin(TF4_GPIO_PORT, TF4_PIN, 1); /* Power on the battery  Powerbus */
            charger_state = CHARGER_STATE_CHARGING_CC;
        }

        break;

    case CHARGER_STATE_CHARGING_CC:
        // cap charge current at 1.5 Amps
        if ((charge_current > MAX_CHARGE_CURRENT) && (chargecontrol_pwm_val > 50))
        {
            chargecontrol_pwm_val--;
        }
        if ((charge_current < MAX_CHARGE_CURRENT) && (chargecontrol_pwm_val < 1350))
        {
            chargecontrol_pwm_val++;
        }

        if(charge_voltage >= (LIMIT_VOLTAGE_150MA )){
            charger_state = CHARGER_STATE_CHARGING_CV;
        }

        break;

    case CHARGER_STATE_CHARGING_CV:
        // set PWM to approach 29.4V  charge voltage
        if ((charge_voltage < (MAX_CHARGE_VOLTAGE)) && (chargecontrol_pwm_val < 1350))
        {
          chargecontrol_pwm_val++;
        }            
        if ((charge_voltage > (MAX_CHARGE_VOLTAGE)) && (chargecontrol_pwm_val > 50))
        {
          chargecontrol_pwm_val--;
        }

        /* the current is limited to 150ma */
        if ((charge_current > (MAX_CHARGE_CURRENT/10)))
        {
            chargecontrol_pwm_val--;
        }

        /* battery full ? */
        if (charge_current < CHARGE_END_LIMIT_CURRENT) {
          //charger_state = CHARGER_STATE_END_CHARGING;
          /*consider as the battery full */
          ampere_acc.f = 2.8;
          SOC = 100;
        }

        break;

    case CHARGER_STATE_END_CHARGING:

        chargecontrol_pwm_val = 0;

        break;


    case CHARGER_STATE_IDLE:
    default:
       
        if (chargerInputVoltage >= 30.0 ) {
            charger_state = CHARGER_STATE_CONNECTED;
            HAL_GPIO_WritePin(TF4_GPIO_PORT, TF4_PIN, 0); /* Power off the battery  Powerbus */
            timestamp = HAL_GetTick();
        }
        chargecontrol_pwm_val = 0;
        break;
    }
    
    ampere_acc.f += ((current - charge_current_offset.f)/(100*60*60));
    if(ampere_acc.f >= 2.8)ampere_acc.f = 2.8;
    SOC = ampere_acc.f/2.8;

    // Writes a data in a RTC Backup data Register 1
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, ampere_acc.u[0]);    
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, ampere_acc.u[1]);   
    HAL_PWR_DisableBkUpAccess(); 

    chargecontrol_is_charging = charger_state;

    /*Check the PWM value for safety */
    if (chargecontrol_pwm_val > 1350){
        chargecontrol_pwm_val = 1350;
    }
    TIM1->CCR1 = chargecontrol_pwm_val;  
    
}

/*
 * Update the states for the Emergency, Charge and Low Bat LEDs
 */
void StatusLEDUpdate(void)
{
        if (Emergency_State()) {
            DB_TRACE("Emergency !");
            PANEL_Set_LED(PANEL_LED_LIFTED, PANEL_LED_FLASH_FAST);
        } else {
            PANEL_Set_LED(PANEL_LED_LIFTED, PANEL_LED_OFF);
        }

        if (chargecontrol_is_charging == 1) //Connected to charger
        {
            PANEL_Set_LED(PANEL_LED_CHARGING, PANEL_LED_ON);
        }
        else if (chargecontrol_is_charging == 2) //CC mode 1A
        {
            PANEL_Set_LED(PANEL_LED_CHARGING, PANEL_LED_FLASH_FAST);
        }
        else if(chargecontrol_is_charging == 3) //CV mode
        {
            PANEL_Set_LED(PANEL_LED_CHARGING, PANEL_LED_FLASH_SLOW);
        }
        else{
            PANEL_Set_LED(PANEL_LED_CHARGING, PANEL_LED_OFF);
        }

        // show a lowbat warning if battery voltage drops below LOW_BAT_THRESHOLD ? (random guess, needs more testing or a compare to the stock firmware)                   
        if (battery_voltage <= LOW_CRI_THRESHOLD)
        {
            PANEL_Set_LED(PANEL_LED_BATTERY_LOW, PANEL_LED_FLASH_FAST); // low
        }
        else if (battery_voltage <= LOW_BAT_THRESHOLD)
        {
            PANEL_Set_LED(PANEL_LED_BATTERY_LOW, PANEL_LED_FLASH_SLOW); // really low
        }
        else
        {
            PANEL_Set_LED(PANEL_LED_BATTERY_LOW, PANEL_LED_OFF); // bat ok
        }

        HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);         // flash LED                                              
}


/*
 * print hex bytes
 */
void msgPrint(uint8_t *msg, uint8_t msg_len)
{
     int i;
     DB_TRACE("msg: ");
     for (i=0;i<msg_len;i++)
     {
        DB_TRACE(" %02x", msg[i]);
     }            
     DB_TRACE("\r\n");
}

/*
 * calc crc byte
 */
uint8_t crcCalc(uint8_t *msg, uint8_t msg_len)
{
    uint8_t crc = 0x0;
    uint8_t i;

    for (i=0;i<msg_len;i++)
    {
        crc += msg[i];
    }
    return(crc);
}

/*
 * 2khz chirps
 */
void chirp(uint8_t count)
{    
    uint8_t i;

    for (i=0;i<count;i++)
    {
        TIM3_Handle.Instance->CCR4 = 10;    
        HAL_Delay(100);
        TIM3_Handle.Instance->CCR4 = 0;   
        HAL_Delay(50);
    }
}

/*
 * Debug print via MASTER USART
 */
void vprint(const char *fmt, va_list argp)
{
    char string[200];    
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
         for (int i = 0; i < strlen(string); i++)
         {
             ITM_SendChar(string[i]);
         }
       // MASTER_Transmit((unsigned char*)string, strlen(string));

        // HAL_UART_Transmit(&MASTER_USART_Handler, (uint8_t*)string, strlen(string), HAL_MAX_DELAY); // send message via UART               
        //while (master_tx_busy) {  }
        //master_tx_busy = 1;        
        //master_tx_buffer_len = strlen(string);
        //memcpy(master_tx_buffer, string, master_tx_buffer_len);
        //HAL_UART_Transmit_DMA(&MASTER_USART_Handler, (uint8_t*)master_tx_buffer, master_tx_buffer_len); // send message via UART                
        // HAL_Delay(10);
    }
}

/*
 * Debug print
 */
void debug_printf(const char *fmt, ...) 
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}

/*
 * Send message via MASTER USART (DMA Normal Mode)
 */
void MASTER_Transmit(uint8_t *buffer, uint8_t len)
{    
    // wait until tx buffers are free (send complete)
    while (master_tx_busy) {  }
    master_tx_busy = 1;  
    // copy into our master_tx_buffer 
    master_tx_buffer_len = len;
    memcpy(master_tx_buffer, buffer, master_tx_buffer_len);
    HAL_UART_Transmit_DMA(&MASTER_USART_Handler, (uint8_t*)master_tx_buffer, master_tx_buffer_len); // send message via UART       
}

static void WATCHDOG_vInit(void)
{
  #if defined(DB_ACTIVE)
    /* setup DBGMCU block - stop IWDG at break in debug mode */
    __DBGMCU_CLK_ENABLE();
    __HAL_FREEZE_IWDG_DBGMCU();
  #endif  /* DB_ACTIVE */

  /* change the period to 50ms */
  IwdgHandle.Instance = IWDG;
  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_256;
  IwdgHandle.Init.Reload = 8U;
  /* Enable IWDG (LSI automatically enabled by HW) */

  /* if window feature is not applied Init() precedes Start() */
  if( HAL_IWDG_Init(&IwdgHandle) != HAL_OK )
  {
    #ifdef DB_ACTIVE
      DB_TRACE(" IWDG init Error\n\r");
    #endif  /* DB_ACTIVE */
  }

  /* Initialize WWDG for run time if applicable */
  #if defined(DB_ACTIVE)
    /* setup DBGMCU block - stop WWDG at break in debug mode */
    __DBGMCU_CLK_ENABLE();
    __HAL_FREEZE_WWDG_DBGMCU();
  #endif  /* DB_ACTIVE */

  /* Setup period - 20ms */
  __WWDG_CLK_ENABLE();
  WwdgHandle.Instance = WWDG;
  WwdgHandle.Init.Prescaler = WWDG_PRESCALER_8;
  WwdgHandle.Init.Counter = 22; /* 20.02 ms*/
  WwdgHandle.Init.Window = 9; /* 8.19 ms */
  if( HAL_WWDG_Init(&WwdgHandle) != HAL_OK )
  {
    #ifdef DB_ACTIVE
      DB_TRACE(" WWDG init Error\n\r");
    #endif  /* DB_ACTIVE */
  }
} /* WATCHDOG_vInit() */

static void WATCHDOG_Refresh(void){
  /* Update WWDG counter */
  WwdgHandle.Instance = WWDG;
  if( HAL_WWDG_Refresh(&WwdgHandle) != HAL_OK )
  {
    #ifdef DB_ACTIVE
      DB_TRACE(" WWDG refresh error\n\r");
    #endif  /* DB_ACTIVE */

  }

  /* Reload IWDG counter */
  IwdgHandle.Instance = IWDG;
  if( HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK )
  {
    #ifdef DB_ACTIVE
      DB_TRACE(" IWDG refresh error\n\r");
    #endif  /* DB_ACTIVE */
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  volatile int i = 0;
  volatile uint32_t l_u32Current_sum = 0;
  volatile uint32_t l_u32BatVoltage_sum = 0;
  volatile uint32_t l_u32ChargerVoltage_sum = 0;
  volatile uint32_t l_u32ChargerInputVoltage_sum = 0;
  
  
  for(i= 40;i<80;++i){
        l_u32Current_sum += adc_buffer[i].current_raw;
        l_u32ChargerVoltage_sum += adc_buffer[i].charge_voltage_raw;
        l_u32BatVoltage_sum += adc_buffer[i].battery_voltage_raw;
        l_u32ChargerInputVoltage_sum += adc_buffer[i].chargerInput_voltage_raw;
    }
   
  current = (float)l_u32Current_sum/(40);
  chargerVoltage = (float)l_u32ChargerVoltage_sum/(40);
  batteryVoltage = (float)l_u32BatVoltage_sum/(40);
  chargerInputVoltage = (float)l_u32ChargerInputVoltage_sum/(40);

  current = ((float)(current/4095.0f)*3.3f - 2.5f) * 100/12.0;
  chargerVoltage = (float)(chargerVoltage/4095.0f)*3.3f*16;
  batteryVoltage = (float)(batteryVoltage/4095.0f)*3.3f*10.09 + 0.6f;
  chargerInputVoltage = (chargerInputVoltage/4095.0f)*3.3f * (32/2);


}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
    volatile int i = 0;
    volatile uint32_t l_u32Current_sum = 0;
    volatile uint32_t l_u32BatVoltage_sum = 0;
    volatile uint32_t l_u32ChargerVoltage_sum = 0;
    volatile uint32_t l_u32ChargerInputVoltage_sum = 0;

    for(i= 0;i<40;++i){
        l_u32Current_sum += adc_buffer[i].current_raw;
        l_u32ChargerVoltage_sum += adc_buffer[i].charge_voltage_raw;
        l_u32BatVoltage_sum += adc_buffer[i].battery_voltage_raw;
        l_u32ChargerInputVoltage_sum += adc_buffer[i].chargerInput_voltage_raw;
    }
    
    current = (float)l_u32Current_sum/(40);
    chargerVoltage = (float)l_u32ChargerVoltage_sum/(40);
    batteryVoltage = (float)l_u32BatVoltage_sum/(40);
    chargerInputVoltage = (float)l_u32ChargerInputVoltage_sum/(40);

    current = ((float)(current/4095.0f)*3.3f - 2.5f) * 100/12.0;
    chargerVoltage = (float)(chargerVoltage/4095.0f)*3.3f*16;
    batteryVoltage = (float)(batteryVoltage/4095.0f)*3.3f*10.09 + 0.6f;
    chargerInputVoltage = (chargerInputVoltage/4095.0f)*3.3f * (32/2);
}