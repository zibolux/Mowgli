/**
  ******************************************************************************
  * @file    panel.c
  * @author  Georg Swoboda <cn@warp.at>
  * @date    21/09/2022
  * @version 1.0.0
  * @brief   panel handling, LED, Buttons
  ******************************************************************************  
  * 
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"

#include "panel.h"
#include "board.h"
#include "main.h"

#define PANEL_LENGTH_INIT_MSG 22
#define PANEL_LENGTH_RQST_MSG 18
#define PANEL_LENGTH_RECEIVED_MSG 20


UART_HandleTypeDef PANEL_USART_Handler;
DMA_HandleTypeDef hdma_uart1_rx;
DMA_HandleTypeDef hdma_uart1_tx;

uint16_t buttonstate[PANEL_BUTTON_BYTES];
uint8_t buttonupdated = 0; // 1 if buttonstate was updated by the panel
uint8_t buttoncleared = 0;


uint8_t Led_States[LED_STATE_SIZE];

static uint8_t SendBuffer[256];
static uint8_t ReceiveBuffer[256];
static uint8_t ReceiveIndex = 0;
static uint8_t ReceiveLength;
static uint8_t ReceiveCRC;
// static uint8_t Key_Pressed;
static uint8_t Frame_Received_Panel = 0;
/* per panel type initializers */
#if PANEL_TYPE == PANEL_TYPE_YARDFORCE_900_ECO
    const uint8_t KEY_INIT_MSG[] = {0x03, 0x90, 0x28};     
#elif PANEL_TYPE == PANEL_TYPE_YARDFORCE_500_CLASSIC
    const uint8_t KEY_INIT_MSG[] = {0x06, 0x50, 0xe0};       
#elif PANEL_TYPE ==PANEL_TYPE_YARDFORCE_LUV1000RI
    const uint8_t KEY_INIT_MSG[] = {0x03, 0x99, 0x21};
#else 
    #error "No panel type define in board.h"
#endif

const uint8_t KEY_ACTIVATE[] = {0x0, 0x0, 0x1};


static uint8_t panel_pu8ReceivedData[50] = {0};
static uint8_t panel_pu8RqstMessage[50]  = {0};

const uint8_t panel_pcu8PreAmbule[5]  = {0x55,0xAA,0x0A,0x50,0x3C};

void PANEL_Send_Message(uint8_t *data, uint8_t dataLength, uint16_t command);

/*
 * Initialize HW, USART and send init sequence to panel
 */
void PANEL_Init(void)
{    
#ifdef PANEL_USART_ENABLED
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // enable port and usart clocks
    PANEL_USART_GPIO_CLK_ENABLE();

    // RX
    GPIO_InitStruct.Pin = PANEL_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(PANEL_USART_RX_PORT, &GPIO_InitStruct);

    // TX
    GPIO_InitStruct.Pin = PANEL_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(PANEL_USART_TX_PORT, &GPIO_InitStruct);

    PANEL_USART_USART_CLK_ENABLE();

    PANEL_USART_Handler.Instance = PANEL_USART_INSTANCE;      // USART1
    PANEL_USART_Handler.Init.BaudRate = 115200;               // Baud rate
    PANEL_USART_Handler.Init.WordLength = UART_WORDLENGTH_8B; // The word is  8  Bit format
    PANEL_USART_Handler.Init.StopBits = USART_STOPBITS_1;     // A stop bit
    PANEL_USART_Handler.Init.Parity = UART_PARITY_NONE;       // No parity bit
    PANEL_USART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No hardware flow control
    PANEL_USART_Handler.Init.Mode = USART_MODE_TX_RX;         // Transceiver mode

    HAL_UART_Init(&PANEL_USART_Handler); // HAL_UART_Init() Will enable UART1

    
    /* UART1 DMA Init */
    /* UART1_RX Init */    
    hdma_uart1_rx.Instance = DMA1_Channel5;
    hdma_uart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart1_rx.Init.Mode = DMA_NORMAL;
    hdma_uart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_uart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&PANEL_USART_Handler,hdmarx,hdma_uart1_rx);
    
    /* UART4 DMA Init */
    /* UART4_TX Init */
    hdma_uart1_tx.Instance = DMA1_Channel4;
    hdma_uart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart1_tx.Init.Mode = DMA_NORMAL;
    hdma_uart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_uart1_tx) != HAL_OK)
    {
      Error_Handler();
    }
   

    __HAL_LINKDMA(&PANEL_USART_Handler,hdmatx,hdma_uart1_tx);

    HAL_NVIC_SetPriority(PANEL_USART_IRQ, 0, 0);
	HAL_NVIC_EnableIRQ(PANEL_USART_IRQ);     
    __HAL_UART_ENABLE_IT(&PANEL_USART_Handler, UART_IT_TC);

    /* TODO maybe put this sequence in the loop */
    memset(Led_States, 0x0, LED_STATE_SIZE);       // all LEDs OFF
    // Initialize Panel Sequence
    PANEL_Send_Message(NULL, 0, 0xffff);
    HAL_Delay(100);
    PANEL_Send_Message(NULL, 0, 0xfffe);
    HAL_Delay(100);    
    PANEL_Send_Message((uint8_t*)KEY_INIT_MSG, sizeof(KEY_INIT_MSG), 0xfffd);
    HAL_Delay(100);
    PANEL_Send_Message(NULL, 0, 0xfffb);
    HAL_Delay(100);
    // knight rider <3
    uint8_t i,j=0;
    for (j=0;j<2;j++)
    {
        for (i=4;i<11;i++)
        {
            memset(Led_States, 0x0, LED_STATE_SIZE);
            PANEL_Set_LED(i, PANEL_LED_ON);
            PANEL_SendLEDMessage();
            HAL_Delay(50);
        }
        for (i=11;i>=4;i--)
        {
            memset(Led_States, 0x0, LED_STATE_SIZE);
            PANEL_Set_LED(i, PANEL_LED_ON);
            PANEL_SendLEDMessage();
            HAL_Delay(50);
        }
    }
    // all off
    HAL_Delay(50);
    memset(Led_States, 0x0, LED_STATE_SIZE);    
    PANEL_SendLEDMessage();

    /* prepare to receive the next message */
    HAL_UARTEx_ReceiveToIdle_DMA(&PANEL_USART_Handler,panel_pu8ReceivedData,PANEL_LENGTH_RECEIVED_MSG);
    __HAL_DMA_DISABLE_IT(&hdma_uart1_rx, DMA_IT_HT);

#endif
}

void PANEL_Set_LED(uint8_t led, PANEL_LED_STATE state)
{
    if (led >= 0 && led < LED_STATE_SIZE)
    {
        switch (state)
        {
        case PANEL_LED_OFF:
            Led_States[led] = 0x00;
            break;

        case PANEL_LED_ON:
            Led_States[led] = 0x10;
            break;

        case PANEL_LED_FLASH_SLOW:
            Led_States[led] = 0x20;
            break;

        case PANEL_LED_FLASH_FAST:
            Led_States[led] = 0x22;
            break;
        }
    }
}

/*
 * feed panel messages to uart
 * needs to be called regularly or led states will timeout 
 */
void PANEL_Tick(void)
{   
     if (Frame_Received_Panel == 1)
     {
            //debug_printf("%x %x %x | %x %x %x \r\n",ReceiveBuffer[2],ReceiveBuffer[3],ReceiveBuffer[4], ReceiveBuffer[5], ReceiveBuffer[6], ReceiveBuffer[7]);
            /* if (ReceiveBuffer[5]==0x02) debug_printf("key: timer\r\n");
            if (ReceiveBuffer[6]==0x02) debug_printf("key: S1\r\n");
            if (ReceiveBuffer[7]==0x02) debug_printf("key: S2\r\n");
            if (ReceiveBuffer[8]==0x02) debug_printf("key: Lock\r\n");
            if (ReceiveBuffer[9]==0x02) debug_printf("key: OK\r\n");
            if (ReceiveBuffer[10]==0x02) debug_printf("key: Mon\r\n");
            if (ReceiveBuffer[11]==0x02) debug_printf("key: Tue\r\n");
            if (ReceiveBuffer[12]==0x02) debug_printf("key: Wed\r\n");
            if (ReceiveBuffer[13]==0x02) debug_printf("key: Thu\r\n");
            if (ReceiveBuffer[14]==0x02) debug_printf("key: Fri\r\n");
            if (ReceiveBuffer[15]==0x02) debug_printf("key: Sat\r\n");
            if (ReceiveBuffer[16]==0x02) debug_printf("key: Sun\r\n");
            */
            if ((panel_pu8ReceivedData[5]&0x1) == 0) // any button pressed
            {
               for(int button_byte=0;button_byte < PANEL_BUTTON_BYTES;button_byte++)
                {
               
                    buttonstate[button_byte] = panel_pu8ReceivedData[button_byte+5];//&0x3;
                    buttonupdated = 1;
                    buttoncleared = 0;
                }
            }
            else
            {   // no button pressed
                if (!buttoncleared) // latch to detect first button release only
                {
                    for(int button_byte=0;button_byte < PANEL_BUTTON_BYTES;button_byte++)
                    {
                      buttonstate[button_byte] = 0;
                    }
                    buttonupdated = 1;
                    buttoncleared = 1;
                }
            }
    
      Frame_Received_Panel=0;
     }
     

    // uncomment to flash charging led as a test
    // PANEL_Set_LED(PANEL_LED_CHARGING, PANEL_LED_FLASH_FAST);
    
#ifdef PANEL_USART_ENABLED   
    PANEL_SendLEDMessage();
#endif
}

void PANEL_SendLEDMessage(void){
    uint8_t ptr = 0;
    uint8_t crc = 0;

    while( __HAL_UART_GET_FLAG(&PANEL_USART_Handler, USART_FLAG_TC) != 1);

    panel_pu8RqstMessage[ptr++] = 0x55;
    panel_pu8RqstMessage[ptr++] = 0xaa;
    panel_pu8RqstMessage[ptr++] = LED_STATE_SIZE + 0x02;
    panel_pu8RqstMessage[ptr++] = LED_CMD >> 8;
    panel_pu8RqstMessage[ptr++] = LED_CMD & 0xff;

    for (int i = 0; i < LED_STATE_SIZE; ++i)
    {
        panel_pu8RqstMessage[ptr++] = Led_States[i];
    }


    for (int i = 0; i < LED_STATE_SIZE + 5; ++i)
    {
        crc += panel_pu8RqstMessage[i];
    }
    panel_pu8RqstMessage[ptr++] = crc;

    panel_pu8RqstMessage[ptr++] = 0x55;
    panel_pu8RqstMessage[ptr++] = 0xaa;
    panel_pu8RqstMessage[ptr++] = 0x05;
    panel_pu8RqstMessage[ptr++] = 0x50;
    panel_pu8RqstMessage[ptr++] = 0x84;
    panel_pu8RqstMessage[ptr++] = KEY_ACTIVATE[0];
    panel_pu8RqstMessage[ptr++] = KEY_ACTIVATE[1];
    panel_pu8RqstMessage[ptr++] = KEY_ACTIVATE[2];
    panel_pu8RqstMessage[ptr++] = 0xD9; /* will change if key change */

#ifdef PANEL_USART_ENABLED
    HAL_UART_Transmit_DMA(&PANEL_USART_Handler, (uint8_t*)&panel_pu8RqstMessage[0], ptr); 
#endif

}

void PANEL_Send_Message(uint8_t *data, uint8_t dataLength, uint16_t command)
{
    uint8_t ptr = 0;

    while( __HAL_UART_GET_FLAG(&PANEL_USART_Handler, USART_FLAG_TC) != 1);

    panel_pu8RqstMessage[ptr++] = 0x55;
    panel_pu8RqstMessage[ptr++] = 0xaa;
    panel_pu8RqstMessage[ptr++] = dataLength + 0x02;
    panel_pu8RqstMessage[ptr++] = command >> 8;
    panel_pu8RqstMessage[ptr++] = command & 0xff;

    for (int i = 0; i < dataLength; ++i)
    {
        panel_pu8RqstMessage[ptr++] = data[i];
    }

    uint8_t crc = 0;
    for (int i = 0; i < dataLength + 5; ++i)
    {
        crc += panel_pu8RqstMessage[i];
    }
    panel_pu8RqstMessage[dataLength + 5] = crc;

    
#ifdef PANEL_USART_ENABLED
    HAL_UART_Transmit_DMA(&PANEL_USART_Handler, (uint8_t*)&panel_pu8RqstMessage[0], dataLength + 6); 
#endif
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == PANEL_USART_INSTANCE)
	{
        /* take only the buttons message */
        if(Size == PANEL_LENGTH_RECEIVED_MSG ){
                    /* decode the frame */
            if(memcmp(panel_pcu8PreAmbule,panel_pu8ReceivedData,5) == 0){
                uint8_t l_u8crc = crcCalc(panel_pu8ReceivedData,14-1);
                if(panel_pu8ReceivedData[14-1] == l_u8crc ){
                    Frame_Received_Panel = 1;
                }
            }
        }


        /* prepare to receive the next message */
        HAL_UARTEx_ReceiveToIdle_DMA(&PANEL_USART_Handler,panel_pu8ReceivedData,PANEL_LENGTH_RECEIVED_MSG);
        __HAL_DMA_DISABLE_IT(&hdma_uart1_rx, DMA_IT_HT);

	}
}