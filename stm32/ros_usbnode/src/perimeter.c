/****************************************************************************
* Title                 :   Perimeter wire module
* Filename              :   perimeter.c
* Author                :   Nekraus
* Origin Date           :   24/09/2022
* Version               :   1.0.0

*****************************************************************************/
/** \file perimeter.c
*  \brief 
*
*/
/******************************************************************************
* Includes
*******************************************************************************/
#include "main.h"
#include "perimeter.h" 
#include <math.h>


/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define PERIMETER_NBPTS 430
/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
ADC_HandleTypeDef ADC_Handle;
DMA_HandleTypeDef hdma_adc;


int32_t sigcode[] = {1, 1, -1, -1, -1, 1, 1, 1, 1}; 
uint16_t pu16_PerimeterADC_buffer[PERIMETER_NBPTS]; /* 28µs by ADC sample and we need 12ms measure 10s/28µs = 427 */
uint16_t u16_zeroADCraw = 2400;

bool perimeter_bFlagIT = false;
bool perimeter_bFlagStart = false;
int32_t mag[COIL_MAX];
float smoothMag[COIL_MAX] = {0};
float quality[COIL_MAX];
perimeter_CoilNumber_e idxCoil = COIL_LEFT;

/******************************************************************************
* Function Prototypes
*******************************************************************************/
int32_t corrFilter(int32_t *H, int32_t subsample, int32_t M, uint16_t *ip, int32_t nPts, float *quality);
void perimeter_SetCoil(perimeter_CoilNumber_e idx);

/******************************************************************************
*  Public Functions
*******************************************************************************/
void Perimeter_vInit(void){
    
 ADC_ChannelConfTypeDef sConfig = {0};
    
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /**ADC1 GPIO Configuration
    PA6     ------> Perometer Sense
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    ADC_Handle.Instance = ADC1;
    ADC_Handle.Init.ScanConvMode = ADC_SCAN_DISABLE;
    ADC_Handle.Init.ContinuousConvMode = ENABLE;
    ADC_Handle.Init.DiscontinuousConvMode = DISABLE;
    ADC_Handle.Init.ExternalTrigConv = ADC_EXTERNALTRIG_EDGE_NONE;
    ADC_Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    ADC_Handle.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&ADC_Handle) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_6; // PA6 Perimeter sense
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5; /* 28 µs with the adc clock to 9mhz */
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
    hdma_adc.Init.Mode = DMA_NORMAL;
    hdma_adc.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc) != HAL_OK)
    {
      Error_Handler();
    }

   __HAL_LINKDMA(&ADC_Handle,DMA_Handle,hdma_adc);

  // calibrate  - important for accuracy !
  HAL_ADCEx_Calibration_Start(&ADC_Handle); 

  perimeter_bFlagIT = false;
  idxCoil = COIL_MAX;
  perimeter_SetCoil(idxCoil);
  HAL_ADC_Start_DMA(&ADC_Handle,(uint32_t*)&pu16_PerimeterADC_buffer[0],PERIMETER_NBPTS);
  __HAL_DMA_DISABLE_IT(&hdma_adc,DMA_IT_HT);
}

void Perimeter_vApp(void){
  if(perimeter_bFlagIT == true){
    perimeter_bFlagIT = false;
    /* the zero need to be calculated */
    if(idxCoil == COIL_MAX){
      uint32_t sum = 0;
      for (int32_t i=0; i<PERIMETER_NBPTS; i++){
        sum += pu16_PerimeterADC_buffer[i];
      }
      sum /= PERIMETER_NBPTS;
      u16_zeroADCraw = (uint16_t)sum;
    }
    else{
      mag[idxCoil] = corrFilter(sigcode,1,sizeof(sigcode)/4,&(pu16_PerimeterADC_buffer[0]),PERIMETER_NBPTS-(sizeof(sigcode)/4),&quality);
      smoothMag[idxCoil] = 0.99 * smoothMag[idxCoil] + 0.01 * ((float)abs(mag[idxCoil]));
    }

    idxCoil ++;
    if(idxCoil == COIL_MAX ){
      idxCoil = COIL_LEFT;
    }
    perimeter_SetCoil(idxCoil);
    if(perimeter_bFlagStart){
      HAL_ADC_Start_DMA(&ADC_Handle,(uint32_t*)&pu16_PerimeterADC_buffer[0],PERIMETER_NBPTS);
      __HAL_DMA_DISABLE_IT(&hdma_adc,DMA_IT_HT);
    }

  }
}

void Perimeter_vStart(bool on){
  perimeter_bFlagStart = on;
}

void PERIMETER_vITHandle(void){
  /* stop DMA and set flag */
  HAL_ADC_Stop_DMA(&ADC_Handle);
  perimeter_bFlagIT = true;
}


/******************************************************************************
*  Private Functions
*******************************************************************************/
// digital matched filter (cross correlation)
// http://en.wikipedia.org/wiki/Cross-correlation
// H[] holds the double sided filter coeffs, M = H.length (number of points in FIR)
// subsample is the number of times for each filter coeff to repeat 
// ip[] holds input data (length > nPts + M )
// nPts is the length of the required output data 

int32_t corrFilter(int32_t *H, int32_t subsample, int32_t M, uint16_t *ip, int32_t nPts, float *quality){  
  int32_t sumMax = 0; // max correlation sum
  int32_t sumMin = 0; // min correlation sum
  int32_t Ms = M * subsample; // number of filter coeffs including subsampling

  // compute sum of absolute filter coeffs
  int32_t Hsum = 0;
  for (int32_t i=0; i<M; i++) Hsum += abs(H[i]); 
  Hsum *= subsample;

  // compute correlation
  // for each input value
  for (int32_t j=0; j<nPts; j++)
  {
      int32_t sum = 0;      
      int32_t *Hi = H;
      int32_t ss = 0;
      uint16_t *ipi = ip;      
      // for each filter coeffs
      for (int32_t i=0; i<Ms; i++)
      {  
        int32_t  tmp;
        tmp =   (int32_t)(*ipi) - (int32_t)u16_zeroADCraw;  
        sum += ((*Hi)) * ((int32_t)(tmp));
        ss++;
        if (ss == subsample) {
          ss=0;
          Hi++; // next filter coeffs
        }
        ipi++;
      }      
      if (sum > sumMax) sumMax = sum;
      if (sum < sumMin) sumMin = sum;
      ip++;
  }   
 
  // compute ratio min/max 
  if (sumMax > -sumMin) {
    *quality = ((float)sumMax) / ((float)-sumMin);
    return sumMax;
  } else {
    *quality = ((float)-sumMin) / ((float)sumMax);
    return sumMin;
  }  
}

void perimeter_SetCoil(perimeter_CoilNumber_e idx){
  switch (idx)
  {
  case COIL_RIGHT:
    {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    break;
    }
  case COIL_MIDDLE:
    {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    break;
    }
  case COIL_LEFT:
    {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    break;
    }
  case COIL_MAX:
  default:
    {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    break;
    }

    
  }
}