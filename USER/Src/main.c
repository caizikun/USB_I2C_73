/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "usb_device.h"
#include "i2c.h"
#include "soft_i2c.h"
#include "stm32l0xx_nucleo.h"
#include "usbd_cdc_if.h"
#include "modsel.h"

/* Buffer used for transmission */
//uint8_t aTxBuffer[] = {0x33, 0x34};
//uint8_t aTxBuffer[] = {0x46, 0x54};//{"FT"}
uint8_t aTxBuffer[RXBUFFERSIZE];

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
__IO uint16_t hTxNumData = 0;
__IO uint16_t hRxNumData = 0;
uint8_t bTransferRequest = 0;

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef I2cHandle;
IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MODSELL_Init();
  /* I2C2 init function */ 
#ifdef HARDWARE_I2C  
  I2C_Init();  
#else 
  Soft_I2C_Init();
#endif /* HARDWARE_I2C */
  
  MX_IWDG_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Configure User push-button */
//  BSP_PB_Init(BUTTON_KEY,BUTTON_MODE_GPIO);
//  
//  /* Wait for User push-button press before starting the Communication */
//  while (BSP_PB_GetState(BUTTON_KEY) != GPIO_PIN_RESET)
//  {
//    
//  }
//
//  /* Wait for User push-button release before starting the Communication */
//  while (BSP_PB_GetState(BUTTON_KEY) != GPIO_PIN_SET)
//  {
//    
//  }
  
  uint8_t len;
  uint8_t t;
  uint8_t deviceAddress;
  uint8_t regAddress;
  uint8_t length;
  uint8_t read_write;
  
#ifdef HARDWARE_I2C
  while(1)
  {
    if(USB_USART_RX_STA&0x8000)
    {
      read_write = USB_USART_RX_Buffer[3];
      deviceAddress = USB_USART_RX_Buffer[4];
      regAddress = USB_USART_RX_Buffer[5];
      length = USB_USART_RX_Buffer[6];
      
      len = USB_USART_RX_STA&0x3FFF;
      USB_USART_RX_STA=0;
      
      for(t=0;t<len-8;t++)
      {
        aTxBuffer[t]=USB_USART_RX_Buffer[t+8];
      }
      
      if(read_write==MASTER_REQ_READ)
      {
        do
        {
          if(HAL_I2C_Mem_Read_IT(&I2cHandle, (uint16_t)deviceAddress, regAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)aRxBuffer, length)!= HAL_OK)
          {
            /* Error_Handler() function is called when error occurs. */
            Error_Handler();
          }          
          while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
        }
        while(HAL_I2C_GetError(&I2cHandle) == HAL_I2C_ERROR_AF);
        
        CDC_Transmit_FS(aRxBuffer, length);       
      }
      
      if(read_write==MASTER_REQ_WRITE)
      {
        do
        {
          if(HAL_I2C_Mem_Write_IT(&I2cHandle, (uint16_t)deviceAddress, regAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&aTxBuffer, len-8)!= HAL_OK)
          {
            /* Error_Handler() function is called when error occurs. */
            Error_Handler();
          }      
          while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
        }
        while(HAL_I2C_GetError(&I2cHandle) == HAL_I2C_ERROR_AF);
      }
    }   
    
    
    /* Refresh IWDG: reload counter */
    if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
    {
      /* Refresh Error */
      Error_Handler();
    }
  }
#else
while(1)
  { 
    if(USB_USART_RX_STA&0x8000)
    { 
      read_write = USB_USART_RX_Buffer[3];
      deviceAddress = USB_USART_RX_Buffer[4];
      regAddress = USB_USART_RX_Buffer[5];
      length = USB_USART_RX_Buffer[6];
      
      len = USB_USART_RX_STA&0x3FFF;
      USB_USART_RX_STA=0;
      
      for(t=0;t<len-8;t++)
      {
        aTxBuffer[t]=USB_USART_RX_Buffer[t+8];
      }
      
      if(read_write==MASTER_REQ_READ)
      {        
        Soft_I2C_Read_Reg(deviceAddress, regAddress, aRxBuffer, length);       
        CDC_Transmit_FS(aRxBuffer, length);
      }  
      
      if(read_write==MASTER_REQ_WRITE)
      {  
        Soft_I2C_Send_Reg(deviceAddress, regAddress, aTxBuffer, len-8);
      }
    }
    
    /* Refresh IWDG: reload counter */
    if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
    {
      /* Refresh Error */
      Error_Handler();
    }
  }
#endif /* HARDWARE_I2C */
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}



/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  //__HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 0xfff;
  hiwdg.Init.Reload = 0xfff;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}



#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
