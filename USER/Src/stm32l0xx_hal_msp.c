/**
  ******************************************************************************
  * File Name          : stm32l0xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
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
#include "stm32l0xx_hal.h"
#include "i2c.h"
#include "max5417.h"
    
extern void _Error_Handler(char *, int);
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  if(hi2c->Instance==I2C1)
  {
    RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
    
    /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /*##-2- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    I2C1_SCL_GPIO_CLK_ENABLE();
    I2C1_SDA_GPIO_CLK_ENABLE();
    /* Enable I2Cx clock */
    I2C1_CLK_ENABLE(); 

    /*##-3- Configure peripheral GPIO ##########################################*/  
    /* I2C TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = I2C1_SCL_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = I2C1_SCL_SDA_AF;
    HAL_GPIO_Init(I2C1_SCL_GPIO_PORT, &GPIO_InitStruct);
      
    /* I2C RX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = I2C1_SDA_PIN;
    GPIO_InitStruct.Alternate = I2C1_SCL_SDA_AF;
    HAL_GPIO_Init(I2C1_SDA_GPIO_PORT, &GPIO_InitStruct);
      
    /*##-4- Configure the NVIC for I2C ########################################*/   
    /* NVIC for I2Cx */
    HAL_NVIC_SetPriority(I2C1_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
  }
  else if(hi2c->Instance==I2C2)
  {
    RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
    
    /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    //RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2CxCLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /*##-2- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    I2C2_SCL_GPIO_CLK_ENABLE();
    I2C2_SDA_GPIO_CLK_ENABLE();
    /* Enable I2Cx clock */
    I2C2_CLK_ENABLE(); 

    /*##-3- Configure peripheral GPIO ##########################################*/  
    /* I2C TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = I2C2_SCL_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = I2C2_SCL_SDA_AF;
    HAL_GPIO_Init(I2C2_SCL_GPIO_PORT, &GPIO_InitStruct);
      
    /* I2C RX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = I2C2_SDA_PIN;
    GPIO_InitStruct.Alternate = I2C2_SCL_SDA_AF;
    HAL_GPIO_Init(I2C2_SDA_GPIO_PORT, &GPIO_InitStruct);
      
    /*##-4- Configure the NVIC for I2C ########################################*/   
    /* NVIC for I2Cx */
    HAL_NVIC_SetPriority(I2C2_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(I2C2_IRQn);
  }
  else if(hi2c->Instance==I2C3)
  { 
 
  }
}

/**
  * @brief I2C MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if(hi2c->Instance==I2C1)
  {
  
    /*##-1- Reset peripherals ##################################################*/
    I2C1_FORCE_RESET();
    I2C1_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks #################################*/
    /* Configure I2C Tx as alternate function  */
    HAL_GPIO_DeInit(I2C1_SCL_GPIO_PORT, I2C1_SCL_PIN);
    /* Configure I2C Rx as alternate function  */
    HAL_GPIO_DeInit(I2C1_SDA_GPIO_PORT, I2C1_SDA_PIN);
    
    /*##-3- Disable the NVIC for I2C ##########################################*/
    HAL_NVIC_DisableIRQ(I2C1_IRQn);
  }
  else if(hi2c->Instance==I2C2)
  {
    /*##-1- Reset peripherals ##################################################*/
    I2C2_FORCE_RESET();
    I2C2_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks #################################*/
    /* Configure I2C Tx as alternate function  */
    HAL_GPIO_DeInit(I2C2_SCL_GPIO_PORT, I2C2_SCL_PIN);
    /* Configure I2C Rx as alternate function  */
    HAL_GPIO_DeInit(I2C2_SDA_GPIO_PORT, I2C2_SDA_PIN);
    
    /*##-3- Disable the NVIC for I2C ##########################################*/
    HAL_NVIC_DisableIRQ(I2C2_IRQn);
  }
  else if(hi2c->Instance==I2C3)
  {
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
