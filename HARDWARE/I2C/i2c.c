#include "i2c.h"
#include "stm32l0xx_hal_i2c.h"

extern I2C_HandleTypeDef hi2c2;

void I2C2_Init(void)
{  
  hi2c2.Instance             = I2C2;
  hi2c2.Init.Timing          = I2C2_TIMING;
  hi2c2.Init.OwnAddress1     = I2C2_ADDRESS;
  hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.OwnAddress2     = 0xFF;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;  

  if(HAL_I2C_Init(&hi2c2) != HAL_OK)
  {

  }

  /* Enable the Analog I2C Filter */
  //HAL_I2CEx_ConfigAnalogFilter(&I2cHandle,I2C_ANALOGFILTER_ENABLE);

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {

  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    
  }
}


