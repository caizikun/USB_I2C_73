#include "max5417.h"

I2C_HandleTypeDef hi2c1;

void max5417_Init(void)
{  
  hi2c1.Instance             = I2C1;
  hi2c1.Init.Timing          = I2C1_TIMING;
  hi2c1.Init.OwnAddress1     = I2C1_ADDRESS;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.OwnAddress2     = 0xFF;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;  

  if(HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
      Error_Handler();
  }

  /* Enable the Analog I2C Filter */
  //HAL_I2CEx_ConfigAnalogFilter(&I2cHandle,I2C_ANALOGFILTER_ENABLE);

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
      Error_Handler();
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
      Error_Handler();
  }
}

void max5417_set(uint8_t vout)
{
  __IO uint8_t value = vout;

  do
  {
    if(HAL_I2C_Mem_Write_IT(&hi2c1, (uint16_t)I2C1_ADDRESS, 0x11, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&value, 1)!= HAL_OK)
    {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }      
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
  }
  while(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);
}

void max5417_save(uint8_t vout)
{
  __IO uint8_t value = vout;

  do
  {
    if(HAL_I2C_Mem_Write_IT(&hi2c1, (uint16_t)I2C1_ADDRESS, 0x21, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&value, 1)!= HAL_OK)
    {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }      
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
  }
  while(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);
  
  do
  {
    if(HAL_I2C_Mem_Write_IT(&hi2c1, (uint16_t)I2C1_ADDRESS, 0x61, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&value, 1)!= HAL_OK)
    {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }      
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
  }
  while(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);
}

void max5417_3v45()
{
   max5417_save(0x79);
}

void max5417_3v15()
{
   max5417_save(0x90);
}

void max5417_3v3()
{
   max5417_save(0x84);
}