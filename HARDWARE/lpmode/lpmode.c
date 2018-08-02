#include "lpmode.h"
#include "main.h"

void LPMODE_Init(void)
{  
  GPIO_InitTypeDef  GPIO_InitStruct;

  LPMODE_GPIO_CLK_ENABLE();
  
  GPIO_InitStruct.Pin       = LPMODE_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  
  HAL_GPIO_Init(LPMODE_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LPMODE_GPIO_PORT, LPMODE_PIN, GPIO_PIN_RESET);
}