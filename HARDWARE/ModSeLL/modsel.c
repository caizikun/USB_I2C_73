#include "modsel.h"
#include "main.h"

void MODSELL_Init(void)
{  
  GPIO_InitTypeDef  GPIO_InitStruct;

  MODSELL_GPIO_CLK_ENABLE();
  
  GPIO_InitStruct.Pin       = MODSELL_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  
  HAL_GPIO_Init(MODSELL_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(MODSELL_GPIO_PORT, MODSELL_PIN, GPIO_PIN_RESET);
}