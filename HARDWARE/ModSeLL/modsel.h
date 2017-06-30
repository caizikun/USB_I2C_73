#ifndef __MODSEL_H
#define __MODSEL_H

#define MODSELL_PIN                    GPIO_PIN_3
#define MODSELL_GPIO_PORT              GPIOA

#define MODSELL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

void MODSELL_Init(void);

#endif