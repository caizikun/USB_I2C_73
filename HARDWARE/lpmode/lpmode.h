#ifndef __LPMODE_H
#define __LPMODE_H

#define LPMODE_PIN                    GPIO_PIN_1
#define LPMODE_GPIO_PORT              GPIOA

#define LPMODE_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

void LPMODE_Init(void);

#endif