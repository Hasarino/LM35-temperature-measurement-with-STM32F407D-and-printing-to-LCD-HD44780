#ifndef __LCD_H
#define __LCD_H

#include "stm32f4xx_hal.h"

// Pin tanımlamaları
#define LCD_RS_Pin GPIO_PIN_8
#define LCD_RS_Port GPIOA

#define LCD_E_Pin GPIO_PIN_9
#define LCD_E_Port GPIOC

#define LCD_D4_Pin GPIO_PIN_8
#define LCD_D4_Port GPIOC

#define LCD_D5_Pin GPIO_PIN_7
#define LCD_D5_Port GPIOC

#define LCD_D6_Pin GPIO_PIN_6
#define LCD_D6_Port GPIOC

#define LCD_D7_Pin GPIO_PIN_15
#define LCD_D7_Port GPIOD

void LCD_Init(void);
void LCD_SendCommand(uint8_t);
void LCD_SendData(uint8_t);
void LCD_SetCursor(uint8_t, uint8_t);
void LCD_Print(char*);
void LCD_Clear(void);

#endif
