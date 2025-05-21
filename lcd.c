#include "lcd.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "main.h"

void LCD_EnablePulse() {
    HAL_GPIO_WritePin(LCD_E_Port, LCD_E_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_E_Port, LCD_E_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}

void LCD_Send4Bits(uint8_t data) {
    HAL_GPIO_WritePin(LCD_D4_Port, LCD_D4_Pin, (data >> 0) & 0x01);
    HAL_GPIO_WritePin(LCD_D5_Port, LCD_D5_Pin, (data >> 1) & 0x01);
    HAL_GPIO_WritePin(LCD_D6_Port, LCD_D6_Pin, (data >> 2) & 0x01);
    HAL_GPIO_WritePin(LCD_D7_Port, LCD_D7_Pin, (data >> 3) & 0x01);
    LCD_EnablePulse();
}

void LCD_SendCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(LCD_RS_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    LCD_Send4Bits(cmd >> 4);
    LCD_Send4Bits(cmd & 0x0F);
    HAL_Delay(2);
}

void LCD_SendData(uint8_t data) {
    HAL_GPIO_WritePin(LCD_RS_Port, LCD_RS_Pin, GPIO_PIN_SET);
    LCD_Send4Bits(data >> 4);
    LCD_Send4Bits(data & 0x0F);
    HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x00 + col : 0x40 + col;
    LCD_SendCommand(0x80 | addr);
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01);
    HAL_Delay(2);
}

void LCD_Print(char* str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}

void LCD_Init(void) {
    HAL_Delay(50);
    LCD_Send4Bits(0x03);
    HAL_Delay(5);
    LCD_Send4Bits(0x03);
    HAL_Delay(5);
    LCD_Send4Bits(0x03);
    HAL_Delay(1);
    LCD_Send4Bits(0x02); // 4-bit mode

    LCD_SendCommand(0x28); // 2 lines, 5x8 font
    LCD_SendCommand(0x0C); // Display on, cursor off
    LCD_SendCommand(0x06); // Entry mode
    LCD_Clear();
}
