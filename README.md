# LM35-temperature-measurement-with-STM32F407D-and-printing-to-LCD-HD44780
Printing the ADC data received from LM35 on a 16x2 LCD by encoding it via STM32


Selecting pins and coding main.c main.h lcd.c lcd.h using STM32cubeide

    LCD Pin | STM32 Pin
         -- | --
         RS | PA8
          E | PC9
        D4 | PC8
        D5 | PC7
        D6 | PC6
        D7 | PD15

             Signal | STM32 Pin
                 -- | --
        lm32 output | PA1 (ADC2_IN1)

