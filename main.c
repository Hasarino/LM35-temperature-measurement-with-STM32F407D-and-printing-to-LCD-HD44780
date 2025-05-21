/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lcd.h"
#include <stdio.h>
#include <string.h>
#include "usb_host.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */



char buffer[16];
uint32_t adc_value;
float voltaj, sicaklik;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void MX_USB_HOST_Process(void);

uint16_t Read_ADC_Value(void);
float Read_Temperature(void);
void Display_Temperature(float temp);



float Read_Sicaklik(void)
{
    #define ORNEK_SAYISI 25
    uint32_t toplam = 0;
    uint32_t ortalama_adc;
    float voltaj, sicaklik;

    for (int i = 0; i < ORNEK_SAYISI; i++)
    {
        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
        toplam += HAL_ADC_GetValue(&hadc2);
        HAL_ADC_Stop(&hadc2);
        HAL_Delay(10);
    }
    ortalama_adc = toplam / ORNEK_SAYISI;

    voltaj = (ortalama_adc / 4095.0f) * 3.3f;
    sicaklik = voltaj * 100.0f;
    return sicaklik;
}
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /*
  uint8_t pressCount = 0;        //butonların başlangıç değerleri
  uint8_t prevButtonState = 0;
  uint8_t prevButton2State = 0;
  uint8_t prevButton3State = 0;
  uint8_t prevButton4State = 0;
  uint8_t prevButton5State = 0;
  uint8_t prevButton6State = 0;
  uint8_t prevButton7State = 0;
  uint8_t prevButton8State = 0; */
  LCD_Init();
  LCD_Clear();
  LCD_SetCursor(0, 0);
  LCD_Print(" ODA SICAKLIGI ");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      // LED güncellemesi (her 50 ms'de bir)

          // ADC başlat, oku
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
      uint16_t adcValue = HAL_ADC_GetValue(&hadc1);

      uint8_t ledCount = (adcValue * 7) / 1023 + 1;

      for(uint8_t i = 0; i < 8; i++)
      {
          if(i < ledCount)
              HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7 << i, GPIO_PIN_SET);
          else
              HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7 << i, GPIO_PIN_RESET);
          HAL_Delay(50);
      }

        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
        uint32_t adc_value = HAL_ADC_GetValue(&hadc2);
        HAL_ADC_Stop(&hadc2);
      float sicaklik = Read_Sicaklik();  // ADC üzerinden sıcaklığı hesaplayan fonksiyon

       snprintf(buffer, sizeof(buffer), "%.1f\xDF""C", sicaklik);
       LCD_SetCursor(1, (16 - strlen(buffer)) / 2);
       LCD_Print(buffer);

       HAL_Delay(1000);
       }














	  /*{
	  void turnOffAllLEDs(void);  // Her döngüde önce tüm LED'leri kapat

	    // button1'i oku ve 3 defa basıldığında led1'i yak
	    uint8_t currentButtonState = HAL_GPIO_ReadPin(buton1_GPIO_Port, buton1_Pin);
	    if (prevButtonState == 1 && currentButtonState == 0)  // button1'e basıldığında
	    {
	        pressCount++;
	        HAL_Delay(250);  // Debounce için kısa bekleme
	    }

	    // 3 defa basıldıysa led1'i yak
	    if (pressCount == 3)
	    {
	        turnOnLED(led1_Pin); // led1 yanar
	        pressCount = 0; // Sayacı sıfırla
	    }

	    prevButtonState = currentButtonState; // Önceki buton durumunu güncelle

	    // button2'ye 1 defa basıldığında led'leri sırayla yak
	    uint8_t currentButton2State = HAL_GPIO_ReadPin(buton2_GPIO_Port, buton2_Pin);
	    if (prevButton2State == 1 && currentButton2State == 0)  // button2'ye basıldığında
	    {
	        // Tüm led'leri kapat
	        turnOffAllLEDs();
	        HAL_Delay(200);

	        turnOnLED(led1_Pin); HAL_Delay(350);
	        turnOnLED(led2_Pin); HAL_Delay(350);
	        turnOnLED(led3_Pin); HAL_Delay(350);
	        turnOnLED(led4_Pin); HAL_Delay(350);
	        turnOnLED(led5_Pin); HAL_Delay(350);
	        turnOnLED(led6_Pin); HAL_Delay(350);
	        turnOnLED(led7_Pin); HAL_Delay(350);
	        turnOnLED(led8_Pin); HAL_Delay(350);
	    }

	    prevButton2State = currentButton2State; // Önceki button2 durumunu güncelle
	    uint8_t currentButton3State = HAL_GPIO_ReadPin(buton3_GPIO_Port, buton3_Pin);

	    if (prevButton3State == 1 && currentButton3State == 0)  // buton3'e basıldığında
	    {
	        turnOffAllLEDs();  // önce tüm ledleri kapat
	        HAL_Delay(200);

	        turnOnLED(led8_Pin); HAL_Delay(350);
	        turnOnLED(led7_Pin); HAL_Delay(350);
	        turnOnLED(led6_Pin); HAL_Delay(350);
	        turnOnLED(led5_Pin); HAL_Delay(350);
	        turnOnLED(led4_Pin); HAL_Delay(350);
	        turnOnLED(led3_Pin); HAL_Delay(350);
	        turnOnLED(led2_Pin); HAL_Delay(350);
	        turnOnLED(led1_Pin); HAL_Delay(350);

	    }

	    prevButton3State = currentButton3State;

	    uint8_t currentButton4State = HAL_GPIO_ReadPin(buton4_GPIO_Port, buton4_Pin);

	    if (prevButton4State == 1 && currentButton4State == 0)  // buton4'e basıldıysa
	    {
	        turnOffAllLEDs();
	        HAL_Delay(150);

	        for (uint16_t i = 1; i <= 255; i++)
	        {
	            turnOffAllLEDs();

	            if (i & 0x01) HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
	            if (i & 0x02) HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
	            if (i & 0x04) HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_SET);
	            if (i & 0x08) HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
	            if (i & 0x10) HAL_GPIO_WritePin(led5_GPIO_Port, led5_Pin, GPIO_PIN_SET);
	            if (i & 0x20) HAL_GPIO_WritePin(led6_GPIO_Port, led6_Pin, GPIO_PIN_SET);
	            if (i & 0x40) HAL_GPIO_WritePin(led7_GPIO_Port, led7_Pin, GPIO_PIN_SET);
	            if (i & 0x80) HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_SET);

	            HAL_Delay(150);
	         }

	      }     prevButton4State = currentButton4State;
	            uint8_t currentButton5State = HAL_GPIO_ReadPin(buton5_GPIO_Port, buton5_Pin);

	     if (prevButton5State == 1 && currentButton5State == 0)  // buton5'e basıldığında
	      {
	              turnOffAllLEDs();
	              HAL_Delay(200);

	              HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
	              HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_SET);
	              HAL_GPIO_WritePin(led5_GPIO_Port, led5_Pin, GPIO_PIN_SET);
	              HAL_GPIO_WritePin(led7_GPIO_Port, led7_Pin, GPIO_PIN_SET);
	         }

	            prevButton5State = currentButton5State;

               uint8_t currentButton6State = HAL_GPIO_ReadPin(buton6_GPIO_Port, buton6_Pin);

          if (prevButton6State == 1 && currentButton6State == 0)  // buton6'ya basıldığında
           {
                turnOffAllLEDs();
                HAL_Delay(200);

                HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);  // led2
                HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);  // led4
                HAL_GPIO_WritePin(led6_GPIO_Port, led6_Pin, GPIO_PIN_SET);  // led6
                HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_SET);  // led8
              }

                prevButton6State = currentButton6State;

                uint8_t currentButton7State = HAL_GPIO_ReadPin(buton7_GPIO_Port, buton7_Pin);

           if (prevButton7State == 1 && currentButton7State == 0)  // buton7'ye basıldığında
            {
        	   pressCount7++;
        	       if (pressCount7 > 8) pressCount7 = 1;

        	       turnOffAllLEDs();      // Önce tüm LED'leri söndür
        	       HAL_Delay(200);        // Debounce için kısa gecikme

        	       switch (pressCount7) {
        	           case 1: HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET); break;
        	           case 2: HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET); break;
        	           case 3: HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_SET); break;
        	           case 4: HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET); break;
        	           case 5: HAL_GPIO_WritePin(led5_GPIO_Port, led5_Pin, GPIO_PIN_SET); break;
        	           case 6: HAL_GPIO_WritePin(led6_GPIO_Port, led6_Pin, GPIO_PIN_SET); break;
        	           case 7: HAL_GPIO_WritePin(led7_GPIO_Port, led7_Pin, GPIO_PIN_SET); break;
        	           case 8: HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_SET); break;
        	       }
        	   }
                  uint8_t currentButton8State = HAL_GPIO_ReadPin(buton8_GPIO_Port, buton8_Pin);

           if (prevButton8State == 1 && currentButton8State == 0)  // Butona basılıp bırakıldıysa
           {
               turnOffAllLEDs();  // Tüm LED'leri söndür
               HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_SET);  // Buzzer aç
               HAL_Delay(500);  // 500 ms süreyle ses versin (isteğe göre azaltılabilir)
               HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);  // Buzzer kapat
               HAL_Delay(200);  // Debounce gecikmesi
           }

                 prevButton8State = currentButton8State;
        	     prevButton7State = currentButton7State;  // Önceki buton durumunu güncelle

        	     uint8_t solState = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14);
        	     uint8_t ortaState = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);
        	     uint8_t sagState = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);  //buton pinlerini okuma

        	     // Sol buton için kontrol
        	     solState = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14);
        	     if (solState == 0 && prevSol == 1) {
        	         position--;  // Konumu bir azalt
        	         updateLCD(); // LCD'yi güncelle
        	     }

        	     // Sağ buton kontrolü
        	     sagState = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
        	     if (sagState == 0 && prevSag == 1) {
        	         position++;  // Konumu bir artır
        	         updateLCD(); // LCD'yi güncelle
        	     }

        	     // Orta buton kontrolü (LCD'yi ortala)
        	     ortaState = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);
        	     if (ortaState == 0 && prevOrta == 1) {
        	         position = 0;  // LCD'yi tam ortada tut
        	         updateLCD(); // LCD'yi güncelle
        	     }

        	         prevSol = solState;
        	         prevOrta = ortaState;
        	         prevSag = sagState;

           }*/

    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|led1_Pin|led2_Pin|led3_Pin
                          |led4_Pin|led5_Pin|led6_Pin|led7_Pin
                          |led8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LCD_D7_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin|LCD_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin led1_Pin led2_Pin led3_Pin
                           led4_Pin led5_Pin led6_Pin led7_Pin
                           led8_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|led1_Pin|led2_Pin|led3_Pin
                          |led4_Pin|led5_Pin|led6_Pin|led7_Pin
                          |led8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin LCD_D6_Pin LCD_D5_Pin LCD_D4_Pin
                           LCD_E_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin
                          |LCD_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BOOT1_Pin buton8_Pin buton7_Pin buton6_Pin
                           buton5_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin|buton8_Pin|buton7_Pin|buton6_Pin
                          |buton5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : buton4_Pin buton3_Pin buton2_Pin buton1_Pin
                           OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = buton4_Pin|buton3_Pin|buton2_Pin|buton1_Pin
                          |OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D7_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = LCD_D7_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
