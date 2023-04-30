/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include "w25qxx.h"
//#include "laki.h"
#include "pic07.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


#define laki_count   1024*150/2
#define size_laki    1024*150


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

SRAM_HandleTypeDef hsram1;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM6_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint16_t part[44032];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void lcd_setup_picture(uint8_t pic_nr)
{
    uint8_t dana ;
    uint16_t x, y;
    lcdOrientationTypeDef dana2;

    lcdOrientationTypeDef setup_pic[] = {
            LCD_ORIENTATION_LANDSCAPE,
            LCD_ORIENTATION_PORTRAIT_MIRROR,
            LCD_ORIENTATION_PORTRAIT,
            LCD_ORIENTATION_LANDSCAPE_MIRROR,
            LCD_ORIENTATION_LANDSCAPE_MIRROR,
            LCD_ORIENTATION_LANDSCAPE,
            LCD_ORIENTATION_LANDSCAPE,
            LCD_ORIENTATION_LANDSCAPE,
            LCD_ORIENTATION_LANDSCAPE,
            LCD_ORIENTATION_LANDSCAPE,
            LCD_ORIENTATION_LANDSCAPE
            };

    dana = pic_nr;
    if (dana > 10) {dana =0;}
    dana2 =  setup_pic[dana];
    lcdSetOrientation(dana2);

    x=319;
    y=239;

    if((dana2 == LCD_ORIENTATION_PORTRAIT  ) || (dana2 == LCD_ORIENTATION_PORTRAIT_MIRROR))
    {
        y=319;
        x=239;
    }
    lcdSetWindow(0, 0, x, y);


}

void showPic(uint8_t pic_nr)
{

    uint32_t index=0;
    lcd_setup_picture(pic_nr);


    while (index<= laki_count)
    {
        LCD_DataWrite(laki[index]);
        index++;
    }
}

void readPicFromFlash(uint8_t pic_nr){

#define rest_pic (150*1024 - 131072)
#define rest0  rest_pic / 2
#define  CCM_ADDRESS   0x10000000UL

    uint32_t i, blk_nr;
    uint16_t * CCM_ADD  = (uint16_t *) CCM_ADDRESS;




    lcd_setup_picture(pic_nr);
    blk_nr = pic_nr*3;

    W25qxx_ReadBlock((uint8_t*)CCM_ADD, blk_nr+0, 0,w25qxx.BlockSize);
    W25qxx_ReadBlock((uint8_t*)&part[0], blk_nr+1, 0,w25qxx.BlockSize);
    W25qxx_ReadBlock((uint8_t*)&part[32768], blk_nr+2, 0, rest_pic);


    for(i=0;i<32768;i++)
    {
        LCD_DataWrite(CCM_ADD[i]);
    }

    for(i=0;i<44032;i++)
    {
        LCD_DataWrite(part[i]);
    }




//    for(i2=0; i2<2; i2++)
//    {
//        W25qxx_ReadBlock((uint8_t*)&part[0], blk_nr+i2, 0,w25qxx.BlockSize);
//        for(i=0;i<32768;i++)
//        {
//            LCD_DataWrite(part[i]);
//        }
//    }
//
//
//    W25qxx_ReadBlock((uint8_t*)&part[0], blk_nr+i2, 0, rest_pic);
//    for(i=0;i<rest0;i++)
//    {
//        LCD_DataWrite(part[i]);
//    }

}



void savePicToFlash(uint8_t pic_nr){
    uint8_t block;
    uint32_t index=0;
    uint32_t laki_size =  sizeof(laki);
    block = pic_nr * 3;

    W25qxx_EraseBlock(block);
    W25qxx_EraseBlock(block+1);
    W25qxx_EraseBlock(block+2);

    W25qxx_WriteBlock((uint8_t*)&laki[index], block, 0,w25qxx.BlockSize);index+=65536/2;
    W25qxx_WriteBlock((uint8_t*)&laki[index], block+1, 0,w25qxx.BlockSize);index+=65536/2;
    W25qxx_WriteBlock((uint8_t*)&laki[index], block+2, 0,laki_size-(65536));index+=65536/2;
}

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
    MX_FSMC_Init();
    MX_RNG_Init();
    MX_TIM6_Init();
    MX_CRC_Init();
    MX_SPI1_Init();
    /* USER CODE BEGIN 2 */


    lcdInit();
    lcdBacklightOn();
    //lcdTest();




    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */


        lcdFillRGB(0);
        lcdHome();
        lcdSetTextFont(&Font16);
        lcdSetOrientation(LCD_ORIENTATION_LANDSCAPE);

//showPic(6);
//for(;;){};

        lcdPrintf("INIT OF SPI FLASH W25Q16 :");
        if (W25qxx_Init()){
            lcdPrintf("OK\n");
            lcdPrintf("CHIP ID :%d\n",w25qxx.ID);
            lcdPrintf("SECTOR COUNT :%d\n",w25qxx.SectorCount);
            lcdPrintf("SECTOR SIZE :%d\n",w25qxx.SectorSize);
            lcdPrintf("BLOCK COUNT :%d\n",w25qxx.BlockCount);
            lcdPrintf("BLOCK SIZE :%d\n",w25qxx.BlockSize);
            lcdPrintf("PAGE COUNT :%d\n",w25qxx.PageCount);
            lcdPrintf("PAGE SIZE :%d\n",w25qxx.PageSize);

//savePicToFlash(6);
//readPicFromFlash(6);
            lcdPrintf("DATA EXIST IN EXT. FLASH\n");
            lcdPrintf("---------------------------\n");
            lcdPrintf("- ODCZYT Z EXT. SPI FLASH -\n");
            lcdPrintf("---------------------------\n");
//for(;;){}

        }
        else {
            lcdPrintf("ERROR\n");
            while(1);;
        }

        /* USER CODE END WHILE */

        while(1)
        {
            for(uint8_t pic_cnt = 0; pic_cnt <7; pic_cnt++)
            {
                //lcdFillRGB(0);
                readPicFromFlash(pic_cnt);
                //HAL_Delay(3000);
            }
        }
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */


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
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 168;
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
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

    /* USER CODE BEGIN CRC_Init 0 */

    /* USER CODE END CRC_Init 0 */

    /* USER CODE BEGIN CRC_Init 1 */

    /* USER CODE END CRC_Init 1 */
    hcrc.Instance = CRC;
    if (HAL_CRC_Init(&hcrc) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CRC_Init 2 */

    /* USER CODE END CRC_Init 2 */

}

/**
 * @brief RNG Initialization Function
 * @param None
 * @retval None
 */
static void MX_RNG_Init(void)
{

    /* USER CODE BEGIN RNG_Init 0 */

    /* USER CODE END RNG_Init 0 */

    /* USER CODE BEGIN RNG_Init 1 */

    /* USER CODE END RNG_Init 1 */
    hrng.Instance = RNG;
    if (HAL_RNG_Init(&hrng) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN RNG_Init 2 */

    /* USER CODE END RNG_Init 2 */

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
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

    /* USER CODE BEGIN TIM6_Init 0 */

    /* USER CODE END TIM6_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM6_Init 1 */

    /* USER CODE END TIM6_Init 1 */
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 0;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 65535;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM6_Init 2 */

    /* USER CODE END TIM6_Init 2 */

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
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, Flash_CS_Pin|LCD_BL_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : Flash_CS_Pin */
    GPIO_InitStruct.Pin = Flash_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(Flash_CS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LCD_BL_Pin */
    GPIO_InitStruct.Pin = LCD_BL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

    /* USER CODE BEGIN FSMC_Init 0 */

    /* USER CODE END FSMC_Init 0 */

    FSMC_NORSRAM_TimingTypeDef Timing = {0};
    FSMC_NORSRAM_TimingTypeDef ExtTiming = {0};

    /* USER CODE BEGIN FSMC_Init 1 */

    /* USER CODE END FSMC_Init 1 */

    /** Perform the SRAM1 memory initialization sequence
     */
    hsram1.Instance = FSMC_NORSRAM_DEVICE;
    hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
    /* hsram1.Init */
    hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
    hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
    hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
    hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
    hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
    hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
    hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
    hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
    hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
    hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_ENABLE;
    hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
    hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
    /* Timing */
    Timing.AddressSetupTime = 15;
    Timing.AddressHoldTime = 15;
    Timing.DataSetupTime = 60;
    Timing.BusTurnAroundDuration = 0;
    Timing.CLKDivision = 16;
    Timing.DataLatency = 17;
    Timing.AccessMode = FSMC_ACCESS_MODE_A;
    /* ExtTiming */
    ExtTiming.AddressSetupTime = 9;
    ExtTiming.AddressHoldTime = 15;
    ExtTiming.DataSetupTime = 8;
    ExtTiming.BusTurnAroundDuration = 0;
    ExtTiming.CLKDivision = 16;
    ExtTiming.DataLatency = 17;
    ExtTiming.AccessMode = FSMC_ACCESS_MODE_A;

    if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK)
    {
        Error_Handler( );
    }

    /* USER CODE BEGIN FSMC_Init 2 */

    /* USER CODE END FSMC_Init 2 */
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

unsigned long randomX(uint32_t min,uint32_t max) {

    while (!(RNG->SR & (RNG_SR_DRDY))); /* Wait until one RNG number is ready */
    return (uint32_t)((RNG->DR % (max - min + 1)) + min);                   /* Get a 32-bit Random number */
}
