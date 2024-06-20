/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include "font.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	SD_NEW=0,
	SD_EXIST,
	SD_APPEND
}
SD_MODE;

typedef enum {
	RED = 0,
	BLUE,
	GREEN,
	BLACK,
	PURPLE,
	WHITE
}color;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//control pin
#define BackLightON  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
#define BackLightOFF 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

#define DataSend 		 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
#define CommandSend  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

#define ChipSelectHigh	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
#define ChipSelectLow	 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

//cmd set
#define SleepOut 0x11
#define SleepIn 0x10

#define DisplayOff 0x28
#define DisplayOn 0x29

#define WriteData 0x2C
#define MADCTL 0x36
#define COLMOD 0x3A

#define CASET 0x2A //set column address
#define RASET 0x2B //set raw address

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uart_rx_data;


uint8_t count = 0;
uint32_t pretime = 0;
uint32_t pretime2 = 0;
bool EXTI12_flag = false;
bool EXTI13_flag = false;
bool LightOn = false;

//color
uint8_t COLOR[6][3]=
{
		{0xFC,0,0}, //RED
		{0,0,0xFC}, //GREEN
		{0,0xFC,0}, //BLUE
		{0,0,0},		//BLACK
		{0xFC,0,0xFC},//PURPLE
		{0xFC,0xFC,0xFC}//WHITE
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void uart_send(uint8_t uart_num,char *fmt,...);

//SD
void SD_WRITE(SD_MODE sd,char file_route[],char *fmt,...);
void SD_READ(char file_route[],uint8_t *buf, uint8_t read_byte);
void sd_init_user();

//ST7735
void ST7735_LCD_Init();
void ST7735_CMD(uint8_t command,uint8_t CS);
void ST7735_LCD_FillFullScreen(color color);
void ST7735_LCD_ADD_SET(uint8_t startx,uint8_t endx, uint8_t starty,uint8_t endy);
void ST7735_Send_Packet(uint8_t command,uint8_t *buf, size_t size);
void SEL_COLOR(color col, uint8_t *color_data);
void ST7735_LCD_FILLBlock(color col,uint8_t startx, uint8_t endx, uint8_t starty, uint8_t endy);


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_12 && HAL_GetTick()-pretime > 200){
		count=(count+1)%8;
		pretime = HAL_GetTick();
		EXTI12_flag = true;
	}
	else if(GPIO_Pin == GPIO_PIN_0 && HAL_GetTick()-pretime2 > 200){
		EXTI13_flag = true;
		pretime2 = HAL_GetTick();
		LightOn = !LightOn;
	}
  UNUSED(GPIO_Pin);

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1){
		HAL_UART_Receive_DMA(&huart1, &uart_rx_data, 1);
	}
  UNUSED(huart);
}

void uart_send(uint8_t uart_num,char *fmt,...){
	va_list arg;
	uint8_t uart_send_data[40];
	memset(uart_send_data,0,40);

	va_start(arg,fmt);
	vsnprintf((char *)uart_send_data,32,fmt,arg);

	if(uart_num==1){
		HAL_UART_Transmit(&huart1, uart_send_data, 32, HAL_MAX_DELAY);
	}

	va_end(arg);
}

void SD_WRITE(SD_MODE sd,char file_route[],char *fmt,...){
	va_list arg;
	uint8_t buffer[32];

	memset(buffer,0,32);

	va_start(arg,fmt);
	vsnprintf((char *)buffer,32,fmt,arg);
	va_end(arg);

	uint8_t mode;
	if(sd == SD_NEW){
		mode = FA_CREATE_NEW|FA_WRITE;
	}
	else if(sd == SD_EXIST){
		mode = FA_OPEN_EXISTING|FA_WRITE;
	}
	else if(sd == SD_APPEND){
		mode = FA_OPEN_APPEND|FA_WRITE;
	}

	retSD = f_open(&SDFile, file_route, mode);

	if(retSD==FR_OK){
		uart_send(1,"file open success!\n");
	}
	else{
	 uart_send(1,"file error with error code : %d\n",retSD);
	 return;
	}


	UINT bw;
	retSD = f_write(&SDFile, buffer, sizeof(buffer), &bw);
	if(retSD==FR_OK){
		f_close(&SDFile);
	  uart_send(1,"file write success with bw:%d\n",bw);
	}
	else{
		f_close(&SDFile);
		uart_send(1,"file error with %d\n",retSD);
	}
}

void SD_READ(char file_route[],uint8_t *buf, uint8_t read_byte){
	retSD = f_open(&SDFile, file_route, FA_OPEN_EXISTING|FA_READ);

	if(retSD==FR_OK){
		uart_send(1,"file open success!\n");
	}
	else{
	 uart_send(1,"file error with error code : %d\n",retSD);
	 return;
	}

	UINT br;
	retSD = f_read(&SDFile, buf, read_byte, &br);

	if(retSD==FR_OK){
		f_close(&SDFile);
	  uart_send(1,"file read success with bw:%d\n",br);
	}
	else{
		f_close(&SDFile);
		uart_send(1,"file error with %d\n",retSD);
	}

}

void sd_init_user(){
  retSD = f_mount(&SDFatFS, SDPath, 1);
  if(retSD==FR_OK){
  	uart_send(1,"file mount success!\n");
  }
  else {
    uart_send(1,"file error with error code : %d\n",retSD);
    return;
  }
}

void ST7735_LCD_Init(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_Delay(10);

  ST7735_CMD(SleepOut,0);
  ST7735_CMD(DisplayOn,0);
  HAL_Delay(120);

  uint8_t command = 0xC8;
  ST7735_Send_Packet(MADCTL, &command, 1);

  command = 0x06;
  ST7735_Send_Packet(COLMOD, &command, 1);

  ST7735_LCD_ADD_SET(0, 127, 0, 127);
  ST7735_LCD_FillFullScreen(BLACK);
}

void ST7735_LCD_ADD_SET(uint8_t startx,uint8_t endx, uint8_t starty,uint8_t endy){
	uint8_t send[4]={0,startx + 2,0,endx + 2};

	ST7735_Send_Packet(CASET, send, 4);

	send[1] = starty+3;
	send[3] = endy+3;

	ST7735_Send_Packet(0x2B, send, 4);
}

void ST7735_Send_Packet(uint8_t command,uint8_t *buf, size_t size){
	ST7735_CMD(command,1);

	DataSend;
	if(HAL_SPI_Transmit(&hspi1, &buf[0], size, HAL_MAX_DELAY)!=HAL_OK){
		uart_send(1, "SPI Fail.\n");
	}

	ChipSelectHigh;
}

void ST7735_CMD(uint8_t command,uint8_t CS){
	CommandSend;
	ChipSelectLow;

	while(HAL_SPI_GetState(&hspi1)!=HAL_SPI_STATE_READY);

	if(HAL_SPI_Transmit(&hspi1, &command, 1, HAL_MAX_DELAY)!=HAL_OK){
		uart_send(1, "SPI Fail.\n");
	}

	if(CS == 0){
		ChipSelectHigh;
	}
}

void ST7735_LCD_FillFullScreen(color col) {
	ST7735_LCD_FILLBlock(col,0,127,0,127);
}

void ST7735_LCD_FILLBlock(color col,uint8_t startx, uint8_t endx, uint8_t starty, uint8_t endy) {
    uint8_t color_data[3];
    SEL_COLOR(col,color_data);

    ST7735_LCD_ADD_SET(startx,endx,starty,endy);
    ST7735_CMD(WriteData, 1);

    DataSend;
    for (uint32_t i = 0; i < (endx - startx + 1) * (endy - starty + 1); i++) {
      if (HAL_SPI_Transmit(&hspi1, color_data, 3, HAL_MAX_DELAY) != HAL_OK) {
        uart_send(1, "SPI Fail.\n");
        break;
      }
    }
    ChipSelectHigh;
}

void SEL_COLOR(color col, uint8_t *color_data){
	for(uint8_t i = 0; i < 3 ; i++){
		color_data[i] = COLOR[col][i];
	}
}



void ST7735_LCD_FILLBlock_Char(color col_back,color col_buf, uint8_t buf[font_len_y][font_len_x], uint8_t startx,uint8_t starty){
	uint8_t back_color[3], buf_color[3];
	SEL_COLOR(col_back,back_color);
	SEL_COLOR(col_buf,buf_color);

	ST7735_LCD_ADD_SET(startx, startx+font_len_x - 1, starty, starty+font_len_y - 1);

	ST7735_CMD(WriteData, 1);

	DataSend;
	for(uint8_t i=0; i<font_len_y; i++){
		for(uint8_t j=0; j<font_len_x; j++){
			if(buf[i][j]){
				HAL_SPI_Transmit(&hspi1, buf_color, 3, HAL_MAX_DELAY);
			}
			else{
				HAL_SPI_Transmit(&hspi1, back_color, 3, HAL_MAX_DELAY);
			}
		}
	}
	ChipSelectHigh;

}

void ST7735_LCD_Write_String(char *fmt, uint8_t *startx, uint8_t *starty, color font_color, color back_color){
	uint8_t string_len = strlen(fmt);
	for(uint8_t i=0;i<string_len;i++){
		switch (fmt[i]){
		case 'A':
			ST7735_LCD_FILLBlock_Char(back_color, font_color, buffer_A, *startx, *starty);
			break;
		case 'B':
			ST7735_LCD_FILLBlock_Char(back_color, font_color, buffer_B, *startx, *starty);
			break;
		case 'D':
			ST7735_LCD_FILLBlock_Char(back_color, font_color, buffer_D, *startx, *starty);
			break;
		case 'E':
			ST7735_LCD_FILLBlock_Char(back_color, font_color, buffer_E, *startx, *starty);
			break;
		case 'H':
			ST7735_LCD_FILLBlock_Char(back_color, font_color, buffer_H, *startx, *starty);
			break;
		case 'L':
			ST7735_LCD_FILLBlock_Char(back_color, font_color, buffer_L, *startx, *starty);
			break;
		case 'O':
			ST7735_LCD_FILLBlock_Char(back_color, font_color, buffer_O, *startx, *starty);
			break;
		case 'R':
			ST7735_LCD_FILLBlock_Char(back_color, font_color, buffer_R, *startx, *starty);
			break;
		case 'W':
			ST7735_LCD_FILLBlock_Char(back_color, font_color, buffer_W, *startx, *starty);
			break;
		}
		if(fmt[i] == '\n'){
			*startx = 0;
			*starty = *starty+font_len_y+1;
		}
		else if((fmt[i] >= 'A' && fmt[i] <= 'Z' )||(fmt[i] >= 'a' && fmt[i] <= 'z' )||
						(fmt[i] == ' ')){
			*startx = *startx+font_len_x+1;
		}
	}

}
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  UART_Start_Receive_DMA(&huart1, &uart_rx_data, 1);
  sd_init_user();

  SD_WRITE(SD_EXIST,"0:/hard.txt","test test%d",166);
  uint8_t buf[32];
  SD_READ("0:/hard.txt",buf,32);

  ST7735_LCD_Init();
  uart_send(1,"%s",buf);
  uint8_t startx = 0;
  uint8_t starty = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	if(EXTI12_flag){
  		switch (count){
  		case 0:
    		ST7735_LCD_FillFullScreen(RED);
    		break;
  		case 1:
    		ST7735_LCD_FillFullScreen(GREEN);
    		break;
  		case 2:
    	  ST7735_LCD_FillFullScreen(BLUE);
    	  break;
  		case 3:
    	  ST7735_LCD_FillFullScreen(BLACK);
    	  break;
  		case 4:
    	  ST7735_LCD_FillFullScreen(PURPLE);
    	  break;
  		case 5:
  			ST7735_LCD_FillFullScreen(WHITE);
  			break;
  		case 6:
        ST7735_LCD_FILLBlock(WHITE, 0, 63, 0, 63);
        ST7735_LCD_FILLBlock(BLACK, 0, 63, 64, 127);
        ST7735_LCD_FILLBlock(PURPLE, 64, 127, 0, 63);
        ST7735_LCD_FILLBlock(RED, 64, 127, 64, 127);
        break;
  		case 7:
    		ST7735_LCD_FillFullScreen(BLACK);
    	  ST7735_LCD_Write_String("HELLOW\nWORLD",&startx,&starty,PURPLE,BLACK);
    	  ST7735_LCD_Write_String("\nHELLOW\nWORLD",&startx,&starty,RED,GREEN);
    	  ST7735_LCD_Write_String("\nHELLOW\nWORLD",&startx,&starty,WHITE,BLACK);

    	  startx = 0;
    	  starty = 0;
    	  break;
  		}
  		EXTI12_flag = false;
  	}

  	if(EXTI13_flag && LightOn){
  		BackLightON;
  		EXTI13_flag = false;
  	}
  	else if(EXTI13_flag && !LightOn){
  		BackLightOFF;
  	  EXTI13_flag = false;
  	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
