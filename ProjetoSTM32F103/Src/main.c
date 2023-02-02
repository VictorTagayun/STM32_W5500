/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
#include "spi.h"
#include "modbus.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "socket.h"	// Just include one header for WIZCHIP
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{ Bit_RESET = 0,
  Bit_SET
}BitAction;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LDR_VALUE  ADC_CHANNEL_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


char writeValue[60];
uint8_t cnt;
BitAction operatorOk = 0;
uint32_t address_mem = 0;

uint8_t TX_buffer[128];
uint8_t RX_buffer[128];
uint16_t holding_registers[64];

uint8_t S1 = 0;
uint8_t S2 = 0;
uint8_t S3 = 0;
uint8_t flag_busy_RTU = 0;
uint8_t cntBufferRTU = 0;
uint8_t FrameNrBytes = 0;

////////////////////////////////////////////////
// Shared Buffer Definition for LOOPBACK TEST //
////////////////////////////////////////////////
#define DATA_BUF_SIZE   2048
uint8_t gDATABUF[DATA_BUF_SIZE];

///////////////////////////////////
// Default Network Configuration //
///////////////////////////////////
wiz_NetInfo gWIZNETINFO = { .mac = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED},
							.ip = {169, 254, 216, 240},
							.sn = {255,255,255,0},
							.gw = {169, 254, 216, 1},
							.dns = {0,0,0,0},
							.dhcp = NETINFO_STATIC };

//states for multythread http
#define HTTP_IDLE 0
#define HTTP_SENDING 1

//variables for multythread http
uint32_t sentsize[_WIZCHIP_SOCK_NUM_];
uint32_t filesize[_WIZCHIP_SOCK_NUM_];
uint8_t http_state[_WIZCHIP_SOCK_NUM_];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

// initialize the dependent host peripheral
void platform_init(void);
void reverse(char s[]);
void new_itoa(int n, char s[]);

//////////
// TODO //
//////////////////////////////////////////////////////////////////////////////////////////////
// Call back function for W5500 SPI - Theses used as parameter of reg_wizchip_xxx_cbfunc()  //
// Should be implemented by WIZCHIP users because host is dependent                         //
//////////////////////////////////////////////////////////////////////////////////////////////
void  wizchip_select(void);
void  wizchip_deselect(void);
void  wizchip_write(uint8_t wb);
uint8_t wizchip_read();
//////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////
// For example of ioLibrary_BSD //
//////////////////////////////////
void network_init(void);								// Initialize Network information and display it
int32_t tcp_http_mt(uint8_t, uint8_t*, uint16_t);		// Multythread TCP server
void HTTP_reset(uint8_t sockn);
//////////////////////////////////

//Define protótipos para funções de leitura dos canais analógicos configurados
uint32_t Ain_Read_value(uint32_t channel);

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
	uint8_t i;
	uint8_t memsize[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
#if (ENABLE_MB_RTU == 1)&&(ENABLE_MB_TCP == 0)
  __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
#endif

   ////////////////////////////////////////////////////////////////////////////////////////////////////
   // First of all, Should register SPI callback functions implemented by user for accessing WIZCHIP //
   ////////////////////////////////////////////////////////////////////////////////////////////////////

   /* Chip selection call back */

#if   _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_VDM_
	reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
#elif _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_FDM_
	reg_wizchip_cs_cbfunc(wizchip_select, wizchip_select);  // CS must be tried with LOW.
#else
   #if (_WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SIP_) != _WIZCHIP_IO_MODE_SIP_
	  #error "Unknown _WIZCHIP_IO_MODE_"
   #else
	  reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
   #endif
#endif

	 /* SPI Read & Write callback function */
	reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);

	////////////////////////////////////////////////////////////////////////
	/* WIZCHIP SOCKET Buffer initialize */
	if(ctlwizchip(CW_INIT_WIZCHIP,(void*)memsize) == -1)
	{
	   //init fail
	   while(1);
	}

	/* Network initialization */
	network_init();

	//all connections inactive
	for(i=0;i<_WIZCHIP_SOCK_NUM_;i++)
	HTTP_reset(i);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	/* Loopback Test */
  	// TCP server
	//4 sockets para webserver http
	for(i=4;i<_WIZCHIP_SOCK_NUM_;i++)
	{
		tcp_http_mt(i, gDATABUF, 80);
	}
	//4 sockets para modbus TCP
	for(i=0;i<4;i++)
	{
		tcp_http_mt(i, gDATABUF, 502);
	}
	//Delayms(10);

	#if (ENABLE_MB_RTU == 1)&&(ENABLE_MB_TCP == 0)
  		//Se estava recebendo frames no canal USART e parou de receber por mais de 3ms
  		//trata como se tivesse finalizado a recepção do frame MODBUS
  		if((flag_busy_RTU == 1)&&(DELAY_Time_2() >= 10))
  		{
  			flag_busy_RTU = 0;
  			holding_registers[1] = Ain_Read_value(LDR_VALUE);
			MODBUS_receive_task(RX_buffer, TX_buffer, holding_registers, sizeof(TX_buffer), sizeof(RX_buffer), &FrameNrBytes);
			HAL_UART_Transmit(&huart2, TX_buffer, FrameNrBytes, 100);
			__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
			memset(RX_buffer,0,sizeof(RX_buffer));
			memset(TX_buffer,0,sizeof(TX_buffer));
			cntBufferRTU = 0;

			if(holding_registers[0]&0b0000000000000001){HAL_GPIO_WritePin(DOUT_LED1_GPIO_Port, DOUT_LED1_Pin, GPIO_PIN_SET);}
			else{HAL_GPIO_WritePin(DOUT_LED1_GPIO_Port, DOUT_LED1_Pin, GPIO_PIN_RESET);}
			if(holding_registers[0]&0b0000000000000010){HAL_GPIO_WritePin(DOUT_LED2_GPIO_Port, DOUT_LED2_Pin, GPIO_PIN_SET);}
			else{HAL_GPIO_WritePin(DOUT_LED2_GPIO_Port, DOUT_LED2_Pin, GPIO_PIN_RESET);}
			if(holding_registers[0]&0b0000000000000100){HAL_GPIO_WritePin(DOUT_LED3_GPIO_Port, DOUT_LED3_Pin, GPIO_PIN_SET);}
			else{HAL_GPIO_WritePin(DOUT_LED3_GPIO_Port, DOUT_LED3_Pin, GPIO_PIN_RESET);}
			if(holding_registers[0]&0b0000000000001000){HAL_GPIO_WritePin(DOUT_LED4_GPIO_Port, DOUT_LED4_Pin, GPIO_PIN_SET);}
			else{HAL_GPIO_WritePin(DOUT_LED4_GPIO_Port, DOUT_LED4_Pin, GPIO_PIN_RESET);}
			if(holding_registers[0]&0b0000000000010000){HAL_GPIO_WritePin(DOUT_LED5_GPIO_Port, DOUT_LED5_Pin, GPIO_PIN_SET);}
			else{HAL_GPIO_WritePin(DOUT_LED5_GPIO_Port, DOUT_LED5_Pin, GPIO_PIN_RESET);}
			if(holding_registers[0]&0b0000000000100000){HAL_GPIO_WritePin(DOUT_LED6_GPIO_Port, DOUT_LED6_Pin, GPIO_PIN_SET);}
			else{HAL_GPIO_WritePin(DOUT_LED6_GPIO_Port, DOUT_LED6_Pin, GPIO_PIN_RESET);}
			if(holding_registers[0]&0b0000000001000000){HAL_GPIO_WritePin(DOUT_LED7_GPIO_Port, DOUT_LED7_Pin, GPIO_PIN_SET);}
			else{HAL_GPIO_WritePin(DOUT_LED7_GPIO_Port, DOUT_LED7_Pin, GPIO_PIN_RESET);}
			if(holding_registers[0]&0b0000000010000000){HAL_GPIO_WritePin(DOUT_LED8_GPIO_Port, DOUT_LED8_Pin, GPIO_PIN_SET);}
			else{HAL_GPIO_WritePin(DOUT_LED8_GPIO_Port, DOUT_LED8_Pin, GPIO_PIN_RESET);}
  		}
	#endif
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_W5500_CS_Pin|DOUT_LED5_Pin|DOUT_LED6_Pin|DOUT_LED7_Pin
                          |DOUT_LED8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DOUT_LED1_Pin|DOUT_LED2_Pin|DOUT_LED3_Pin|DOUT_LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPIO_W5500_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_W5500_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIO_W5500_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DOUT_LED1_Pin DOUT_LED2_Pin DOUT_LED3_Pin DOUT_LED4_Pin */
  GPIO_InitStruct.Pin = DOUT_LED1_Pin|DOUT_LED2_Pin|DOUT_LED3_Pin|DOUT_LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DOUT_LED5_Pin DOUT_LED6_Pin DOUT_LED7_Pin DOUT_LED8_Pin */
  GPIO_InitStruct.Pin = DOUT_LED5_Pin|DOUT_LED6_Pin|DOUT_LED7_Pin|DOUT_LED8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//Chamada da função de callback do systick (1ms)
void HAL_SYSTICK_Callback()
{
	//Chama função de incremento de contadores da biblioteca de delay
	DelayIncCnt();
}


uint8_t W5500_rxtx(uint8_t data)
{
	uint8_t rxdata;

	HAL_SPI_TransmitReceive(&hspi1, &data, &rxdata, 1, 50);

	return (rxdata);
}

//////////
// TODO //
/////////////////////////////////////////////////////////////////
// SPI Callback function for accessing WIZCHIP                 //
// WIZCHIP user should implement with your host spi peripheral //
/////////////////////////////////////////////////////////////////
void  wizchip_select(void)
{
	W5500_select();
}

void  wizchip_deselect(void)
{
	W5500_release();
}

void  wizchip_write(uint8_t wb)
{
	W5500_tx(wb);
}

uint8_t wizchip_read()
{
   return W5500_rx();
}
//////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////
// Intialize the network information to be used in WIZCHIP //
/////////////////////////////////////////////////////////////
void network_init(void)
{
   uint8_t tmpstr[6];

	ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);

	ctlwizchip(CW_GET_ID,(void*)tmpstr);
}
/////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////
// HTTP Multythread Example Code using ioLibrary_BSD         //
///////////////////////////////////////////////////////////////
#define len(some) (sizeof(some)/sizeof(some[0]))


const char http_404_full[] =
	"HTTP/1.0 404 Not Found\r\n"
	"Content-Type: text/html;"
	"Server: STM32+W5500\r\n"
	"\r\n"
	"<pre>Page not found\r\n\r\n";


const char  http_200[] = "HTTP/1.0 200 OK\r\n";
const char http_server[] = "Server: STM32+W5500\r\n";
const char http_connection_close[] = "Connection: close\r\n";
const char http_content_type[] = "Content-Type: ";
const char http_content_length[] = "Content-Length: ";
const char  http_linebreak[] = "\r\n";
const char  http_header_end[] = "\r\n\r\n";
const char http_not_found[] = "<h1>404 - Not Found</h1>";

const char http_text_plain[] = "text/plain";
const char http_text_html[] = "text/html";
const char http_text_js[] = "text/javascript";
const char  http_text_css[] = "text/css";
const char http_image_gif[] = "image/gif";
const char http_image_png[] = "image/png";
const char  http_image_jpeg[] = "image/jpeg";
const char  http_video_mp4[] = "video/mp4";
const char  http_video_avi[] = "video/avi";

char default_page[]="index.htm";


// get mime type from filename extension
const char *httpd_get_mime_type(char *url)
{
	const char *t_type;
	char *ext;

     t_type = http_text_plain;

	if((ext = strrchr(url, '.')))
	{
		ext++;
		strlwr(ext);

	    if(strcmp(ext, "txt")==0)
		t_type = http_text_plain;

	    else if(strcmp(ext, "htm")==0)
		t_type = http_text_html;

	    else if(strcmp(ext, "html")==0)
		t_type = http_text_html;

	    else if(strcmp(ext, "js")==0)
		t_type = http_text_js;

	    else if(strcmp(ext, "css")==0)
		t_type = http_text_css;

	    else if(strcmp(ext, "gif")==0)
		t_type = http_image_gif;

	    else if(strcmp(ext, "png")==0)
		t_type = http_image_png;

	    else if(strcmp(ext, "jpg")==0)
		t_type = http_image_jpeg;

	    else if(strcmp(ext, "jpeg")==0)
		t_type = http_image_jpeg;

	    else if(strcmp(ext, "mp4")==0)
		t_type = http_video_mp4;

	    else if(strcmp(ext, "avi")==0)
		t_type = http_video_avi;

	}

	return t_type;
}


//http server

void HTTP_reset(uint8_t sockn)
{
    sentsize[sockn]=0;
	http_state[sockn]=HTTP_IDLE;
}

int32_t tcp_http_mt(uint8_t sn, uint8_t* buf, uint16_t port)
{
   int32_t ret;
   uint32_t size = 0;
   uint16_t cntBytesTX;
   uint32_t sampleTX;
   uint8_t flagHtmlGen = 0;
   char *url,*p;
   uint16_t blocklen=0;

   switch(getSn_SR(sn))
   {
      case SOCK_ESTABLISHED :

         if(getSn_IR(sn) & Sn_IR_CON)
         {
            setSn_IR(sn,Sn_IR_CON);
         }

         if((size = getSn_RX_RSR(sn)) > 0)
         {
            if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
            ret = recv(sn,buf,size);

            HTTP_reset(sn);

            if(ret <= 0)
            return ret;

            url =(char*) buf + 4;

            if((http_state[sn]==HTTP_IDLE)&&(memcmp(buf, "GET ", 4)==0)&&((p = strchr(url, ' '))))// extract URL from request header
            {
              *(p++) = 0;//making zeroed url string

				if ((url != NULL)&&(strncmp("/favicon.ico",url,12) != 0))
				{
					flagHtmlGen = 1;
					if (strncmp("/S1/ON",url,6) == 0)
					{
						S1 = 1;
						HAL_GPIO_WritePin(DOUT_LED1_GPIO_Port, DOUT_LED1_Pin, GPIO_PIN_SET);
					}
					else if (strncmp("/S1/OFF",url,6) == 0)
					{
						S1 = 0;
						HAL_GPIO_WritePin(DOUT_LED1_GPIO_Port, DOUT_LED1_Pin, GPIO_PIN_RESET);
					}
					else if (strncmp("/S2/ON",url,6) == 0)
					{
						S2 = 1;
						HAL_GPIO_WritePin(DOUT_LED2_GPIO_Port, DOUT_LED2_Pin, GPIO_PIN_SET);
					}
					else if (strncmp("/S2/OFF",url,6) == 0)
					{
						S2 = 0;
						HAL_GPIO_WritePin(DOUT_LED2_GPIO_Port, DOUT_LED2_Pin, GPIO_PIN_RESET);
					}
					else if (strncmp("/S3/ON",url,6) == 0)
					{
						S3 = 1;
						HAL_GPIO_WritePin(DOUT_LED3_GPIO_Port, DOUT_LED3_Pin, GPIO_PIN_SET);
					}
					else if (strncmp("/S3/OFF",url,6) == 0)
					{
						S3 = 0;
						HAL_GPIO_WritePin(DOUT_LED3_GPIO_Port, DOUT_LED3_Pin, GPIO_PIN_RESET);
					}
					else if (strcmp("/DOWNLOAD.txt",url) == 0)
					{
						strcpy((char*)buf,"HTTP/1.0 200 OK\r\nContent-Type: application/octet-stream\r\n\r\n");
						cntBytesTX = 0;
						sampleTX = 0;
						do
						{

							memset(&writeValue,0,sizeof(writeValue));
							strcpy(writeValue, "23/12/2017 ; 10:50:00 ; 28ºC \r\n");
							//print_text(&writeValue[0]);
							strcat((char*)buf, writeValue);
							cntBytesTX += strlen(writeValue);
							sampleTX++;

							if (cntBytesTX>=1000)
							{
								cntBytesTX = 0;
								blocklen = strlen((char*)buf);
								ret = send(sn,buf,blocklen);
								memset(buf,0,DATA_BUF_SIZE);
								if(ret < 0)
								{
									close(sn);
									return ret;
								}
								Delayus(100);
							}
						} while (sampleTX < 100000);

						Delayms(1);
						flagHtmlGen = 0;
						HTTP_reset(sn);
						disconnect(sn);

					}

					//Geração da HTML
					if(flagHtmlGen == 1)
					{
						strcpy((char*)buf,"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n");
						strcat((char*)buf, "<html><head>");
						strcat((char*)buf, "<title>Eletrocursos W5500 Web Server</title>");
						strcat((char*)buf, "<link href='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.1/css/bootstrap.min.css' rel='stylesheet'></link>");
						strcat((char*)buf, "</head>");
						strcat((char*)buf, "<body>");
						strcat((char*)buf, "<div class='jumbotron'>");
						strcat((char*)buf, "<h2>ELETROCURSOS - Interface De Comando</h2>");

						strcat((char*)buf, "<div class='row'>");
						strcat((char*)buf, "<div class='col-md-3'>");
						strcat((char*)buf, "<table class='table table-bordered'>");
						strcat((char*)buf, "<tbody>");

						strcat((char*)buf, "<tr>");
						strcat((char*)buf, "<td>Saida 1 - ");
						if(S1 == 1)
						{
							strcat((char*)buf, "On");
							strcat((char*)buf, "</td><td><a href='/S1/OFF'>DESLIGAR</a>");
						}
						else
						{
							strcat((char*)buf, "Off");
							strcat((char*)buf, "</td><td><a href='/S1/ON'>LIGAR</a>");
						}

						strcat((char*)buf, "</td>");
						strcat((char*)buf, "</tr>");
						strcat((char*)buf, "<td>Saida 2 - ");
						if(S2 == 1)
						{
							 strcat((char*)buf, "On");
							 strcat((char*)buf, "</td><td><a href='/S2/OFF'>DESLIGAR</a>");
						}
						else
						{
							 strcat((char*)buf, "Off");
							 strcat((char*)buf, "</td><td><a href='/S2/ON'>LIGAR</a>");
						}

						strcat((char*)buf, "</td>");
						strcat((char*)buf, "</tr>");
						strcat((char*)buf, "<td>Saida 3 - ");
						if(S3 == 1)
						{
							strcat((char*)buf, "On");
							strcat((char*)buf, "</td><td><a href='/S3/OFF'>DESLIGAR</a>");
						}
						else
						{
							 strcat((char*)buf, "Off");
							 strcat((char*)buf, "</td><td><a href='/S3/ON'>LIGAR</a>");
						}

						strcat((char*)buf, "</td>");
						strcat((char*)buf, "</tr>");

						strcat((char*)buf, "<td>Download");
						strcat((char*)buf, "</td><td><a href='/DOWNLOAD.txt'>DOWNLOAD</a>");
						strcat((char*)buf, "</td>");
						strcat((char*)buf, "</html>");

						blocklen = strlen((char*)buf);

						ret = send(sn,buf,blocklen);
						if(ret < 0)
						{
							close(sn);
							return ret;
						}
						else
						{
							HTTP_reset(sn);
							disconnect(sn);
						}
					}
				}
				else
				{
//					strcpy((char*)buf,http_404_full);
//					size=strlen((char*)buf);
//					ret=send(sn,buf,size);
//					if(ret < 0)
//					{
//						close(sn);
//						return ret;
//					}
					HTTP_reset(sn);
					disconnect(sn);
				}

        	  }
			#if (ENABLE_MB_RTU == 0)&&(ENABLE_MB_TCP == 1)
			else if(http_state[sn]==HTTP_IDLE)
            {
				holding_registers[1] = Ain_Read_value(LDR_VALUE);

				memcpy(RX_buffer, buf, sizeof(RX_buffer));
				MODBUS_receive_task(RX_buffer, TX_buffer, holding_registers, sizeof(TX_buffer), sizeof(RX_buffer), &FrameNrBytes);
				memset(buf,0,DATA_BUF_SIZE);
				memcpy(buf, TX_buffer, FrameNrBytes);

				ret = send(sn,buf,FrameNrBytes);
				if(ret < 0)
				{
					close(sn);
					return ret;
				}

				if(holding_registers[0]&0b0000000000000001){HAL_GPIO_WritePin(DOUT_LED1_GPIO_Port, DOUT_LED1_Pin, GPIO_PIN_SET);}
				else{HAL_GPIO_WritePin(DOUT_LED1_GPIO_Port, DOUT_LED1_Pin, GPIO_PIN_RESET);}
				if(holding_registers[0]&0b0000000000000010){HAL_GPIO_WritePin(DOUT_LED2_GPIO_Port, DOUT_LED2_Pin, GPIO_PIN_SET);}
				else{HAL_GPIO_WritePin(DOUT_LED2_GPIO_Port, DOUT_LED2_Pin, GPIO_PIN_RESET);}
				if(holding_registers[0]&0b0000000000000100){HAL_GPIO_WritePin(DOUT_LED3_GPIO_Port, DOUT_LED3_Pin, GPIO_PIN_SET);}
				else{HAL_GPIO_WritePin(DOUT_LED3_GPIO_Port, DOUT_LED3_Pin, GPIO_PIN_RESET);}
				if(holding_registers[0]&0b0000000000001000){HAL_GPIO_WritePin(DOUT_LED4_GPIO_Port, DOUT_LED4_Pin, GPIO_PIN_SET);}
				else{HAL_GPIO_WritePin(DOUT_LED4_GPIO_Port, DOUT_LED4_Pin, GPIO_PIN_RESET);}
				if(holding_registers[0]&0b0000000000010000){HAL_GPIO_WritePin(DOUT_LED5_GPIO_Port, DOUT_LED5_Pin, GPIO_PIN_SET);}
				else{HAL_GPIO_WritePin(DOUT_LED5_GPIO_Port, DOUT_LED5_Pin, GPIO_PIN_RESET);}
				if(holding_registers[0]&0b0000000000100000){HAL_GPIO_WritePin(DOUT_LED6_GPIO_Port, DOUT_LED6_Pin, GPIO_PIN_SET);}
				else{HAL_GPIO_WritePin(DOUT_LED6_GPIO_Port, DOUT_LED6_Pin, GPIO_PIN_RESET);}
				if(holding_registers[0]&0b0000000001000000){HAL_GPIO_WritePin(DOUT_LED7_GPIO_Port, DOUT_LED7_Pin, GPIO_PIN_SET);}
				else{HAL_GPIO_WritePin(DOUT_LED7_GPIO_Port, DOUT_LED7_Pin, GPIO_PIN_RESET);}
				if(holding_registers[0]&0b0000000010000000){HAL_GPIO_WritePin(DOUT_LED8_GPIO_Port, DOUT_LED8_Pin, GPIO_PIN_SET);}
				else{HAL_GPIO_WritePin(DOUT_LED8_GPIO_Port, DOUT_LED8_Pin, GPIO_PIN_RESET);}

         }
		 #endif
         }
         break;
      case SOCK_CLOSE_WAIT :

    	  HTTP_reset(sn);

         if((ret=disconnect(sn)) != SOCK_OK)
         return ret;

         break;
      case SOCK_INIT :

    	  HTTP_reset(sn);

         if( (ret = listen(sn)) != SOCK_OK) return ret;
         break;
      case SOCK_CLOSED:

    	  HTTP_reset(sn);

         if((ret=socket(sn,Sn_MR_TCP,port,0x00)) != sn)
         return ret;

         break;

      default:
    	  HTTP_reset(sn);
         break;
   }
   return 1;
}



//****************************************************
// reverse:
void reverse(char s[])
{
    int i, j;
    char c;

    for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}

//*******************************************************
// itoa
 void new_itoa(int n, char s[])
 {
     int i, sign;

     if ((sign = n) < 0)  /* sign */
         n = -n;          /* make positive n */
     i = 0;
     do {       /* generate reverse */
         s[i++] = n % 10 + '0';   /* next digit */
     } while ((n /= 10) > 0);     /* delete */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
 }

 uint32_t Ain_Read_value(uint32_t channel)
 {
   uint32_t result;
   ADC_ChannelConfTypeDef sConfig;

   sConfig.Channel = channel;
   sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
   sConfig.Rank = ADC_REGULAR_RANK_1;

   HAL_ADC_ConfigChannel(&hadc1, &sConfig);

   HAL_ADCEx_Calibration_Start(&hadc1);

   HAL_ADC_Start(&hadc1);
   HAL_ADC_PollForConversion(&hadc1, 1000);
   result = HAL_ADC_GetValue(&hadc1);
   HAL_ADC_Stop(&hadc1);

   return result;
 }

#if (ENABLE_MB_RTU == 1)&&(ENABLE_MB_TCP == 0)
 //Armazena a informação recebida via ModbusRTU em um buffer
void RTU_RX_Int(UART_HandleTypeDef* huart)
{
	uint8_t c;

	if(huart->Instance == USART2)
	{
		HAL_UART_Receive(huart, &c, 1,100);
		if(cntBufferRTU < 128)
		{
			RX_buffer[cntBufferRTU] = c;
			cntBufferRTU++;
		}
		flag_busy_RTU = 1;
		DELAY_SetTime_2(0);
	}

}
#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
