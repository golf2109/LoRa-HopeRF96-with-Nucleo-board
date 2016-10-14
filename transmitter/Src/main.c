/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */

#define	CS		GPIO_PIN_12	//A4 - CS
#define	DIO0	GPIO_PIN_6
#define	DIO5	GPIO_PIN_7
#define PACKAGE_LENGTH 16

#define	len	0x10

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t	package_length = 16;
uint8_t spiTxData[2];
uint8_t spiRxData[2];
uint8_t	send_SPI[0x04];
uint8_t	get_SPI[0x04];
uint8_t	iii,temp0,temp1,temp2,temp3,base_addr;
uint8_t	transmitLoRa[len] ={0x55,0xAA};
uint8_t	RegNumber[0x80];
uint16_t ttt,ttt1,x,y;
uint8_t	dig[3];
uint8_t buffer[10];
uint32_t	packet_number=0;
char	bbb[16] =  {0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x0A,0x0D};
char* ptrbbb;
uint8_t	receive_UART[16] = {0};
uint8_t	buffer_receive_UART[16] = {0};
uint8_t	transmit_UART[32] = {0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
													   0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x0A,0x0D};
uint8_t	receive_pack[16] = {0};
uint8_t	addr_lora,addr_lora1,addr_lora2,data_lora,data_lora1,data_lora2;
  unsigned char i;
  unsigned char UART_Data;
  unsigned char RFM_Package[256]={0};

uint8_t	read_reg;
uint8_t	F_23_16,F_15_8,F_7_0;
uint8_t	l_100,l_10,l_1;
uint8_t	f_100000,f_10000,f_1000,f_100,f_10,f_1;
uint32_t freq_hex1,freq_hex2;
uint8_t power_bits;
uint8_t spread;
uint8_t	time_sec;
extern	uint8_t* out1,out2;
extern	uint8_t spiTxData[2];
extern	uint8_t spiRxData[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*
*****************************************************************************************
* Description:  Function is used to read a value from a specific register of the RFM module
*
* Arguments:    RFM_Adress  Adress of the register to read
*
* Return:       The value of the register is returned
*****************************************************************************************
*/
unsigned char RFM_Read(uint8_t RFM_Address)
{
spiTxData[0] = 	RFM_Address & 0x7F;
  //Set NSS pin low to start SPI communication
HAL_GPIO_WritePin(GPIOB, CS, GPIO_PIN_RESET);//CS = 0	

HAL_SPI_TransmitReceive_DMA(&hspi2, spiTxData , spiRxData, 2);
  //Set NSS high to end communication

HAL_GPIO_WritePin(GPIOB, CS, GPIO_PIN_SET);//CS = 1	
  //Return received data
  return spiRxData[1];
}

/*
*****************************************************************************************
* Description:    Function is used to write a value to a specific register of the RFM module
*
* Arguments:      RFM_Adress  Adress of the register to be written
                  RFM_Data    Data that will be written
*****************************************************************************************
*/
void RFM_Write(uint8_t RFM_Address, uint8_t RFM_Data)
{
  //Set NSS pin Low to start communication
HAL_GPIO_WritePin(GPIOB, CS, GPIO_PIN_RESET);//CS = 0	

  //Send Addres with MSB 1 to make it a writ command
spiTxData[0] = 	RFM_Address | 0x80;
spiTxData[1] = 	RFM_Data;	
  //Send Data
HAL_SPI_TransmitReceive_DMA(&hspi2, spiTxData , spiRxData, 2);

  //Set NSS pin High to end communication
HAL_GPIO_WritePin(GPIOB, CS, GPIO_PIN_SET);//CS = 1		
}

/*
*****************************************************************************************
* Description  :   Function is used to send a message of 10 bytes
*
* Arguments   :   *RFM_Package Pointer to the array with the data to send
*****************************************************************************************
*/
void RFM_SendPackage(unsigned char *RFM_Package)
{
  unsigned char i;

  //Switch RFM to standby
  RFM_Write(0x01,0x81);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 

  //Set SPI pointer to Tx base address
  RFM_Write(0x0D,0x80);

  //Switch DIO to TxDone
  RFM_Write(0x40,0x40);

  //Write payload in to the fifo
  for(i = 0; i < package_length; i++)
  {
    RFM_Write(0x00,*RFM_Package);
    RFM_Package++;
  }

  //Switch RFM to TX
  RFM_Write(0x01,0x83);

  //Wait on TxDone
  while(HAL_GPIO_ReadPin(GPIOC, DIO0) == 0){} 

  //Clear interrupt
  RFM_Write(0x12,0x08);

  //Set DIO0 to RxDone
  RFM_Write(0x40,0x00);

   //Set RFM in continues receive
  RFM_Write(0x01,0x85);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){}   
}

/*
*****************************************************************************************
* Description:  Function is to get a message of 10 bytes received by the RFM
*
* Arguments:    *RFM_Package  Pointer to the array with the data to send
*****************************************************************************************
*/
void RFM_ReadPackage(unsigned char *RFM_Package)
{
  unsigned char i;
//  unsigned char RFM_Interrupt;
  unsigned char RFM_PackageLocation;

  //Get interrupt register
//  RFM_Interrupt = RFM_Read(0x12);

  //Switch RFM to Standby
  RFM_Write(0x01,0x81);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 

  //Clear interrupt register
  RFM_Write(0x12,0x60);

  //Get package location
  RFM_PackageLocation = RFM_Read(0x10);

  //Set SPI pointer to Packagelocation
  RFM_Write(0x0D,RFM_PackageLocation);

  //Get message
  for(i = 0; i < package_length; i++)
  {
    *RFM_Package = RFM_Read(0x00);
    RFM_Package++;
  }

  //Switch RFM to receive
  RFM_Write(0x01,0x85);
  //Wait for mode ready   
} 

/*
*****************************************************************************************
* Description  :   Initialisation of the RFM module. 
*                  Check the datasheet if you want to use other settings
*****************************************************************************************
*/
void RFM_Init()
{
  //Setting RFM in sleep
  RFM_Write(0x01,0x00);

  //Switch to LoRa mode
  RFM_Write(0x01,0x80);

  //Setting RFM to standby
  RFM_Write(0x01,0x81);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 

  //Set carrier frequency
  //433.175 MHz / 61.035 Hz = 7097157 = 0x6C4B45
  RFM_Write(0x06,0x6C);
  RFM_Write(0x07,0x4B);
  RFM_Write(0x08,0x45);

  //Set Pa pin to maximal power
  RFM_Write(0x09,0xFF);

  //Bandwith 250 kHz, Coding rate = 4/8, Implicit header mode
  RFM_Write(0x1D,0x8B);

  //Spreading factor 6, PayloadCRC on
  RFM_Write(0x1E,0x64);
  //Setting additional register for SF6 use other settings for the other SF
  RFM_Write(0x31,0xC5);
  RFM_Write(0x37,0x0C);

  //Preamble length 0x0018 + 4 = 28
  RFM_Write(0x20,0x00);
  RFM_Write(0x21,0x18);

  //Payload length
  RFM_Write(0x22,package_length);

  //Set RFM in continues receive
  RFM_Write(0x01,0x85);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 
}

void	digit(uint8_t	hexdig)
{
dig[0] = 0x30 + hexdig/100;
dig[1] = 0x30 + (hexdig - 100*(hexdig/100))/10;
dig[2] = 0x30 + (hexdig - 100*(hexdig/100) - 10*((hexdig - 100*(hexdig/100))/10));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
void	hex_string(uint32_t packet_number, uint8_t* string_packet)
{
	uint8_t u[8];
	u[7] = packet_number/10000000;
	*string_packet = 0x30 + u[7];
	u[6] = (packet_number - u[7]*10000000)/1000000;
	*(string_packet + 1) = 0x30 + u[6];
	u[5] = (packet_number - u[7]*10000000 - u[6]*1000000)/100000;
	*(string_packet + 2) = 0x30 + u[5];
	u[4] = (packet_number - u[7]*10000000 -u [6]*1000000 - u[5]*100000)/10000;
	*(string_packet + 3) = 0x30  + u[4];
	u[3] = (packet_number - u[7]*10000000 - u[6]*1000000 - u[5]*100000 - u[4]*10000)/1000;
	*(string_packet + 4) = 0x30 + u[3];
	u[2] = (packet_number - u[7]*10000000 - u[6]*1000000 - u[5]*100000 - u[4]*10000 - u[3]*1000)/100;
	*(string_packet + 5) = 0x30 + u[2];
	u[1] = (packet_number - u[7]*10000000 - u[6]*1000000 - u[5]*100000 - u[4]*10000 - u[3]*1000 - u[2]*100)/10;
	*(string_packet + 6) = 0x30 + u[1];
	u[0] = (packet_number - u[7]*10000000 - u[6]*1000000 - u[5]*100000 - u[4]*10000 - u[3]*1000 - u[2]*100 - u[1]*10);
	*(string_packet + 7) = 0x30 + u[0];
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */


  RFM_Package[0] = 0x20; RFM_Package[1] = 0x20; RFM_Package[2] = 0x20; RFM_Package[3] = 0x20; RFM_Package[4] = 0x20;
  RFM_Package[5] = 0x20; RFM_Package[6] = 0x20; RFM_Package[7] = 0x20; RFM_Package[8] = 0x20; RFM_Package[9] = 0x20;
  RFM_Package[0xA] = 0x20; RFM_Package[0xB] = 0x20; RFM_Package[0xC] = 0x20; RFM_Package[0xD] = 0x20;
	RFM_Package[0xE] = 0x0D;
	RFM_Package[0xF] = 0x0A;	

  //Initialieze the RFM module
  RFM_Init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
HAL_UART_Receive_DMA(&huart2, buffer_receive_UART, 12);
//-------     TEST receive command S (send to registers)		
		if((buffer_receive_UART[0] == 0x53) && (buffer_receive_UART[11] == 0x29)) 
		{
						for(iii=0;iii<=15;iii++)
							{
								receive_UART[iii] = buffer_receive_UART[iii];
							}
						switch (receive_UART [4])
						{
							case 0x41:
								addr_lora1 = 0xA0;
								break;
							case 0x42:
								addr_lora1 = 0xB0;
								break;														
							case 0x43:
								addr_lora1 = 0xC0;
								break;
							case 0x44:
								addr_lora1 = 0xD0;
								break;
							case 0x45:
								addr_lora1 = 0xE0;
								break;	
							case 0x46:
								addr_lora1 = 0xF0;
								break;		
							default:
							addr_lora1 = (0x0F & receive_UART[4]) << 4;		
						}
						
						switch (receive_UART [5])
						{
							case 0x41:
								addr_lora2 = 0xA;
								break;
							case 0x42:
								addr_lora2 = 0xB;
								break;														
							case 0x43:
								addr_lora2 = 0xC;
								break;
							case 0x44:
								addr_lora2 = 0xD;
								break;
							case 0x45:
								addr_lora2 = 0xE;
								break;	
							case 0x46:
								addr_lora2 = 0xF;
								break;		
							default:
							addr_lora2 = 0x0F & receive_UART[5];		
						}						
						addr_lora = addr_lora1 + addr_lora2;
							
						switch (receive_UART [9])
						{
							case 0x41:
								data_lora1 = 0xA0;
								break;
							case 0x42:
								data_lora1 = 0xB0;
								break;														
							case 0x43:
								data_lora1 = 0xC0;
								break;
							case 0x44:
								data_lora1 = 0xD0;
								break;
							case 0x45:
								data_lora1 = 0xE0;
								break;	
							case 0x46:
								data_lora1 = 0xF0;
								break;		
							default:
							data_lora1 = (0x0F & receive_UART[9]) << 4;		
						}
						
						switch (receive_UART [0x0A])
						{
							case 0x41:
								data_lora2 = 0xA;
								break;
							case 0x42:
								data_lora2 = 0xB;
								break;														
							case 0x43:
								data_lora2 = 0xC;
								break;
							case 0x44:
								data_lora2 = 0xD;
								break;
							case 0x45:
								data_lora2 = 0xE;
								break;	
							case 0x46:
								data_lora2 = 0xF;
								break;		
							default:
							data_lora2 = 0x0F & receive_UART[0x0A];		
						}						
						data_lora = data_lora1 + data_lora2;		

  //Setting RFM to standby
  RFM_Write(0x01,0x81);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 						
		
						RFM_Write(addr_lora,data_lora);	//go SLEEP MODE !!!!
						read_reg = RFM_Read(addr_lora);
  //Set RFM in continues receive
  RFM_Write(0x01,0x85);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 						
						
						char *ok = "  OK               ";			
						for(iii=0;iii<=13;iii++)
						{
							transmit_UART[iii] = *(ok + iii);
						}									
						HAL_UART_Transmit_DMA(&huart2, transmit_UART, 32); 									
						
						for(iii=0;iii<=15;iii++)
							{
								buffer_receive_UART[iii] = 0;
							}						
		}
		
//-------     TEST receive command F (set frequency) 
		if((buffer_receive_UART[0] == 0x46) && (buffer_receive_UART[8] == 0x29))
		{
						for(iii=0;iii<=15;iii++)
							{
								receive_UART[iii] = buffer_receive_UART[iii];
							}		
			f_100000 = 
			f_10000	= 0xF & receive_UART[3];
			f_1000 = 0xF & receive_UART[4];
			f_100	= 0xF & receive_UART[5];
			f_10 = 0xF & receive_UART[6];
			f_1	= 0xF & receive_UART[7];
					
			freq_hex1 = (100000*f_100000 + 10000*f_10000 + 1000*f_1000 + 100*f_100 + 10*f_10 +f_1)*1000;
			freq_hex2 = freq_hex1 / 61.035;
			F_7_0 = 0xFF & freq_hex2;
			F_15_8 = 0xFF & (freq_hex2 >>8);
			F_23_16 = 0xFF & (freq_hex2 >>16);		

  //Setting RFM to standby
  RFM_Write(0x01,0x81);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 						
		
  //Set carrier frequency
  //433.175 MHz / 61.035 Hz = 7097157 = 0x6C4B45
  RFM_Write(0x06,F_23_16);
  RFM_Write(0x07,F_15_8);
  RFM_Write(0x08,F_7_0);		
				
  //Set RFM in continues receive
  RFM_Write(0x01,0x85);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 						
						
						char *ok = "  OK               ";			
						for(iii=0;iii<=13;iii++)
						{
							transmit_UART[iii] = *(ok + iii);
						}									
						HAL_UART_Transmit_DMA(&huart2, transmit_UART, 32); 									
						
						for(iii=0;iii<=15;iii++)
							{
								buffer_receive_UART[iii] = 0;
							}									
		}
		
		
		
//-------     TEST receive command P (set power) 1- min 1,  7 - max
		if((buffer_receive_UART[0] == 0x50) && (buffer_receive_UART[3] == 0x29))
		{
						for(iii=0;iii<=15;iii++)
							{
								receive_UART[iii] = buffer_receive_UART[iii];
							}		
						power_bits = 0x7 & receive_UART[2];						
						power_bits = power_bits << 4;
						power_bits = 0x1F | power_bits;

  //Setting RFM to standby
  RFM_Write(0x01,0x81);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 						
		

  RFM_Write(0x09,power_bits);		
				
  //Set RFM in continues receive
  RFM_Write(0x01,0x85);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 						
						
						char *ok = "  OK               ";			
						for(iii=0;iii<=13;iii++)
						{
							transmit_UART[iii] = *(ok + iii);
						}									
						HAL_UART_Transmit_DMA(&huart2, transmit_UART, 32); 									
						
						for(iii=0;iii<=15;iii++)
							{
								buffer_receive_UART[iii] = 0;
							}																
							
		}			
//-------     TEST receive command B (set signal bandwidth) 1 - 7.8 kHz, 9 - 500 kHz
		if((buffer_receive_UART[0] == 0x42) && (buffer_receive_UART[3] == 0x29))
		{
						for(iii=0;iii<=15;iii++)
							{
								receive_UART[iii] = buffer_receive_UART[iii];
							}		
						power_bits = 0xF & receive_UART[2];						
						power_bits = power_bits << 4;
						power_bits = 0xB | power_bits;

  //Setting RFM to standby
  RFM_Write(0x01,0x81);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 						
		

  RFM_Write(0x1D,power_bits);		
				
  //Set RFM in continues receive
  RFM_Write(0x01,0x85);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 						
						
						char *ok = "  OK               ";			
						for(iii=0;iii<=13;iii++)
						{
							transmit_UART[iii] = *(ok + iii);
						}									
						HAL_UART_Transmit_DMA(&huart2, transmit_UART, 32); 									
						
						for(iii=0;iii<=15;iii++)
							{
								buffer_receive_UART[iii] = 0;
							}																
		}					
//-------     TEST receive command D (set Spreading Factor) 6 - 64 chips / symbol, 12 - 4096 chips / symbol
		if((buffer_receive_UART[0] == 0x44) && (buffer_receive_UART[3] == 0x29))
		{
						for(iii=0;iii<=15;iii++)
							{
								receive_UART[iii] = buffer_receive_UART[iii];
							}		
						spread = 0x7 & receive_UART[2];						
						spread = spread + 5;
						spread = 0x4 | spread;

  //Spreading factor 6, PayloadCRC on
  //RFM_Write(0x1E,0x64);							
							
  //Setting RFM to standby
  RFM_Write(0x01,0x81);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 						
		
  RFM_Write(0x1E,spread);		
				
  //Set RFM in continues receive
  RFM_Write(0x01,0x85);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 						
						
						char *ok = "  OK               ";			
						for(iii=0;iii<=13;iii++)
						{
							transmit_UART[iii] = *(ok + iii);
						}									
						HAL_UART_Transmit_DMA(&huart2, transmit_UART, 32); 									
						
						for(iii=0;iii<=15;iii++)
							{
								buffer_receive_UART[iii] = 0;
							}																
		}		
//-------     TEST receive command T (set time) in sec
		if((buffer_receive_UART[0] == 0x54) && (buffer_receive_UART[4] == 0x29))
		{
						for(iii=0;iii<=15;iii++)
							{
								receive_UART[iii] = buffer_receive_UART[iii];
							}		
						time_sec = 16*(0xf & receive_UART[2]) + 0xf & receive_UART[3];
										
						char *ok = "  OK               ";			
						for(iii=0;iii<=13;iii++)
						{
							transmit_UART[iii] = *(ok + iii);
						}									
						HAL_UART_Transmit_DMA(&huart2, transmit_UART, 32); 									
						
						for(iii=0;iii<=15;iii++)
							{
								buffer_receive_UART[iii] = 0;
							}																
		}						
		
//-------     TEST receive command L (set package length) 
		if((buffer_receive_UART[0] == 0x4C) && (buffer_receive_UART[8] == 0x29))
		{
						for(iii=0;iii<=15;iii++)
							{
								receive_UART[iii] = buffer_receive_UART[iii];
							}		
			l_100	= 0xF & receive_UART[2];
			l_10 = 0xF & receive_UART[3];
			l_1	= 0xF & receive_UART[4];
			package_length = 100*l_100+10*l_10+l_1;
							

  //Setting RFM to standby
  RFM_Write(0x01,0x81);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 						
		
  //Set payload length in bytes
  RFM_Write(0x22,package_length);		
				

  RFM_Write(0x01,0x85);
  //Wait for mode ready
  while(HAL_GPIO_ReadPin(GPIOC, DIO5) == 0){} 						
						
						char *ok = "  OK               ";			
						for(iii=0;iii<=13;iii++)
						{
							transmit_UART[iii] = *(ok + iii);
						}									
						HAL_UART_Transmit_DMA(&huart2, transmit_UART, 32); 									
						
						for(iii=0;iii<=15;iii++)
							{
								buffer_receive_UART[iii] = 0;
							}										
		}
		
		
		
			RFM_SendPackage(RFM_Package);
			packet_number++;
			hex_string(packet_number, RFM_Package);
			HAL_UART_Transmit_DMA(&huart2, RFM_Package, 16); 		
			for(iii=0; iii<=10;iii++)
			{
			transmit_UART[iii] = receive_UART[iii];
      receive_UART[iii] = 0x00;
			}
			
//    }
			for(iii=0; iii<=time_sec;iii++)
			{
			HAL_Delay(1000);
			}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_Pin CS_Pin */
  GPIO_InitStruct.Pin = RST_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO0_Pin DIO5_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin|DIO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RST_Pin|CS_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
