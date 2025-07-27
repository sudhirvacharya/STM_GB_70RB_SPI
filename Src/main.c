#include <stdint.h>
#include "stm32g0xx.h"
#include<stdbool.h>

#define MAX_RDATA 100
uint32_t ID = 0;
uint8_t rdata[MAX_RDATA];
uint8_t buff[16];
uint8_t currIdx;
uint8_t checkIdx;



#define MAX7219_REG_NO_OP 0x00
#define MAX7219_REG_DIGIT0 0x01
#define MAX7219_REG_DIGIT1 0x02
#define MAX7219_REG_DIGIT2 0x03
#define MAX7219_REG_DIGIT3 0x04
#define MAX7219_REG_DIGIT4 0x05
#define MAX7219_REG_DIGIT5 0x06
#define MAX7219_REG_DIGIT6 0x07
#define MAX7219_REG_DIGIT7 0x08
#define MAX7219_REG_DECODE_MODE 0x09
#define MAX7219_REG_INTENSITY 0x0A
#define MAX7219_REG_SCAN_LIMIT 0x0B
#define MAX7219_REG_SHUTDOWN 0x0C
#define MAX7219_REG_DISPLAY_TEST 0x0F

const uint8_t digit[][9]= {
{
0b00000000, //dummy at index 0 , NOP
0b11111111, //digit 0
0b10000001,
0b10000001,
0b10000001,
0b10000001,
0b10000001,
0b10000001,
0b11111111 //digit 7
},
{
0b00000000, //dummy at index 0
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b11111111
}

};

/*******************SysTick ***************/

static volatile uint32_t timeDelay=0;
//volatile int32_t clockfreqinKhz=16000;//as for 10ms SysTick interval
volatile int32_t csrRegister;
uint32_t timeout;
#define POLLING 1
#define MAXCOUNT 50000

void SysTick_initialize(uint32_t clockFreqinKhz)
{
SysTick->LOAD=clockFreqinKhz-1;
SysTick->VAL=0x0;
SysTick->CTRL=0x4;//Processor clock
#if !POLLING
SysTick->CTRL |=0x2;
#endif
SysTick->CTRL |=0x1;
}

void updateTimeDelay(void){
#if POLLING
csrRegister=SysTick->CTRL;
#endif
csrRegister &= (1<<16);
if(csrRegister){
timeDelay++;
if(timeout > 0) timeout--;
}
}

void delay1ms(uint8_t nmsecs)
{
timeDelay=0;
while(timeDelay != nmsecs)
updateTimeDelay();
}


void delay(uint32_t k)
{
uint32_t i, j;
for(i=0; i<=j; i++)
for(j=0; j<=1000; j++);
}
//Generic code for GPIO
void set_GPIO_mode(char port , uint8_t pin , uint8_t mode )
{
switch (port)
{
case 'A':
#ifdef STM32F072xB
RCC->AHBENR |= (1<<17);
#elif defined STM32G071xx
RCC->IOPENR |= (1<<0);
#endif
GPIOA->MODER &= ~(3 << (pin*2));
GPIOA->MODER |= (mode << (pin*2));
break;
case 'B':
#ifdef STM32F072xB
RCC->AHBENR |= (1 << 18);
#elif defined STM32G071xx
RCC->IOPENR |= (1<<1);
#endif
GPIOB->MODER &= ~(3 << (pin*2));
GPIOB->MODER |= (mode << (pin*2));
break;
case 'C':
#ifdef STM32F072xB
RCC->AHBENR |= (1 << 19);
#elif defined STM32G071xx
RCC->IOPENR |= (1<<2);
#endif
GPIOC->MODER &= ~(3 << (pin*2));
GPIOC->MODER |= (mode << (pin*2));
break;

default:
break;

}
}




void config_PLL()
{
uint32_t pllStatus =0;

RCC->CR |= (1<<8) ;//HSI on
while (!(RCC->CR &(1 <<10)));

FLASH->ACR |= (2<<0); //To adjust with new frequency
while( ((FLASH->ACR) & (7<<0)) != 0x02);

RCC->CR &= (~(1<< 24)); //Disable main PLL
while(RCC->CR & (1<<25));

RCC->PLLCFGR = (2 <<0); //HSI16
//RCC->PLLCFGR |= (2 << 4); //M=2 , 16/4=4Mhz input to PLL
RCC->PLLCFGR |= (8<< 8); //N=8 , 128MHz PLL output
RCC->PLLCFGR |= (1<< 17); //P=2 ,32/2=16mhz
RCC->PLLCFGR |= (1<< 29); //R=2 ,128/2=64mhz
RCC->PLLCFGR |= (1<< 25); //Q=2 ,32/2=16mhz

RCC->CR |= (1<< 24); //PLLEN
do
{
pllStatus = RCC->CR ;
pllStatus &= (1<< 25);
}while (!pllStatus);
RCC->PLLCFGR |= (1<< 28); //PLLR EN

#if MCO_OUT
//MCO out
set_GPIO_mode('A',8,2);
GPIOA->AFR[1] &= (~(0xF<<0)); //Alternate function for PA8
RCC->CFGR |= (4 << 28); // MCO divide by 16
RCC->CFGR |= (1 << 24); // 1 is SYSCLK, 3 is HSI16, 5 is Connect PLLRCLK to MCO
#endif
RCC->CFGR |= (2 <<0); //PLLRCLK as SYSCLK If you change freq from 1 to 4Mhz , then siiue

}


void W25QXX_CS_LOW() {
GPIOC->BSRR = (1 << 23); // PC7=0

}
void W25QXX_CS_HIGH()
{
GPIOC->BSRR = (1 << 7); // PC7=1

}
void SPI1_Transmit(uint8_t *data,uint32_t size)
{
uint32_t i=0;

while(i<size)
{
/*Wait until TXE is set*/
while(!(SPI1->SR & (SPI_SR_TXE))){}

/*Write the data to the data register*/
//*(uint8_t*)&SPI1->DR = data[i];
*(uint8_t*)&SPI1->DR = data[i];
i++;
while((SPI1->SR & (SPI_SR_BSY))){}
}
/*Wait until TXE is set*/
while(!(SPI1->SR & (SPI_SR_TXE))){}

/*Wait for BUSY flag to reset*/
while((SPI1->SR & (SPI_SR_BSY))){}
/*Clear OVR flag*/
(void)SPI1->DR;
(void)SPI1->SR;
}
void sent_data_dotMx(uint8_t reg, uint16_t data)
{
rdata[0]=reg;
rdata[1]=data;

W25QXX_CS_LOW();
//SPI1_Transmit16(rdata); //Not gauranteed to work
SPI1_Transmit(&rdata[0], 1);
//delay(1);
SPI1_Transmit(&rdata[1], 1);
//SPI1_Transmit(&temp, 1);
W25QXX_CS_HIGH();
}
void SPI1_Transmit16(uint16_t *data,uint32_t size)
{
uint32_t i=0;

while(i<size)
{
/*Wait until TXE is set*/
while(!(SPI1->SR & (SPI_SR_TXE))){}

/*Write the data to the data register*/
//*(uint8_t*)&SPI1->DR = data[i];
//*(uint8_t*)&
SPI1->DR = data[i];
i++;
while((SPI1->SR & (SPI_SR_BSY))){}
}
/*Wait until TXE is set*/
while(!(SPI1->SR & (SPI_SR_TXE))){}

/*Wait for BUSY flag to reset*/
while((SPI1->SR & (SPI_SR_BSY))){}

/*Clear OVR flag*/
(void)SPI1->DR;
(void)SPI1->SR;
}

void init_dotMx()
{
sent_data_dotMx( MAX7219_REG_SHUTDOWN ,0x01);//Normal operation mode
delay(10);
sent_data_dotMx(MAX7219_REG_SCAN_LIMIT ,0x07);//Scan limit display digit 00-07
delay(10);
sent_data_dotMx(MAX7219_REG_DECODE_MODE ,0x00);//No-decode Mode
delay(10);
sent_data_dotMx(MAX7219_REG_DISPLAY_TEST,0x00);//Display Normal operation
delay(1);
sent_data_dotMx( MAX7219_REG_INTENSITY ,0x0F);
delay(10);
}


void MAX7219_Write(uint8_t reg, uint8_t data)
{
uint8_t spiData[2];
spiData[0] = reg; // BCD for row number
spiData[1] = data; //seven segment data
{
uint16_t spiData1 = (reg <<8 ) | data;
W25QXX_CS_LOW();// CS LOW
SPI1_Transmit(spiData, 2);
// SPI1_Transmit16(spiData, 1);
W25QXX_CS_HIGH(); // CS HIGH
}
}
void MAX7219_WriteRegister(uint8_t reg, uint8_t data) {
uint8_t spiData[2];
spiData[0] = reg;
spiData[1] = data;
W25QXX_CS_LOW();// CS LOW
SPI1_Transmit(spiData, 2);
W25QXX_CS_HIGH(); // CS HIGH
}
void MAX7219_Init(void)
{
MAX7219_Write(MAX7219_REG_SHUTDOWN, 0x01); // Shutdown register: Normal operation
MAX7219_Write(MAX7219_REG_SCAN_LIMIT, 0x07); // Scan limit: Display digits 0-7
MAX7219_Write(MAX7219_REG_DECODE_MODE, 0x00); // Decode mode: No decode
MAX7219_Write(MAX7219_REG_DISPLAY_TEST, 0x00); // Display test: Off
MAX7219_Write(MAX7219_REG_INTENSITY, 0x0F); // Intensity: Maximum brightness
}
void MAX7219_DisplayTest(void)
{
for (uint8_t i = MAX7219_REG_DIGIT0; i <= MAX7219_REG_DIGIT7; i++)
{
MAX7219_Write(i, 0xFF); // Turn on all LEDs
}
}
void MAX7219_DisplayPattern(uint8_t *pattern)
{
for (uint8_t i = 1; i <= 8; i++)
{
MAX7219_Write( i, pattern[i]);
}
}


void MAX7219_Clear(void) {
for (uint8_t i = 1; i <= 8; i++) {
MAX7219_Write(i, 0x00); // Turn off all LEDs
delay(1);
}
}

void init_SPI(void)
{
RCC->CFGR |= (6 <<12); //APB = HCLK/8 = 8Mhz
RCC->APBENR2 |= (1<<12); //SPI1 clock
set_GPIO_mode('C',7,1);
set_GPIO_mode('A',5,2);
set_GPIO_mode('A',6,2);
set_GPIO_mode('A',7,2);
//GPIOA->AFR[0] &= ~((0xF<<20) | (0xF<<24) | (0xF<<28));
GPIOA->OSPEEDR|=(3 <<10)|(3<<12)|(3<<14) ; //very high speed
SPI1->CR1 |= (1<<9); //SSM=1
SPI1->CR1 |= (1<<8); //SSI=1 required to get data from other device
SPI1->CR1 |= (1<<2); //Master
SPI1->CR1 &= ~((1<<1)|(1<<0));//clock phase=0 , clock polarity =0
SPI1->CR1 |= (1<<6);//Peripheral enabled
}
void init_dotMx_DEBUG()
{
	sent_data_dotMx( MAX7219_REG_SHUTDOWN ,0x00);//Shutdownl operation mode
	delay(10);
	sent_data_dotMx( MAX7219_REG_SHUTDOWN ,0x01);//Normal operation mode
	delay(10);
	sent_data_dotMx(MAX7219_REG_SCAN_LIMIT ,0x07);//Scan limit display digit 00-07
	 delay(10);
	sent_data_dotMx(MAX7219_REG_DECODE_MODE ,0x00);//No-decode Mode
	 delay(10);
	 sent_data_dotMx( MAX7219_REG_INTENSITY ,0x0F);
	 delay(10);
    sent_data_dotMx(MAX7219_REG_DISPLAY_TEST,0x01);//Display Normal operat
delay(1);
}
#define DEBUG 1
int main(void)
{
config_PLL();
init_SPI();
//init_dotMx();

#if DEBUG
  init_dotMx_DEBUG();
  while(1)
  {

  }
#else
  init_dotMx();
#endif

while (1)
{

	MAX7219_DisplayTest();
	delay(1000);
	MAX7219_Clear();
	/*

MAX7219_Clear();
MAX7219_DisplayPattern(&digit[0][0]);
delay(1000);
MAX7219_Clear();
// Clear display
MAX7219_DisplayPattern(&digit[1][0]);
delay(1000);
MAX7219_Clear();
*/

}


}
