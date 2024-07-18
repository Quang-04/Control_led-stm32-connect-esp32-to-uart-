#include "main.h"
#include <stdio.h>
#include <string.h>
#define SETBIT(REG, VAL, POS) *REG=*REG|(VAL<<POS)
#define CLEARBIT(REG, VAL, POS) *REG=*REG&~(VAL<<POS)
#define READBIT(REG, POS) ((*REG>>POS)&1)
#define GPIOA_BASE_ADDR 0x40020000
#define GPIOD_BASE_ADDR 0x40020C00
/*
	Init 4 led - set PD12, PD13, PD14 va PD15 => OUTPUT
	input: No
	output: No
*/
void leds_init(void)
{
	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t* GPIOD_MODER = (uint32_t*)(GPIOD_BASE_ADDR + 0x00);
	CLEARBIT(GPIOD_MODER, 0b11111111, 24);
	SETBIT(GPIOD_MODER, 0b01010101, 24);
}

typedef enum
{
	GREEN_LED = 12,
	ORANGE_LED,
	RED_LED,
	BLUE_LED
} led_t;

typedef enum
{
	LED_OFF, LED_ON
} led_state_t;

/*
	Control led on/off
	input:
		led_name: name led of control
		led_state: led status of ON/OFF
	output: No
*/
void led_control(led_t led_name, led_state_t state)
{
	uint32_t* GPIOD_ODR = (uint32_t*)(GPIOD_BASE_ADDR + 0x14);
	if(state == LED_ON)
	{
		SETBIT(GPIOD_ODR, 1, led_name);
	}
	else
	{
		CLEARBIT(GPIOD_ODR, 1, led_name);
	}
}
#define UART1_BASE_ADDR 0x40011000
#define GPIOB_BASE_ADDR 0x40020400
/*
    UART1:
        + BR: 9600bps
        + Frame:
            * data size: 8 bit
            * parity: none
*/
void UartInit()
{
    // set up: PB6 -> UART1_TX, PB7 -> UART1_RX
    __HAL_RCC_GPIOB_CLK_ENABLE();
    uint32_t* MODER = (uint32_t*)(GPIOB_BASE_ADDR + 0x00);
    *MODER &= (0b1111 << 12);
    *MODER |= (0b10 << 12) | (0b10 << 14);  //set Alternate function mode for PB6 and PB7
    uint32_t* AFRL = (uint32_t*)(GPIOB_BASE_ADDR + 0x20);
    *AFRL &= ~(0xff << 24);
    *AFRL |= (07 << 24) | (07 << 28);

    __HAL_RCC_USART1_CLK_ENABLE();
    uint32_t *BRR = (uint32_t *)(UART1_BASE_ADDR + 0x08);
    *BRR = 8 << 4 | 11; // set baudrate 115200 bps

    uint32_t *CR1 = (uint32_t *)(UART1_BASE_ADDR + 0x0C);
    *CR1 &= ~(1 << 12); // clear bit 12 to set data frame as 8bit data
    *CR1 &= ~(1 << 10); // clear bit 10 to disable parity bit
    /*
        set bit 2: enable Receiver
        set bit 3: enable Trasmitter
        set bit 13: enable UART1
    */
    *CR1 |= (1<<2) | (1<<3) | (1<<13);

/*
   Send 1 byte (8bit) through UART
    inputs:
        data: data you want to send
*/
void UartTransmitOneByte(char data)
{
    uint8_t* DR = (uint8_t *)(UART1_BASE_ADDR + 0x04);
    uint8_t* SR = (uint8_t *)(UART1_BASE_ADDR + 0x00);
    while (((*SR >> 7) & 1) == 0);	//wait TxD empty
    *DR = data; 					//write data to DR to send data via UART
    while (((*SR >> 6) & 1) == 0);  //wait TC is set to 1 (data transmission complete)
}

void UartTransmitBytes(char* data_buffer, int size)
{
	for(int i = 0; i < size; i++)
	{
		UartTransmitOneByte(data_buffer[i]);
	}
}

int __io_putchar(char c)
{
	UartTransmitOneByte(c);
	return 0;
}
/*
 *read 1 byte data from UART
 * input: No
 * output: char: 1 byte read from UART
 */
char UartRecievData()
{
	uint8_t* DR = (uint8_t *)(UART1_BASE_ADDR + 0x04);
	uint8_t* SR = (uint8_t *)(UART1_BASE_ADDR + 0x00);
	//step 1: wait RXNE(in SR - status register) is set to 1 - Received data is ready to be read.
	while(READBIT(SR, 5) == 0);
	//step 2: read DR value
	uint8_t recv_data = *DR;
	//step 3: return this value
	return recv_data;
}
#define WIFI_SSID "QUANG SANG"
#define WIFI_PASS "chochutda"
void Wifi_Init()
{
	char rx_buff[64] = {0};
	char rx_index = 0;
	/* check connection */
	printf("AT\r\n");
	//recv resp from esp32
	while((!strstr(rx_buff, "OK\r\n")) && (!strstr(rx_buff, "ERROR\r\n")))
	{
		rx_buff[rx_index++] = UartRecievData();
	}
	memset(rx_buff, 0, sizeof(rx_buff));
	rx_index = 0;
	/* set esp32 in station mode */
	printf("AT+CWMODE=1\r\n");
	//recv resp from esp32
	while((!strstr(rx_buff, "OK\r\n")) && (!strstr(rx_buff, "ERROR\r\n")))
	{
		rx_buff[rx_index++] = UartRecievData();
	}
	memset(rx_buff, 0, sizeof(rx_buff));
	rx_index = 0;
	/* set esp32 ssid and pass for wifi */
	printf("AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PASS);
	//recv resp from esp32
	while((!strstr(rx_buff, "OK\r\n")) && (!strstr(rx_buff, "ERROR\r\n")))
	{
		rx_buff[rx_index++] = UartRecievData();
	}
	memset(rx_buff, 0, sizeof(rx_buff));
	rx_index = 0;
}
#define SERVER_IP 	"192.168.1.9"
#define SERVER_PORT	1234
void Connect_Server()
{
	char rx_buff[64] = {0};
	char rx_index = 0;
	/* set esp32 in station mode */
	printf("AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", SERVER_IP, SERVER_PORT);
	//recv resp from esp32
	while((!strstr(rx_buff, "OK\r\n")) && (!strstr(rx_buff, "ERROR\r\n")))
	{
		rx_buff[rx_index++] = UartRecievData();
	}
	memset(rx_buff, 0, sizeof(rx_buff));
	rx_index = 0;

	char msg[] = "hello server, I'm ESP32 \r\n";
	printf("AT+CIPSEND=%d\r\n", strlen(msg));
	//recv resp from esp32
	while((!strstr(rx_buff, "OK\r\n\r\n>")) && (!strstr(rx_buff, "ERROR\r\n")))
	{
		rx_buff[rx_index++] = UartRecievData();
	}
	memset(rx_buff, 0, sizeof(rx_buff));
	rx_index = 0;
	printf(msg);
	//recv resp from esp32
	while((!strstr(rx_buff, "SEND OK\r\n")) && (!strstr(rx_buff, "ERROR\r\n")))
	{
		rx_buff[rx_index++] = UartRecievData();
	}
	memset(rx_buff, 0, sizeof(rx_buff));
	rx_index = 0;
}

void Create_TCP_Server()
{
	char rx_buff[64] = {0};
	char rx_index = 0;
	/* A TCP/SSL server can only be created when multiple connections are activated */
	printf("AT+CIPMUX=1\r\n");
	//recv resp from esp32
	while((!strstr(rx_buff, "OK\r\n")) && (!strstr(rx_buff, "ERROR\r\n")))
	{
		rx_buff[rx_index++] = UartRecievData();
	}
	memset(rx_buff, 0, sizeof(rx_buff));
	rx_index = 0;

	/* A TCP/SSL server can only be created when multiple connections are activated */
	printf("AT+CIPSERVER=1\r\n");
	//recv resp from esp32
	while((!strstr(rx_buff, "OK\r\n")) && (!strstr(rx_buff, "ERROR\r\n")))
	{
		rx_buff[rx_index++] = UartRecievData();
	}
	memset(rx_buff, 0, sizeof(rx_buff));
	rx_index = 0;
}

void LED_Control_From_Server()
{
	char rx_buff[128] = {0};
	char rx_index = 0;
	while(1)
	{
		rx_buff[rx_index++] = UartRecievData();
		if(strstr(rx_buff, "\r\n"))
		{
			if(strstr(rx_buff, "LED ON"))
			{
				led_control(RED_LED, LED_ON);
			}
			else if(strstr(rx_buff, "LED OFF"))
			{
				led_control(RED_LED, LED_OFF);
			}
			memset(rx_buff, 0, sizeof(rx_buff));
			rx_index = 0;
		}
	}
}
int main()
{
	HAL_Init();
	UartInit();
	leds_init();
	Wifi_Init();
	Create_TCP_Server();
	LED_Control_From_Server();

	while(1)
	{

	}
	return 0;
}
