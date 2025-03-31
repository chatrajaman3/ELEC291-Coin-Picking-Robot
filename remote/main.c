/*
 * Coin Picking Robot (Remote)
 * main.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "include/stm32l051xx.h"
#include "include/serial.h"
#include "util.h"

#define VER_MAJOR 0
#define VER_MINOR 1

#ifndef NDEBUG
	#define DEBUG_PRINT(fmt, ...)                                          \
		fprintf(stderr, "DEBUG: %s:%d: %s(): " fmt "\r\n",             \
			__FILE__, __LINE__, __func__, __VA_ARGS__)
#else
	#define DEBUG_PRINT(fmt, ...) do {} while (0)
#endif

/*
 *                   ----------
 *             VDD -|1       32|- VSS
 *            PC14 -|2       31|- BOOT0
 *            PC15 -|3       30|- PB7
 *            NRST -|4       29|- PB6
 *            VDDA -|5       28|- PB5
 *     LCD_RS  PA0 -|6       27|- PB4
 *     LCD_E   PA1 -|7       26|- PB3
 *     LCD_D4  PA2 -|8       25|- PA15  UART2_RXD/JDY40_TXD
 *     LCD_D5  PA3 -|9       24|- PA14  UART2_TXD/JDY40_RXD
 *     LCD_D6  PA4 -|10      23|- PA13  JDY40_SET
 *     LCD_D7  PA5 -|11      22|- PA12
 *             PA6 -|12      21|- PA11
 *             PA7 -|13      20|- PA10  UART1_RXD
 * JOYSTICK_Y  PB0 -|14      19|- PA9   UART1_TXD
 * JOYSTICK_X  PB1 -|15      18|- PA8   JOYSTICK_SW
 *             VSS -|16      17|- VDD
 *                   ----------
 */

void init(void);
void send_command(char *s);
void reception_off(void);
int pb_read(void);

int main(void) {
	int x, y, timeout;
	char *buf;

	init();
	uart2_init(9600);
	adc_init();
	lcd_init();

	DEBUG_PRINT("Coin picking robot, version %d.%d (%s %s)", \
		VER_MAJOR, VER_MINOR, __DATE__, __TIME__);

	sleep(1000);

	reception_off();

	/* Retrieve current configuration. */
	send_command("AT+VER\r\n");
	send_command("AT+BAUD\r\n");
	send_command("AT+RFID\r\n");
	send_command("AT+DVID\r\n");
	send_command("AT+RFC\r\n");
	send_command("AT+POWE\r\n");
	send_command("AT+CLSS\r\n");

	/* Set device ID to 0xC0A8 and switch to channel 108. */
	send_command("AT+DVIDC0A8\r\n");
	send_command("AT+RFC108\r\n");

	buf = malloc(80);
	strcpy(buf, 0);

	sleep(500);

	while (1) {
		x = adc_read(ADC_CHSELR_CHSEL8);
		DEBUG_PRINT("x: %d", x);
		y = adc_read(ADC_CHSELR_CHSEL9); 
		DEBUG_PRINT("y: %d", y);

		buf[0] = 0;

		/* Read joystick inputs. */

		if (x <= 1800) {
			/* Left. */
			buf[0] |= 0b01 << 2;
		} else if (x >= 2300) {
			/* Right. */
			buf[0] |= 0b10 << 2;
		} else {
			/* Centre. */
			buf[0] |= 0b11 << 2;
		}

		if (y <= 1800) {
			/* Down. */
			buf[0] |= 0b01;
		} else if(y >= 2300) {
			/* Up. */
			buf[0] |= 0b10;
		} else {
			/* Centre. */
			buf[0] |= 0b11;
		}

		/* Joystick button. */
		if (GPIOA->IDR & BIT8) {
			buf[0] |= 0b1 << 4;
		}

		buf[1] = pb_read();

		DEBUG_PRINT("buf: [%d, %d]", buf[0], buf[1]);

		com2_eputc('!');
		sleep(5);
		com2_eputc(buf[0]);
		sleep(5);
		com2_eputc(buf[1]);
		sleep(5);
		com2_eputc('\n');

		timeout = 0;

		/* Wait for response. */

		do {
			if (uart2_received() > 0)
				break;
			usleep(100);
		} while (++timeout < 250);

		if (uart2_received() > 0) {
			com2_egets(buf, sizeof(buf) - 1);
			DEBUG_PRINT("Response: %s\n", buf);
		} else {
			DEBUG_PRINT("No response\n", 0);
		}

		fflush(NULL);
	}
}

void init(void) {
	/* Configure port A for very high speed (page 201). */
	GPIOA->OSPEEDR=0xFFFFFFFF;

	RCC->IOPENR |= BIT0; 
	RCC->IOPENR |= BIT1;

	/* LCD display output. */
	GPIOA->MODER = (GPIOA->MODER & ~(BIT0|BIT1)) | BIT0;
	GPIOA->OTYPER &= ~BIT0;
	GPIOA->MODER = (GPIOA->MODER & ~(BIT2|BIT3)) | BIT2;
	GPIOA->OTYPER &= ~BIT1;
	GPIOA->MODER = (GPIOA->MODER & ~(BIT4|BIT5)) | BIT4;
	GPIOA->OTYPER &= ~BIT2;
	GPIOA->MODER = (GPIOA->MODER & ~(BIT6|BIT7)) | BIT6;
	GPIOA->OTYPER &= ~BIT3;
	GPIOA->MODER = (GPIOA->MODER & ~(BIT8|BIT9)) | BIT8;
	GPIOA->OTYPER &= ~BIT4;
	GPIOA->MODER = (GPIOA->MODER & ~(BIT10|BIT11)) | BIT10;
	GPIOA->OTYPER &= ~BIT5;

	/* Radio output. */
	GPIOA->MODER = (GPIOA->MODER & ~(BIT27|BIT26)) | BIT26;
	GPIOA->ODR |= BIT13;

	/* Joystick button input. */
	GPIOA->MODER &= ~(BIT16|BIT17);
	GPIOA->PUPDR |= BIT16; 
	GPIOA->PUPDR &= ~BIT17;

	/* Push-button array ADC input. */
	GPIOA->MODER &= ~(BIT12|BIT13);
	GPIOA->PUPDR |= BIT12;
	GPIOA->PUPDR &= ~BIT13;
}

void send_command(char *s) {
	char buff[40];
	printf("Command: %s", s);
	GPIOA->ODR &= ~BIT13;
	sleep(10);
	com2_eputs(s);
	com2_egets(buff, sizeof(buff) - 1);
	GPIOA->ODR |= BIT13;
	sleep(10);
	printf("Response: %s", buff);
}

void reception_off(void) {
	GPIOA->ODR &= ~(BIT13);
	sleep(10);
	com2_eputs("AT+DVID0000\r\n");
	sleep(10);
	GPIOA->ODR |= BIT13;
	while (uart2_received() > 0)
		com2_egetc();
}

int pb_read(void) {
	int pb_adc;
	pb_adc = adc_read(ADC_CHSELR_CHSEL6);
	DEBUG_PRINT("pb_adc: %d", pb_adc);

	/* A 1000 ohm resistor is connected to the analogue push button
	 * array input pin. The following values were read for their
	 * respective buttons:
	 * - button 1: 320;
	 * - button 2: 460;
	 * - button 3: 580;
	 * - button 4: 730;
	 * - button 5: 930;
	 * - button 6: 1260;
	 * - button 7: 1920;
	 * - button 8: 4090. */
	if (pb_adc > 0xF00)
		return 0b1 << 7;
	if (pb_adc > 0x700)
		return 0b1 << 6;
	if (pb_adc > 0x480)
		return 0b1 << 5;
	if (pb_adc > 0x380)
		return 0b1 << 4;
	if (pb_adc > 0x280)
		return 0b1 << 3;
	if (pb_adc > 0x200)
		return 0b1 << 2;
	if (pb_adc > 0x180)
		return 0b1 << 1;
	if (pb_adc > 0x100)
		return 0b1;
	return 0;
}
