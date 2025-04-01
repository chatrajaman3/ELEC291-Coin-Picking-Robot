#ifndef UTIL_H
#define UTIL_H

#define SYSCLK 72000000L
#define BAUDRATE 115200L

#define SARCLK 18000000L
#define RELOAD_10US (0x10000L-(SYSCLK/(12L*100000L)))
#define RELOAD_10MS (0x10000L-(SYSCLK/(12L*100L)))

#define VDD 3.3035

char _c51_external_startup(void) {
	/* Disable watchdog. */
	SFRPAGE = 0x00;
	WDTCN = 0xDE;
	WDTCN = 0xAD;
	/* Enable VDD monitor. */
	VDM0CN = 0x80;
	/* Enable reset on missing clock detector and VDD. */
	RSTSRC = 0x02|0x04;
	/* Enable interrupts. */
	EA = 1;

	#if (SYSCLK == 48000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x10;
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20;
		SFRPAGE = 0x00;
	#endif

	#if (SYSCLK == 12250000L)
		CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 24500000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 48000000L)
		/* Before setting clock to 48 MHz, we must transition to
		 * 24.5 MHz first. */
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		/* Before setting clock to 72 MHz, we must transition to
		 * 24.5 MHz first. */
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L,             \
			48000000L, or 72000000L
	#endif

	/* Configure the pins used as outputs. */

	/* Configure UART0 TX (P0.4) and UART1 TX (P0.0) as push-pull output. */
	P0MDOUT |= 0b00010001;
	/* Motor controls via H-bridge and coin detector. */
	P1MDOUT |= 0b10101111;
	/* Arm servo motors and electromagnet. */
	P2MDOUT |= 0b01010011;

	/* Enable UART0 on P0.4 (TX) and P0.5 (RX). */
	XBR0 = 0x01;
	// XBR1 = 0x00;
	/* Enable crossbar and UART1. */
	XBR2 = 0x41;

	#if (((SYSCLK/BAUDRATE)/(2L*12L)) > 0xFFL)
		#error Timer 0 reload value is incorrect because               \
			(SYSCLK/12L) / (2L*BAUDRATE) > 0xFF
	#endif

	/* Configure UART0. */

	SCON0 = 0x10;
	CKCON0 |= 0b00000000 ;
	TH1 = 0x100 - (SYSCLK/12L) / (2L*BAUDRATE);
	TL1 = TH1;
	/* Timer 1 in auto-reload mode. */
	TMOD &= 0x0F;
	TMOD |= 0x20;
	TR1 = 1;
	/* Indicate TX0 ready. */
	TI = 1;

	/* Initialize timer 5 for periodic interrupts. */

	SFRPAGE = 0x10;
	TMR5CN0 = 0x00;
	/* Set to reload immediately. */
	TMR5 = 0xFFFF;
	/* Enable Timer 5 interrupts. */
	EIE2 |= 0b00001000;
	/* Start timer 5. */
	TR5=1;
	SFRPAGE=0x00;

	return 0;
}

void usleep(unsigned char us) {
	unsigned char i;

	/* Select SYSCLK as input .*/
	CKCON0 |= 0b01000000;

	TMR3RL = -SYSCLK/1000000L;
	TMR3 = TMR3RL;

	TMR3CN0 = 0x04;

	for (i = 0; i < us; ++i) {
		while (!(TMR3CN0 & 0x80));
		TMR3CN0 &= ~(0x80);
	}

	TMR3CN0 = 0x00;
}

void sleep(unsigned int ms) {
	unsigned int i;
	for (i = 4*ms; i != 0; --i)
		usleep(250);
}

void adc_init(void) {
	SFRPAGE = 0x00;
	ADEN = 0;

	ADC0CN1 = (0x2 << 6) | (0x0 << 3) | (0x0 << 0);
	ADC0CF0 = (SYSCLK/SARCLK) << 3;
	ADC0CF1 = 0x1E;
	ADC0CN0 = 0x00;
	ADC0CF2 = (0x1 << 5) | (0x1F);
	ADC0CN2 = 0x00;

	ADEN = 1;
}

void adc_pin_init(unsigned char port, unsigned char pin) {
	unsigned char mask;

	mask = 1 << pin;

	SFRPAGE = 0x20;
	switch (port) {
	case 0:
		P0MDIN &= ~mask;
		P0SKIP |= mask;
		break;
	case 1:
		P1MDIN &= ~mask;
		P1SKIP |= mask;
		break;
	case 2:
		P2MDIN &= ~mask;
		P2SKIP |= mask;
		break;
	default:
		break;
	}
	SFRPAGE = 0x00;
}

unsigned int adc_value(unsigned char pin) {
	ADC0MX = pin;
	ADINT = 0;
	ADBUSY = 1;
	while (!ADINT);
	return ADC0;
}

float adc_volts(unsigned char pin) {
	 return adc_value(pin) * VDD / 0x3FFF;
}

void uart1_init(unsigned long baudrate) {
	SFRPAGE = 0x20;
	SMOD1 = 0x0C; /* 8N1. */
	SCON1 = 0x10;
	SBCON1 =0x00;
	SBRL1 = 0x10000L - (SYSCLK/12L) / (2L*baudrate);
	TI1 = 1;
	SBCON1 |= 0x40;
	SFRPAGE = 0x00;
}

void uart1_putchar(char c) {
	SFRPAGE = 0x20;
	while (!TI1);
	TI1 = 0;
	SBUF1 = c;
	SFRPAGE = 0x00;
}

void uart1_putstr(char * s) {
	while (*s) {
		uart1_putchar(*s);
		++s;
	}
}

char uart1_getchar(void) {
	char c;
	SFRPAGE = 0x20;
	while (!RI1);
	RI1 = 0;
	/* Clear overrun and parity error flags. */
	SCON1 &= 0b00111111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return c;
}

char uart1_getchar_with_timeout(void) {
	char c;
	unsigned int timeout;
	SFRPAGE = 0x20;
	timeout = 0;
	while (!RI1) {
		SFRPAGE = 0x00;
		usleep(20);
		SFRPAGE = 0x20;
		++timeout;
		if (timeout == 25000) {
			SFRPAGE = 0x00;
			return '\n'; // Timeout after half second
		}
	}
	RI1 = 0;
	/* Clear overrun and parity error flags. */
	SCON1 &= 0b00111111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return c;
}

void uart1_getstr(char * s, unsigned char n) {
	char c;
	unsigned char cnt;

	cnt = 0;
	while (1) {
		c = uart1_getchar_with_timeout();
		if (c == '\n') {
			*s = 0;
			return;
		}

		if (cnt < n) {
			++cnt;
			*s = c;
			++s;
		} else {
			*s = 0;
			return;
		}
	}
}

bit uart1_received(void) {
	bit mybit;
	SFRPAGE = 0x20;
	mybit = RI1;
	SFRPAGE = 0x00;
	return mybit;
}

void uart1_sleep_until_received(unsigned int ms) {
	unsigned int i;
	for (i = 0; i < 4*ms; ++i) {
		if (uart1_received())
			return;
		usleep(250);
	}
}

#endif /* UTIL_H */
