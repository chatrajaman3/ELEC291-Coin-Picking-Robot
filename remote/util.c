/*
 * Coin Picking Robot (Remote)
 * util.c
 */

#include "include/stm32l051xx.h"
#include "util.h"

unsigned com2_open, com2_error, com2_busy;
ComBuffer com_tx_buf, com_rx_buf;

unsigned char com_getbuf(ComBuffer *buf);
int com_putbuf(ComBuffer *buf, unsigned char data);
unsigned com_length(ComBuffer *buf);
void usart2_tx(void);
void usart2_rx(void);

void USART2_Handler(void) {
	if (USART2->ISR & BIT7)
		usart2_tx();
	if (USART2->ISR & BIT5)
		usart2_rx();
}

void sleep(unsigned int ms) {
	unsigned int i;
	for (i = 0; i < 4*ms; ++i)
		usleep(250);
	return;
}

void usleep(unsigned char us) {
	SysTick->LOAD = (SYSCLK / 1000000L * us) - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
	while ((SysTick->CTRL & BIT16) == 0);
	SysTick->CTRL = 0x00;
	return;
}

void lcd_init(void) {
	LCD_E_0;
	sleep(20);

	lcd_write_command(0x33);
	lcd_write_command(0x33);
	lcd_write_command(0x32);

	lcd_write_command(0x28);
	lcd_write_command(0x0C);
	lcd_write_command(0x01);
	sleep(20);

	return;
}

void lcd_print(char *s, unsigned char line, unsigned char clear) {
	int i;

	lcd_write_command(line == 1 ? 0x80 : 0xC0);
	sleep(5);
	for (i = 0; s[i] != 0; ++i)
		lcd_write_data(s[i]);
	if (clear)
		for (; i < CHARS_PER_LINE; ++i)
			lcd_write_data(' ');
	return;
}

void lcd_write_command(unsigned char x) {
	LCD_RS_0;
	lcd_byte(x);
	sleep(5);
	return;
}

void lcd_write_data(unsigned char x) {
	LCD_RS_1;
	lcd_byte(x);
	sleep(2);
	return;
}

void lcd_byte(unsigned char x) {
	if (x & 0x80) LCD_D7_1; else LCD_D7_0;
	if (x & 0x40) LCD_D6_1; else LCD_D6_0;
	if (x & 0x20) LCD_D5_1; else LCD_D5_0;
	if (x & 0x10) LCD_D4_1; else LCD_D4_0;
	lcd_pulse();

	usleep(40);

	if (x & 0x08) LCD_D7_1; else LCD_D7_0;
	if (x & 0x04) LCD_D6_1; else LCD_D6_0;
	if (x & 0x02) LCD_D5_1; else LCD_D5_0;
	if (x & 0x01) LCD_D4_1; else LCD_D4_0;
	lcd_pulse();

	return;
}

void lcd_pulse(void) {
	LCD_E_1;
	usleep(40);
	LCD_E_0;
	return;
}

void adc_init(void) {
	RCC->APB2ENR |= BIT9;

	/* ADC clock selection procedure (page 746 of RM0451). */
	ADC1->CFGR2 |= ADC_CFGR2_CKMODE;

	/* ADC enable sequence procedure (page 745 of RM0451). */
	ADC1->ISR |= ADC_ISR_ADRDY;
	ADC1->CR |= ADC_CR_ADEN;
	if ((ADC1->CFGR1 & ADC_CFGR1_AUTOFF) == 0) {
		while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);
	}

	/* Calibration code procedure (page 745 of RM0451). */
	if ((ADC1->CR & ADC_CR_ADEN) != 0) {
		ADC1->CR |= ADC_CR_ADDIS;
	}
	ADC1->CR |= ADC_CR_ADCAL;
	while ((ADC1->ISR & ADC_ISR_EOCAL) == 0);
	ADC1->ISR |= ADC_ISR_EOCAL;
}

int adc_read(unsigned int channel) {
	/* Single conversion sequence code example - software trigger
	 * (page 746 of RM0451). */
	ADC1->CFGR1 |= ADC_CFGR1_AUTOFF;
	ADC1->CHSELR = channel;
	ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2;
	if (channel == ADC_CHSELR_CHSEL17) {
		ADC->CCR |= ADC_CCR_VREFEN;
	}

	/* Perform the AD conversion. */
	ADC1->CR |= ADC_CR_ADSTART;
	while ((ADC1->ISR & ADC_ISR_EOC) == 0);

	return ADC1->DR;
}

void uart2_init(int baud) {
	int baud_rate_divisor;

	__disable_irq();

	com_rx_buf.head = com_rx_buf.tail = com_rx_buf.count = 0;
	com_tx_buf.head = com_tx_buf.tail = com_tx_buf.count = 0;
	com2_open = 1;
	com2_error = 0;

	RCC->IOPENR |= BIT0;
	
	baud_rate_divisor = SYSCLK;
	baud_rate_divisor = baud_rate_divisor / (long)baud;

	GPIOA->OSPEEDR |= BIT28;
	GPIOA->OTYPER &= ~BIT14;
	GPIOA->MODER = (GPIOA->MODER & ~(BIT28)) | BIT29;
	GPIOA->AFR[1] |= BIT26;

	GPIOA->MODER = (GPIOA->MODER & ~(BIT30)) | BIT31;
	GPIOA->AFR[1] |= BIT30;

	RCC->APB1ENR |= BIT17;

	USART2->CR1 |= (BIT2|BIT3|BIT5|BIT6);
	USART2->CR2 = 0x00000000;
	USART2->CR3 = 0x00000000;
	USART2->BRR = baud_rate_divisor;
	USART2->CR1 |= BIT0;

	NVIC->ISER[0] |= BIT28;

	__enable_irq();
}

int uart2_received(void) {
	return com_length(&com_rx_buf);
}

int com2_read(int max, unsigned char *buf) {
	unsigned i;

	if (!com2_open)
		return -1;

	i = 0;
	while ((i < max-1) && (com_length(&com_rx_buf)))
		buf[i++] = com_getbuf(&com_rx_buf);

	if (i > 0) {
		buf[i]=0;
		return i;
	} else {
		return 0;
	}
}

int com2_write(int count, unsigned char *buf) {
	unsigned i;

	if (!com2_open)
		return -1;

	if (count < MAXBUFFER)
		while ((MAXBUFFER - com_length(&com_tx_buf)) < count);
	else
		return -2;

	for (i = 0; i < count; ++i)
		com_putbuf(&com_tx_buf, buf[i]);

	if ((USART2->CR1 & BIT3) == 0) {
		USART2->CR1 |= BIT3;
		USART2->TDR = com_getbuf(&com_tx_buf);
	}

	return 0;
}

int com2_egets(char *s, int max) {
	int len;
	char c;

	if (!com2_open)
		return -1;

	len = 0;
	c = 0;
	while ((len < max-1) && (c != '\n')) {
		while (!com_length(&com_rx_buf));
		c = com_getbuf(&com_rx_buf);
		s[len++] = c;
	}

	if (len > 0) {
		s[len] = 0;
	}

	return len;
}

int com2_eputs(char *s) {
	if (!com2_open)
		return -1;

	while (*s)
		com2_write(1, s++);

	return 0;
}

int com2_echos(char *s, int max) {
	int len;
	char c;

	if (!com2_open)
		return -1;

	len = 0;
	c = 0;

	while ((len < max-1) && (c != '\r')) {
		while (!com_length(&com_rx_buf));
		c = com_getbuf(&com_rx_buf);
		com2_eputc(c);
		s[len++] = c;
	}

	if (len > 0) {
		s[len] = 0;
	}

	return len;
}

char com2_egetc(void) {
	return com_getbuf(&com_rx_buf);
}

void com2_eputc(char c) {
	com2_write(1, &c);
}

char com2_echoc(void) {
	char c;
	c = com2_egetc();
	com2_eputc(c);
	return c;
}

unsigned char com_getbuf(ComBuffer *buf) {
	unsigned char data;

	if (buf->count==0)
		return 0;

	__disable_irq();

	data = buf->buffer[buf->tail++];
	if (buf->tail == MAXBUFFER) buf->tail = 0;
	buf->count--;

	__enable_irq();

	return data;
}

int com_putbuf(ComBuffer *buf, unsigned char data) {
	if ((buf->head == buf->tail) && (buf->count != 0))
		return 1;

	__disable_irq();

	buf->buffer[buf->head++] = data;
	buf->count++;

	if (buf->head == MAXBUFFER)
		buf->head = 0;

	__enable_irq();

	return 0;
}

unsigned int com_length(ComBuffer *buf) {
	return buf->count;
}

void usart2_tx(void) {
	if (com_length(&com_tx_buf)) {
		USART2->TDR = com_getbuf(&com_tx_buf);
	} else {
		USART2->CR1 &= ~BIT3;
		if (USART2->ISR & BIT6) USART2->ICR |= BIT6;
		if (USART2->ISR & BIT7) USART2->RQR |= BIT4;
	}
}

void usart2_rx(void) {
	if (com_putbuf(&com_rx_buf, USART2->RDR))
		com2_error = 1;
}
