#include "../Common/Include/stm32l051xx.h"
#include "util_stm.h"


// PIN CONFIGURATIONS -> RCC, GPIO, etc.
void Configure_Pins (void)
{
	// Configure the pin used for a blinking LED: PA8 (pin 18)
  RCC->IOPENR  |= BIT0; // peripheral clock enable for port A
  GPIOA->MODER  = (GPIOA->MODER & ~(BIT17|BIT16) ) | BIT16; // Make pin PA8 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0))
	
	// Configure the pin used for analog input: PB1 (pin 15)
	RCC->IOPENR  |= BIT1;         // peripheral clock enable for port B
	GPIOB->MODER |= (BIT2|BIT3);  // Select analog mode for PB1 (pin 15 of LQFP32 package)
}


// LCD FUNCTIONS
void lcd_pulse(void) {
	LCD_E_1;
	sleep_us(40);
	LCD_E_0;
	return;
}

void lcd_byte(unsigned char x) {
	if (x & 0x80) LCD_D7_1; else LCD_D7_0;
	if (x & 0x40) LCD_D6_1; else LCD_D6_0;
	if (x & 0x20) LCD_D5_1; else LCD_D5_0;
	if (x & 0x10) LCD_D4_1; else LCD_D4_0;
	lcd_pulse();
	sleep_us(40);

	if (x & 0x08) LCD_D7_1; else LCD_D7_0;
	if (x & 0x04) LCD_D6_1; else LCD_D6_0;
	if (x & 0x02) LCD_D5_1; else LCD_D5_0;
	if (x & 0x01) LCD_D4_1; else LCD_D4_0;
	lcd_pulse();

	return;
}

void lcd_write_data(unsigned char x) {
	LCD_RS_1;
	lcd_byte(x);
	sleep(2);
	return;
}

void lcd_write_command(unsigned char x) {
	LCD_RS_0;
	lcd_byte(x);
	sleep(5);
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

void lcd_print(char *string, unsigned char line, unsigned char clear) {
	int i;

	lcd_write_command(line == 1 ? 0x80 : 0xC0);
	sleep(5);
	for (i = 0; string[i] != 0; ++i)
		lcd_write_data(string[i]);
	if (clear)
		for (; i < CHARS_PER_LINE; ++i)
			lcd_write_data(' ');
	return;
}


// ADC FUNCTIONS

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
	ADC1->CR |= ADC_CR_ADCAL; /* (3) */
	while ((ADC1->ISR & ADC_ISR_EOCAL) == 0);
	ADC1->ISR |= ADC_ISR_EOCAL;
}

int adc_read(unsigned int channel) {
	/* Single conversion sequence code example - Software trigger
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


// DELAY FUNCTIONS

void wait_1ms(void)
{
	// For SysTick info check the STM32l0xxx Cortex-M0 programming manual.
	SysTick->LOAD = (F_CPU/1000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	//SysTick->CTRL = 0x05; // Bit 0: ENABLE, BIT 1: TICKINT, BIT 2:CLKSOURCE
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while((SysTick->CTRL & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
}

void delayms(int len)
{
	while(len--) wait_1ms();
}

void sleep_us(unsigned char us) {
	// For SysTick info check the STM32L0xxx Cortex-M0 programming manual page 85.
	SysTick->LOAD = (F_CPU/(1000000L/us)) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while ((SysTick->CTRL & BIT16) == 0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
	return;
}

void sleep(unsigned int ms) {
	unsigned int i;
	unsigned char j;
	for (i = 0; i < ms; ++i)
		for (j = 0; j < 4; ++j)
			sleep_us(250);
	return;
}

