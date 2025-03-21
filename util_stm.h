#ifndef UTIL_H
#define UTIL_H

// #define F_CPU 32000000L 

#define LCD_RS_0 (GPIOA->ODR &= ~BIT0)
#define LCD_RS_1 (GPIOA->ODR |= BIT0)
#define LCD_E_0 (GPIOA->ODR &= ~BIT1)
#define LCD_E_1 (GPIOA->ODR |= BIT1)
#define LCD_D4_0 (GPIOA->ODR &= ~BIT2)
#define LCD_D4_1 (GPIOA->ODR |= BIT2)
#define LCD_D5_0 (GPIOA->ODR &= ~BIT3)
#define LCD_D5_1 (GPIOA->ODR |= BIT3)
#define LCD_D6_0 (GPIOA->ODR &= ~BIT4)
#define LCD_D6_1 (GPIOA->ODR |= BIT4)
#define LCD_D7_0 (GPIOA->ODR &= ~BIT5)
#define LCD_D7_1 (GPIOA->ODR |= BIT5)

void Configure_Pins (void);
void lcd_pulse(void);
void lcd_byte(unsigned char x);
void lcd_write_data(unsigned char x);
void lcd_write_command(unsigned char x);
void lcd_init(void);
void lcd_print(char *string, unsigned char line, unsigned char clear);
void adc_init(void);
int adc_read(unsigned int channel);
void wait_1ms(void);
void delayms(int len);
void sleep_us(unsigned char us);
void sleep(unsigned int ms);

#endif /* UTIL_H */
