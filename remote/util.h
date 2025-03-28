/*
 * Coin Picking Robot (Remote)
 * util.h
 */

#ifndef UTIL_H
#define UTIL_H

#define F_CPU 32000000L

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
#define CHARS_PER_LINE 16

void sleep(unsigned int ms);
void sleep_us(unsigned char us);

void lcd_init(void);
void lcd_print(char *s, unsigned char line, unsigned char clear);
void lcd_write_command(unsigned char x);
void lcd_write_data(unsigned char x);
void lcd_byte(unsigned char x);
void lcd_pulse(void);

void adc_init(void);
int adc_read(unsigned int channel);

void uart2_init(int baud);
int uart2_received(void);
int com2_read(int max, unsigned char *buf);
int com2_write(int count, unsigned char *buf);
int com2_egets(char *s, int size);
int com2_eputs(char *s);
int com2_echos(char *s, int size);
char com2_egetc(void);
void com2_eputc(char c);
char com2_echoc(void);

#endif /* UTIL_H */
