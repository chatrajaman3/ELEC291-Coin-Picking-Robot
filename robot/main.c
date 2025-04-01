/*
 * Coin Picking Robot (Robot Subsystem)
 * main.c
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <EFM8LB1.h>

#include "util.h"

#define PERIOD_PIN P1_5
#define TRIG_PIN P3_0
#define ECHO_PIN P3_1

#define RIGHT_FWD P1_0
#define RIGHT_REV P1_1
#define LEFT_FWD P1_2
#define LEFT_REV P1_3

#define ELBOW_SERVO P2_4
#define SHOULDER_SERVO P2_1
#define MAGNET P2_6
#define ELBOW_MODE 0
#define SHOULDER_MODE 1
#define ARM_DELAY 500

volatile unsigned int pwm_reload;
volatile unsigned char pwm_state;
volatile unsigned char count20ms;
volatile unsigned int servo_switch;

idata char buf[20];

#ifndef NDEBUG
	#define DEBUG_PRINT(fmt, ...) printf("DEBUG: %s:%d: " fmt "\r\n",      \
		__FILE__, __LINE__, __VA_ARGS__)
#else
	#define DEBUG_PRINT(fmt, ...) do {} while (0)
#endif

void Timer5_ISR (void) interrupt INTERRUPT_TIMER5 {
	SFRPAGE = 0x10;
	TF5H = 0;

	if (servo_switch == ELBOW_MODE){
		switch (pwm_state) {
		case 0:
			ELBOW_SERVO = 1;
			TMR5RL = RELOAD_10MS;
			pwm_state = 1;
			++count20ms;
			break;
		case 1:
			ELBOW_SERVO = 0;
			TMR5RL = RELOAD_10MS - pwm_reload;
			pwm_state = 2;
			break;
		default:
			ELBOW_SERVO = 0;
			TMR5RL = pwm_reload;
			pwm_state = 0;
			break;
		}
	} else {
		switch (pwm_state) {
		case 0:
			SHOULDER_SERVO = 1;
			TMR5RL = RELOAD_10MS;
			pwm_state = 1;
			++count20ms;
			break;
		case 1:
			SHOULDER_SERVO = 0;
			TMR5RL = RELOAD_10MS - pwm_reload;
			pwm_state = 2;
			break;
		default:
			SHOULDER_SERVO = 0;
			TMR5RL = pwm_reload;
			pwm_state = 0;
			break;
		}
	}
}

void elbow_control(float pulse) {
	servo_switch = ELBOW_MODE;
	pwm_reload = 0x10000L - (SYSCLK*pulse*1.0e-3)/12.0;
}

void shoulder_control(float pulse) {
	servo_switch = SHOULDER_MODE;
	pwm_reload = 0x10000L - (SYSCLK*pulse*1.0e-3)/12.0;
}

void coin_pickup(void) {
	MAGNET = 1;
	shoulder_control(1.5);
	sleep(ARM_DELAY);
	elbow_control(2.4);
	sleep(ARM_DELAY);
	shoulder_control(2.4);
	sleep(ARM_DELAY);
	shoulder_control(1.5);
	sleep(ARM_DELAY);
	elbow_control(1.8);
	sleep(ARM_DELAY);
	elbow_control(1.0);
	sleep(ARM_DELAY);
	shoulder_control(1.0);
	sleep(ARM_DELAY);
	MAGNET = 0;
	sleep(ARM_DELAY);
	elbow_control(1.0);
	sleep(ARM_DELAY);
	shoulder_control(1.2);
}

void send_command(char *s) {
	printf("Command: %s", s);
	P2_0 = 0;
	sleep(5);
	uart1_putstr(s);
	uart1_getstr(buf, sizeof(buf)-1);
	sleep(10);
	P2_0 = 1;
	printf("Response: %s\r\n", buf);
}

void reception_off(void) {
	P2_0 = 0;
	sleep(10);
	uart1_putstr("AT+DVID0000\r\n");
	sleep(10);
	SCON1 &= 0b00111111;
	P2_0 = 1;
}

unsigned long get_period(int n) {
	/* If it overflows too many times assume no signal is present. */
	unsigned int overflow;
	unsigned char i;

	TR0 = 0;
	TMOD &= 0b11110000;
	TMOD |= 0b00000001;

	TR0 = 0;
	TL0 = 0; TH0 = 0; TF0 = 0; overflow = 0;
	TR0 = 1;

	while (PERIOD_PIN != 0) {
		if (TF0 == 1) {
			TF0 = 0;
			++overflow;
			if (overflow == 10) {
				TR0 = 0;
				return 0;
			}
		}
	}

	TR0 = 0;
	TL0 = 0; TH0 = 0; TF0 = 0; overflow = 0;
	TR0 = 1;

	while (PERIOD_PIN != 1) {
		if (TF0 == 1) {
			TF0 = 0;
			++overflow;
			if (overflow == 10) {
				TR0 = 0;
				return 0;
			}
		}
	}

	TR0 = 0;
	TL0 = 0; TH0 = 0; TF0 = 0; overflow = 0;
	TR0 = 1;

	for (i = 0; i < n; ++i) {
		while (PERIOD_PIN != 0) {
			if (TF0 == 1) {
				TF0 = 0;
				++overflow;
			}
		}
		while (PERIOD_PIN != 1) {
			if (TF0 == 1) {
				TF0 = 0;
				++overflow;
			}
		}
	}

	TR0 = 0;

	return 65536*overflow + 256*TH0 + TL0;
}

void motor_stop(void) {
	LEFT_FWD = LEFT_REV = RIGHT_FWD = RIGHT_REV = 0;
}

void motor_forward(void) {
	LEFT_FWD = RIGHT_FWD = 1;
	LEFT_REV = RIGHT_REV = 0;
}

void motor_backward(void) {
	LEFT_REV = RIGHT_REV = 1;
	LEFT_FWD = RIGHT_FWD = 0;
}

void motor_left(void) {
	LEFT_REV = RIGHT_FWD = 1;
	LEFT_FWD = RIGHT_REV = 0;
}

void motor_right(void) {
	LEFT_FWD = RIGHT_REV = 1;
	LEFT_REV = RIGHT_FWD = 0;
}

void random_turn(void) {
	unsigned int turn_time;

	motor_left();

	sleep(turn_time = (rand()%1000 + 500));
	motor_stop();
	sleep(500);
}

unsigned long get_base_freq(void) {
	long int f, f_max;
	int i;

	f = f_max = 0;

	for (i = 0; i < 5; ++i) {
		if ((f = SYSCLK * 30.0 / get_period(30) / 12) > f_max)
			f_max = f;

		sleep(100);
	}

	DEBUG_PRINT("Base frequency: %ld", f_max);
	return f_max;
}

float get_base_volt(unsigned char pin) {
	float v, v_max;
	int i;

	v = v_max = 0;

	for (i = 0; i < 10; ++i) {
		if ((v = adc_volts(pin)) > v_max)
			v_max = v;

		sleep(100);
	}

	DEBUG_PRINT("Base voltage: %f", v_max);
	return v_max;
}

void victory_dance(void) {
	motor_left();
	sleep(3000);

	motor_stop();

	shoulder_control(1.5);
	sleep(500);
	elbow_control(2.4);
	sleep(500);
	elbow_control(1.0);
	sleep(500);
	elbow_control(2.4);
	sleep(500);
	elbow_control(1.0);
	sleep(500);
	shoulder_control(1.2);
}

void send_trigger_pulse(void) {
	TRIG_PIN = 1;
	usleep(10);
	TRIG_PIN = 0;
}

unsigned int measure_echo_pulse(void) {
    unsigned int duration;

    duration = 0;
    while (!ECHO_PIN);
    while (ECHO_PIN) {
        usleep(1);
        ++duration;
    }

    return duration;
}

void main(void) {
	char c;
	unsigned int echo_time;
	int is_auto_mode, coins;
	long int count, f, f_base;
	float v_perim_1, v_perim_2, v_base_1, v_base_2;

	MAGNET = 0;
	TRIG_PIN = 0;
	pwm_state = 0;
	servo_switch = 0;
	count20ms = 0;
	is_auto_mode = 0;
	f = 0;
	coins = 0;

	sleep(500);

	adc_pin_init(2, 2);
	adc_pin_init(2, 3);
	adc_init();

	uart1_init(9600);

	reception_off();

	send_command("AT+VER\r\n");
	send_command("AT+BAUD\r\n");
	send_command("AT+RFID\r\n");
	send_command("AT+DVID\r\n");
	send_command("AT+RFC\r\n");
	send_command("AT+POWE\r\n");
	send_command("AT+CLSS\r\n");

	send_command("AT+DVIDC0A8\r\n");
	send_command("AT+RFC108\r\n");

	motor_stop();

	sleep(1000);

	/* Calibrate on boot. */
	f_base = get_base_freq();
	v_base_1 = get_base_volt(QFP32_MUX_P2_2);
	v_base_2 = get_base_volt(QFP32_MUX_P2_3);

	while (1) {
		/* For metal detection, changes in frequency
		 * represent coin detection. */
		count = get_period(30);
		if (count > 0)
			f = SYSCLK * 30.0 / get_period(30) / 12;

		c = 0;
		if (uart1_received()) {
			c = uart1_getchar();
			DEBUG_PRINT("%c", c);
		}

		if (c == '#')
			is_auto_mode = !is_auto_mode;

		if (is_auto_mode) {
			DEBUG_PRINT("Automatic", 0);
			goto automatic;
		} else {
			DEBUG_PRINT("Manual", 0);
			goto manual;
		}

manual:
		if (c == '!') {
			uart1_getstr(buf, sizeof(buf)-1);
			DEBUG_PRINT("%c", buf[0]);

			if (buf[0] & (0b1 << 4)) { 
				DEBUG_PRINT("Pickup", 0);
				motor_stop();
				coin_pickup();
				SFRPAGE = 0x20;
				c = SBUF1;
				SFRPAGE = 0x00;
			} else {
				switch (buf[0] & 0b1111) {
				case 0b0101:
				case 0b0110:
				case 0b0111:
					DEBUG_PRINT("Left", 0);
					motor_left();
					break;
				case 0b1001:
				case 0b1010:
				case 0b1011:
					DEBUG_PRINT("Right", 0);
					motor_right();
					break;
				case 0b1101:
					DEBUG_PRINT("Back", 0);
					motor_backward();
					break;
				case 0b1110:
					DEBUG_PRINT("Forward", 0);
					motor_forward();
					break;
				default:
					DEBUG_PRINT("Stop", 0);
					motor_stop();
				}
			}
		} else if (c == '@') {
		}

		continue;

automatic:
		v_perim_1 = adc_volts(QFP32_MUX_P2_2);
		v_perim_2 = adc_volts(QFP32_MUX_P2_3);

		send_trigger_pulse();
        	echo_time = measure_echo_pulse();

		if (f > f_base + 100) {
			DEBUG_PRINT("Coin detected", 0);
			motor_stop();
			motor_backward();
			sleep(200);
			motor_stop();
			coin_pickup();
			sleep(300);
			++coins;
		} else if (v_perim_1 > v_base_1 + 0.2
			|| v_perim_2 > v_base_2 + 0.2) {
			DEBUG_PRINT("Perimeter detected", 0);
			motor_stop();
			motor_backward();
			sleep(400);
			random_turn();
			motor_stop();
		} else if ((float)echo_time/58 < 3.0) {
			DEBUG_PRINT("Object detected", 0);
			motor_stop();
			motor_backward();
			sleep(400);
			random_turn();
			motor_stop();
		}

		if (coins == 20) {
			motor_stop();
		} else {
			motor_forward();
		}
	}
}
