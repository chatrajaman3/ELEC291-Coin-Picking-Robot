#include <EFM8LB1.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

idata char buff[20];
#define PERIOD_PIN P1_5

//Motor Pins 
#define OUTPIN1    P1_0
#define OUTPIN2    P1_1
#define OUTPIN3    P1_2
#define OUTPIN4    P1_3

#define SYSCLK 72000000L // SYSCLK frequency in Hz
#define BAUDRATE 115200L

#define SARCLK 18000000L
#define RELOAD_10us (0x10000L-(SYSCLK/(12L*100000L))) // 10us rate

#define VDD 3.3035 // The measured value of VDD in volts

//Servo Vars 
volatile unsigned int pwm_reload;
volatile unsigned char pwm_state=0;
volatile unsigned char count20ms;
volatile unsigned int servo_switch = 0; //0 for elbow, 1 for shoulder 

#define ELBOW_SERVO P2_4
#define SHOULDER_SERVO P2_1
#define MAGNET    P2_6
#define ELBOW_MODE 0 
#define SHOULDER_MODE 1 
#define RELOAD_10MS (0x10000L-(SYSCLK/(12L*100L)))
#define ARM_DELAY 500

#define TRIG_PIN P3_0
#define ECHO_PIN P3_1

#define BACKLED_PIN P0_3

#define NDEBUG

#ifndef NDEBUG
	#define DEBUG_PRINT(fmt, ...) printf("DEBUG: %s:%d: " fmt "\r\n", __FILE__, __LINE__, __VA_ARGS__)
#else
	#define DEBUG_PRINT(fmt, ...) do {} while (0)
#endif
//-----------------------------------------------------------------------//

char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key
  

	//VDM0CN |= 0x80;
	//RSTSRC = 0x02;
	VDM0CN=0x80;       // enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD


	#if (SYSCLK == 48000000L)	
		SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20; // SYSCLK < 75 MHz.
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
		// Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		// Before setting clock to 72 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif
	
	// Configure the pins used as outputs
	P0MDOUT |= 0b_0001_1001; // Configure UART0 TX (P0.4) and UART1 TX (P0.0) as push-pull output
	P1MDOUT |= 0b_1010_1111; // OUPTUT1 to OUTPUT4, coin detector
	P2MDOUT|=0b_0101_0011; //Shoulder and elbow servo, electromagnet 

	P3MDOUT |= 0x01;      // P3.0 (TRIG) push-pull
    P3MDOUT &= ~0x02;     // P3.1 (ECHO) open-drain
   // P3SKIP |= 0x03;       // Skip P3.0 and P3.1
	XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0X00; // 
	XBR2     = 0x41; // Enable crossbar and uart 1

	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	// Configure Uart 0
	SCON0 = 0x10;
	CKCON0 |= 0b_0000_0000 ; // Timer 1 uses the system clock divided by 12.
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready

	//added
	//P2_0=1; // 'set' pin to 1 is normal operation mode. ------------------------------------------------------

	// Initialize timer 5 for periodic interrupts
	SFRPAGE=0x10;
	TMR5CN0=0x00;
	TMR5=0xffff;   // Set to reload immediately
	EIE2|=0b_0000_1000; // Enable Timer5 interrupts
	TR5=1;         // Start Timer5 (TMR5CN0 is bit addressable)
	
	EA=1;
	
	SFRPAGE=0x00;
	
	return 0;
}


//----------------------------------------------------------------------------------//
void Timer5_ISR (void) interrupt INTERRUPT_TIMER5
{
	SFRPAGE=0x10;
	TF5H = 0; // Clear Timer5 interrupt flag
	// Since the maximum time we can achieve with this timer in the
	// configuration above is about 10ms, implement a simple state
	// machine to produce the required 20ms period.

	//Case for elbow servo 
	if (servo_switch == ELBOW_MODE){
		switch (pwm_state)
		{
				case 0:
					ELBOW_SERVO=1;
					TMR5RL=RELOAD_10MS;
					pwm_state=1;
					count20ms++;
				break;
				case 1:
					ELBOW_SERVO=0;
					TMR5RL=RELOAD_10MS-pwm_reload;
					pwm_state=2;
				break;
				default:
					ELBOW_SERVO=0;
					TMR5RL=pwm_reload;
					pwm_state=0;
				break;
		}
	}	

	//case for shoulder servo 
	else{
		switch (pwm_state)
		{
				case 0:
					SHOULDER_SERVO=1;
					TMR5RL=RELOAD_10MS;
					pwm_state=1;
					count20ms++;
				break;
				case 1:
					SHOULDER_SERVO=0;
					TMR5RL=RELOAD_10MS-pwm_reload;
					pwm_state=2;
				break;
				default:
					SHOULDER_SERVO=0;
					TMR5RL=pwm_reload;
					pwm_state=0;
				break;
		}
	}
}


// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
	CKCON0|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN0 & 0x80));  // Wait for overflow
		TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
	unsigned int j;
	for(j=ms; j!=0; j--)
	{
		Timer3us(249);
		Timer3us(249);
		Timer3us(249);
		Timer3us(250);
	}
}

void elbow_control(float pulse){
	servo_switch = ELBOW_MODE;
	pwm_reload=0x10000L-(SYSCLK*pulse*1.0e-3)/12.0;
}

void shoulder_control(float pulse){
	servo_switch = SHOULDER_MODE;
	pwm_reload=0x10000L-(SYSCLK*pulse*1.0e-3)/12.0;
}

void coin_pickup(void){
	MAGNET = 1; 
    shoulder_control(1.5);
    waitms(ARM_DELAY);
    elbow_control(2.4);
    waitms(ARM_DELAY);
    shoulder_control(2.4); 
    waitms(ARM_DELAY);
    shoulder_control(1.5); 
    waitms(ARM_DELAY);
    elbow_control(1.8); 
    waitms(ARM_DELAY);
    elbow_control(1.0);
    waitms(ARM_DELAY);
    shoulder_control(1.0);
    waitms(ARM_DELAY);
    //elbow_control(1.5);
    //waitms(1000);
    MAGNET = 0;
    waitms(ARM_DELAY);
    elbow_control(1.0);
    waitms(ARM_DELAY);
    shoulder_control(1.2);
}

void InitADC (void)
{
	SFRPAGE = 0x00;
	ADEN=0; // Disable ADC
	
	ADC0CN1=
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.		
		(0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
	
	ADC0CF0=
	    ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
	
	ADC0CF1=
		(0 << 7)   | // 0: Disable low power mode. 1: Enable low power mode.
		(0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
	
	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
		(0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
		(0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
		(0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
		(0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
		(0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

	ADC0CF2= 
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
		(0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
		(0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
	
	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
		(0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

	ADEN=1; // Enable ADC
}

void InitPinADC (unsigned char portno, unsigned char pin_num)
{
	unsigned char mask;
	
	mask=1<<pin_num;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
	SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

void UART1_Init (unsigned long baudrate)
{
    SFRPAGE = 0x20;
	SMOD1 = 0x0C; // no parity, 8 data bits, 1 stop bit
	SCON1 = 0x10;
	SBCON1 =0x00;   // disable baud rate generator
	SBRL1 = 0x10000L-((SYSCLK/baudrate)/(12L*2L));
	TI1 = 1; // indicate ready for TX
	SBCON1 |= 0x40;   // enable baud rate generator
	SFRPAGE = 0x00;
}

void putchar1 (char c) 
{
    SFRPAGE = 0x20;
	while (!TI1);
	TI1=0;
	SBUF1 = c;
	SFRPAGE = 0x00;
}

void sendstr1 (char * s)
{
	while(*s)
	{
		putchar1(*s);
		s++;	
	}
}

char getchar1 (void)
{
	char c;
    SFRPAGE = 0x20;
	while (!RI1);
	RI1=0;
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return (c);
}

char getchar1_with_timeout (void)
{
	char c;
	unsigned int timeout;
    SFRPAGE = 0x20;
    timeout=0;
	while (!RI1)
	{
		SFRPAGE = 0x00;
		Timer3us(20);
		SFRPAGE = 0x20;
		timeout++;
		if(timeout==25000)
		{
			SFRPAGE = 0x00;
			return ('\n'); // Timeout after half second
		}
	}
	RI1=0;
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return (c);
}

void getstr1 (char * s, unsigned char n)
{
	char c;
	unsigned char cnt;
	
	cnt=0;
	while(1)
	{
		c=getchar1_with_timeout();
		if(c=='\n')
		{
			*s=0;
			return;
		}
		
		if (cnt<n)
		{
			cnt++;
			*s=c;
			s++;
		}
		else
		{
			*s=0;
			return;
		}
	}
}

// RXU1 returns '1' if there is a byte available in the receive buffer of UART1
bit RXU1 (void)
{
	bit mybit;
    SFRPAGE = 0x20;
	mybit=RI1;
	SFRPAGE = 0x00;
	return mybit;
}

void waitms_or_RI1 (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
	{
		for (k=0; k<4; k++)
		{
			if(RXU1()) return;
			Timer3us(250);
		}
	}
}

void SendATCommand (char * s)
{
	printf("Command: %s", s);
	P2_0=0; // 'set' pin to 0 is 'AT' mode.
	waitms(5);
	sendstr1(s);
	getstr1(buff, sizeof(buff)-1);
	waitms(10);
	P2_0=1; // 'set' pin to 1 is normal operation mode.
	printf("Response: %s\r\n", buff);
}

void ReceptionOff (void)
{
	P2_0=0; // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	sendstr1("AT+DVID0000\r\n"); // Some unused id, so that we get nothing in RXD1.
	waitms(10);
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	P2_0=1; // 'set' pin to 1 is normal operation mode.
}

float Volts_at_Pin(unsigned char pin)
{
	 return ((ADC_at_Pin(pin)*VDD)/0b_0011_1111_1111_1111);
}

// Measure the period of a square signal at PERIOD_PIN
unsigned long GetPeriod (int n)
{
	unsigned int overflow_count;
	unsigned char i;
	
	TR0=0; // Stop Timer/Counter 0
	TMOD&=0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
	TMOD|=0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer

	// Reset the counter
	TR0=0;
	TL0=0; TH0=0; TF0=0; overflow_count=0;
	TR0=1;
	while(PERIOD_PIN!=0) // Wait for the signal to be zero
	{
		if(TF0==1) // Did the 16-bit timer overflow?
		{
			TF0=0;
			overflow_count++;
			if(overflow_count==10) // If it overflows too many times assume no signal is present
			{
				TR0=0;
				return 0; // No signal
			}
		}
	}
	
	// Reset the counter
	TR0=0;
	TL0=0; TH0=0; TF0=0; overflow_count=0;
	TR0=1;
	while(PERIOD_PIN!=1) // Wait for the signal to be one
	{
		if(TF0==1) // Did the 16-bit timer overflow?
		{
			TF0=0;
			overflow_count++;
			if(overflow_count==10) // If it overflows too many times assume no signal is present
			{
				TR0=0;
				return 0; // No signal
			}
		}
	}
	
	// Reset the counter
	TR0=0;
	TL0=0; TH0=0; TF0=0; overflow_count=0;
	TR0=1; // Start the timer
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while(PERIOD_PIN!=0) // Wait for the signal to be zero
		{
			if(TF0==1) // Did the 16-bit timer overflow?
			{
				TF0=0;
				overflow_count++;
			}
		}
		while(PERIOD_PIN!=1) // Wait for the signal to be one
		{
			if(TF0==1) // Did the 16-bit timer overflow?
			{
				TF0=0;
				overflow_count++;
			}
		}
	}
	TR0=0; // Stop timer 0, the 24-bit number [overflow_count-TH0-TL0] has the period in clock cycles!
	
	return (overflow_count*65536+TH0*256+TL0);
}

void eputs(char *String)
{	
	while(*String)
	{
		putchar(*String);
		String++;
	}
}

void PrintNumber(long int val, int Base, int digits)
{ 
	code const char HexDigit[]="0123456789ABCDEF";
	int j;
	#define NBITS 32
	xdata char buff[NBITS+1];
	buff[NBITS]=0;
	
	if(val<0)
	{
		putchar('-');
		val*=-1;
	}

	j=NBITS-1;
	while ( (val>0) | (digits>0) )
	{
		buff[j--]=HexDigit[val%Base];
		val/=Base;
		if(digits!=0) digits--;
	}
	eputs(&buff[j+1]);
}


void motor_stop (void) {

	OUTPIN1=0;
	OUTPIN2=0;
	OUTPIN3=0;
	OUTPIN4=0;
}


void motor_forward (void) {

	BACKLED_PIN = 1;
	OUTPIN1=1;
	OUTPIN2=0;
	OUTPIN3=1;
	OUTPIN4=0;
}

void motor_backward (void) {

	BACKLED_PIN = 1;
	OUTPIN1=0;
	OUTPIN2=1;
	OUTPIN3=0;
	OUTPIN4=1;
}


void motor_left (void) {

	OUTPIN1=1;
	OUTPIN2=0;
	OUTPIN3=0;
	OUTPIN4=1;
}


void motor_right (void) {

	OUTPIN1=0;
	OUTPIN2=1;
	OUTPIN3=1;
	OUTPIN4=0;
}



void random_turn(void) {
    unsigned int turn_time = (rand() % 1000) + 500; // Random turn duration between 500ms and 1500ms
    
	motor_left();
    
    waitms(turn_time);
    motor_stop();
    waitms(500); // Pause before moving forward again
}

void sonar_turn(void) {
    unsigned int turn_direction = rand() % 2; // Randomly choose left (0) or right (1) // not needed
    unsigned int turn_time_sonar = (rand() % 500) + 250; // Random turn duration between 500ms and 1500ms


    if (turn_direction == 1) {
        motor_right();
    }
    else {
        motor_left();
    }

    waitms(turn_time_sonar);
    motor_stop();
    waitms(500); // Pause before moving forward again
}

unsigned long base_freq (void) {
	
	long int count, f = 0;
	long int base_f = 0;
	int n = 0;

	while (n < 5) {
		count=GetPeriod(30);
		f=(SYSCLK*30.0)/(count*12);
		if (base_f < f) {
			base_f = f;
		}
		n++;

		waitms(100);
	}

	DEBUG_PRINT("Base frequency: %ld", base_f);
	return base_f;
}


float base_volt(unsigned char pin) {
	float volt = 0;
	float base_v = 0;
	int n = 0;

	while (n < 10) {
		volt = Volts_at_Pin(pin);
		if (base_v < volt){
			base_v = volt;
		}
		n++;
		waitms(100);	
	}
	
	DEBUG_PRINT("Base voltage: %f", base_v);
	return base_v;
}

void victory_dance(void) {
    motor_left();
    waitms(3000);
    motor_stop();
    shoulder_control(1.5);
    waitms(500);
    elbow_control(2.4);
    waitms(500);
    elbow_control(1.0);
    waitms(500);
    elbow_control(2.4);
    waitms(500);
    elbow_control(1.0);
    waitms(500);
    shoulder_control(1.2);
}

void send_trigger_pulse() {
    TRIG_PIN = 1;
    Timer3us(10);  // 10 µs pulse
    TRIG_PIN = 0;
}

unsigned int measure_echo_pulse() {
    unsigned int duration = 0;

    while (!ECHO_PIN);          // Wait for echo to go HIGH
    while (ECHO_PIN) {          // Measure time echo stays HIGH
        Timer3us(1);
        duration++;
    }

    return duration;
}

void main (void)
{
	int is_auto_mode;
    long int j, v;
    //var for perimeter voltage
    float perimeter_1, perimeter_2, base_volt_1, base_volt_2;
	long int count, f, threshold_freq;
	
	//coin count var
	int coin = 0;

	//added
    char c;

	//ultrasonic sensor 
	unsigned int echo_time;
    //float distance_cm;
    
	//servos 
	float pulse_width;
    count20ms=0;
	is_auto_mode = 0;



	
	waitms(500);
	printf("\r\nEFM8LB12 JDY-40 Slave Test.\r\n");

	//******change this later please optmize and just measure threshold from start, only for testing */
	

	
	InitPinADC(2, 2); // Configure P2.2 as analog input
	InitPinADC(2, 3); // Configure P2.3 as analog input
	InitADC();

	TRIG_PIN = 0;  // Ensure TRIG is LOW

	//added
	UART1_Init(9600);

	ReceptionOff();

	// To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");

	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	SendATCommand("AT+DVIDC0A8\r\n");
	SendATCommand("AT+RFC108\r\n");

	//added end
    
    waitms(1000); // Wait a second to give PuTTy a chance to start

	motor_stop();
	
	f = 0;
	threshold_freq = base_freq() + 100;
	base_volt_1 = base_volt(QFP32_MUX_P2_2);
	base_volt_2 = base_volt(QFP32_MUX_P2_3);
	
	while (1)
	{
		//for metal detection, changes in frequency represent coin detection
		count=GetPeriod(30);
		if(count>0)
		{
			f=(SYSCLK*30.0)/(count*12);
			eputs("f=");
			PrintNumber(f, 10, 7);
			// eputs("Hz, count=");
			// PrintNumber(count, 10, 8);
			// eputs("          \r");
			
		}
		else
		{
			eputs("NO SIGNAL                     \r");
		}
		
		c = 0;
		if (RXU1()) {
			c = getchar1();
			DEBUG_PRINT("%c", c);
		}

		if (c == '#')
			is_auto_mode = !is_auto_mode;

restart:
		if (is_auto_mode) {
			DEBUG_PRINT("Automatic", 0);
			goto automatic;
		} else {
			DEBUG_PRINT("Manual", 0);
			goto manual;
		}

manual:		

		if(c=='!') // Master is sending message
		{
			getstr1(buff, sizeof(buff)-1);
			DEBUG_PRINT("%c", buff[0]);
			#if 0
			switch ((buff[0] & 0b1100) >> 2) {
			case 0b01:
				printf("motor_left");
				break;
			case 0b10:
				printf("Right");
				break;
			default:
				printf("Centre");
				break;
			}

			switch (buff[0] & 0b11) {
			case 0b01:
				printf("Down");
				break;
			case 0b10:
				printf("Up");
				break;
			default:
				printf("Centre");
				break;
			}
			printf("\r\n");
			#else
			if (buff[0] & (0b1 << 4)) { 
				DEBUG_PRINT("Pickup", 0);
				motor_stop();
				// waitms(1000);
				coin_pickup();
				// continue;
				SFRPAGE = 0x20;
				c = SBUF1;
				SFRPAGE = 0x00;
			} else {
				switch (buff[0] & 0b1111) {
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
			#endif
		}
	
		goto telemetry;
	
automatic:



		j=ADC_at_Pin(QFP32_MUX_P2_2);
		v=(j*33000)/0x3fff; 
		eputs("ADC[P2.2]=0x");
		PrintNumber(j, 16, 4);
		eputs(", ");
		PrintNumber(v/10000, 10, 1);

		// perimeter_1 = v/10000;
		 putchar('.');
		 PrintNumber(v%10000, 10, 4);
		eputs("V ");

		perimeter_1 = Volts_at_Pin(QFP32_MUX_P2_2);


		
		//store var for perimeter 1

		j=ADC_at_Pin(QFP32_MUX_P2_3);
		v=(j*33000)/0x3fff;
		eputs("ADC[P2.3]=0x");
		PrintNumber(j, 16, 4);
		eputs(", ");
		PrintNumber(v/10000, 10, 1);
		// perimeter_2 = v/10000;


		// perimeter_2 = perimeter_2 + v%10000; 
		perimeter_2 = Volts_at_Pin(QFP32_MUX_P2_3);
		putchar('.');
		PrintNumber(v%10000, 10, 4);
		eputs("V ");
		
		//store var for perimeter 2

		// Not very good for high frequencies because of all the interrupts in the background
		// but decent for low frequencies around 10kHz.
		
		//ultrasonic code 
		send_trigger_pulse();
        echo_time = measure_echo_pulse();
		SFRPAGE = 0x20;
		c = SBUF1;
		SFRPAGE = 0x00;

		//will later change to > threshold_freq + 100->300
		if (f > threshold_freq) {

			motor_stop();
			motor_backward();
			//adjust this
			waitms(200);
			eputs("coin det");
			motor_stop();
			//pick up coin function
			coin_pickup();
			waitms(300);
			SFRPAGE = 0x20;
			c = SBUF1;
			SFRPAGE = 0x00;
			
			//after pick up coin, incr coin counter
			coin++;

			eputs("Coin detected");
		} else if (perimeter_1 > base_volt_1 + 0.2 || perimeter_2 > base_volt_2 + 0.2) {
			eputs("Perimeter detected");
			motor_stop();
			//backup a little 
			motor_backward();
			//adjust this
			waitms(400);
			//change angle randomly
			random_turn();
			SFRPAGE = 0x20;
			c = SBUF1;
			SFRPAGE = 0x00;

			motor_stop();
		}

		else if ((float) echo_time/58 < 3.0){
			motor_stop();
			//backup a little 
			motor_backward();
	
			waitms(500);

			//change angle randomly
			sonar_turn();
			SFRPAGE = 0x20;
			c = SBUF1;
			SFRPAGE = 0x00;
			
			motor_stop();

		}
		
		
		//***can change this later to variable coin limit
		if (coin == 20) {
			//victory_dance();
			motor_stop();
			is_auto_mode = 0;
			coin = 0;
			// ReceptionOff();
			waitms(10);
			SFRPAGE = 0x20;
			c = SBUF1;
			SFRPAGE = 0x00;
			waitms(10);
			// goto restart;
			continue;
		}
			//other end of task functions or features below:

		motor_forward();

telemetry:
		if (c=='@') // Master wants slave data
		{
			sprintf(buff, "%03ld\n", f - threshold_freq + 200);
			waitms(5); // The radio seems to need this delay...
			sendstr1(buff);
		}
			//eputs("      moving    \r");
	}

}
