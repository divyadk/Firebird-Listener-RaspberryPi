/*
 * ReadSensors.c
 *
 * Created: 5/21/2018 3:51:26 PM
 * Author : divya kulkarni
 */ 
/********************************************************************************
Character codes to read the sensors:

1. 'a' to 'e' for the 5 IR sensors
2. 'S','T' and 'U' for centre, left and the right sharp IR sensors respectively (this is
measure distance)

Character codes for the motor speed:

for the motor speed, pass the string:
m150#150! this indicates speed of left and right motor 
********************************************************************************/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "lcd.c"

unsigned char data; //to store received data from UDR1
int hashe=0,ende=1,incoming=0;
char m1[20];
char m2[20];
int res,ss;
void move(void);
void read(void);
unsigned char ADC_Conversion(unsigned char);
void move_robot(void);
unsigned char ADC_Value;
unsigned char sharp, distance, adc_reading;
unsigned int value;
unsigned char slen[12];
unsigned char sharpa[12];

void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
		return distanceInt;
}


//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00; //set PORTF direction as input
	PORTF = 0x00; //set PORTF pins floating
	DDRK = 0x00; //set PORTK direction as input
	PORTK = 0x00; //set PORTK pins floating
}

void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void sendsharplen(void){
	unsigned int th,temp1;
	
	sprintf(sharpa,"%d", value);
	unsigned int len=strlen(sharpa);
	sprintf(slen,"%d",len);
	int len1=strlen(slen);
	/*for(int i=0;i<len;i++){
		UDR2=slen[i];
	}
		temp1 = value/1000;
		th = temp1%10 + 48;
		lcd_wr_char(th);
		UDR2=th;
	//}*/
	//last attempt god help this
	UDR2=value/256;
	UDR2=value%256;
}

void sendsharp(void){
	
	int len=strlen(sharpa);
	for(int i=0;i<len;i++){
		UDR2='a';
		UDR2=sharpa[i];
	}
		UDR2='e';
}

void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//to set the direction of the motors
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 			// removing upper nibbel as it is not needed
	PortARestore = PORTA; 			// reading the PORTA's original status
	PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
	PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
	PORTA = PortARestore; 			// setting the command to the port
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void)
{
	motion_set(0x00);
}
//Function to initialize ports
void port_init()
{
	lcd_port_config();
	motion_pin_config();
	buzzer_pin_config();
	adc_pin_config();
}

//Function to Initialize ADC
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

// Function for robot velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}


//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}


void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

//Function To Initialize UART2
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart2_init(void)
{
 UCSR2B = 0x00; //disable while setting baud rate
 UCSR2A = 0x00;
 UCSR2C = 0x06;
 UBRR2L = 0x5F; //set baud rate lo
 UBRR2H = 0x00; //set baud rate hi
 UCSR2B = 0x98;
}

/*ISR(USART2_TX_vect)
{
	unsigned char val;
	val=ADC_Conversion(4);
	UDR2='a';
	UDR2=val;
	val=ADC_Conversion(5);
	UDR2='b';
	UDR2=val;
	val=ADC_Conversion(6);
	UDR2='c';
	UDR2=val;
	val=ADC_Conversion(7);
	UDR2='d';
	UDR2=val;
	val=ADC_Conversion(8);
	UDR2='e';
	UDR2=val;
}*/

ISR(USART2_RX_vect)	// ISR for receive complete interrupt
{
	data = UDR2; 				//making copy of data from UDR2 in 'data' variable 
	//UDR2 = data; 				//echo data back to PC
	if(data=='a'){
		ADC_Value=ADC_Conversion(4);
		UDR2=ADC_Value;
	}
	if(data=='b'){
		ADC_Value=ADC_Conversion(5);
		UDR2=ADC_Value;
	}
	if(data=='c'){
		ADC_Value=ADC_Conversion(6);
		UDR2=ADC_Value;
	}
	if(data=='d'){
		ADC_Value=ADC_Conversion(7);
		UDR2=ADC_Value;
	}
	if(data=='e'){
		ADC_Value=ADC_Conversion(8);
		UDR2=ADC_Value;
	}
	if(data=='S'){
		sharp=ADC_Conversion(11);
		value = Sharp_GP2D12_estimation(sharp);
		sendsharplen();
		//ss=1;
		//sendsharp();
	}
	//leftmost val
	if(data=='T'){
		sharp=ADC_Conversion(9);
		value=Sharp_GP2D12_estimation(sharp);
		sendsharplen();
	}
	//righmost val
	if(data=='U'){
		sharp=ADC_Conversion(13);
		value=Sharp_GP2D12_estimation(sharp);
		sendsharplen();
	}
	if(data=='1' && ss==1)
	{
		UDR2=sharpa[0];
		if(strlen(sharpa)==1)
			ss=0;
	}
	if(data=='2' && ss==1){
		UDR2=sharpa[1];
		if(strlen(sharpa)==2)
			ss=0;
	}
	if(data=='3' && ss==1){
		UDR2=sharpa[2];
		if(strlen(sharpa)==3)
			ss=0;
	}
	if(data=='4' && ss==1){
		UDR2=sharpa[3];
		if(strlen(sharpa)>=4)
			ss=0;
	}
	//here starts the incoming of the motor values
	if(data=='m'){
		incoming=1;res=0;
		lcd_wr_char(data);
	}
	if (incoming==1 && hashe==0 && ende==1 && data!='m'){
		res=strlen(m1);
		m1[res++]=data;
		m1[res]='\0';
		lcd_wr_char(data);
	}
	if (data== '#')
	{
		hashe=1;ende=0;res=0;
		lcd_wr_char(data);
	}
	if(data=='!')
	{
		ende=1;hashe=0;incoming=0;
		lcd_wr_char(data);
		move_robot();
	}

	if (hashe==1 && ende==0 && data!='#')
	{
		lcd_wr_char(data);
		res=strlen(m2);
		m2[res++]=data;
		m2[res]='\0';
	}
	
	/*char *token=strtok(data,"#");
		
	char *token1=strtok(NULL,"#");
		
		if(token == 0x38) //ASCII value of 8
		{
			PORTA=0x06;  //forward
			_delay_ms(500);
		}

		if(token1 == 0x35) //ASCII value of 5
		{
			PORTA=0x09; //back
			_delay_ms(500);
		}*/

	/*	
	
	if(token1 == "0x32") //ASCII value of 2
	{
		PORTA=0x09; //back
		_delay_ms(500);
	}
	
	if(data == 0x34) //ASCII value of 4
		{
			PORTA=0x05;  //left
		}

		if(data == 0x36) //ASCII value of 6
		{
			PORTA=0x0A; //right
		}

		if(data == 0x35) //ASCII value of 5
		{
			PORTA=0x00; //stop
		}

		if(data == 0x37) //ASCII value of 7
		{
			buzzer_on();
		}

		if(data == 0x39) //ASCII value of 9
		{
			buzzer_off();
		} */

}




//Function To Initialize all The Devices
void init_devices()
{
 cli(); //Clears the global interrupts
 port_init();  //Initializes all the ports
 timer5_init();
 adc_init();
 uart2_init(); //Initailize UART1 for serial communiaction
 sei();   //Enables the global interrupts
}

//Main Function
int main(void)
{
	init_devices();

	lcd_init();
	lcd_cursor(1,1);
	//read();
	while(1);
}

//to read and send the analog sensor values to the rasp pi
void read(void){
	unsigned char val;
	int len;
	val=ADC_Conversion(4);
	//UDR2='a';
	UDR2=val;
	val=ADC_Conversion(5);
	//UDR2='b';
	UDR2=val;
	val=ADC_Conversion(6);
	//UDR2='c';
	UDR2=val;
	val=ADC_Conversion(7);
	//UDR2='d';
	UDR2=val;
	val=ADC_Conversion(8);
	//UDR2='e';
	UDR2=val;
	UDR2='e';
}

void move_robot()
{
	int dir; //dir 0x06-> straight 0x05->left back,right forward 3->left forward,right back 4->both back
	//m1=ceil(output[0][0]*200);
	//m2=ceil(output[1][0]*200);
	int m1d=0;
	int m2d=0;
	m1d=atoi(m1);
	m2d=atoi(m2);
	//lcd_string(m1); 
	//lcd_string(m2); 
	velocity(m1d,m2d);
	if(m1d>0 && m2d>0)
	forward();
	else if(m1d<0 && m2d>0)
	right();
	else if(m1d>0 && m2d<0)
	left();
	else if(m1d<0 && m2d<0)
	back();
	else if(m1d==0 && m2d==0)
	stop();
	/*motion_set(dir);
	_delay_ms(1000);*/
	m1[0]='\0';
	m2[0]='\0';
}


void move(void){
	UDR2='c';
	
	double m1d=atof(m1);
	double m2d=atof(m2);
	
	if(m1d==120.5){
	PORTA=0x06;  //forward
	_delay_ms(1000);}
	
	if(m2d==100.5)
	PORTA=0x00;
}
