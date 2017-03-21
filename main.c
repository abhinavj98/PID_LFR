/*
 * PID_LFR.c
 *
 * Created: 2/5/2017 5:19:43 PM
 * Author : Abhinav
 */ 

/*

Sensor map error
 00001 4
 00011 3
 00111 2 
 00010 1
 00110 1
 00100 0
 01110 0
 01000 -1
 01100 -1
 11100 -2
 11000 -3
 10000 -4
 00000 -5 or 5 
 */

#include <stdio.h>
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>

#define s_left 0  
#define s_midleft 1 
#define s_centre 2  
#define s_midright 3
#define s_right 4
#define port1 PORTD
#define port2 PORTD
#define pin1 PIND4
#define pin2 PIND5

float prop = 0; //WHen car left, right sensors off, take right control positive
float integral =0;
float derivative = 0;
float prevprop = 0;
int co3ntrol=0;
int sl=0;
int sml=0;
int sc=0;
int smr=0;
int sr=0;
double error=0;
int avg_speed = 600; 
int dutycycle=0;
int a;
int control;
struct
{
	float kd;
	float kp;
	float ki;
} gain;

int PID(float req,float current);
double getError();
void PIDinit(float,float,float);	
void PWMinit();
void setMotorSpeed(int);

int main(void)
{
	DDRB =(1<<PIND4) | (1<<PIND5);
	PORTB |= (1<<PINB0) | (1<<PINB2);
	PWMinit();
	PIDinit(20,0,0); 
	while (1) 
    {
		float value = PID(0,getError());			
		setMotorSpeed(value);
	}
}
void PIDinit(float kp,float kd, float ki)
{
	gain.kp=kp;
	gain.kd=kd;
	gain.ki=ki;
}

int PID(float req,float current)
{
	prop = req-current; //WHen car left, right sensors off, take right control positive
	integral += prop;
	derivative = prevprop-prop;
	prevprop = prop;
	
	control = gain.kp*prop + gain.ki*integral + gain.kd*derivative;
	
	if(control>600)
	control = 600;
	if(control<-600)
	control = -600;
	
	return control;
}

double getError()
{
	if(bit_is_clear(PINA,s_right))
		{
			sr=1;
			a=1;
		}
	else
	{
		sr=0;
		a=0;
	}
	if(bit_is_clear(PINA,s_midright))
		{
		smr=1;
		a=0;
		}
	else
		smr=0;
	if(bit_is_clear(PINA,s_centre))
		{
		sc=1;
		a=0;
		}

	else
		sc=0;
	if(bit_is_clear(PINA,s_midleft))
		{
			sml=1;
			a=0;
		}
	else
		sml=0;
	if(bit_is_clear(PINA,s_left))
		{
		sl=1;
		a=2;
		}
	else
	{
		sl=0;
		a=0;
	}
	error=(sr*5 +smr*2 +sc*0 +sml*(-2) +sl*(-5))*(sr+smr+sml+sl);
	if(error==0)
	{
		if(a==1)//go left
		error=-16;
		if(a==2)//go right
		error =16;
	}
	return error;
}
//383

void PWMinit()
{
    /* OCIA == PD5  left
	   OCIB == PD4  right
	   +ve dutycycle left ones on
	 */
	DDRD=(1<<PIND5)|(1<<PIND4);
	TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);
	TCCR1B |= (1<<WGM12) | (1<<WGM13) |(1<<CS11);
	ICR1 = 1000;
}

void setMotorSpeed(int dutycycle)
	{
		
		if(dutycycle > 0)
			{
				  OCR1A = avg_speed-dutycycle;
				  OCR1B = avg_speed+dutycycle;  //Turn left
			}
		if(dutycycle < 0)
			{
				dutycycle=-dutycycle;
				OCR1A = avg_speed+dutycycle;
				OCR1B = avg_speed-dutycycle;  //Turn Right
			}
	    if(dutycycle == 0)
			{
				OCR1A = avg_speed;
				OCR1B = avg_speed;
			}
					
	}
	
	