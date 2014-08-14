
//**********************************************************
////////////////////////////////////////////////////////////
//    AKM Sabbir
//    
//             --  Embedded program for
//                 motor control, encoder input, 
//                 Sensor input (ADC), and UART communication
//    			   FRTOS implimented	
////////////////////////////////////////////////////////////
//**********************************************************



//INCLUDE FILES FOR AVR
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>


//  FreeRTOS includes  
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
#include "FRTOS/FreeRTOS.h"
#include "FRTOS/task.h"
#include "FRTOS/semphr.h"
#include "FRTOS/queue.h"


//  Define the task priorities to be one higher than the IDLE task 
//////////////////////////////////////////////////////////////////
#define mainCHECK_TASK_PRIORITY			( tskIDLE_PRIORITY + 1 )

//  Task Declarations  
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
static void vMain( void *pvParameters );
static void vOdometry( void *pvParameters );
static void vDrive( void *pvParameters );


// Definitions
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

#define one_ms         62
#define one_five_ms    94
#define two_ms         125
#define one_eight_ms   103         // was 113 // 106 // 102
#define one_ninteen_ms 87          // was 78  //84 // 88
#define twenty_ms      1240

// Semephore Decleration
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

xSemaphoreHandle AdcChannels;


// Global Variables
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

volatile int stateRH = 0;          // rh encoder interrupt state flag
volatile int stateLH = 0;          // lh encoder interrupt state flag
volatile int encodercountRH = 0;   // rotary encoder pulse count RH side
volatile int encodercountLH = 0;   // rotary encoder pulse count LH side
volatile float seconds = 0;                 // keeps track of time
unsigned char sreg;                // stores status reg
unsigned int ADCchan[5] = {0};     // used for ADC conversions
volatile unsigned int IRnum = 0;   // used for ADC conversion from IR sensors
volatile unsigned int CUR_STATE = 0;  // used for movement state machine
volatile char temp_buffer;         // char buffers for data
char roll_buffer[20];              // used for roll pitch and yaw from imu
char pitch_buffer[20];
char yaw_buffer[20];
int ind = 0;
volatile float W_LH = 0;
volatile float W_RH = 0;
volatile float revRH = 0;                     // wheel revolutions RH side
volatile float revLH = 0;                     // wheel revolutions LH side
volatile float pastRevRH = 0;
volatile float pastRevLH = 0;
float temptimeRH = 0;                // keeps track of old time
float temptimeLH = 0;
float currtime = 0;                // keeps track of current time count
volatile float distRH = 0;                  // .398982 meters per revolution
volatile float distLH = 0;                  // .398982 meters per revolution 
volatile float m_per_sRH = 0;               // meters per second
volatile float m_per_sLH = 0;               // meters per second            
float time;                        // time in ms
char* valRH_dis;
char* valRH_Vel;
char* valLH_dis;
char* valLH_Vel;
char* val;
float t;
float x = 0; 
float y = 0;
float theta = 0;
float del_x = 0;
float del_y = 0;
float del_theta = 0;

// Prototypes
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
int readadchan( char n);
void Init();
//void send_message( char *s);
char * float_to_string( float fnum); 
char * IRtable ( int adcV);
void get_string_IMU( void);
char get_char( void);
void put_char( unsigned char chartr);
void put_string(char* string);
void drive( void);
void velocity( void);
void Odom(float W_LH, float W_RH, float time, float theta); 
void turnLeft( void);
void turnRight( void);
void foward( void);
void halt( void);
xSemaphoreHandle ADCchannels;


struct queue{
   char string[100];
   int length;
   int start;
  }Q;




///////////////////////////////////////////////////////////
//*******************************************************//
///////////////////////////////////////////////////////////    


 
int main (void)
{

 
 

//CREATE TASKS
////////////////////////////////////////////////////
////////////////////////////////////////////////////


	ADCchannels = xSemaphoreCreateMutex();
	

	// Creates RTOS tasks for odometry calculation, drive routine, and main routine
	// and sets priority levels
	xTaskCreate( vOdometry, ( signed char * ) "Odometry", 150, NULL, mainCHECK_TASK_PRIORITY + 1, NULL );
	xTaskCreate( vDrive, ( signed char * ) "Drive", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY + 2, NULL );
    xTaskCreate( vMain,  ( signed char * ) "Main", 400, NULL, mainCHECK_TASK_PRIORITY, NULL );

 
	vTaskStartScheduler();				// starts the scheduler
	for(;;);							// loops forever

	return 0;
}




//**************************************************
// This is a funcion for all of the setup and config
//
void Init()
{
    // Init Ports///////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    
    
    DDRA = 0x00;            // set port A as input
    PORTA = 0x00;          // pull it low
    
    
    DDRB = 0xFF;            // sets port B for output
    PORTB = 0xFF;            // PWM output
    
     
     
    DDRE = 0x3F;            
    PORTE = 0xFF;  
    
    DDRF = 0x00;
  
    DDRG = 0xFF;            // set portG as output
    PORTG = 0x00;           // set port G low
                            // Setup the timer tmr0
                            
                            
    sei();                  // enable the global int flag 
    
   // Init timer interrupt////////////////////////////////////// 
   /////////////////////////////////////////////////////////////
   
     
   // Timer 3 16 bit timer
   TCCR3A = 0b10101010;      // Set OC0A/B/C on Compare Match when up-counting. 
							 // Fast PWM with ICR3 => top, update of OCR3A on top, TOV3 flag set on bottom
   TCCR3B = 0b00011100;      // Prescale by 256 so 16MHz / 256 is 16 us tick 
                             // 256 * 16us = 4.096ms   
   
                             // disable interrupts
   TCNT3 = 0;                // Initialize TCNT3 to 0 
   
   
                              // one_ms         62
                              // one_five_ms    94
                              // two_ms         125
                              // one_eight_ms   106
                              // one_ninteen_ms 85
                              
   ICR3 = 1250;               // period of ~20ms
   OCR3A = one_five_ms;       // pwm pin 11   initialized to deadband
   OCR3B = one_five_ms;       // pwm pin 12   initialized to deadband
   OCR3C = twenty_ms;         // pwm pin 13 and twenty ms added to seconds
   
                              
   TIMSK3 = 0x0E;             // COMPARE A MATCH
   
   sei();                     // enable global interrupt
   
   // Init Extern interrupts////////////////////////////////////////
   /////////////////////////////////////////////////////////////////
   
   
   EIMSK |= 0xC0;              // Enable external pin ints 6 and 7   (on pin PE6 and PE7)
   
   EICRB = 0xF0;               // Enable ext enerupts on rising edge (3), fall ->2,
                               // any edge ->1, low level ->0 
   
   
   // Init USART/////////////////////////////////////////////////////
   //////////////////////////////////////////////////////////////////
   
   
   UCSR0A=0x00;
   UCSR0B=0x58;           // enable recieve and transmit interrupt and enable transmitter
   UCSR0C=0x06;           // asenchronious mode 8 bit word frame 
   UBRR0H=0x00;              
   UBRR0L=0x1A;           //1A for 38400 //10 for 57600    // set baudrate of 38400
   
   UCSR1A=0x00;
   UCSR1B=0x58;           // enable recieve and transmit interrupt and enable transmitter
   UCSR1C=0x06;           // asynchronious mode 8 bit word frame 
   UBRR1H=0x00;              
   UBRR1L=0x68;           // 68 for 9600 // set Bd rate low for better transmission
   

   // Init ADC//////////////////////////////////////////////////////
   /////////////////////////////////////////////////////////////////
   
   
   ADMUX = 0x40;    // start with channel 0 and use AVCC
   ADCSRA |= 0x8F;  // prescale at 120 ~= 200KHz
   //ADCSRB |= 0x08;
 
  
} // end init()



//****************************************************    
// TMR1 compA Interrupt
//
ISR( TIMER3_COMPA_vect)
{
    //PORTB ^= 0X80;     // toggle the PORTB pin 7
}


//*****************************************************
// TMR1 CompB Interrupt
//
ISR( TIMER3_COMPB_vect)
{
    //PORTB ^= 0x80;   // toggle PORTB pin 7  
}




//******************************************************
// ISR for the 20 ms counter
//
ISR( TIMER3_COMPC_vect)
{

}


//******************************************************
// external interrupts for encoders
//******************************************************


//******************************************************
// external interrupt 5 on pin 3   
//
ISR( INT7_vect)
{
  
	unsigned char Data = 0b00000010;

	encodercountRH ++;


	revRH = .011 * encodercountRH;

	//distRH = revRH * 0.399;                // distance =  1 revolution * .399 meters distRH is total distance traveled



	// IF STATE RH IS 0 THEN REVERSE
	Data &= PINA;

	if(Data)
		stateRH = 1;
	else
		stateRH = 0;

  
  
}


//*******************************************************
// external interrupt 4 on pin 2
//
ISR( INT6_vect)
{
	unsigned char Data = 0b00000001;

	encodercountLH ++;


	revLH = .011 * encodercountLH;

				//1 revolution is .399 meters 



	// IF STATE LH IS 0 THEN REVERSE
	Data &= PINA;

	if(Data)
		stateLH = 1;
	else
		stateLH = 0;
  
  
  
}




//**************************************************
//  ISR for usart0 transmit int to comp
//
ISR( USART0_TX_vect)
{
 //   if( Q.start != Q.length)
   // UDR0=Q.string[ Q.start++];
}


/*
 void send_message(char *s)
  {
      
    Q.start=0;
    Q.length=0;
    // Q.str[Q.lenght++]=0x0d;
    //Q.str[Q.lenght++]=0x0a;
    while(*s)
       Q.string[ Q.length++] = *s++;
       UDR0 = Q.string[ Q.start++];
      
    }
*/


//**************************************************
// This is the ISR to read the ADC 
//
ISR(ADC_vect)
{
  
  //ADCchan[IRnum] = ADCW;
   
	switch (IRnum)
	{
	case 0:
	  
		ADMUX = 0x40;
		ADCchan[0] = ADC;
		IRnum++;
		break;

	case 1: 
	  
		ADMUX = 0x41;
		ADCchan[1] = ADC;
		IRnum++;
		break;
	  
	case 2:

		ADMUX = 0x42;
		ADCchan[2] = ADC;
		IRnum++;
		break;

	case 3:
	  
		ADMUX = 0x43;
		ADCchan[3] = ADC;
		IRnum++;
		break;
	  
	case 4:

		ADMUX = 0x44;
		ADCchan[4] = ADC;
		IRnum = 0;
		break;

	default: break;
	}
  
	ADCSRA |= 0x40;
  
   
}




//*********************************************************
// This is a funciton to read a channel on the adc
// without the use of an interrupt, its currently 
// unused
int readadchan(char n)
{
//read ch n of internal 10 bit a/d

	ADMUX = n;
	ADMUX |= 0x40;
	ADCSRA |= 0x44;

	while((ADCSRA & 0x40) != 0); //wait for conv complete
	//while((ADCSRA & ADIF) ==0); 

	return ADC;
}


//******************************************************
// This is a funciton to convert a float to a string for 
// printing via serial transmission
char * float_to_string(float fnum)
{
	char temp[10];
	int intfrac;
	float frac = fnum - floor(fnum);    // pull off dec and store in c
	fnum = fnum - frac;                 // sepeates int part (a) from frac part (c)
	frac = frac * 1000;              // make c into an int
	intfrac = (int)frac;               // and store it in b
	int i = 0;                 // start an index (i) at 0
	while( i < 3)              // while loop to covert floats to chars
	{

		temp[i++] = ((char)(intfrac%10)) + 48;

		intfrac = intfrac / 10;
	}
  
	temp[i++] = '.';
	if( fnum == 0)
		temp[ i++]= '0';
	while( fnum)
	{
		temp[i++] = (((char)fmod( fnum, 10)) + 48);
		fnum = floor(( fnum / 10));
	}

	temp[i] = '\0';

  
  
	return strrev(temp);
}
    
 
    
//************************************************    
// This is a function to closely approximate the 
// IR sensor distace from the ADC reading
char * IRtable ( int adcV)
{
  
  
  // ADCval = 6941.6 * CM ^ -0.867 
  // cm = 25441*(adcV)^(-1.143)  characterized dist
  
  if(adcV > 540)
  return " 15 cm ";
  else if(adcV <= 540 && adcV > 544) 
  return " 20 cm ";
  else if(adcV <= 544 && adcV > 495) 
  return " 25 cm ";
  else if(adcV <= 495 && adcV > 444) 
  return " 30 cm ";
  else if(adcV <= 444 && adcV > 392) 
  return " 35 cm ";
  else if(adcV <= 392 && adcV > 340) 
  return " 40 cm ";
  else if(adcV <= 340 && adcV > 294) 
  return " 45 cm ";
  else if(adcV <= 294 && adcV > 263) 
  return " 50 cm ";
  else if(adcV <= 263 && adcV > 240) 
  return "55 cm ";
  else if(adcV <= 240 && adcV > 222) 
  return " 60 cm ";
  else if(adcV <= 222 && adcV > 203) 
  return " 65 cm ";
  else if(adcV <= 203 && adcV > 190) 
  return " 70 cm ";
  else if(adcV <= 190 && adcV > 175) 
  return " 75 cm ";
  else if(adcV <= 175 && adcV > 168) 
  return " 80 cm ";
  else if(adcV <= 168 && adcV >= 160) 
  return " 85 cm ";
  else if(adcV <= 160 && adcV >= 155) 
  return " 90 cm ";
  else if(adcV <= 155 && adcV >= 140) 
  return " 95 cm ";
  else if(adcV <= 140 && adcV >= 134) 
  return " 100 cm ";
  else if(adcV <= 134 && adcV >= 130) 
  return " 105 cm ";
  else if(adcV <= 130 && adcV >= 121) 
  return " 110 cm ";
  else if(adcV <= 121 && adcV >= 116) 
  return " 115 cm ";
  else if(adcV <= 116 && adcV >= 113) 
  return " 120 cm ";
  else if(adcV <= 113 && adcV >= 109) 
  return " 125 cm ";
  else if(adcV <= 109 && adcV >= 103) 
  return " 130 cm ";
  else if(adcV <= 103 && adcV >= 99) 
  return " 135 cm ";
  else if(adcV <= 99 && adcV >= 95) 
  return " 140 cm ";
  else if(adcV <= 95 && adcV >= 91) 
  return " 145 cm ";
  else if(adcV <= 91 && adcV >= 0.0) 
  return " 150 cm ";
  
  
}

//*********************************************
// This is the function used to gather data 
// from the IMU, it uses USART1 for a recieving
// not currently using imu 
void get_string_IMU(void)
{

    
    int i, j = 0;
  
    while(get_char() != '$');   // wait for the $ char
     
   
   
        // roll state***************************************
//        put_string("R: ");
        do
        {
			roll_buffer[i] = get_char();
			i++;
        }while((roll_buffer[i-1] != ',') && (roll_buffer[i-1] != '>'));
			roll_buffer[i] = '\0';
          
          
//      for(j = 0; j < i-1; j++)
//      {       
//      	put_char(roll_buffer[j]);
//      }
        i = 0;
          
       
       
        // pitch state****************************************
//        put_string("    P: ");
        do
        {  
          
			pitch_buffer[i] = get_char();
			i++;
        }while((pitch_buffer[i-1] != ',') && (pitch_buffer[i-1] != '>'));
			pitch_buffer[i] = '\0';
          
          
//      for(j = 0; j < i-1; j++)
//      {     
//         	put_char(pitch_buffer[j]);
//      }
		i = 0;
       
       
       
        // yaw state*********************************************
//        put_string("    Y: ");
        do
        {  
          
			yaw_buffer[i] = get_char();
			i++;
        }while((yaw_buffer[i-1] != ',') && (yaw_buffer[i-1] != '>'));
		
		yaw_buffer[i] = '\0';
          
          
//      for(j = 0; j < i-1; j++)
//      {       
//      	put_char(yaw_buffer[j]);
//      }
		i = 0;
//      put_string(" \n");

}    
    
    
    

//**********************************************
//  Funciton to get characters from UART1 
//  
char get_char( void)
{
	while ( !( UCSR1A & ( 1<<RXC1)));        // Wait for char 

	return UDR1;                        //Get and return char
}


//***********************************************
// function to put individual characters in the usart0 buffer
//
void put_char( unsigned char chartr)
{

	while ( !( UCSR0A & (1<<UDRE0)));   // wait for buffer to be empty


	UDR0 = chartr;        // put data into usart0 buffer
}


//**************************************************
// Function to disperse string chars into usart0 buffer
//
void put_string( char* string)
{
	int string_length;
	int count;

	string_length = strlen( string);   // find string length

	for( count = 0; count < string_length; count++)
	{
		put_char(string[count]);            // put each char from string
	}
  
}// end put_string


//***************************************************
// Function to compute odometry data
//

void Odom(float W_LH, float W_RH, float time, float theta)
{

	// difference equations for odometry
	del_x = (31.75)*(W_LH + W_RH)*cos(theta);
	del_y = (31.75)*(W_LH + W_RH)*sin(theta);
	del_theta = 0.219*(- W_LH + W_RH);


}


void turnLeft( void)
{
    sreg = SREG;  
    cli();
    
    OCR3A = one_eight_ms;           // pwm pin 6 set slow
    OCR3B = one_eight_ms;			// pwm pin 2 set slow
    
    sei();
    SREG = sreg;
    
}


void turnRight( void)
{
  
    sreg = SREG;  
    cli();
    
    OCR3A = one_ninteen_ms;           // pwm pin 6 set slow
    OCR3B = one_ninteen_ms;			  // pwm pin 2 set slow
       
    sei();
    SREG = sreg;
}

void foward( void)
{
  
    sreg = SREG;  
    cli();
  
    OCR3A = one_eight_ms;            // pwm pin 6   set to slow
    OCR3B = one_ninteen_ms;         // pwm pin 2   set to slow
    
    sei();
    SREG = sreg;
}

void halt( void)
{
  
    sreg = SREG;  
    cli();
    
    OCR3A = one_five_ms;         
    OCR3B = one_five_ms;
    
    sei();
    SREG = sreg;   
    
}









//RTOS TASKS 
///////////////////////////////////////////////
///////////////////////////////////////////////



// Main task pri = 1
///////////////////////////////////////////////////////////
static void vMain( void *pvParameters)
{




 //******************************************************
  ///// INITALIZE LOCAL VARIABLES ////////////////////////
  //******************************************************
 int three_ft = 0;
 float temp_distRH = 0; 
 float temp_distLH = 0; 
 int target_yaw = 0;
 int delta_yaw = 0;
 char chr_target_yaw[5];
 int turn_done = 1;
 int int_yaw_buffer = 0;
 char* valW_RH; 
 char* valW_LH;
 signed int intx;
 signed int inty;
 signed int inttheta;
 char chrx[6];
 char chry[6];
 char chrtheta[6];
 int k = 0;
 float RTx = 0;
 float LTx = 0;
 float RTy = 0;
 float LTy = 0;
 float RTIR = 0;
 float LTIR = 0;
 float RTxAve = 0;
 float LTxAve = 0;
 char chrRTx[6];
 char chrLTx[6];
 signed int intRTx;
 signed int intLTx;
 signed int intRTy;
 char chrRTy[6];
 signed int intLTy;
 char chrLTy[6];
 int IRval[6];
 int i = 0;
 
   	Init();                            // initialize and setup
  
  	sei();					
  
  	ADCSRA |= 0x40; 				// start an adc conversion
 
 
 

	while( 1)
	{
	  
		
		
		   
		// DISPLAY IMU VALUES AND IR SENSOR VALUES//////////
		//        not in use
		////////////////////////////////////////////////////
		
		
		//get_string_IMU();                   // displays imu values
		
		/*
		put_string(" IR0 dist  = ");        // display IRnum0
		val=IRtable( ADCchan[0]);
		put_string( val);

		 
		put_string(" IR1 dist  = ");        // display IRnum1
		val=IRtable( ADCchan[1]);
		put_string( val);

		
		put_string(" IR2 dist  = ");        // display IRnum2
		val=IRtable( ADCchan[2]);
		put_string( val);

		
		put_string(" IR3 dist  = ");        // display IRnum3
		val=IRtable( ADCchan[3]);
		put_string( val);

		
		put_string(" IR4 dist  = ");        // display IRnum4
		val=IRtable( ADCchan[4]);
		put_string( val);
	   
		   
			   
		put_string(" \n ");
		*/
		
		
		
		
		
		
		//*********** MAP SENSOR DATA *********************************//
	   //////////////////////////////////////////////////////////////////
	 
		// ADCval = 6941.6 * CM ^ -0.867
		// cm = 25441*(adcV)^(-1.143)
		
		// semaphore used to show functionality of semaphore
		xSemaphoreTake( ADCchannels, portMAX_DELAY);
		for(i = 0; i < 5; i++)
		{
			IRval[i] = ADCchan[i];
		}
		xSemaphoreGive( ADCchannels);
		
		if(IRval[0] < 600)			//only look at ir values tha make sense
		{
			RTIR = ADCchan[0];
			RTIR = 254410 * pow(RTIR,(-1.143));
		}
		 
		// caclulate distance x and y components based
		// on local vs global angles (theta)
		RTx = x + (RTIR * sin(theta));
		RTy = y - (RTIR * cos(theta));
		 
		if(ADCchan[1] < 600)
		{
			LTIR = ADCchan[1];
			LTIR = 254410 * pow(LTIR,(-1.143));
		}
		 
		LTx = x - (LTIR * sin(theta));
		LTy = y + (LTIR * cos(theta));
		
		
		 
		
		// DISPLAY ODOMETRY DATA ///////////////////////////
		////////////////////////////////////////////////////
		
		
		if(k == 2)  // used to slow data rate 
		{
		  
		// robot path
			put_string("  ");
			intx = (signed int)(x );
			itoa(intx, chrx, 10);
			put_string(chrx);
			
			put_string(",  ");
			inty = (signed int)(y );
			itoa(inty, chry, 10);
			put_string(chry);
			
			put_string(",  ");
		  
				  
			// rt side walls
			intRTx = (signed int)(RTx );
			itoa(intRTx, chrRTx, 10);
			put_string(chrRTx);
			
			put_string(",  ");
			intRTy = (signed int)(RTy );
			itoa(intRTy, chrRTy, 10);
			put_string(chrRTy);
			
			
			// lt side walls  
			put_string(",  ");
			intLTx = (signed int)(LTx );
			itoa(intLTx, chrLTx, 10);
			put_string(chrLTx);
			
			put_string(",  ");
			intLTy = (signed int)(LTy );
			itoa(intLTy, chrLTy, 10);
			put_string(chrLTy);
			
			
				  
			put_string("\n");
			
			RTxAve = 0;
			LTxAve = 0;
			k=0;
		}
		k++;

	} // end while
 

}


// RTOS task to compute odometry pri = 2
///////////////////////////////////////////////////////////////////////////
static void vOdometry( void *pvParameters )
{

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();  //  Get the last tick count before the delay.

	while (1)
    {
		// toggle PORTB pin 7 
    	//PORTB ^= 0x80;
		  seconds = seconds + .02;  // used to add 20 ms to seconds
 
		// compute radian velocity for lh 
	    W_LH = (revLH - pastRevLH);
	    W_LH = (W_LH * 2 * 3.14);
	    W_LH = W_LH / .02; 
	   
	    if(!stateLH)
			W_LH = -1*W_LH;
	   
	   
		// compute radian velocity for rh  
	    W_RH = (revRH - pastRevRH);
	    W_RH = (W_RH * 2 * 3.14);
	    W_RH = W_RH / .02; 
	   
	    if(!stateRH)
			W_RH = -1*W_RH;
 
 
 
  
	// ODOMETRY /////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
     
		Odom( W_LH, W_RH, .02, theta);
	   
		x += .02 * del_x;
		y += .02 * del_y;
		theta += .02 * del_theta;
	  		
		pastRevRH = revRH;
		pastRevLH = revLH;
	    // block task for exactly 20 ms for timing
		vTaskDelayUntil(&xLastWakeTime, 20 / portTICK_RATE_MS ); 
		
	}// end while
}


//RTOS Task to drive robot pri = 3
/////////////////////////////////////////////////////////////////////
static void vDrive( void *pvParameters )
{
	int avoid_F = 0;
	int rt_f = 0;
	int lt_f = 0;
	int tempEncoderLH = 0;
	int tempEncoderRH = 0;
	int tempEncoderLH1 = 0;
	int tempEncoderRH1 = 0;
	int tempEncoderLH2 = 0;
 
	int cont_F = 0;
	
	
	
	while (1)
    {
		
		
		/*
		// DRIVE ROBOT IN 3 FT SQUARES TO TEST MAPPING ///////////////
		//////////////////////////////////////////////////////////////
		
		if(ADCchan[3] >= 400)          // used to stop robot if object closer 30 cm
		{
		  halt();
		} 
		
		else 
		{      
		 
		  foward();
		  
		  
		  
		  if((encodercountLH - tempEncoderLH) >= 270 || (encodercountRH - tempEncoderRH) >= 270)
		  {
			  turnLeft();
			  
			  _delay_ms(2180);
		  
		
			tempEncoderLH = encodercountLH;
			tempEncoderRH = encodercountRH;
		  }
		}
		*/
	   
		
		//// STEER ROBOT DOWN HALLWAY
		////////////////////////////////////////////////////
		
		
		if(!avoid_F)
		{
		  

			if(ADCchan[1] <= ADCchan[0] + 50 && ADCchan[1] >= ADCchan[0] - 50)
			{
				foward();
			}
			else if(ADCchan[2] <= ADCchan[4] + 100 && ADCchan[2] >= ADCchan[4] - 100)
			{
				foward();
			}
			else if(ADCchan[1] > 270 || ADCchan[2] > 200)
			{
				halt();
				_delay_ms(150);
				turnRight();
				_delay_ms(450);
				if(ADCchan[3] < 400)
				{
					foward();
					_delay_ms(200);
				}
			}



			else if( ADCchan[0] > 270|| ADCchan[4] > 200)
			{
				halt();
				_delay_ms(150);
				turnLeft();
				_delay_ms(450);
				if(ADCchan[3] < 400)
				{
					foward();
					_delay_ms(200);
				}
			}

			else
				foward();
		  
		 
		  
		}
		
		// NOTE TRY INREASING THE DELAY TIME ABOVE TO SMOOTH OUT ROBOT MOVEMENT
		// ALSO BELOW MIGHT BE ABLE TO TAKE OFF THE avoid_F IN IF STATEMENT
		// TO ALLOW ROBOT TO STOP IF NEEDED IN AN AVOID STATE

		/* 
		// NEW AVOIDANCE ALGO ////////////////////////////////////////
		////////////////////////////////////////////////////////////// 

		if(rt_f)
		{
		 rt_f = 0;
		 foward();
		 if(ADCchan[3] < 420)
		 _delay_ms(200);
		 if(ADCchan[3] < 420)
		 _delay_ms(200);
		 if(ADCchan[3] < 420)
		 _delay_ms(200);
		 if(ADCchan[3] < 420)
		 _delay_ms(200);
		 turnLeft();
		   _delay_ms(2400);
		}
		else if(lt_f)
		{
		 lt_f = 0;
		 foward();
		 if(ADCchan[3] < 420)
		 _delay_ms(200);
		 if(ADCchan[3] < 420)
		 _delay_ms(200);
		 if(ADCchan[3] < 420)
		 _delay_ms(200);
		 if(ADCchan[3] < 420)
		 _delay_ms(200);
		 turnRight();
		   _delay_ms(2400);
		}
	   
	   
	   
	   
	   
		if(ADCchan[3] >= 420 && (ADCchan[1] >= 420 || ADCchan[2] >= 420))
		{
		   rt_f = 1;
		   turnRight();
		   _delay_ms(2745);
		}
		else if (ADCchan[3] >= 420 && (ADCchan[0] >= 420 || ADCchan[4] >= 420))
		{
		   lt_f = 1;
		   turnLeft();
		   _delay_ms(2745);
		}
		else if  (ADCchan[3] >=420 && ADCchan[0] >= 420 && ADCchan[1] >= 420)
		{
		   turnLeft();
		   _delay_ms(6490);
		}
	   
	   
	  */
	   
	     
	   
	   
	   
	   ////////////////////////////////////////////
	   //// STEER ROBOT DOWN HALLWAY
		////////////////////////////////////////////////////
		/*
		
		if(!avoid_F)
		{
		  
		  
		  if(ADCchan[1] <= ADCchan[0] + 50 && ADCchan[1] >= ADCchan[0] - 50)
		  {
			foward();
		  }
		  else if(ADCchan[2] <= ADCchan[4] + 50 && ADCchan[2] >= ADCchan[4] - 50)
		  {
			foward();
		  }
		  else if(ADCchan[1] > 200 || ADCchan[2] > 140)
		  {
			halt();
			_delay_ms(150);
			turnRight();
			_delay_ms(350);
		  }
		  
		  else if( ADCchan[0] > 200|| ADCchan[3] > 140)
		  {
			halt();
			_delay_ms(150);
			turnLeft();
			_delay_ms(350);
		  }
		  
		  else
		  foward();
		  
		 
		  
		}
		
		   
		// DRIVE ROBOT and AVOID OBJECT //////////////////////////////
		//////////////////////////////////////////////////////////////
		
		if(ADCchan[3] >= 420 || avoid_F)          // used to stop robot if object closer 30 cm
		{
		  
			if(ADCchan[3] >= 420)
			{
				halt();
		  
				_delay_ms(300);
		  
				turnRight();
		  
				_delay_ms(2000);
		  
				halt();
		  
				_delay_ms(300);
				if(!avoid_F)
				{
				tempEncoderLH1 = encodercountLH;
				tempEncoderRH1 = encodercountRH;
				}
		  
				avoid_F = 1;                  // in an avoid state
			}
			
			if(ADCchan[1] < 300)
			{      
				_delay_ms(700);
				tempEncoderLH = encodercountLH;
				tempEncoderRH = encodercountRH;    
				
				turnLeft();
		 
				_delay_ms(2000);  /////////////////
		  
				foward();
				_delay_ms(2000);
				
				
				turnLeft();
				_delay_ms(2000);      ////////////////////
				
				////////////////////////////////////////////////////left off
		  
		  
				tempEncoderLH2 = encodercountLH + (tempEncoderLH - tempEncoderLH1);
		  
		  
			   // _delay_ms(300);
				avoid_F = 0;
				cont_F = 1;
			}
			else if((ADCchan[1] > 300 || ADCchan[2] > 300) && avoid_F)    //ADCchan[2] >= 300 || 
			{
			
				foward();
				
		   
				
			}
		} 
		
		
		   
		else if(cont_F)// CONTENUE WITH NORMAL OPERATION 
		{      
		 
		  
		  if(encodercountLH < tempEncoderLH2)
		  foward();
		  else
		  {
		  turnRight();
		  _delay_ms(2000);
		  foward();
		  cont_F = 0;
		  }
		}
		else
		{
		  foward();
		} 
		  
			 
		
		*/
			
		// block talk for 10 ms
 		vTaskDelay(10);
	}
}
