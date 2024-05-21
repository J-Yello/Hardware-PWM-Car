// PWMtest.c
// Runs on TM4C123
// Use PWM0/PB6 and PWM1/PB7 to generate pulse-width modulated outputs.
// Min He
// September 7, 2021

// This is an example program to show how to use hardware PWM on TM4C123.
#include <stdint.h>
#include "Initialization.h"
#include "tm4c123gh6pm.h"
#include "ADCSWTrigger.h"

// Function Prototypes (external functions from startup.s)
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // Go to low power mode while waiting for the next interrupt

#define PERIOD 20000

// Duty cycle of PWM to stop the motor 
#define STOP 1

int LW_Speed[5] = {STOP, 0.30 * PERIOD, 0.55 * PERIOD, 0.8 * PERIOD, 0.98 * PERIOD};
int RW_Speed[5] = {STOP, 0.30 * PERIOD, 0.60 * PERIOD, 0.8 * PERIOD, 0.98 * PERIOD};

// Direction the car goes in 
#define BACKWARD   0x0A  //1010 
#define FORWARD  0x05   //0101 
#define NO_MOVE  0x0

// Color of on-board LEDs
#define RED 0x02 // Stoped/ No motion 
#define BLUE 0x04 // Backward Directions 
#define GREEN 0x08 // Forward Direction 
#define YELLOW 0xA // Iniital in yellow 2s 
#define PURPLE 0x06// Reached end of track 
#define WHITE 0xE// Speed mode 
#define OFF   0x0// midddle

// Data pins PF1-3 for on-board LEDs 
#define LED       (*((volatile unsigned long *)0x40025038))
// Data pins PB0-3 for the direction the car will go on 
#define CONTROL 	(*((volatile unsigned long *)0x4000503C))
//used to debounce buttons 
#define debouncing_number 727240*2 

// Constant definitions
// constants used in euqarion y=A+B/x
#define A  -0.96771004
#define B  41177.73274
// determines how many samples are taken 
#define NUM_SAMPLES 3

// variable to indicate time to do the ADC sampling
unsigned char sample = 0; 

//volatile unsigned long ADCvalue;
//Stores the values the sensors will used to estimate distance 
const int distance2[] = {3688,	2582,	1873,	1572,	1305,	1150,	1018,	929, 790,	748, 690,	641, 556 }; 
const double measurement2[] = {10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70};
// variable to indicate the obstacle is too close to the right sensor
unsigned char too_close_right = 0; 
// variable to indicate the obstacle is too close to the left sensor
unsigned char too_close_left = 0; 
// distance estimation for tabel lookup and equation calculation.
unsigned char tabledist, eqdist; 
// Functions that will calculate the distance 
unsigned char eq_calcution(unsigned int ADC_Value);
unsigned char tb_estimation(unsigned int ADC_Value);

//Use a median approace to approimate distance 
void ReadADCFIRFilter(unsigned long *ain2, unsigned long *ain9, unsigned long *ain8);
// combines the values from the two functions and avervgaes them together 
int combine_avg = 0; 
// Keep track of what state we are in 
char car_go = 0;
// Used to generate 2second delay
int time = 0;

unsigned char count=0;

int combine_avg_left = 0;
int combine_avg_right = 0;
int combine_avg_front = 0; 
int tabledist_left = 0;
int eqdist_left = 0;
int tabledist_right = 0;  
int eqdist_right = 0; 
int diff = 0;
void Delay(void);

int main(void){
	
	unsigned long left_measuremnet, right_measurement, front_measurment; 
	 
	DisableInterrupts();
	ADC0_InitSWTriggerSeq3_Ch1();   // ADC initialization PE2/AIN1
	ADC_Init298();
  PWM0A_Init(PERIOD );         // initialize PWM0, PB6
  PWM0B_Init(PERIOD);         // initialize PWM0, PB7
	Control_Int(); 							// initialize PB0-3 as output pins 
	PortF_LEDInit();            // Initialize PF1-3 for onboard LEDs 
	Switch_Init();              // Initialize PF0 & PF4 to be switches 
	SysTick_Init(); 						// Controls the sampling rate of ADC and changes Duty cycle of motors 
	EnableInterrupts();
	
	// Starts the LED on yellow for 2 seconds with no movement 
	LED = YELLOW;
	// waits 2 seconds 
	while(time < 400){} 
	LED = OFF; 
	time = 0; 
	
		
	// Default car starts in forward mode 
	CONTROL = FORWARD;
	
	// Default Car is not moving 
	L_Wheel_Duty(LW_Speed[0]);	
	R_Wheel_Duty(RW_Speed[0]);
		
	while(1){
		
		// Software filter: average for 50 samples
		
		while (count<NUM_SAMPLES) {
			while (!sample){} // sample one value every 1/20=0.05 second 
			sample = 0;
			//ADCvalue += ADC0_InSeq3();	
			ReadADCFIRFilter(&left_measuremnet, &right_measurement, &front_measurment);				
			count++;
	  }
		
		// Saves the distance for the left and right sensor
		tabledist_left = tb_estimation(left_measuremnet);
		eqdist_left = eq_calcution(left_measuremnet);
		tabledist_right = tb_estimation(right_measurement);
		eqdist_right = eq_calcution(right_measurement); 
		combine_avg_left = (tabledist_left + eqdist_left)  / 2; 
		combine_avg_right = ( tabledist_right + eqdist_right) / 2; 
		// Save the distance for the front sensor 
		int tabledist_front = tb_estimation(front_measurment);
		int eqdist_front = eq_calcution(front_measurment);
		combine_avg_front = ( tabledist_front + eqdist_front) / 2; 
		//calculates the differencae between the two 
		diff = combine_avg_left - combine_avg_right; 
		
		//Speed setting mode 
		if (LED == WHITE){
			while(LED == WHITE){
				float a = 90; 
				float divide = a / 2771; 
				float pot_speed = ADC0_InSeq3();
				pot_speed = pot_speed * divide / 100;
				int pot_duty = pot_speed * PERIOD;
				L_Wheel_Duty(pot_duty);	
				R_Wheel_Duty(pot_duty);
			}
		}
		else if (combine_avg_left < 15 || combine_avg_right < 15){// | combine_avg_front < 13){
			LED = RED;
			too_close_right = too_close_left = 0;
		 	// 1 second wait
			while(time < 200){}
			L_Wheel_Duty(LW_Speed[0]);
			R_Wheel_Duty(RW_Speed[0]);
			time = 0; 
			LED = OFF; 
			CONTROL = BACKWARD; 
		}
		else if (CONTROL == BACKWARD){
			while(time < 50){}
			time = 0;
			CONTROL = FORWARD; 
		}
		else if (combine_avg_left > 45 && combine_avg_right > 45){
			too_close_right = too_close_left = 0; 
			if (combine_avg_front > 50){
				LED = PURPLE;
				L_Wheel_Duty(LW_Speed[0]);
				R_Wheel_Duty(RW_Speed[0]);
				
			}
			 
		}
		// Detects if too close to the rightside 
		else if (diff > 6){
			LED = BLUE;
			too_close_right = 1; 
		}
		// Detects if too close to the leftside 
		else if( diff < -6){
			LED = GREEN;
			too_close_left = 1; 
		}
		else{
			LED = OFF; 
			too_close_right = too_close_left = 0; 
			L_Wheel_Duty(LW_Speed[1]);	
			R_Wheel_Duty(RW_Speed[1]);
		}
		
		count = 0;
	}	
}

void SysTick_Handler(void){
	//controls when to sample 
	sample = 1;
	time++; 
	// too close to right side push the car left 
	if (too_close_right) { 
		R_Wheel_Duty(RW_Speed[2]); 
		L_Wheel_Duty(RW_Speed[0]);
	}
	// too close to the left side push the car right 
	else if (too_close_left){
		L_Wheel_Duty(LW_Speed[2]);
		R_Wheel_Duty(RW_Speed[0]);
	}
	// if car is too close and set to move backwards go back  
	else if (CONTROL == BACKWARD && (LED == RED)){
		L_Wheel_Duty(LW_Speed[1]);	
		R_Wheel_Duty(RW_Speed[1]);
	}

}
// This is the function that calculate distance based on ADC value passed in
unsigned char eq_calcution(unsigned int ADC_Value){
	unsigned char dist=0;
	//dist = A + (B / ADC_Value);
	dist = A + B / ADC_Value; 
	return dist;
}

unsigned char tb_estimation(unsigned int ADC_Value){
	unsigned char dist=0;
	
	int run = 1;
	int count = 0;
	while(run == 1){
		if( distance2[count] > ADC_Value ){
			count++;
		}
		else{
			run = 0; 
		}			
	}
	if (count >= 1){
		float slope = ( measurement2[count - 1] - measurement2[count] ) / ( distance2[count-1] - distance2[count] ); 
		float y_intercept = -slope * distance2[count-1] + measurement2[count-1]; 
		dist = slope * ADC_Value + y_intercept; 
	}
	else{
		dist = measurement2[count]; 
	}
	
	return dist;
}

void GPIOPortF_Handler(void) {
	for (int time = 0; time < debouncing_number; time = time + 1){} //deboucning 
	 
	if (GPIO_PORTF_RIS_R & 0x10) { // switch 1 
		GPIO_PORTF_ICR_R = 0x10; // ackniwkege flag 0 
		car_go = car_go ^ 1; //inverts the bit 
		if (car_go){
			// Disable Systick Timer 
			NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;	
			CONTROL = NO_MOVE;
		}
		else{
			// Reenable SysTick Timmer 
			NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; 
			CONTROL = FORWARD; 
		}
	}
	
	// Controls the direction the car moves in 
	else if (GPIO_PORTF_RIS_R & 0x01){ //swtich 2 
		GPIO_PORTF_ICR_R = 0x01; // acknowledge flag 4 
		
		//Checks which direction the motors are current spining change accordingly 
		if( LED != WHITE){
			LED = WHITE; 
		}
		else{
			LED = YELLOW;
		}	
	}	
}

void ReadADCFIRFilter(unsigned long *ain2, unsigned long *ain9, unsigned long *ain8){
  //                   x(n-1)
  static unsigned long ain2previous=0; // after the first call, the value changed to 12
  static unsigned long ain9previous=0;
  static unsigned long ain8previous=0;
  // save some memory; these do not need to be 'static'
  //            x(n)
  unsigned long ain2newest;
  unsigned long ain9newest;
  unsigned long ain8newest;
  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = (ain2newest + ain2previous)/2;
  *ain9 = (ain9newest + ain9previous)/2;
  *ain8 = (ain8newest + ain8previous)/2;
  ain2previous = ain2newest; ain9previous = ain9newest; ain8previous = ain8newest;

}
