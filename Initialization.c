// PWM.c
// Runs on TM4C123
// Use PWM0/PB6 and PWM1/PB7 to generate pulse-width modulated outputs.
// Daniel Valvano
// March 28, 2014
// Modified by Min He, September 7, 2021

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
  Program 6.7, section 6.3.2

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
#include <stdint.h>
#include "tm4c123gh6pm.h"

#define SYSDIV2 7 //used to generate 50MHz clock frequency

// period is 16-bit number of PWM clock cycles in one period 
// Output on PB6/M0PWM0
void PWM0A_Init(uint16_t period){
  SYSCTL_RCGCPWM_R |= 0x01;             // 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= 0x02;            // 2) activate port B: 000010
  while((SYSCTL_RCGCGPIO_R&0x02) == 0){};
	GPIO_PORTB_AFSEL_R |= 0x40;           // enable alt funct on PB6: 0100 0000
  GPIO_PORTB_PCTL_R &= ~0x0F000000;     // configure PB6 as PWM0
  GPIO_PORTB_PCTL_R |= 0x04000000;
  GPIO_PORTB_AMSEL_R &= ~0x40;          // disable analog functionality on PB6
  GPIO_PORTB_DEN_R |= 0x40;             // enable digital I/O on PB6
  GPIO_PORTB_DR8R_R |= 0xC0;    // enable 8 mA drive on PB6,7
  SYSCTL_RCC_R = 0x00100000 |           // 3) use PWM divider
    (SYSCTL_RCC_R & (~0x001E0000));   //    configure for /2 divider: PWM clock: 80Mhz/2=40MHz
  PWM0_0_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_0_GENA_R = 0xC8;                 // low on LOAD, high on CMPA down
  // PB6 goes low on LOAD
  // PB6 goes high on CMPA down
  PWM0_0_LOAD_R = period - 1;           // 5) cycles needed to count down to 0
  PWM0_0_CMPA_R = 0;             // 6) count value when output rises
  PWM0_0_CTL_R |= 0x00000001;           // 7) start PWM0
  PWM0_ENABLE_R |= 0x00000001;          // enable PB6/M0PWM0 0100 0000 
}

// change duty cycle of PB6
// duty is number of PWM clock cycles output is high  
void R_Wheel_Duty(uint16_t duty){
  PWM0_0_CMPA_R = duty - 1;             // 6) count value when output rises
}

// period is 16-bit number of PWM clock cycles in one period 
// Output on PB7/M0PWM1
void PWM0B_Init(uint16_t period){
  volatile unsigned long delay;
  SYSCTL_RCGCPWM_R |= 0x01;             // 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= 0x02;            // 2) activate port B
  delay = SYSCTL_RCGCGPIO_R;            // allow time to finish activating
  GPIO_PORTB_AFSEL_R |= 0x80;           // enable alt funct on PB7
  GPIO_PORTB_PCTL_R &= ~0xF0000000;     // configure PB7 as M0PWM1
  GPIO_PORTB_PCTL_R |= 0x40000000;
  GPIO_PORTB_AMSEL_R &= ~0x80;          // disable analog functionality on PB7
  GPIO_PORTB_DEN_R |= 0x80;             // enable digital I/O on PB7
  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // 3) use PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; //    clear PWM divider field
  SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;  //    configure for /2 divider
  PWM0_0_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_0_GENB_R = (PWM_0_GENB_ACTCMPBD_ONE|PWM_0_GENB_ACTLOAD_ZERO); // 0xC08
  // PB7 goes low on LOAD
  // PB7 goes high on CMPB down
  PWM0_0_LOAD_R = period - 1;           // 5) cycles needed to count down to 0: LAB2: PF1->M1PWM5:PWM1_2_
  PWM0_0_CMPB_R = 0;             // 6) count value when output rises: Lab 2:5%2=1->CMPB, GENB
  PWM0_0_CTL_R |= 0x00000001;           // 7) start PWM0: odd->B, even->A
  PWM0_ENABLE_R |= 0x00000002;          // enable PB7/M0PWM1
}

// change duty cycle of PB7
// duty is number of PWM clock cycles output is high  
void L_Wheel_Duty(uint16_t duty){
  PWM0_0_CMPB_R = duty - 1;             // 6) count value when output rises
}

//Initalizes PB0~3 for H-Bridge Control 
void Control_Int(void){
	GPIO_PORTB_DIR_R |= 0x0F; // sets pins PB0-3 to be output 
	GPIO_PORTB_AFSEL_R &= ~0x0F; //disable alt fun for PB0-3 
	GPIO_PORTB_DEN_R |= 0x0F; // Enable digital I/O for PB0-3 
	GPIO_PORTB_PCTL_R &= ~0x0000FFFF; //Configure PB0-3 as GPIO 
	GPIO_PORTB_AMSEL_R &= ~0x0F; //Disable analog fun for PB0-3 
}

// Initialize Port F LEDs
void PortF_LEDInit(void) {
	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;            // 2) activate port F: 000010
  while((SYSCTL_RCGCGPIO_R&SYSCTL_RCGCGPIO_R5) == 0){};

  GPIO_PORTF_AMSEL_R &= ~0x0E;        // disable analog function
  GPIO_PORTF_PCTL_R &= ~0x0000FFF0; 	// GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R |= 0x0E;          	// PF3-PF1 output   
	GPIO_PORTF_AFSEL_R &= ~0x0E;        // no alternate function
  GPIO_PORTF_DEN_R |= 0x0E;          	// enable digital pins PF3-PF1  
}

// Initialize edge trigger interrupt for PF0 and PF4 (falling edge) 
void Switch_Init(void) {  
	if ((SYSCTL_RCGCGPIO_R & SYSCTL_RCGCGPIO_R5) != SYSCTL_RCGCGPIO_R5){
		SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;     	// activate F clock
		while((SYSCTL_RCGCGPIO_R&SYSCTL_RCGCGPIO_R5) == 0){}; // wait for the clock to be ready
	}	
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   	// unlock PortF PF0
	GPIO_PORTF_CR_R |= 0x1F;         		// allow changes to PF4-0 :11111->0x0F 
	GPIO_PORTF_DIR_R &= ~0x11;          // PF0 & PF4 input   
	GPIO_PORTF_AFSEL_R &= ~0x11;        // no alternate function
	GPIO_PORTF_DEN_R |= 0x11;          	// enable digital pins PF0
	GPIO_PORTF_PCTL_R &= ~0x000F000F; 	// GPIO clear bit PCTL
	GPIO_PORTF_AMSEL_R &= ~0x11;        // disable analog function
	GPIO_PORTF_PUR_R |= 0x11;          	// enable pullup resistors on PF0  
	
	GPIO_PORTF_IS_R &= ~0x11; 					//PF0 & PF4 are edge-sensitive 
	GPIO_PORTF_IBE_R &= ~0x11; 					//PF0 & PF4 are not both edge sensative
	GPIO_PORTF_IEV_R &= ~0x11; 					//PF0 & PF4 are falling edge event
	GPIO_PORTF_ICR_R = 0x11; //clear flag 4, 0 
	GPIO_PORTF_IM_R |= 0x11; //arm interrupt on PF4,0
	NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF1FFFFF)| 0x00400000; //PF has priority 2 
	NVIC_EN0_R = 0x40000000;  //enable interrupt 30(PORT F) in NVIC
}

void PLL_Init(void){
		SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2; //enable the use of advance clock control 
		SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2; //bypass PPL while initializing 
		// 2) select the crystal value and oscillator source
		SYSCTL_RCC_R &= ~SYSCTL_RCC_XTAL_M;   // clear XTAL field
		SYSCTL_RCC_R += SYSCTL_RCC_XTAL_16MHZ;// configure for 16 MHz crystal
		SYSCTL_RCC2_R &= ~SYSCTL_RCC2_OSCSRC2_M;// clear oscillator source field
		SYSCTL_RCC2_R += SYSCTL_RCC2_OSCSRC2_MO;// configure for main oscillator source
		// 3) activate PLL by clearing PWRDN
		SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2;
		// 4) set the desired system divider and the system divider least significant bit
		SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400;  // use 400 MHz PLL
		SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~0x1FC00000) // clear system clock divider field
                  + (SYSDIV2<<22);      // configure for 50 MHz clock
		// 5) wait for the PLL to lock by polling PLLLRIS
		while((SYSCTL_RIS_R&SYSCTL_RIS_PLLLRIS)==0){};
		// 6) enable use of PLL by clearing BYPASS
		SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2;
}

// Initialize SysTick timer with interrupt enabled
void SysTick_Init(void){ 
	
	NVIC_ST_CTRL_R = 0x00; //disable the SysTick tmer 
	NVIC_ST_RELOAD_R = 80000 - 1; //value to generates a 0.005s delay
	NVIC_ST_CURRENT_R = 0; //clear the value 
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x1FFFFFFF) | 0xA0000000; //SysTick Timer has priority 5
	NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC + NVIC_ST_CTRL_INTEN + NVIC_ST_CTRL_ENABLE;  
}
