// 2DX4_Knowledge_Thread_3_Session_1
// This program illustrates the use of SysTick in the C language.
// Note the library headers asscoaited are PLL.h and SysTick.h,
// which define functions and variables used in PLL.c and SysTick.c.
// This program uses code directly from your course textbook.

//  Written by Samer Rafidi, 400333524, rafidis, CLK = 16 MHz, LED = PF4
//  March 13, 2022



#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"



//PF4 is output
void PortF_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; //activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){}; //allow time for clock to stabilize
	GPIO_PORTF_DIR_R=0b00010000; //Make PF4 output, to turn on LED's
	GPIO_PORTF_DEN_R=0b00010000; // Enable PF4
	return;
}

//Port L is used for the push button input
void PortL_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10; // activate the clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){}; // allow time for clock to stabilize
	GPIO_PORTL_DIR_R = 0b00000000; // Set as inputs     
	GPIO_PORTL_DEN_R = 0b00000001; // Enable PL0 
	return;
}

//Port M is used for the stepper motor 
void PortM_Init(void){
    //Use PortM pins for output
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};    // allow time for clock to stabilize
    GPIO_PORTM_DIR_R |= 0xFF;                                        // make PN0 out (PN0 built-in LED1)
    GPIO_PORTM_AFSEL_R &= ~0xFF;                                     // disable alt funct on PN0
    GPIO_PORTM_DEN_R |= 0xFF;                                        // enable digital I/O on PN0                                                                                             // configure PN1 as GPIO
    //GPIO_PORTM_PCTL_R = (GPIO_PORTM_PCTL_R&0xFFFFFF0F)+0x00000000;
    GPIO_PORTM_AMSEL_R &= ~0xFF;                                     // disable analog functionality on PN0        
    return;
}


void spinCW(){
    for(int i=0; i<512; i++){  // 2048 rotations but we do 512 because there is 4 steps in every loop. 2048/512
       if(((i+1)%64) == 0){ //Check if angle is a multiple of 45. 512/360 = 1.4222. 1.4222 x 45 = 64
            GPIO_PORTF_DATA_R = 0b00010000; //flash LED at PF4         
        }
			 else {
            GPIO_PORTF_DATA_R = 0b00000000; //turn off LED at PF4
        }
        GPIO_PORTM_DATA_R = 0b00001001;
				SysTick_Wait10ms(1); 
				GPIO_PORTM_DATA_R = 0b00000011;
				SysTick_Wait10ms(1);
				GPIO_PORTM_DATA_R = 0b00000110;
				SysTick_Wait10ms(1);
				GPIO_PORTM_DATA_R = 0b00001100; 
				SysTick_Wait10ms(1); 
    }
    GPIO_PORTM_DATA_R = 0b00000000;
    GPIO_PORTF_DATA_R = 0b00000000;
}

int main(void){
    PLL_Init();                                                   // Default Set System Clock to 16MHz
    SysTick_Init();                                               // Initialize SysTick configuration
    PortM_Init();    
    PortL_Init();
    PortF_Init();
    while(1){
		GPIO_PORTF_DATA_R = 0b00000000;
		if ((GPIO_PORTL_DATA_R&0b00000001) == 0) {
			spinCW();
			}	
	}
}		