/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Samer Rafidi, 400333524
            Last Update: April 5th, 2022
						

						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file

*/
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"




#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

void PortL_Init(void){                //From MS1
    //Use PortM pins for output
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};    // allow time for clock to stabilize
    GPIO_PORTL_DIR_R &= 0x04;                                        // make PN0 out (PN0 built-in LED1)
    GPIO_PORTL_DIR_R |= 0x04; 
  GPIO_PORTL_AFSEL_R &= ~0x07;                                     // disable alt funct on PN0
  GPIO_PORTL_DEN_R |= 0x07;                                        // enable digital I/O on PN0
                                                                                                    // configure PN1 as GPIO
  GPIO_PORTL_AMSEL_R &= ~0x07;                                     // disable analog functionality on PN0        
    return;
}

void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;                    // activate clock for Port J
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};    // allow time for clock to stabilize
    GPIO_PORTJ_DIR_R &= ~0x02;                                            // make PJ1 in 
    GPIO_PORTJ_DEN_R |= 0x02;                                             // enable digital I/O on PJ1
    GPIO_PORTJ_PCTL_R &= ~0x000000F0;                                     //  configure PJ1 as GPIO 
    GPIO_PORTJ_AMSEL_R &= ~0x02;                                            //  disable analog functionality on PJ1        
    GPIO_PORTJ_PUR_R |= 0x02;                                                    //    enable weak pull up resistor
}

void PortF_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; //activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){}; //allow time for clock to stabilize
	GPIO_PORTF_DIR_R=0b00010000; //Make PF4 output, to turn on LED's
	GPIO_PORTF_DEN_R=0b00010000; // Enable PF4
	return;
}

void PortM_Init(void){				
	//Use PortM pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0xFF;        								// make PM0 out (PM0 built-in LED1)
  GPIO_PORTM_AFSEL_R &= ~0xFF;     								// disable alt funct on PN0
  GPIO_PORTM_DEN_R |= 0xFF;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
  //GPIO_PORTM_PCTL_R = (GPIO_PORTM_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTM_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0		
	return;
}

void spin(int direction){
    for(int i=0; i<8; i++){
			if(direction == 1) { // Direction = CCW
				GPIO_PORTM_DATA_R = 0b00001100;
        SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0b00000110;
        SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0b00000011;
        SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0b00001001;
        SysTick_Wait10ms(1);
      }
      else if(direction == 0) { // Direction = CW
        GPIO_PORTM_DATA_R = 0b00001001;
        SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0b00000011;
        SysTick_Wait10ms(1);
				GPIO_PORTM_DATA_R = 0b00000110;
        SysTick_Wait10ms(1);
				GPIO_PORTM_DATA_R = 0b00001100;
        SysTick_Wait10ms(1);
      }
      else {
        GPIO_PORTH_DATA_R = 0b00000000; 
        SysTick_Wait10ms(1);
			}        
    }
        GPIO_PORTM_DATA_R = 0b00000000; 
}
//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t sensorState = 0;
  uint16_t wordData;
  uint16_t Distance;
  uint8_t dataReady;

	//initialize
	PortJ_Init();
	PortM_Init();
	PortF_Init();
	PortL_Init();
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	
	// hello world!
	// Prints some text in the output box when the code is being ran
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX4 Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
	// Gets the ToF model ID
	status = VL53L1X_GetSensorId(dev, &wordData);
	//Prints the model ID
	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// Booting ToF chip
	// Goes through a loop that keeps running until the system is booted (1)
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState); // Checks if the sensor is booted (Good to check that the sensor finishes booting before it access I2C)
		SysTick_Wait10ms(10); // 10ms delay between each iteration
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); // clear interrupt has to be called to enable next interrupt
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev); // SensorInit is called to initialize the sensor (default = 10 Hz)
	Status_Check("SensorInit", status); // CHecks the status of the sensor and handles any errors

	// 1000/16 mill * 10^6
	  /*for(int i = 0; i<24000000; i++){
    GPIO_PORTL_DATA_R ^= 0b00000100;
        SysTick_Wait(500);
   }
		*/
	while(1){
		status = VL53L1X_StartRanging(dev);            //This function has to be called to enable the ranging
		if((GPIO_PORTJ_DATA_R&0b00000010) == 0){
			int j = 64;
			Restart:
				for(int i = 0; i < j; i++) {  // Get the Distance Measures 64 times because j = 64
					int degrees = 5.625*i; // 360 degrees
					while (dataReady == 0){  //wait until the ToF sensor's data is ready
						status = VL53L1X_CheckForDataReady(dev, &dataReady); //checks if the ToF sensors data is ready to be read
						FlashLED3(1);
						VL53L1_WaitMs(dev, 5);
					}
					dataReady = 0;
					
					//read the data values from ToF sensor
					status = VL53L1X_GetDistance(dev, &Distance);	//The Measured Distance value
					FlashLED4(1);
					status = VL53L1X_ClearInterrupt(dev); 		//clear interrupt has to be called to enable next interrupt
					double distance = Distance;			// Create a new variable to store decimals																															
					sprintf(printf_buffer,"%f\r\n", distance/1000);            	// print the resulted readings to UART
					UART_printf(printf_buffer);   // Print the resulting reads to UART
					spin(0);
					SysTick_Wait10ms(1);
				}
		for(int i = 0; i<64;i++){
			spin(1);
		}
		VL53L1X_StopRanging(dev);
		}
	}
}

