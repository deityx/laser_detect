
//-----------------------------------------------------------------------------
// Copyright 2006 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
// --------------------
//
// This example code for the C8051F350 takes measurements from input A1N0.0
// using ADC0 then prints the results to a terminal window via the UART to 
// indicate whether the laser is working normally.
// The system is clocked by the external 3.6864MHz oscillator. The completion of
// this conversion in turn triggers an interrupt service routine (ISR). The ISR
// calculates the ADC0 result into the equivalent mV and then prints the value
// to the terminal via printf before starting another conversion.
//
// The analog multiplexer selects A1N0 as the positive ADC0 input.  This
// port is configured as an analog input in the port initialization routine.
// The negative ADC0 input is connected via mux to ground, which provides
// for a single-ended ADC input.
//
//----------
// C8051F350-TB
//
// Terminal output is done via printf, which directs the characters to
// UART0.
//
// F350 Delta-Sigma ADC
// --------------------
// Please see Silicon Labs Applicaton Note AN217 for more information
// on the C8051F35x Delta-Sigma ADC.  AN217 can be found on the Applications
// webpage by going to the Silicon Labs Microcontrollers homepage
// (www.silabs.com -> select Microcontrollers under "Products at the top) and
// clicking the gray link on the left.
//
// Direct link:
//   http://www.silabs.com/products/microcontroller/applications.asp
//
// F350 Resources:
// ---------------
// Timer1: clocks UART
//
// How To Test:
// ------------
// 1) Download code to a 'F350 device on a C8051F350-TB development board
// 2) Connect serial cable from the transceiver to a PC
// 3) On the PC, open HyperTerminal (or any other terminal program) and connect
//    to the COM port at <BAUDRATE> and 8-N-1
// 4) Connect a variable voltage source (between 0 and Vref)
//    to AIN2, or a potentiometer voltage divider as shown above.
// 5) HyperTerminal will print the voltage measured by the device.
//
// Target:         C8051F350
//
// Release 1.0
//    -Initial Revision (SM / TP)
//    - 19 MAR 2016

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <C8051F350.h>                 	// SFR declarations
#include <stdio.h>

//-----------------------------------------------------------------------------
// 16-bit SFR Definitions for 'F35x
//-----------------------------------------------------------------------------

sfr16 TMR2RL = 0xCA;                   	// Timer2 reload value
sfr16 TMR2 = 0xCC;                     	// Timer2 counter
sfr16 ADC0DEC = 0x9A;                 	// ADC0 Decimation Ratio Register

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

#define SYSCLK        3686400          	// SYSCLK frequency in Hz

// XFCN Setting Macro
#if  (SYSCLK <= 32000)
   #define XFCN 0
#elif(SYSCLK <= 84000)
   #define XFCN 1
#elif(SYSCLK <= 225000)
   #define XFCN 2
#elif(SYSCLK <= 590000)
   #define XFCN 3
#elif(SYSCLK <= 1500000)
   #define XFCN 4
#elif(SYSCLK <= 4000000)
   #define XFCN 5
#elif(SYSCLK <= 10000000)
   #define XFCN 6
#elif(SYSCLK <= 30000000)
   #define XFCN 7
#else
   #error "Crystal Frequency must be less than 30MHz"
#endif


#define MDCLK         1228800          	// Modulator clock in Hz (ideal is
                                       	// 2.4576 MHz) Because the SYSCLK is too slow
                                       
#define OWR                20          	// Desired Output Word Rate in Hz

#define BAUDRATE       115200          	// Baud rate of UART in bps


sbit SWITCH = P0^6 ;

sbit BAT_SHOW = P1^2 ;				   	// BAT_SHOW='0' means LED ON
sbit ON = P1^3 ;					   	// ON='0' means LED ON
sbit OFF = P1^4 ;			           	// OFF='0' means LED ON


unsigned char BATTERY;
unsigned char LASER;




//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

void Port_Init (void);
void Oscillator_Init (void);
void PCA_Init (void);
//void UART0_Init (void);
void ADC0_Init(void);
void Ext_Interrupt_Init (void);
void Timer2_Init (int counts);
void Timer2_ISR (void);
//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------
void main (void)
{
		
	 
   PCA0MD &= ~0x40;                    	// WDTE = 0 (clear watchdog timer)

   Port_Init();                        	// Initialize Crossbar and GPIO  
 
    
   Oscillator_Init();                  	// Initialize system clock
                                      	// enable)
   PCA_Init();

//   UART0_Init();                       	// Initialize UART0 for printf's
   Ext_Interrupt_Init();                // Initialize External Interrupts
            
   ADC0_Init();                        	// Initialize ADC0
 

   if ((RSTSRC & 0x02) == 0x00)        // First check the PORSF bit. if PORSF
   {                                   // is set, all other RSTSRC flags are
                                       // invalid
   	// Check if the last reset was due to the Watch Dog Timer
   	if (RSTSRC == 0x08)                    	
   	{ 
	   	Timer2_Init (SYSCLK / 12 / 50); 
        EIE1   |= 0x08;                // Enable ADC0 Interrupts
	   	while(1);                      // wait forever
   	}	
   	else 
   	{
	   	Timer2_Init (SYSCLK / 12 / 10); 
	   	// Init Timer2 to generate interrupts at a 10Hz rate.
	   }                                   
   
   }
    // Calculate Watchdog Timer Timeout
    // Offset calculated in PCA clocks
    // Offset = ( 256 x PCA0CPL2 ) + 256 - PCA0L
	//        = ( 256 x 255(0xFF)) + 256 - 0
	// Time   = Offset * (12/SYSCLK)   
	//        = 213 ms ( PCA uses SYSCLK/12 as its clock source)		
	   	
   PCA0MD  &= ~0x40;                   	// WDTE = 0 (clear watchdog timer 
                                       	// enable)
   PCA0L    = 0x00;               		// Set lower byte of PCA counter to 0  
   PCA0H    = 0x00;               		// Set higher byte of PCA counter to 0
   PCA0CPL2 = 0xFF;               		// Write offset for the WDT 
   PCA0MD  |= 0x40;               		// enable the WDT

   AD0INT = 0;
   ADC0MD = 0x83;                      	// Start continuous conversions

   EA = 1;



   while (1) 
   { 
   		PCA0CPH2 = 0x00;			     	    
				                        // Write a 'dummy' value to the PCA0CPH2 
						   				// register to reset the watchdog timer 
						   				// timeout. If a delay longer than the 
						   				// watchdog timer delay occurs between 
						   				// successive writes to this register,
									    // the device will be reset by the watch
	   							 		// dog timer.     

                                        // Spin forever
   }
}

//-----------------------------------------------------------------------------
// Initialization Subroutines
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This function initializes the GPIO and the Crossbar
//
// Pinout:
//
//	 P0.0   Testpoint	 (digital,push-pull)
//	 P0.1   Testpoint	 (digital,push-pull)
//   P0.2   analog     don't care     XTAL1
//   P0.3   analog     don't care     XTAL2
//   P0.4 - UART TX (digital, push-pull)
//   P0.5 - UART RX (digital, open-drain)
//	 P0.7   Testpoint	 (digital,push-pull)
//
//   AIN0.0 - ADC0 input Laser
//   AIN0.1 - Battery power not included yet
//
//-----------------------------------------------------------------------------
void Port_Init (void)
{

   P0MDIN &= ~0x0C;                    	// P0.2, P0.3 are analog
   
   P0MDOUT |= 0x40;                    	// TX, LEDs = Push-pull

  

   P0SKIP |= 0x00;                     	// none skipped in the Crossbar
     
   XBR0 = 0x00;                        	// UART0 Selected

   P1MDOUT |= 0x1C;

   XBR1 = 0xC0;                        	// Enable crossbar and disable weak pull-ups

   SWITCH = 1;							//MDCEN on

}




//-----------------------------------------------------------------------------
// Oscillator_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This routine initializes the system clock to use the external
// oscillator as its clock source.  Also enables missing clock detector reset.
//
//-----------------------------------------------------------------------------


void OSCILLATOR_Init (void)
{

   OSCXCN = (0x60 | XFCN);             	// Start external oscillator with
                                       	// based on crystal frequency

   while (!(OSCXCN & 0x80));           	// Wait for crystal osc. to settle

   CLKSEL = 0x01;                      	// Select external oscillator as system

   RSTSRC = 0x06;                      	// Enable missing clock detector and
                                       	// VDD Monitor reset

   OSCICN = 0x00;                      	// Disable the internal oscillator.


}



void PCA_Init()
{
    PCA0CN     =  0x40;        			// PCA counter enable
    PCA0MD    &= ~0x40 ;       			// Watchdog timer disabled-clearing bit 6
    PCA0MD    &=  0xF1;        			// timebase selected - System clock / 12
    PCA0CPL2   =  0xFF;        			// Offset value
}


//-----------------------------------------------------------------------------
// UART0_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Configure the UART0 using Timer1, for <BAUDRATE> and 8-N-1.
//
//-----------------------------------------------------------------------------
/*

void UART0_Init (void)
{
   SCON0 = 0x10;                       	// SCON0: 8-bit variable bit rate
                                       	//        level of STOP bit is ignored
                                       	//        RX enabled
                                       	//        ninth bits are zeros
                                       	//        clear RI0 and TI0 bits
   if (SYSCLK/BAUDRATE/2/256 < 1) {
      TH1 = -(SYSCLK/BAUDRATE/2);
      CKCON |=  0x08;                  	// T1M = 1; SCA1:0 = xx
   } else if (SYSCLK/BAUDRATE/2/256 < 4) {
      TH1 = -(SYSCLK/BAUDRATE/2/4);
      CKCON &= ~0x0B;                  	// T1M = 0; SCA1:0 = 01
      CKCON |=  0x01;
   } else if (SYSCLK/BAUDRATE/2/256 < 12) {
      TH1 = -(SYSCLK/BAUDRATE/2/12);
      CKCON &= ~0x0B;                  	// T1M = 0; SCA1:0 = 00
   } else if (SYSCLK/BAUDRATE/2/256 < 48) {
      TH1 = -(SYSCLK/BAUDRATE/2/48);
      CKCON &= ~0x0B;                  	// T1M = 0; SCA1:0 = 10
      CKCON |=  0x02;
   } else {
      while (1);                       	// Error.  Unsupported baud rate
   }

   TL1 = TH1;                          	// Init Timer1
   TMOD &= ~0xf0;                      	// TMOD: timer 1 in 8-bit autoreload
   TMOD |=  0x20;
   TR1 = 1;                            	// START Timer1
   TI0 = 1;                            	// Indicate TX0 ready
}

*/
//-----------------------------------------------------------------------------
// ADC0_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Initialize the ADC to use the temperature sensor. (non-differential)
//
//-----------------------------------------------------------------------------

void ADC0_Init (void)
{
   REF0CN &= ~0x01;                    	// disable internal vref
   ADC0CN = 0x00;                      	// Gain = 1, Unipolar mode
   ADC0CF = 0x04;                      	// Interrupts upon SINC3 filter output
                                       	// and uses external VREF

   ADC0CLK = (SYSCLK/MDCLK)-1;         	// Generate MDCLK for modulator.
                                       	// Ideally MDCLK = 2.4576MHz

   // Program decimation rate for desired OWR
   ADC0DEC = ((unsigned long) MDCLK / (unsigned long) OWR /
              (unsigned long) 128) - 1;

   ADC0BUF = 0x00;                     	// Turn off Input Buffers
   ADC0MUX = 0x18;                     	// Select AIN0.0

   ADC0MD = 0x81;                      	// Start internal calibration
   while(AD0CALC != 1);                	// Wait until calibration is complete

   EIE1   |= 0x08;                     	// Enable ADC0 Interrupts
   ADC0MD  = 0x80;                     	// Enable the ADC0 (IDLE Mode)

}


void Ext_Interrupt_Init (void)
{
    
   TCON = 0x01;                        // /INT 0 are edge triggered

   IT01CF = 0x07;                      // /INT0 active low; /INT0 on P1.0;

   EX0 = 1;                            // Enable /INT0 interrupts                           

}

//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// ADC0_ISR
//-----------------------------------------------------------------------------
//
// This ISR prints the result to the UART. The ISR is called after each ADC
// conversion.
//
//-----------------------------------------------------------------------------

void INT0_ISR (void) interrupt 0
{

  if(IT01CF == 0x0F)
   {
   		SWITCH = 0;
		return;
	}

   if(IT01CF == 0x07)
   {
   		SWITCH = 1;

   		EX0 = 1;
   		EA = 1;
   		IT01CF = 0x0F;
  		return;
   }

}



void ADC0_ISR (void) interrupt 10
{
	if(ADC0MUX == 0x18)
	{
        while(!AD0INT);                     	// Wait till conversion complete
        AD0INT = 0;  
		BATTERY = (unsigned char)ADC0H;                       	// Clear ADC0 conversion complete flag

 		if (BATTERY >= 0x99)
		{
			BAT_SHOW = 1;
			//BAT_SHOW = ~BAT_SHOW;

		}        
		
		if (BATTERY < 0x99)
		{
			BAT_SHOW = ~BAT_SHOW;
			//BAT_SHOW = 1;
		}
	}

	
	if(ADC0MUX == 0x08)
	{
	    while(!AD0INT);                     	// Wait till conversion complete
        AD0INT = 0;                         	// Clear ADC0 conversion complete flag
		LASER = (unsigned char)ADC0H;                       	// Clear ADC0 conversion complete flag
 		
		
		if (LASER >= 0x28)
		{
			ON = 0 ;
        	OFF = 1 ;
		}        
		
		if (LASER < 0x28)
		{
			ON = 1;
        	OFF = 0;
		}

	}

	if(ADC0MUX == 0x18)
	{


	   ADC0MD  = 0x80;                     	// Enable the ADC0 (IDLE Mode)
	   ADC0MUX = 0x08;                     	// Select AIN0.0
	   ADC0MD  = 0x83;                     	// Enable the ADC0 (IDLE Mode)

		return;
	}
	
	if(ADC0MUX == 0x08)
	{


	   ADC0MD  = 0x80;                     	// Enable the ADC0 (IDLE Mode)
	   ADC0MUX = 0x18;                     	// Select AIN0.0
	   ADC0MD  = 0x83;                     	// Enable the ADC0 (IDLE Mode)
		return;
	}

}

void Timer2_Init (int counts)
{
   TMR2CN  = 0x00;                     	// Stop Timer2; Clear TF2;
                                       	// use SYSCLK/12 as timebase
   CKCON  &= ~0x60;                    	// Timer2 clocked based on T2XCLK;

   TMR2RL  = -counts;                  	// Init reload values
   TMR2    = 0xffff;                   	// set to reload immediately
   ET2     = 1;                        	// enable Timer2 interrupts
   TR2     = 1;                        	// start Timer2
}


//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Timer2_ISR
//-----------------------------------------------------------------------------
// This routine changes the state of the LED whenever Timer2 overflows.
//-----------------------------------------------------------------------------



void Timer2_ISR (void) interrupt 5
{
   TF2H = 0;                           	// clear Timer2 interrupt flags
}




//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
