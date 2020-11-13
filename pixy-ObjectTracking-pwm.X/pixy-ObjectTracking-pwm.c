// This is a very simplified object tracking with two servos
// The pulse width calculation is NOT dependent on distance from the camera
// There is only one equation for pulse width for the pan servo as a function of X and one for the tilt servo as a function of Y

// Pixy camera receives 6 words of data from a single object after double sync codes (1 for frame and 1 for each object)
// Bytes 0, 1 = sync word (0xAA55)
// Bytes 2, 3 = check sum (sum of bytes 6 to 13)
// Bytes 4, 5 = signature number (signature of the object)
// Bytes 6, 7 = X center of the object
// Bytes 8, 9 = Y center of the object
// Bytes 10, 11 = width of the object
// Bytes 12, 13 = width of the object

#include "xc.h"

// Configuration Register - FOSC
#pragma config FCKSM = 3    // both clock switching and fail-safe modes are disabled
#pragma config OSCIOFNC = 0 // OSCO pin is a general purpose I/O pin
#pragma config POSCMD = 3   // primary oscillators are disabled

// Configuration Register - FOSCSEL
#pragma config IESO = 0     // start with a user-selected oscillator at reset
#pragma config FNOSC = 7    // select FRC oscillator with postscalar at reset

// Configuration Register - FICD
#pragma config JTAGEN = 0   // JTAG is disabled
#pragma config IOL1WAY = 0  // Allow multiple PPS reconfigurations
#pragma config GSS = 3      // User program is not code-protected
#pragma config GWRP = 1     // User program is not write-protected
#pragma config ICS = 3      // Program with PGEC1/PGED1 program ports
#pragma config PWMPIN = 0   // All Motor Control PWM outputs are active at Reset
#pragma config HPOL = 1     // Motor control HIGH outputs are active high
#pragma config LPOL = 1     // Motor control LOW outputs are active high
#pragma config FWDTEN = 0   // Disable Watch Dog timer

#define PPSUnLock       __builtin_write_OSCCONL(OSCCON & 0xBF)
#define PPSLock         __builtin_write_OSCCONL(OSCCON | 0x40)

unsigned int i, j, k;
unsigned int PWpan[3], PWtilt[3];
unsigned int HighByte, LowByte;
unsigned int CheckSum, Signature, Width, Height;
unsigned int X[3], Y[3];
unsigned int StartWord = 0xAA55;
unsigned int Word[20];
unsigned int breakpoint;
    
unsigned char SPI_Exchange (unsigned char Value)
    {
    while (SPI1STATbits.SPITBF == 1); // Wait until the TX buffer is empty due to a prior process
    SPI1BUF = Value;                  // When empty, send the sync byte to the TX buffer             
    while (SPI1STATbits.SPIRBF == 0); // As sync bits shifts out of the SDO port of the CPU, pixy bits are received from the SDI port
                                      // Wait until the RX buffer is full of pixy data
    return SPI1BUF;                   // When full, read the pixy data in RX buffer through SPI1BUF
    }

void delay ()
    {
    for (k = 0; k < 1000; k++);
    }

void main()
{
    
// OSCTUN Register
OSCTUNbits.TUN = 21;   // select FRC = 8.0MHz
CLKDIVbits.FRCDIV = 3; // FOSC = 1MHz, FP = 500KHz
//CLKDIVbits.FRCDIV = 2; // FOSC = 2MHz, FP = 1MHz
CLKDIVbits.DOZE = 2;   // FCY = 125KHz (CPU clock)
//CLKDIVbits.DOZE = 0;   // FCY = 1MHz (CPU clock)
CLKDIVbits.DOZEN = 1;  // Enable CPU clock

// PWM1 timebase control register
P1TCONbits.PTEN   = 0;  // Do not enable the clock delivery to PWM1 timer yet
P1TCONbits.PTCKPS = 0;  // Prescale is 1:1 so Timer1 clock = FP = 500KHz
P1TCONbits.PTMOD  = 0;  // PWM1 is in free-running mode

// PWM1 counter and period
P1TMRbits.PTMR = 0;     // Initial value in PWM1 counter register
P1TPER = 10000;         // PWM1 period register produces 20msec PWM period

// PWM1 control register1
PWM1CON1bits.PMOD3 = 1; // PWM1H3 and PWM1L3 outputs are independent mode
PWM1CON1bits.PMOD2 = 1; // PWM1H2 and PWM1L2 outputs are independent mode
PWM1CON1bits.PMOD1 = 1; // PWM1H1 and PWM1L1 outputs are independent mode
PWM1CON1bits.PEN3H = 0; // PWM1H3 is disabled
PWM1CON1bits.PEN2H = 1; // PWM1H2 is enabled for TILT servo
PWM1CON1bits.PEN1H = 1; // PWM1H1 is enabled for PAN servo
PWM1CON1bits.PEN3L = 0; // PWM1L3 is disabled
PWM1CON1bits.PEN2L = 0; // PWM1L2 is disabled
PWM1CON1bits.PEN1L = 0; // PWM1L1 is disabled
PWM2CON1bits.PEN1H = 0; // PWM2H1 is disabled
PWM2CON1bits.PEN1L = 0; // PWM1L1 is disabled

// PWM1 control register2
PWM1CON2bits.IUE = 0;   // Updates are not immediate
PWM1CON2bits.UDIS = 0;  // Updates from period and duty cycle registers are enabled

// PWM1 duty cycle register1
P1DC1 = 1500;            // Duty cycle for PAN servo
P1DC2 = 1500;            // Duty cycle for TILT servo
delay ();

// Peripheral Pin Select with RP pins 
PPSUnLock;
RPOR4bits.RP8R = 8;     // RB8 is for SCK1 (output)
RPINR20bits.SCK1R = 8;  // RB8 is for SCK1 (input)
RPOR4bits.RP9R = 7;     // RB9 is for SDO1
RPINR20bits.SDI1R = 7;  // RB7 is for SDI1
RPOR3bits.RP6R = 9;     // RB6 is for SS1
PPSLock;

// Define I/O 
//AD1PCFGL=0xFFFF;//////
//TRISAbits.TRISA0=0;///Green LED
//TRISAbits.TRISA1=0;///Blue LED

TRISBbits.TRISB6 = 0; // Configure RB6 as an output for SS
TRISBbits.TRISB8 = 0; // Configure RB8 as an output for SCK1 
TRISBbits.TRISB9 = 0; // Configure RB9 as an output for SDO1
TRISBbits.TRISB7 = 1; // Configure RB7 as an input for SDI1

// SPI configuration MASTER mode sending 8 bits
SPI1CON1bits.DISSCK = 0;  // Enable the internal SPI clock
SPI1CON1bits.DISSDO = 0;  // Enable the SPI data output, SDO
SPI1CON1bits.MODE16 = 0;  // Enable the 8-bit data mode
SPI1CON1bits.SSEN = 0;    // This unit is not a slave so Slave Select pin is not used 
SPI1CON1bits.MSTEN = 1;   // Enable MASTER mode
SPI1CON1bits.SMP = 0;     // Sample input in the middle of data output period (data is sampled at the pos edge when received at the neg edge of SCK)
SPI1CON1bits.CKE = 1;     // SCK edge: Output data changes when SCK goes from ACTIVE to IDLE state (data is transmitted at the neg edge of SCK)
SPI1CON1bits.CKP = 0;     // SCK polarity: IDLE is low phase and ACTIVE is high phase of SCK
SPI1CON1bits.PPRE = 3;    // Primary SPI clock pre-scale is 1:1
SPI1CON1bits.SPRE = 3;    // Secondary SPI clock pre-scale is 1:5 -> SCK = FCY/5 = 25KHz
SPI1STATbits.SPIROV = 0;  // Clear initial overflow bit in case an overflow condition in SPI1BUF
SPI1STATbits.SPIEN = 1;   // Enable the SPI interface

// Enable time-base control register
P1TCONbits.PTEN = 1;   // Enable the clock delivery to PWM1 timer  

//LATAbits.LATA1=1;//Blue LED ON to indicate that program is running

START_OVER:

for (j = 0; j <= 2; j++)      // Inspect a total of 3 frames
{
    for (i = 0; i < 20; i++)  // Scan 20 words (per frame) to see the beginning of double sync codes
    {
       
    LATBbits.LATB6 = 0;       // Lower SS for object data reception
    HighByte = SPI_Exchange(0x5A);
    LowByte  = SPI_Exchange(0x00);
    Word[i] = (HighByte << 8) | LowByte;
    LATBbits.LATB6 = 1;       // Raise SS for object data completion         
    }        

    i = 0;  // Start inspecting the words in a frame
    NEXT_TRY:
    if  ((Word[i] == StartWord) && (Word[i+1] == StartWord) && (Word[i+2] != StartWord))  // Find the beginning of valid object data
        {
        //LATAbits.LATA0=1;//Green LED ON to indicate that pixy Cam is sending data
        CheckSum  = Word[i+2];      
        Signature = Word[i+3];
        X[j]   = Word[i+4];
        Y[j]   = Word[i+5];
        Width  = Word[i+6];
        Height = Word[i+7];
        PWpan [j] = -2.86*X[j] + 1968.6; // PAN servo
        PWtilt[j] =  3.86*Y[j] + 1105.2; // TILT servo 
        }
    else
        {
        i = i + 1;
        goto NEXT_TRY;
        }
    breakpoint = 1;
}

unsigned int PWpanav;
unsigned int PWtiltav;
PWpanav  = (PWpan[0] + PWpan[1] + PWpan[2])/3;
PWtiltav = (PWtilt[0] + PWtilt[1] + PWtilt[2])/3;
P1DC1 = PWpanav;
P1DC2 = PWtiltav;

goto START_OVER;

}
