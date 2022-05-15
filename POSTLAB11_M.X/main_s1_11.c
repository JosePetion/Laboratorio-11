/* 
 * File:   main_s1_11.c
 * Author: Jose Pablo Petion
 * ESCLAVO QUE CONTROLA AL MOTOR
 * Created on May 13, 2022, 4:23 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT    // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF               // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF              // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF              // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF                 // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF                // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF              // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF               // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF              // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF                // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V           // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF                // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000               //1 MHz
#define PLOW 0                           // Valor minimo (ADDRES)
#define PHIGH 255                        // Valor máximo
#define PWMLOW 62                        // Valor minimo (PWM)
#define PWMHIGH 125                      // Valor máximo

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t CCPR;
uint8_t VALOR;               // Variable para almacenar valores temporales

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){    
    
    if (PIR1bits.SSPIF){                                    // RECIBE DATOS DEL ESCLAVO
        
        VALOR = SSPBUF;                                     // VALOR DEL MAESTRO
        CCPR = map(VALOR, PLOW, PHIGH, PWMLOW, PWMHIGH);    
        CCPR1L = (uint8_t)(CCPR>>2);   
        CCP1CONbits.DC1B = CCPR & 0b11;
        PIR1bits.SSPIF = 0;
        
    }return;}

/*------------------------------------------------------------------------------
 * LOOP PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    
    setup();
    while(1){}
    return;
}

/*------------------------------------------------------------------------------
 * SETUP 
 ------------------------------------------------------------------------------*/
void setup(void){   
    ANSEL = 0;
    ANSELH = 0;            
    TRISA = 0b00100000; 
    TRISD = 0;            
    TRISC = 0b00111000;
    PORTCbits.RC5 = 0;
    PORTA = 0;
    PORTC = 0;
    PORTD = 0;
    
    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;
    
    //INTERRUPCIONES
    INTCONbits.GIE = 1; 
    INTCONbits.PEIE = 1;
    PIR1bits.SSPIF = 0;
    PIE1bits.SSPIE = 1;
    
    //SPI ESCLAVO SSPCON <5:0>
    
    SSPCONbits.SSPM = 0b0100;   
    SSPCONbits.CKP = 0;       
    SSPCONbits.SSPEN = 1;
                                        // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1; 
    SSPSTATbits.SMP = 0;
    
    // PWM
    PIR1bits.TMR2IF = 0;
    T2CONbits.T2CKPS = 0b11;            // prescaler 1:16
    T2CONbits.TMR2ON = 1;
    while(!PIR1bits.TMR2IF);
    PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC2 = 1;
    PR2 = 62;                           // T = 4ms
    
    // CCP-CCP1
    CCP1CON = 0;
    CCP1CONbits.P1M = 0;
    CCP1CONbits.CCP1M = 0b1100;        
    CCPR1L = 250>>2;
    CCP1CONbits.DC1B = 250 & 0b11;       
    TRISCbits.TRISC2 = 0;               //ENABLE PWM
}

/*------------------------------------------------------------------------------
 * FUNCIONES 
 ------------------------------------------------------------------------------*/

//INTERPOLACIÓN
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));}