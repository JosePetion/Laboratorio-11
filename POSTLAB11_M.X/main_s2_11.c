/* 
 * File:   main_s2_11.c
 * Author: Jose Pablo Petion
 *
 * Created on May 13, 2022, 4:29 PM
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
#define _XTAL_FREQ 1000000              // Frec. 1 MHz
#define FLAG_SPI 0xFF                   // bandera esclavo

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t COUNTER;

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);

/*------------------------------------------------------------------------------
 * VECTOR DE INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    
    if(INTCONbits.RBIF){                // PULSADOR: PORTB
        
        if(!PORTBbits.RB0){             
            COUNTER++;                 // Contador: esclavo+1
        }
        else if(!PORTBbits.RB1){        
            COUNTER--;                 // Contador: esclavo-1
        }
        INTCONbits.RBIF = 0;            
    }
    
    if (PIR1bits.SSPIF){                // Bandera SPI
        SSPBUF = COUNTER;              // Cargamos envio: maestro
        PIR1bits.SSPIF = 0;      }
    
    return;
}

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
 
    ANSEL = 0x00;
    ANSELH = 0x00; 
    TRISC = 0b00011000;
    PORTCbits.RC5 = 0;    
    TRISA = 0b00100000;
    TRISB = 0b00000011;
    TRISD = 0;
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    
    OSCCONbits.IRCF = 0b100;        // 1MHz
    OSCCONbits.SCS = 1;
    
    //INTERRUPCIONES
    INTCONbits.GIE = 1;
    INTCONbits.RBIE = 1;
    INTCONbits.RBIF = 0;      
    
    // INTERUPCIONES PORTB
    IOCBbits.IOCB0 = 1;
    IOCBbits.IOCB1 = 1;
    OPTION_REGbits.nRBPU = 0;
    WPUBbits.WPUB0 = 1;
    WPUBbits.WPUB1 = 1;
    
    //SPI ESCLAVO2 SSPCON <5:0>
    SSPCONbits.SSPM = 0b0100;
    SSPCONbits.CKP = 0;
    SSPCONbits.SSPEN = 1;
                                // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        
    SSPSTATbits.SMP = 0;
    
    //BANDERAS SPI
    PIR1bits.SSPIF = 0;
    PIE1bits.SSPIE = 1;         
    INTCONbits.GIE = 1;        
    INTCONbits.PEIE = 1;
    
}