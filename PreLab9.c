/*
 * File:   PreLab9.c
 * Author: jorge
 *
 * Created on 26 de abril de 2022, 04:29 PM
 */
// PIC16F887 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>             // int8_t, unit8_t
#define _tmr0_value 251         // Valor para 10 ms
#define _XTAL_FREQ 500000
#define IN_MIN 0                
#define IN_MAX 255              // Valores de entrada a Potenciometro
#define OUT_MIN 31
#define OUT_MAX 63             // Valores para el servomotor

unsigned short CCPR = 0;
//unsigned short val = 0;

void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

void __interrupt() isr (void){
    if(PIR1bits.ADIF){              // Interrupción por ADC
        if(ADCON0bits.CHS == 0){    // Interrupción por AN0
            CCPR = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);   // Valor de ancho de pulso
            CCPR1L = (uint8_t)(CCPR>>2);                            // 8 bits mas significativos en CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11;                         // 2 bits menos significativos en DC1B
        }
        PIR1bits.ADIF = 0;          // Limpiar la bandera de ADC
    }
    return;
}


void main(void) {
    setup();
    while(1){
        if(ADCON0bits.GO == 0){             
            ADCON0bits.GO = 1;  // Iniciar proceso de conversión
        }
    }
    return;
}

void setup(void){
    ANSEL = 0b00000001;         // AN0 como entrada analógica
    ANSELH = 0;                 // I/O digitales

    TRISA = 0b00000001;         // RA0 como entrada analógica y demás como salidas
    PORTA = 0x00;               // Se limpia PORTA
    
    // Configuración de interrupciones
    PIR1bits.ADIF = 0;          // Limpiar la bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitar interrupciones de ADC
    INTCONbits.PEIE = 1;        // Habilitar interrupciones de periféricos
    INTCONbits.GIE = 1;         // Habilitar interrupciones globales
    
    // Configuración del oscilador
    OSCCONbits.IRCF = 0b0011;   // 500kHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuración ADC
    ADCON0bits.ADCS = 0b00000001;   // FOSC/8
    ADCON1bits.VCFG0 = 0;           // VDD
    ADCON1bits.VCFG1 = 0;           // VSS
    ADCON0bits.CHS = 0b0000;        // Selecciona el AN0
    ADCON1bits.ADFM = 0;            // Justificador a la izquierda
    ADCON0bits.ADON = 1;            // Habilitar modulo ADC
    __delay_us(320);                 // Sample time
    
    // Configuración PWM
    TRISCbits.TRISC2 = 1;           // Deshabilitar salida de CCP1 (Se pone como entrada)
    PR2 = 155;                       // Periodo de 20 ms
    
    // Configuracion CCP
    CCP1CON = 0;                    // Apagar CCP1
    CCP1CONbits.P1M = 0;            // Modo sigle output
    CCP1CONbits.CCP1M = 0b1100;     // PWM
    
    CCPR1L = 156>>2;
    CCP1CONbits.DC1B = 156 & 0b11;   // 25% Ancho de pulso
    
    PIR1bits.TMR2IF = 0;            // Limpiar bandera de TMR2
    T2CONbits.T2CKPS = 0b11;        // Prescaler 1:16
    T2CONbits.TMR2ON = 1;           // Encender TMR2
    while (!PIR1bits.TMR2IF);       // Esperar un ciclo del TMR2
    PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC2 = 0;           // Habilitar salida de PWM
    return;
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}