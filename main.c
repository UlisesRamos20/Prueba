/*
 * File:   UART incompleto
 * Author: Ulises
 *
 * Created on 16 de octubre de 2023, 11:43 AM
 */
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRIPLL         // Oscillator Mode (Internal Fast RC (FRC) with divide by N)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = XT              // Primary Oscillator Source (XT Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include "xc.h"
#include "printing.h"

#define FCY 40000000
#define BAUDRATE 115200
#define BRGVAL ((FCY/BAUDRATE)/16)-1
unsigned int i, cont=0;

#include "libpic30.h"

void os_config(void);
void UART_config(void);
void QEI_config(void);


int main(void) {
    os_config();
    UART_config();
    QEI_config();
      
    ADPCFG = 0xFFFF;    //Deshabilita todas las entradas anaogicas
    // Configuración del motor
    TRISBbits.TRISB4 = 0;
    TRISBbits.TRISB5 = 0;
    LATBbits.LATB4 = 0;
    LATBbits.LATB5 = 0;
    
    while(1){
        //Enciende los motores
        LATBbits.LATB4 = 1;
        LATBbits.LATB5 = 1;
        //Lectura del encoder
        cont = POS1CNT;
        printf("Contador: %u \r\n",cont);
        __delay_ms(10);
        //LATBbits.LATB0 = 1;
        //__delay_ms(1000);   
    }
    return 0;
}



void QEI_config(void){
    QEI1CONbits.QEIM = 5; // Modo de 2x y el indice, conteo hasta MAXPOS1
    QEI1CONbits.CNTERR = 0; // Sin detección de error de conteo
    QEI1CONbits.UPDN = 0;
    //QEI1CONbits.QEISIDL = 0; // Continuar funcionando en modo reposo
    QEI1CONbits.SWPAB = 0; // No swap A y B
    QEI1CONbits.PCDOUT = 0; // Sin dirección de conteo
    
    // Configura los pines de entrada de QEI    
    ADPCFGbits.PCFG2 = 1; // Deshabilita entrada analógica en B2
    ADPCFGbits.PCFG3 = 1; // Deshabilita entrada analógica en B3
    TRISBbits.TRISB2 = 1; // Configura B2 como entrada
    TRISBbits.TRISB3 = 1; // Configura B3 como entrada

    RPINR14bits.QEA1R = 2; // Asigna B2 como entrada A del QEI
    RPINR14bits.QEB1R = 3; // Asigna B3 como entrada B del QEI
    MAX1CNT = 65535;
    POS1CNT = 0;
}


void UART_config(void){
    U1BRG = BRGVAL;         //115200 bps
    U1MODEbits.PDSEL=0;     //8 bits de paridad
    U1MODEbits.STSEL=0;     //1 bit de stop
    U1MODEbits.ABAUD = 0;   // Auto-Baud desabilitado 
    U1MODEbits.BRGH = 0;    //Modo de baja velocidad
    RPINR18bits.U1RXR1 = 1; //Pin RX = RP1
    RPOR0bits.RP0R = 3;     //Pin TX = RP0
    
    IEC0bits.U1TXIE = 0;    // Deshbabilita la interrupcion  UART TX 
    U1MODEbits.UARTEN = 1;  // Habilta el UART 
    U1STAbits.UTXEN = 1;    // Enable UART TX   
    
    ADPCFGbits.PCFG0 = 1;   // Deshabilita entrada analógica en B0
    ADPCFGbits.PCFG1 = 1;   // Deshabilita entrada analógica en B1
    TRISBbits.TRISB0 = 1;   // Configura B0 como entrada
    TRISBbits.TRISB1 = 0;   // Configura B1 como salida
}


void os_config(void){
    OSCTUN = 0;                     // Tune FRC oscillator, if FRC is used
    OSCCONbits.CLKLOCK = 0;
    OSCCONbits.COSC = 3;
    OSCCONbits.OSWEN = 0;
    CLKDIVbits.PLLPOST = 0;
    PLLFBDbits.PLLDIV = 78;
    CLKDIVbits.DOZEN  = 1;
    CLKDIVbits.DOZE = 0;
    while(OSCCONbits.LOCK != 1) {}; // Wait for PLL to lock
    
}
