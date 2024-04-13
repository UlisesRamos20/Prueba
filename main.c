/*
 * File:   Double motors
 * Author: Ulises
 *
 * Last modified: February 5th 2024 
 */
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Mode (Internal Fast RC (FRC) with divide by N)
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
//Global variables
volatile unsigned int cont=0, cont_p=0;     //Pulse variables
volatile unsigned int PT=0;                 //Previous time
volatile unsigned int  pwm1 = 2000;               //Duty_cycle minimo 2000
volatile unsigned int  pwm2 = 2000;
int interval = 15625;                       //Time interval
float pv;                                   //Proces variable
float sp;                                   //Set point
float cv;                                   //Current value
float cvp;                                  //Previous value
float error;
float error1;
float error2;

float Kp = 1;
float Ki = 5;
float Kd = 0.1;
float Tm = 0.1;

#include "libpic30.h"

void os_config(void);
void UART_config(void);
void QEI_config(void);
void MCPWM_config1(void);
void MCPWM_config2(void);
void TMR_config1(void);
void TMR_config2(void);


int main(void) {
    os_config();
    UART_config();
    QEI_config();
    MCPWM_config1();
    MCPWM_config2();
    TMR_config1();
    TMR_config2();
      
    ADPCFG = 0xFFFF;    //Deshabilita todas las entradas anaogicas
    // Configuración del motor
    TRISBbits.TRISB4 = 0;           //pin rb4
    TRISBbits.TRISB5 = 0;
    LATBbits.LATB4 = 0;
    LATBbits.LATB5 = 0;
    
    while(1){
        //Enciende los motores
        LATBbits.LATB4 = 1;
        LATBbits.LATB5 = 1;
        //Lectura del encoder
        cont_p=cont;
        cont = POS1CNT;
        pv = cont;
        
        unsigned int CT = TMR1;     //Current time
        unsigned int ET = CT-PT;    //Elapsed time
        
        if(ET >= interval)
        {
            P1DC1 = pwm1;
            P2DC1 = pwm2;
            PT = CT;
            if (cont != 0)
            {
                printf("pULSOS POR SEGUNDO: %u \r\n",cont);
                printf("Timer: %u \r\n",TMR1);
                printf("Timer: %u \r\n",TMR2);
                printf("PWM: %u \r\n",pwm1);
                printf("PWM: %u \r\n",pwm2);
                pwm1 = pwm1;
                pwm2 = pwm2;
                 //------Set point------
                sp = 600;
                error = sp - pv;

                //------Ecuacion diferencial------
                cv = cvp + (Kp +Kd/Tm)*error + (-Kp + Ki*Tm - 2*Kd/Tm)*error1 + (Kd/Tm)*error2;
                cvp = cv;                    //Recursividad
                error2 = error1;
                error1 = error;

                //------Saturacion de la salida del pid------
                if(cv > 346){
                    cv = 346;
                }
                if(cv < 20){
                    cv = 20;
                }
                pwm1 = cv*15.41;
                pwm2 = cv*15.41;
                
                printf("set point: %u \r",sp);
                printf("Process variable: %u \r",pv);
            }
            
           
            
            TMR1 = 0;
            TMR2 = 0;
            POS1CNT = 0;
        }
        
        
        /*
        if(ET > 0)
        {
            pps = (float)(cont - cont_p) / ET * 1000.0;
        }
        */       
            
        //pv = cont*(60/(32/70));
        
        /*
        PT = CT;
        printf("Contador: %u \r\n",cont);
        printf("Timer: %u \r\n",TMR1);
        printf("Pulsos por segundo: %f \r\n",pps);
        P1DC1 = pwm;
        pwm = pwm + 100;
        printf("Pwm: %u \r\n",pwm);
        POS1CNT = 0;
        TMR1 = 0;
        __delay_ms(1000);
         */
        
        //LATBbits.LATB0 = 1;
        //__delay_ms(1000);   
    }
    return 0;
}

void TMR_config1(void) {
    T1CONbits.TON = 0;      // Detiene el temporizador
    T1CONbits.TCS = 0;      // Fuentes de reloj interna (FOSC/2)
    T1CONbits.TGATE = 0;    // Deshabilita Gated Timer mode
    T1CONbits.TCKPS = 0b11; // Configura el pre-escaler a 1:256
    TMR1 = 0;               // Reinicia el contador

    IFS0bits.T1IF = 0;      // Limpia la bandera de interrupción
    T1CONbits.TON = 1;      // Inicia el temporizador
}

void TMR_config2(void) {
    T2CONbits.TON = 0;      // Detiene el temporizador
    T2CONbits.TCS = 0;      // Fuentes de reloj interna (FOSC/2)
    T2CONbits.TGATE = 0;    // Deshabilita Gated Timer mode
    T2CONbits.TCKPS = 0b11; // Configura el pre-escaler a 1:256
    TMR2 = 0;               // Reinicia el contador

    IFS0bits.T2IF = 0;      // Limpia la bandera de interrupción
    T2CONbits.TON = 1;      // Inicia el temporizador
}

void MCPWM_config1(void){
   
    P1TCONbits.PTMOD = 0;
    P1TCONbits.PTCKPS = 0;
    P1TCONbits.PTOPS = 0;
    
    P1TPER = 2666;  
    
    PWM1CON1bits.PMOD1 = 1;
    PWM1CON1bits.PEN1H = 1;
    
    
    PWM1CON2bits.IUE = 1;
    
    P1DC1 = 0;
    P1TCONbits.PTEN = 1;
} 
void MCPWM_config2(void){
   
    P2TCONbits.PTMOD = 0;
    P2TCONbits.PTCKPS = 0;
    P2TCONbits.PTOPS = 0;
    
    P2TPER = 2666;  
    
    PWM2CON1bits.PMOD1 = 1;
    PWM2CON1bits.PEN1H = 1;
    
    
    PWM2CON2bits.IUE = 1;
    
    P2DC1 = 0;
    P2TCONbits.PTEN = 1;
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
