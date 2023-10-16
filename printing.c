/*
 * File:   printing.c
 * Author: Ulises Ramos
 *
 * Created on 16 de octubre de 2023, 08:47 AM
 */

#include "printing.h"
#include <stdio.h>
#include "xc.h"
// Implementaci�n de UART_send_char
void UART_send_char(char c) {
    while (U1STAbits.UTXBF);  // Esperar a que el buffer de transmisi�n est� vac�o
    U1TXREG = c;  // Enviar el car�cter
}

// Implementaci�n de UART_send_string
void UART_send_string(const char *str) {
    while (*str != '\0') {
        UART_send_char(*str);
        str++;
    }
}

// Implementaci�n de my_printf
void _printf(const char *format, ...) {
    va_list args;
    va_start(args, format);

    while (*format != '\0') {
        if (*format == '%') {
            format++;  // Avanzar al siguiente car�cter despu�s de '%'
            int num;
            char num_str[12];
            double num_float;
            char float_str[64];
            switch (*format) {
                case 'c':
                    UART_send_char(va_arg(args, int));
                    break;
                case 's':
                    UART_send_string(va_arg(args, char *));
                    break;
                case 'd':
                    // Manejo de n�meros enteros
                    num = va_arg(args, int);
                    // Convierte el n�mero a una cadena y la env�a
                      // Suficiente para n�meros enteros de 32 bits
                    vsprintf(num_str, "%d", num);
                    UART_send_string(num_str);
                    break;
                case 'f':
                    // Manejo de n�meros de punto flotante
                    num_float = va_arg(args, double);
                    // Convierte el n�mero de punto flotante a una cadena y env�a la cadena
                    sprintf(float_str, "%f", num_float);
                    UART_send_string(float_str);
                    break;
             }
        } else {
            UART_send_char(*format);
        }
        format++;
    }

    va_end(args);
}