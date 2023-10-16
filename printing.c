/*
 * File:   printing.c
 * Author: Ulises Ramos
 *
 * Created on 16 de octubre de 2023, 08:47 AM
 */

#include "printing.h"
#include <stdio.h>
#include "xc.h"
// Implementación de UART_send_char
void UART_send_char(char c) {
    while (U1STAbits.UTXBF);  // Esperar a que el buffer de transmisión esté vacío
    U1TXREG = c;  // Enviar el carácter
}

// Implementación de UART_send_string
void UART_send_string(const char *str) {
    while (*str != '\0') {
        UART_send_char(*str);
        str++;
    }
}

// Implementación de my_printf
void _printf(const char *format, ...) {
    va_list args;
    va_start(args, format);

    while (*format != '\0') {
        if (*format == '%') {
            format++;  // Avanzar al siguiente carácter después de '%'
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
                    // Manejo de números enteros
                    num = va_arg(args, int);
                    // Convierte el número a una cadena y la envía
                      // Suficiente para números enteros de 32 bits
                    vsprintf(num_str, "%d", num);
                    UART_send_string(num_str);
                    break;
                case 'f':
                    // Manejo de números de punto flotante
                    num_float = va_arg(args, double);
                    // Convierte el número de punto flotante a una cadena y envía la cadena
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