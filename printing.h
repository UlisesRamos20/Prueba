/* 
 * File:    printing
 * Author:  Ulises Ramos
 * Comments:
 * Revision history: 16-10-23
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef PRINTING
#define	PRINTING

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdarg.h>
#include <stdio.h>

// Funci�n para enviar un car�cter por UART
void UART_send_char(char c);

// Funci�n para enviar una cadena de caracteres por UART
void UART_send_string(const char *str);

// Funci�n printf simplificada
void _printf(const char *format, ...);

#endif	/* XC_HEADER_TEMPLATE_H */

