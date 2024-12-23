;******************************************************************************
;   This file is a basic template for assembly code for a PIC18F4550. Copy    *
;   this file into your project directory and modify or add to it as needed.  *
;                                                                             *
;   The PIC18FXXXX architecture allows two interrupt configurations. This     *
;   template code is written for priority interrupt levels and the IPEN bit   *
;   in the RCON register must be set to enable priority levels. If IPEN is    *
;   left in its default zero state, only the interrupt vector at 0x008 will   *
;   be used and the WREG_TEMP, BSR_TEMP and STATUS_TEMP variables will not    *
;   be needed.                                                                *
;                                                                             *
;   Refer to the MPASM User's Guide for additional information on the         *
;   features of the assembler.                                                *
;                                                                             *
;   Refer to the PIC18FXX50/XX55 Data Sheet for additional                    *
;   information on the architecture and instruction set.                      *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Filename:    Plantilla                                              *
;    Date:        18/12/19                                                    *
;    File Version: 1.0                                                        *
;                                                                             *
;    Author:   Jose Luis Bravo                             *
;    Company:   Acad. Computación ICE                             *
;                                                                             * 
;******************************************************************************
;                                                                             *
;    Files required: P18F4550.INC                                             *
;                                                                             *
;******************************************************************************

	LIST P=18F4550, F=INHX32	;directive to define processor
	#include <P18F4550.INC>		;processor specific variable definitions

;******************************************************************************
;Configuration bits

	CONFIG PLLDIV   = 5         ;(20 MHz crystal on PICDEM FS USB board)
    CONFIG CPUDIV   = OSC1_PLL2	
    CONFIG USBDIV   = 2         ;Clock source from 96MHz PLL/2
    CONFIG FOSC     = HSPLL_HS
    CONFIG FCMEN    = OFF
    CONFIG IESO     = OFF
    CONFIG PWRT     = OFF
    CONFIG BOR      = ON
    CONFIG BORV     = 3
    CONFIG VREGEN   = ON		;USB Voltage Regulator
    config WDT      = OFF
    config WDTPS    = 32768
    config MCLRE    = ON
    config LPT1OSC  = OFF
    config PBADEN   = OFF		;NOTE: modifying this value here won't have an effect
        							  ;on the application.  See the top of the main() function.
        							  ;By default the RB4 I/O pin is used to detect if the
        							  ;firmware should enter the bootloader or the main application
        							  ;firmware after a reset.  In order to do this, it needs to
        							  ;configure RB4 as a digital input, thereby changing it from
        							  ;the reset value according to this configuration bit.
    config CCP2MX   = ON
    config STVREN   = ON
    config LVP      = OFF
    config ICPRT    = OFF       ; Dedicated In-Circuit Debug/Programming
    config XINST    = OFF       ; Extended Instruction Set
    config CP0      = OFF
    config CP1      = OFF
    config CP2      = OFF
    config CP3      = OFF
    config CPB      = OFF
    config CPD      = OFF
    config WRT0     = OFF
    config WRT1     = OFF
    config WRT2     = OFF
    config WRT3     = OFF
    config WRTB     = OFF       ; Boot Block Write Protection
    config WRTC     = OFF
    config WRTD     = OFF
    config EBTR0    = OFF
    config EBTR1    = OFF
    config EBTR2    = OFF
    config EBTR3    = OFF
    config EBTRB    = OFF
;******************************************************************************
; DEFINICION DE VARIABLES
; 


;******************************************************************************
; Reset vector
; Esta sección se ejecutara cuando ocurra un RESET.

RESET_VECTOR	ORG		0

		goto	INICIO		;go to start of main code

;******************************************************************************

;******************************************************************************
;Start of main program
; el PROGRAMA PRINCIPAL inicia aqui

	ORG		0x1000
INICIO				; *** main code goes here **
	call Cpuertos
loop call LEER
	call PWM
	goto loop
	
					; end of main	
;******************************************************************************
; Espacio para subrutinas
;******************************************************************************


Cpuertos:
	movlw 0x0f
	movwf ADCON1
	movlw 0x00
	movwf TRISA
	movlw 0xff
	movwf TRISB

	return

LEER:
	movf PORTB,0
	andlw 0xf0
	bz etq1
	movwf 0x00
	swapf 0x00,1
	movf 0x00,0
	sublw 0x10
	movwf 0x01

	return
	
etq1 bcf PORTA,1
	goto LEER

PWM:
	movf 0x00,0
	movwf 0x06
	bsf PORTA,1
	call Gtime
	movf 0x01,0
	movwf 0x06
	bcf PORTA,1
	call Gtime
	return
	
Gtime:
	movlw 0x08
	movwf 0x10
etq3 movlw 0xd1
	movwf 0x11
	movlw 0x20
	movwf 0x12
	call TMR0
	decf 0x06,1
	bz etq2
	goto etq3
	

etq2 return

TMR0:
	movf 0x10,0
	movwf T0CON
	movf 0x11,0
	movwf TMR0H
	movf 0x12,0
	movwf TMR0L
	bsf T0CON,7
etq4 btfss INTCON,2
	goto etq4
	bcf INTCON,2
	bcf T0CON,7
	return
    



;******************************************************************************
;Fin del programa
	END
