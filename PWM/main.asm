        list P=PIC16F877,   F=INHX8M,	C=160,	N=80,	ST=OFF,	MM=OFF,	R=DEC
	#include	<p16f877.inc>
        __config _CP_OFF & _WDT_OFF & _BODEN_OFF & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _LVP_OFF & _DEBUG_OFF & _CPD_OFF 
        errorlevel-302

    ORG		0
    GOTO	INIT
INIT	BSF	STATUS, RP0 ;select bank 1
	BCF	INTCON, GIE ;disable global interrupt
	BCF	INTCON, PEIE
	MOVLW	B'00000001' ;configure PR2
	MOVWF	PR2
	BCF	STATUS, RP0 ;select bank 0
	MOVLW	B'00000100' ;configure CCPR1L
	MOVWF	CCPR1L	
	MOVLW	B'00011100' ; only works if 3rd bit is set to 1 for some reason
	MOVWF	CCP1CON
	BSF	STATUS,RP0  ;select bank 1
	CLRF	TRISC	    ;configure PORTC as output
	BCF	STATUS,RP0  ;select bank 0
	MOVLW	B'00000100' ;configure T2CON
	MOVWF	T2CON
	
ENDLP	GOTO ENDLP	    ;endless looop
	
	END




