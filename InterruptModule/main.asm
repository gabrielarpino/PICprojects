
      list p=16f877                 ; list directive to define processor
      #include <p16f877.inc>        ; processor specific variable definitions
      __CONFIG _CP_OFF & _WDT_OFF & _BODEN_ON & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _CPD_OFF & _LVP_OFF     



	ORG 0000

     GOTO MAIN

     ORG 0004

     GOTO INT_SERV
     

	
INT_SERV
	;btfsc INTCON, INTF
	BSF STATUS, RP0
	bsf	PORTE,0
	BCF STATUS, RP0
     BCF INTCON, INTF    ; clear the appropriate flag
     RETFIE 
 
MAIN 
     
          bsf       STATUS,RP0     ; select bank 1
         clrf      TRISA          ; All port A is output
	 clrf	    TRISB
         clrf      TRISC          ; All port C is output
         clrf      TRISD          ; All port D is output
	 

         bcf       STATUS,RP0     ; select bank 0
         clrf      PORTA
         clrf      PORTB
         clrf      PORTC
         clrf      PORTD

	 bsf	PORTA,4
	 bsf	PORTA,5

     BSF STATUS, RP0     ; bank 1
     MOVLW 1
     MOVWF TRISB
     BCF STATUS, RP0 
	
     BSF OPTION_REG, INTEDG   ; interrupt on positive
     BCF INTCON, INTF    ; clear interrupt flag
     BSF INTCON, INTE    ; mask for external interrupts
     BSF INTCON, GIE     ; enable interrupts
     GOTO MAIN
     




END  