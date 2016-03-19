    list p=16f877                 ; list directive to define processor
      #include <p16f877.inc>        ; processor specific variable definitions
      __CONFIG _CP_OFF & _WDT_OFF & _BODEN_ON & _PWRTE_OFF & _HS_OSC & _CPD_OFF & _LVP_OFF


	cblock	0x70
		COUNTH
		COUNTM	
		COUNTL	
		Table_Counter
		lcd_tmp	
		lcd_d1
		lcd_d2
		com	
		dat
		temp
		table_tmp
		table_entries
	endc	
;*******************************************************************************
; MACROS
;*******************************************************************************
	
;***************************************
; BANK0 macro	[TESTED]
;***************************************
BANK0 Macro
    bcf STATUS,RP0 
    bcf STATUS,RP1
    endm
;***************************************
; BANK1 macro	[TESTED]
;***************************************
BANK1 Macro
    bsf STATUS,RP0 
    bcf STATUS,RP1
    endm
	
;***************************************
; BANK2 macro	[TESTED]
;***************************************
BANK2 Macro
    bsf STATUS,RP0 
    bsf STATUS,RP1
    endm
    
;***************************************
; BANK3 macro	[TESTED]
;***************************************
BANK3 Macro
    bcf STATUS,RP0 
    bsf STATUS,RP1
    endm
    
;***************************************
; MOVLF	macro	[TESTED]
;***************************************
MOVLF	Macro	literal, reg
    MOVLW  literal	; move literal into working register
    MOVWF   reg		; move working register into reg
    endm

;***************************************
; MOV	macro  reg2 <- reg1		[TESTED]	
;***************************************
MOV	Macro	reg1, reg2
    MOVF  reg1,W	; move reg1 into working register
    MOVWF   reg2	; reg2 <- reg1
    endm
    
;***************************************
; ADDL macro, Adds literal and a reg	[TESTED]
;***************************************
ADDL	Macro	Destination, reg, literal
    MOVLW  literal	; move literal into working register
    ADDWF   reg,W	; W <- literal + reg
    MOVWF   Destination	; Destination <- literal + reg
    endm
    
;***************************************
; ADD macro, Adds two registers together    [TESTED]
;***************************************
ADD	Macro	Destination, reg1, reg2
    MOVF    reg1,W	; move literal into working register
    ADDWF   reg2,W	; W <- reg1 + reg2
    MOVWF   Destination	; Destination <- reg1 + reg2
    endm

    
         ORG       0x00000000     ;RESET vector must always be at 0x00
         goto      init_PIN       ;Just jump to the main code section.
           
;***************************************
; Initialize
;***************************************
init_PIN
         clrf      INTCON         ; No interrupts
	 
	 ; Set input and output types
         bsf       STATUS,RP0     ; select bank 1
         clrf      TRISA          ; All port A is output
	 BSF	   TRISA, 4	  ; Set Rotary Encoder input pin
         clrf	   TRISB
         clrf      TRISC          ; All port C is output
	 BSF	   TRISC, 0	  ; Set Rotary Encoder input pin
	 BSF	   TRISC, 1	  ; Set Rotary Encoder input pin
         clrf      TRISD          ; All port D is output

	 ; Set output values
         bcf       STATUS,RP0     ; select bank 0
         clrf      PORTA
	 clrf      PORTB
         clrf      PORTC
         clrf      PORTD

    call Init_TMR0	 
	 
Poll_Timer1
    movfw	TMR0
    sublw	5		; Skip if 
    btfsc	STATUS, Z	; WREG is zero
    call	Set_LED
    call	Poll_Timer1

Set_LED
    BSF PORTA, 0
    call STOP
  
STOP	goto STOP

;***************************************
;Init_TMR0
;   Input:  RA4
;   Output: TMR0 register
;   desc:   Counts the clock pulses for the
;	    rotary encoder
;****************************************
Init_TMR0			    ; INITIALIZE TIMER 0   
    BANKSEL OPTION_REG
    MOVLF   B'00101000', OPTION_REG
    ;	    B'0xxxxxxx'	    RBPU - Set pull ups (0-enabled, 1-disabled)
    ;	    B'x1xxxxxx'	    INTEDG - Interupt on rising edge
    ;	    B'xx1xxxxx'	    T0CS - External clock on Pin RA4
    ;	    B'xxx0xxxx'	    T0SE - 0-Increment on low-to-high transition on T0CKI pin
    ;	    B'xxxx1xxx'	    PSA - Prescaler assigned to WDT
    ;	    B'xxxxx0xx'	    PS2 - Prescaler of 1:1
    ;	    B'xxxxxx0x'	    PS1 - Prescaler of 1:1
    ;	    B'xxxxxxx0'	    PS0 - Prescaler of 1:1
    BANKSEL TMR0
    CLRF   TMR0			    ; Clear timer 0
    return   
    
	END
