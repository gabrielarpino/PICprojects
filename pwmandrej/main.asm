list p=16f877A                 ; list directive to define processor
#include <p16f877A.inc>        ; processor specific variable definitions
    __CONFIG _CP_OFF & _WDT_OFF & _BODEN_ON & _PWRTE_OFF & _HS_OSC  & _CPD_OFF & _LVP_OFF
    
    cblock	0x70
		COUNTH
		COUNTM	
		COUNTL
		d1
		d2
		d1_2
		pr2temp
    endc
    
         ORG       0x00000000     ;RESET vector must always be at 0x00
         goto      Main       ;Just jump to the main code section.

	 
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
    
;*******************************************************************************
; MAIN
;******************************************************************************* 
    
;***************************************** PWM ********************************
Main
  
;************ FIRST PWM ******************************		    To stop it, clear CCP1RL and/or CCPR2L 
    
    BANKSEL TRISC
    BCF	    TRISC, 2		;set CCP1 as output		;CCP1 is RC2 and CCP2 is RC1
    
    MOVF     CCP1CON,W		;set CCP1 as PWM
    ANDLW    0xF0
    IORLW    0x0C
    MOVWF    CCP1CON
    
    ;11000011
    MOVLW    b'11000011'	;set highest PWM value
    BANKSEL  PR2		
    MOVWF    PR2		
    BANKSEL  TMR2		
    
    BSF	    T2CON,T2CKPS1	;set prescaler to 16
    
    CLRF    CCPR1L		;set PWM to zero
    
    BSF     T2CON, TOUTPS3	; Set Postscale to 16
    BSF     T2CON, TOUTPS2
    BSF     T2CON, TOUTPS1
    BSF     T2CON, TOUTPS0
    
    ; SET PWM duty cycle
    ;01001110
    BSF	    CCP1CON, 5				; change 1s here to 2 to get two pwm bro
    BSF	    CCP1CON, 4
    MOVLF   B'01101', CCPR1L	
    BSF	    CCP1CON, 3
    BSF	    CCP1CON, 2
    
    BSF     T2CON, TMR2ON	;and start the timer running
    
;************ SECOND PWM ****************************** 
    
    BANKSEL TRISC
    BCF	    TRISC, 1		;set CCP2 as output		;CCP1 is RC2 and CCP2 is RC1
    
    MOVF     CCP2CON,W		;set CCP2 as PWM
    ANDLW    0xF0
    IORLW    0x0C
    MOVWF    CCP2CON
    
    ; save pr2 values
    
    movfw   PR2
    movwf   pr2temp
    
    
    ;11000011
    MOVLW    b'11000011'	;set highest PWM value
    BANKSEL  PR2	
    MOVWF    PR2
    BANKSEL  TMR2		
    
    BSF	    T2CON,T2CKPS1	;set prescaler to 16
    
    CLRF    CCPR2L		;set PWM to zero
    
    BSF     T2CON, TOUTPS3	; Set Postscale to 16
    BSF     T2CON, TOUTPS2
    BSF     T2CON, TOUTPS1
    BSF     T2CON, TOUTPS0
    
    ; SET PWM duty cycle
    ;01001110
    BSF	    CCP2CON, 5				; change 1s here to 2 to get two pwm bro
    BSF	    CCP2CON, 4
    MOVLF   B'01100001', CCPR2L	
    BSF	    CCP2CON, 3
    BSF	    CCP2CON, 2
    
    BSF     T2CON, TMR2ON	;and start the timer running
    
Delay
    
	    ; Delay = 2 seconds
; Clock frequency = 4 MHz

; Actual delay = 2 seconds = 2000000 cycles
; Error = 0 %
			;1999996 cycles
	movlw	0x11
	movwf	d1
	movlw	0x5D
	movwf	d2
	movlw	0x05
	movwf	d1_2
Delay_0
	decfsz	d1, f
	goto	$+2
	decfsz	d2, f
	goto	$+2
	decfsz	d1_2, f
	goto	Delay_0

			;4 cycles (including call)
	 
    
    clrf   CCPR1L
    clrf  CCPR2L
    movfw   pr2temp
    movwf   PR2
    
    
STOP	call	STOP
    
    

    
END