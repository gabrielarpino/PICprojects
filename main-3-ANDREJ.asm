      list p=16f877                 ; list directive to define processor
      #include <p16f877.inc>        ; processor specific variable definitions
      __CONFIG _CP_OFF & _WDT_OFF & _BODEN_ON & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _CPD_OFF & _LVP_OFF


#include <lcd.inc>			   ;Import LCD control functions from lcd.asm
#include <rtc_macros.inc>
      
	cblock	0x70
	    Table_Counter
	    Temp
	    COUNTH
	    COUNTM	
	    COUNTL	
	endc		
	
	;Declare constants for pin assignments (LCD on PORTD)
		#define	RS		PORTD, 2
		#define	E		PORTD, 3
		#define CLRSENSORF	PORTA, 2	
		#define CLRSENSORB	PORTE, 2
		#define ENCODER		PORTA, 4
		#define POLL		PORTA, 0
		#define BWDM1		PORTA, 5
		#define BWDM2		PORTC, 5
		#define FWDM1		PORTC, 6
		#define FWDM2		PORTC, 7
		#define ARMIN		PORTE, 0
		#define ARMOUT		PORTE, 1
		#define BIN		PORTA, 1
         ORG       0x0000     ;RESET vector must always be at 0x00
         goto      init       ;Just jump to the main code section.		
		
;***************************************
; Display macro
;***************************************
Display macro	Message
		local	loop_
		local 	end_
		clrf	Table_Counter
		clrw		
loop_	movf	Table_Counter,W
		call 	Message
		xorlw	B'00000000' ;check WORK reg to see if 0 is returned
		btfsc	STATUS,Z
			goto	end_
		call	WR_DATA
		incf	Table_Counter,F
		goto	loop_
end_
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
; MOVLF	macro	[TESTED]
;***************************************
MOVLF	Macro	literal, reg
    MOVLW  literal	; move literal into working register
    MOVWF   reg		; move working register into reg
    endm	
    
;***************************************
; Initialize LCD
;***************************************
init

         clrf      INTCON         ; No interrupts
	 
	 bsf       STATUS,RP0     ; select bank 1
	 clrf      TRISA          ; All port A is output
         movlw     b'11110010'    ; Set required keypad inputs
         movwf     TRISB    
         clrf      TRISC          ; All port C is output
         clrf      TRISD          ; All port D is output
	 clrf	   TRISE	  ; All port E is output  
         ;Set SDA and SCL to high-Z first as required for I2C
	 bsf	   TRISC, 4		  
	 bsf	   TRISC, 3	
	 ;set inputs for sensors
	 bcf	   TRISA, 2	  ; front colour sensor
	 bsf	   TRISA, 4	  ; rotary encoder
	 bsf	   TRISA, 0	  ; poll distance sensor
	 bsf	   TRISA, 1	  ; bin distance sensor
	 bsf	   TRISE, 2	  ; back colour sensor
	 bcf       STATUS,RP0     ; select bank 0
	 
	 call 	   i2c_common_setup
	 call      InitLCD  	 
	 bcf       STATUS,RP0     ; select bank 0
;***************************************
;Checking key
;***************************************
Key	 macro	value, subroutine
 	 swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
	 andlw		0x0F
	 xorlw		value
	 btfsc		STATUS, Z
	 call		subroutine
	endm
	
;***************************************
;PCLATH for tables
;***************************************
tablepclath	macro	name
		movwf Temp
		movlw HIGH name
		movwf PCLATH
		movf	Temp,w
		addlw LOW name
		btfsc STATUS, C
		incf PCLATH, f
		movwf PCL
		addwf	PCL,F
		name
	endm		 
	
;***************************************
; Main code
;***************************************	
	bcf       STATUS,RP1     ; select bank 1
	bsf       STATUS,RP0     
	
	; SET RA2 AS OUTPUT
	bcf	   TRISA, 2	  ; front colour sensor

	
	bcf       STATUS,RP1     ; select bank 0	
	bcf       STATUS,RP0 

	Display	    welcome2	   ; Display "Press A"


	bsf	    PORTA, 2	   ; Set RA2 HIGH
	btfsc	    PORTA, 2	   ; SKIP IF RA2 IS LOW
	goto finished	
	Display		welcome1	; “Welcome”
finished goto finished
	

; ANOTHER FORM

;Display	    welcome2	   ; Display "Press A"


;	bcf	    PORTA, 2	   ; Set RA2 HIGH
;	btfss	    PORTA, 2	   ; SKIP IF RA2 IS LOW
;	goto finished	
;	Display		welcome1
;finished goto finished


; ANOTHER FORM

;Display	    welcome2	   ; Display "Press A"


;	bcf	    PORTA, 2	   ; Set RA2 HIGH
;	btfss	    PORTA, 2	   ; SKIP IF RA2 IS LOW
;	goto 	finished	
;	goto	here
;finished goto finished
;here	Display		welcome1
;	goto	finished

;***************************************
;  Look up table
;***************************************	
welcome1   
	tablepclath	wel1
	dt		"Welcome", 0
welcome2 
	tablepclath	wel2
	dt		"Press A", 0	

;***************************************
; Delay 0.5s
;***************************************
HalfS	
	local		HalfS_0
	movlw		0x88
	movwf		COUNTH
	movlw		0xBD
	movwf		COUNTM
	movlw		0x03
	movwf		COUNTL

HalfS_0
	decfsz		COUNTH, f
	goto		$+2
	decfsz		COUNTM, f
	goto		$+2
	decfsz		COUNTL, f
	goto		HalfS_0
	goto		$+1
	nop
	nop
	return	
	
	
;***************************************
; LCD control
;***************************************
Switch_Lines
	movlw		B'11000000'
	call		WR_INS
	return

Clear_Display
	movlw		B'00000001'
	call		WR_INS
	return	
	
END