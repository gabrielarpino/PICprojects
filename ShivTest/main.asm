;*******************************************************************
; Keypad/LCD Test Code
; Assembler : mpasm.exe
; Linker    : mplink.exe
; Written By : Kevin Lam
;*******************************************************************

      list p=16f877               ; list directive to define processor
      #include <p16f877.inc>        ; processor specific variable definitions
      __CONFIG _CP_OFF & _WDT_OFF & _BODEN_ON & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _CPD_OFF & _LVP_OFF
      Errorlevel -302		    ; switches off message [302] Register in operand not in bank 0
      Errorlevel -305

      #include <lcd.inc>			   ;Import LCD control functions from lcd.asm
	#include <rtc_macros.inc>

	udata_shr

      
      
	cblock	0x30
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
		Temp
		Location1	;3B PC
		Location2
		Location3
		Location4
		Location5	
		Location6	;40 PC
		Location7
		Front1		;42 PC7
		Front2
		Front3
		Front4
		Front5
		Front6
		Front7
		Back1		; 49 PC
		Back2
		Back3
		Back4
		Back5
		Back6
		Back7		;50
		pr2temp 
		NumH
		NumL
		TenK
		Thou
		Hund
		Tens
		Ones
		distanceMoved
		isColumnThere
		isBinThere
		frontstickerValues
		backstickerValues
		binCounter
		countdown
		counter
		NumOfBins1			
		delay3
		LEDcounter0, LEDcounter1,LEDcounter2,LEDcounter3,LEDcounter4,LEDcounter5,LEDcounter7,LEDcounter8
		cm
		mm
		m
		distreg
		lcd_d1_2
		count_highs		;the number of consecutive high counter for the US sensor
		temp_2
		hour1,hour2,min1,min2,sec1,sec2
	endc	

	;Declare constants for pin assignments (LCD on PORTD)
		#define	RS 	PORTD,2
		#define	E 	PORTD,3
		#define	IR1	PORTA,0
		#define	IR2	PORTA,1
		#define	ColMotor PORTC,7
		#define	UST1	PORTD,0
		#define	US1E1	PORTA,2
		#define	UST2	PORTD,1
		#define	US1E2	PORTA,3
		#define	LED	PORTB,0
		#define	Std1	PORTC,5		    ;std stands for standard motor (not pwm)
		#define	Std2	PORTC,6		    ;Std1 is the negative one
		#define	Std1Backwards	PORTC,0
		#define	Std2Backwards	PORTC,7
		#define	SwitchWhite		PORTB,2	    ; or gated switch to stop motor motion
		#define	SwitchArm		PORTB,3
		#define	NOTPWMFWD		PORTC,2
		#define	NOTPWMBACK		PORTC,1
		#define	MAX_HIGHS	0x3		    ; number of consecutive highs we want to detect US
		#define	MAX_TICKS	d'70'		    ; number of ticks where it reaches 4 metres, max time to get back from 4 meters is 36 seconds
		;SHAFTIR is PORTA,4
		;PWMFWD is RC2
		;PWMBACK is RC1
		

         ORG       0x0000     ;RESET vector must always be at 0x00
         goto      init       ;Just jump to the main code section.

;*******************************************************************************
; MACROS
;*******************************************************************************
Key	 macro	value, subroutine
 	 swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
	 andlw		0x0F
	 xorlw		value
	 btfsc		STATUS, Z
	 call		subroutine
	endm

Rotation macro	value
 	 swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
	 andlw		0x0F
	 xorlw		value
	 btfsc		STATUS, Z
	 call		ShiftLeft
	endm
;***************************************
; Delay: ~160us macro
;***************************************
LCD_DELAY macro
	movlw   0xFF
	movwf   lcd_d1
	decfsz  lcd_d1,f
	goto    $-1
	endm
	
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
;***************************************
; Number to Printable
;***************************************
PrintNumber macro	number
	;movf	    number ,W
	movfw	    number
	call	    DectoChar
	call	    WR_DATA
	endm
;***************************************
; Store_Dist macro --> Stores current
;   distance and stores it in Bin_Dist_reg
;***************************************		
Put_Dist_In_Reg macro    Bin_Dist_reg
	lcall	Dist_Decoder
	movfw	cm
	movwf	Bin_Dist_reg
    endm
    
;***************************************
; Display_Dist macro --> Takes value in Bin_Dist_reg
;   and converts it into a BCD and prints it
;   To the LCD
;***************************************		
Display_Dist macro    Bin_Dist_reg
    MOV	    Bin_Dist_reg, temp  ; Bin_Dist_reg --> temp
    call    Distance_Display
    endm
;*********************************************************
;   Distance_Display		
;   input:	temp
;   Output:	LCD
;   desc:	Decodes the distance and prints to the LCD
;********************************************************* 	
Distance_Display
	call	Dist_Decoder	    ; outputs m and cm regs to display
	PrintNumber	m
	clrf	NumL
	clrf	NumH
	MOV	cm, NumL
	call	bin16_BCD
	PrintNumber     Tens	    ; Print cm
	PrintNumber     Ones
	return
;***************************************
; Number to Colour
;***************************************
PrintCol macro	    number
	movfw	    number
	call	    StickerColours
	call	    WR_DATA
	endm	

;***************************************
;	    US READ MACRO
;****************************************
ReadUltrasonic macro	trigger, trigger_bit, echo, echo_bit
    clrf	TMR1H
    clrf	TMR1L

    bcf	trigger, trigger_bit		;make sure trigger is clear
    call lcdLongDelay

    bsf	trigger, trigger_bit	;trigger high, bottom sensor
    call lcdLongDelay		;10us delay
    bcf	trigger, trigger_bit	;trigger low

    btfss	echo, echo_bit		;wait for echo to go high
    goto	$-1
    bsf	T1CON,TMR1ON	;turn on timer

    btfsc	echo, echo_bit		;wait for echo to go low
    goto	$-1

    bcf	T1CON,TMR1ON	;turn off timer
    endm
	
;*******************************************************
;******             BCD		        ****************
;*******************************************************
	
;---------------- Binary (16-bit) to BCD -----------------------
;
;bin8_BCD:	; --- Takes Binary.number in      NumL
;bin16_BCD:	; --- Takes Binary.number in NumH:NumL 
		; --> Returns decimal.form  in TenK:Thou:Hund:Tens:Ones
;
; Uses variables
; NumH, NumL
; TenK, Thou, Hund, Tens, Ones
		
;BCD	macro	NumH, NumL

bin16_BCD
        swapf   NumH, W
        andlw   0x0F
        addlw   0xF0
        movwf   Thou 
        addwf   Thou, F 
        addlw   0xE2 
        movwf   Hund 
        addlw   0x32 
        movwf   Ones 

        movf    NumH, W 
        andlw   0x0F 
        addwf   Hund, F 
        addwf   Hund, F 
        addwf   Ones, F 
        addlw   0xE9 
        movwf   Tens 
        addwf   Tens, F 
        addwf   Tens, F 

        swapf   NumL, W 
        andlw   0x0F 
        addwf   Tens, F 
        addwf   Ones, F 

        rlf    Tens, F 
        rlf    Ones, F 
        comf	Ones, F 
        rlf    Ones, F 

        movf    NumL, W 
        andlw   0x0F 
        addwf   Ones, F 
        rlf    Thou,F 

        movlw   0x07 
        movwf   TenK 

        movlw   0x0A
Lb1
        decf    Tens, F 
        addwf   Ones, F 
        btfss   STATUS, C 
        goto		Lb1 
Lb2
        decf    Hund, F 
        addwf   Tens, F 
        btfss   STATUS,C 
        goto		Lb2 
Lb3
        decf    Thou, F 
        addwf   Hund, F 
        btfss   STATUS,C
        goto		Lb3 
Lb4
        decf    TenK, F 
        addwf   Thou, F 
        btfss   STATUS,C 
        goto		Lb4 

        retlw   0
	return
;***************************************
; Initialize
;***************************************
init
         clrf      INTCON         ; No interrupts

         bsf       STATUS,RP0     ; select bank 1
	 movlw	    0xFF	    ; need to make A input
         movwf      TRISA          ; All port A is input
         movlw     b'11111110'    ; Set required keypad inputs
         movwf     TRISB
         clrf      TRISC          ; All port C is output
         clrf      TRISD          ; All port D is output
	 clrf	   TRISE	
	 
	          ;Set SDA and SCL to high-Z first as required for I2C
		 bsf	   TRISC,4		  
		 bsf	   TRISC,3

         bcf       STATUS,RP0     ; select bank 0
         clrf      PORTA
         clrf      PORTB
         clrf      PORTC
         clrf      PORTD
	 clrf	   PORTE
	 
	;Set up I2C for communication
		 
	call 	   i2c_common_setup
;*******************************************************************************
;	 UNCOMMENT IF YOU WANT TO CHANGE THE TIME
	rtc_resetAll					;works;\
	call set_rtc_time
;*******************************************************************************          
         call      InitLCD  	  ;Initialize the LCD (code in lcd.asm; imported by lcd.inc)
	 ;A/D converter attempt
	 	 
	; Set ADCON1 to use RA0 as analog input	
	bcf	STATUS,RP1
	bsf	STATUS,RP0	;Select bank 1
	movlw	b'00000110'	;left justified, all inputs digital
	movwf	ADCON1			;All digital input, reference voltage Vdd Vss

	;ADCON0
	bcf	STATUS,RP0	
;***************************************************
; Initialize variables and Displays
;****************************************************
	Display		Welcome_Msg
	call		Switch_Lines
	Display		Welcome_Msg2
	clrf		NumOfBins1
	clrf		distreg		    ; clear register for distance
	call		Init_TMR0	    ; Initialize shaft encoder
	bsf		PORTD,0
	bsf		LEDcounter0,0		    ;counter for LED
	bsf		LEDcounter1,0		    ;forward motor counters
	bsf		LEDcounter2,0
	bsf		LEDcounter3,0		    
	bsf		LEDcounter4,0		    ;backward motor counters
	bsf		LEDcounter5,0	    
	bsf		LEDcounter7,0		    ;pwm counters
	bsf		LEDcounter8,0		    
	
	clrf		hour1
	clrf		hour2
	clrf		min1
	clrf		min2
	clrf		sec1
	clrf		sec2
	
	bcf		Std1			    ;use this one for negative for when need to move full motor			
	bsf		Std2
	bcf		Std1Backwards
	bsf		Std2Backwards
	bsf		PORTC,2		    
	bsf		PORTC,1			; PWM now negative, so start by clearing
	
	;trying to fix the switch bs
	bcf		SwitchWhite
	bcf		SwitchArm

;*************************************
;	Keypad and LCD forms	    
;*************************************
	
KeypadandLCD	btfss		PORTB,1     ;Wait until data is available from the keypad
		goto		$-1 

		swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
		andlw		0x0F
		movwf		temp
		
		Key	0x00, TOTAL
		Key	0x01, DisplayBlackWhiteIR1
		Key	0x02, DisplayBlackWhiteIR2
		Rotation	0x03
		Key	0x04, EXIT
	    	Key	0x05, Read1_US
		Key	0x06, Read2_US
		Key	0x07, set_rtc_time
		Key	0x08, Stickers1
		Key	0x09, LEDControlON
		Key	0x0A, CHECKSWITCH		
		Key	0x0C, StdRotation2
		Key	0x0D, StdRotation2Backwards
		Key	0x0E, PWMFWD
		Key	0x0F, PWMBACK
		btfsc		PORTB,1     ;Wait until key is released
	        goto		$-1
		goto		KeypadandLCD
		
goback
		return	
	

;**************************************************************************************************************************************************
;								  MAIN CODE
;----------------------------------------------------------------------------------------------------------------------------------------------

AVOIDCOLUMN1
    
    bcf	    Std1		;motor will stop move fwd
    
    ;WORKSSSSSSSSSSSSSSSSSSSSSSSSSSSS
    bcf	    Std2		; start white thing fwd
    call    HalfS	
    btfsc   SwitchWhite
    goto    $-2
    call    lcdLongDelay
    btfss   SwitchWhite
    goto    $-2
    
    bsf	    Std2		   ; stop white thing fwd
    
     call    HalfS
    call    HalfS
     call    HalfS
    call    HalfS
    
    call    PWMFWD	    ;start arm forward
    call    HalfS	
    btfsc   SwitchArm
    goto    $-2
    call    lcdLongDelay
    btfss   SwitchArm
    goto    $-2
    
    call    PWMFWD	    ;stop arm forward
 
    call    HalfS
    
    return
    
;-------------------------------------------------
    
AVOIDCOLUMN2
    
    bsf	    Std1		;move forward until no more column in the way
    
    call    OneS
    call    OneS
    call    OneS
    call    OneS
    call    OneS
    ;wait to see the thing
    call    Read2_US		;checks to see if bin present
    movlw   0x8		; read the bin
    subwf   TMR1H
    btfsc   STATUS,C
    goto    $-4
    call    Read2_US		;checks to see if bin present
    movlw   0x6		; read the bin
    subwf   TMR1H
    btfss   STATUS,C
    goto    $-4
    call    RETURNFROMCOLUMN
    call    READBIN	
    goto    ENDTHIS

    
RETURNFROMCOLUMN
    
    bcf	    Std1
    
    call    PWMBACK	    ;start arm BACK
    call    HalfS
    call    PWMBACK
    
    call    PWMBACK	    ;start arm BACK
    call    HalfS
    btfss   SwitchArm
    goto    $-1
    
    call    PWMBACK	    ;stop arm BACK
        
    call    HalfS
    call    HalfS
    
    bcf	    Std2Backwards
    call    HalfS
    btfss   SwitchWhite
    goto    $-1 
    
    bsf	    Std2Backwards
    
    bsf	    Std1	    ; make it start moving right after column
    
    return
    
READBIN
    bsf	    LED
    bcf     Std1
    call    AddBin		;adds bin to list
    
    call    Dist_Decoder	; stores the distance
    Put_Dist_In_Reg	distreg
    Display_Dist	distreg
    call    StoreThisTick
    
    call    Clear_Display    
    
        
    call    DisplayBlackWhiteIR1	;warms up IR
    call    DisplayBlackWhiteIR1	;warms up IR  
    call    StoreBW1		;reads & stores first IR data
    
;*****THRESHHOLD TEST ON IR 1************
;    nothing in front: 02056
;    black in front: 10000-41000
;    white in front: >60000
    
    ;READ 2 ALGORITHM IMPLEMENT HERE
    
    call    HalfS

    call    Clear_Display
    
    call    DisplayBlackWhiteIR2	;warms up IR
    call    DisplayBlackWhiteIR2	;warms up IR
    call    StoreBW2
    
    call    HalfS
    
    bcf	    LED
    
    return
    
    ;goto    EXIT    
     
    
TOTAL
    
    call    Read1_US		;warms up us sensors that was weird
    call    Read2_US
   
TOTAL1
    
    bsf	    Std1		;moving
    clrf    count_highs		;reset the high value counter
    
COLREADINGSTART
    call    Read1_US		;checks to see if column present
    
;********THRESHOLD CALCULATION FOR COLUMN**************:
;   At around cm from col, reading: 02303
;	So, this will be the max value.
;	To be safe, will minus 02603 from the TMR variables.
;	This means, subtract 0x4 from TMR1H 
;	Value before at integration was 0x14, now trying 0x4
;	THE FOLLOWING CODE COUNTS 3 HIGHS
    
    movlw   0xF		; if column present, it'll move forward
    subwf   TMR1H
    btfsc   STATUS,C	    
    goto    $+8
    incf    count_highs
    movlw   MAX_HIGHS
    subwf   count_highs,W	    ; will always be negative UNTIL the high count is the one we want
    btfss   STATUS,Z		    ; if result is zero, Z bit is set.
    goto    COLREADINGSTART
    call    AVOIDCOLUMN1
    call    AVOIDCOLUMN2
    
    clrf    count_highs		;reset the high value counter
 
BINREADINGSTART
    
    bsf	    Std1		;gotta always be moving
    
    call    Read2_US		;checks to see if bin present
    
;********THRESHOLD CALCULATION FOR BIN**************:
;   At around 6cm from white bin, reading: 01103
;	So, this will be the max value.
;	To be safe, will minus 01210 from the TMR variables.
;	This means, subtract 0x4 from TMR1H 
;	Value before at integration was 0x14, now trying 0x4
;	CODE COUNTS 3 HIGHS

    
    movlw   0x8		; read the bin
    subwf   TMR1H
    btfsc   STATUS,C
    goto    $+8
    incf    count_highs
    call    lcdLongDelay
    movlw   MAX_HIGHS
    subwf   count_highs,W	    ; will always be negative UNTIL the high count is the one we want
    btfss   STATUS,Z		    ; if result is zero, Z bit is set.
    goto    BINREADINGSTART
    call    DELAYEDREAD

    goto    ENDTHIS   
    
DELAYEDREAD
    call    DELAY1		; do a 0.25s delay in order to move it forward a lil
    call    Read2_US		;checks to see if bin present
    movlw   0x6		; read the bin
    subwf   TMR1H
    btfss   STATUS,C
    goto    DELAYEDREAD
    call    READBIN
    bcf	    LED			; turn off LED After reading    
ENDTHIS       
    movlw	0X5		;checks if max of 7 bins has been reached
    subwf	NumOfBins1,W		
    btfss	STATUS,Z
    goto	ENDTHIS1
    bsf		Std1
    call	OneS
    call	OneS
    call	HalfS
    goto	EXIT
    
ENDTHIS1  
    
    movfw	TMR0			;checks if max ticks has been reached
    subwf	MAX_TICKS,W
    btfsc	STATUS,Z
    goto	EXIT
    
    goto	TOTAL1    
    
CHECKSWITCH
    ;WORKSSSSSSSSSSSSSSSSSSSSSSSSSSSS
    bcf	    Std2		; start white thing fwd
    call    HalfS	
    btfsc   SwitchWhite
    goto    $-2
    call    lcdLongDelay
    btfss   SwitchWhite
    goto    $-2
    
   
    bsf	    Std2		   ; stop white thing fwd
    
    return

EXIT
    
    bcf		Std1
    call	AVOIDCOLUMN1	 ;RETRACT ARM
    bsf		Std1Backwards
    
    ;add delays
    call	DELAY2

    bcf		Std1Backwards
    
    call	Clear_Display
    Display	FinalMessage     ;display final interface for choosing stuff
    call	Switch_Lines
    call	show_RTC
    
    goto	EXITDISPLAY
    


EXITDISPLAY	btfss		PORTB,1     ;Wait until data is available from the keypad
		goto		$-1 

		swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
		andlw		0x0F
		movwf		temp
		Key	0x00, Stickers1
		Key	0x01, ShowBins
		Key	0x02, Locations
		Rotation	0x03
		Key	0x04, show_RTC
		btfsc		PORTB,1     ;Wait until key is released
	        goto		$-1
		goto		EXITDISPLAY

    goto    EXIT
    
MoveBackwards
    
    call    PWMSTOP
    
    	
;-----------------------------------------------------------------------------------------------------		

;*********************************************************
; A to D conversion with LCD display for IR sensor
;*********************************************************
	
;******************************************
;	Sticker Print Modules
;*****************************************
	
BWScanModule1

	bsf	ADCON0,2		;start conversion and wait for it to complete
	btfsc	ADCON0,2		;LCD CONVERSION MODULE
	goto	$-1
	
	movf	ADRESH, W
	movwf	NumH
	movf	ADRESL, W
	movwf	NumL
	call	Clear_Display
	call	bin16_BCD
	PrintNumber	TenK
	PrintNumber	Thou
	PrintNumber	Hund
	PrintNumber     Tens
	PrintNumber     Ones
	return
BWStoreModule1
	BCF     STATUS, IRP
	movlw	0x42
	movwf	FSR
	movlw	0X0
	decf	NumOfBins1,W		;want bin number to be decreased when back checks it
	addwf	FSR,F
	movlw	0X9c			; roughly 40000
	subwf	NumH
	movlw	0x0
	btfsc	STATUS, C
	movlw	0x1
	movwf	INDF
	return	
	
BWScanModule2

	bsf	ADCON0,2		;start conversion and wait for it to complete
	btfsc	ADCON0,2		;LCD CONVERSION MODULE
	goto	$-1
	
	movf	ADRESH, W
	movwf	NumH
	movf	ADRESL, W
	movwf	NumL
	call	Clear_Display
	call	bin16_BCD
	PrintNumber	TenK
	PrintNumber	Thou
	PrintNumber	Hund
	PrintNumber     Tens
	PrintNumber     Ones
	return
BWStoreModule2
	BCF     STATUS, IRP
	movlw	0x49			;number 49 to try to get registers for back of bin
	movwf	FSR
	movlw	0X0
	decf	NumOfBins1,W
	addwf	FSR,F
	movlw	0XA			;2800 threshold
	subwf	NumH
	movlw	0x0
	btfsc	STATUS, C
	movlw	0x1
	movwf	INDF
	return	
	
;***************************************************
;	US Sensor Modules			    [TESTED]
;***************************************************
	
Read1_US		    
	 
	;call		Read1_US1 
	 
	ReadUltrasonic	UST1,US1E1
	
	call		Clear_Display
	
	movf		TMR1H, W
	movwf		NumH
	movf		TMR1L, W
	movwf		NumL
	
	call		bin16_BCD
	
	PrintNumber	TenK
	PrintNumber	Thou
;	PrintNumber	Hund
;	PrintNumber	Tens
;	PrintNumber	Ones
	call		Clear_Display
	return

Read2_US		    
	 
	;call		Read2_US1 
	
	ReadUltrasonic	UST2,US1E2
	 
	call		Clear_Display
	
	movf		TMR1H, W
	movwf		NumH
	movf		TMR1L, W
	movwf		NumL
	
	call		bin16_BCD
	
	PrintNumber	TenK
	PrintNumber	Thou
;	PrintNumber	Hund
;	PrintNumber	Tens
;	PrintNumber	Ones
	call		Clear_Display
	return
		
;*******************************************************
; Dist_Decoder
;   input:  temp (holds bin_dist_reg)
;   output: cm, mm, m
;   Desc:   Converts the count in the rotary encoder
;	    into physical distance
;*******************************************************
Dist_Decoder
	; Initalize all registers
	clrf	cm		
	clrf	mm
	clrf	m

	; Check if its already zero
	movfw	temp
	sublw	D'0'
	btfsc	STATUS, Z
	return
	
Decode_loop			; Assume each step is 3.6 cm
	; Perform decode
	ADDL	cm, cm, D'5'	    ;perimeter of 2*pi
	ADDL	mm, mm, D'9'
	decf	temp
	
Check_mm_overflow
	; Test if mm has overflowed! (mm >= 10)
	movlw	D'10'
	subwf	mm, w		; mm - 10 --> w (w = f - w, when d = 0) 
	btfsc	STATUS, C	; Y = mm, w = 10, Y-w
	goto	mm_overflow	; Run overflow routine if it did overflow
	
Check_cm_overflow	
	; Test if cm has overflowed! (cm >= 100)
	movfw	cm
	movwf	temp_2
	movlw	D'100'
	subwf	temp_2, w	; cm - 100 --> w (w = f - w, when d = 0) 
	btfsc	STATUS, C	; Y = cm, w = 100, Y-w
	goto	cm_overflow	; Run overflow routine if it did overflow

Decode_check_done	
	; Test if done decoding	
	movfw	temp
	sublw	D'0'		; MAKE THIS A LARGER NUMBER TO ACCOUNT FOR 
	btfss	STATUS, Z	; DIFFERENCE IN DISTANCE
	goto	Decode_loop
	return	
	
mm_overflow
	incf	cm		; cm + 1 --> cm
	movlw	D'10'		; 10 --> w
	subwf	mm, f		; mm - 10 --> mm (f = f - w, when d = 1) 
	goto	Decode_check_done	; Check cm overflow
	
cm_overflow
	incf	m		; m + 1 --> m
	movlw	D'100'		; 100 --> w
	subwf	cm, f		; cm - 100 --> cm (f = f - w, when d = 1) 
	goto	Decode_check_done	; Continue decoding
	
StoreThisTick
	BCF     STATUS, IRP
	movlw	0x3B			;number 56 to try to get location registers in cblock
	movwf	FSR
	movlw	0X0
	decf	NumOfBins1,W
	addwf	FSR,F
	movfw	TMR0		; poll encoder for current state
	movwf	temp
	movwf	INDF
	
	return
;*********************************************
; Keypad Modules
;*******************************************
	 

ShiftLeft
	movlw		b'00011000'		;Move to the left
	call		WR_INS
	return

	goto	KeypadandLCD
	
RTCDisplay
	call	show_RTC
	
	btfsc		PORTB,1     ;Wait until data is available from the keypad
	return
	btfsc		PORTB,1     ;Wait until data is available from the keypad
	return
	btfsc		PORTB,1     ;Wait until data is available from the keypad
	return
	btfsc		PORTB,1     ;Wait until data is available from the keypad
	return
	goto	    RTCDisplay
	
StoreBW1
	bcf	STATUS,RP0
	movlw	b'11000101'           	
	movwf	ADCON0			;choose RA0
	
	call	BWScanModule1	
	call	BWStoreModule1
	
	return				; why was there not a return here before?
	
StoreBW2
	bcf	STATUS,RP0
	movlw	b'11001101'           	
	movwf	ADCON0			;choose RA1
	
	call	BWScanModule2
	call	BWStoreModule2
	
	return
	
AddBin
	Call Clear_Display
	incf	NumOfBins1,F
	PrintNumber	NumOfBins1
	movlw	0X8			;checks if max of 7 bins has been reached
	subwf	NumOfBins1,W		
	btfsc	STATUS,Z
	goto	EXIT
	return
	
ShowBins
	call		Clear_Display
	PrintNumber	NumOfBins1
	return
	
Stickers1		    ;TESTED
	call 		Clear_Display
	movlw		0X0
	movwf		counter
	BCF             STATUS, IRP
	movlw		0x41
	movwf		FSR
	movfw		NumOfBins1
	movwf		countdown
	Display		FrontStickers
FrontLoop1	
	INCF		FSR,1
	incf		counter
	PrintNumber	counter
	Display		Colon
	movfw		INDF
	PrintCol	W
	Display		Spacee
	decfsz		countdown,F
	goto		FrontLoop1
	
	call		Switch_Lines
	
Stickers2			;TESTED
	
	movlw		0X0
	movwf		counter
	BCF             STATUS, IRP
	movlw		0x48
	movwf		FSR
	movfw		NumOfBins1
	movwf		countdown
	Display		BackStickers
FrontLoop2
	INCF		FSR,1
	incf		counter
	PrintNumber	counter
	Display		Colon
	movfw		INDF
	PrintCol	W
	Display		Spacee
	decfsz		countdown,F
	goto		FrontLoop2
	
	return
	
Locations			;TESTED
	call 		Clear_Display
	movlw		0X0
	movwf		counter
	BCF             STATUS, IRP
	movlw		0x3A
	movwf		FSR
	movfw		NumOfBins1
	movwf		countdown
	Display		Distances
	call		Switch_Lines
FrontLoop	
	INCF		FSR,1
	incf		counter
	;PrintNumber	counter
	Display		Colon
	Display_Dist	INDF
	Display		Spacee
	decfsz		countdown,F
	goto		FrontLoop
	
	return
	
LEDControlON
	
	btfsc	    LEDcounter0,0
	goto	    $+4
	bsf	    LED
	bsf	    LEDcounter0,0
	return
	
	
	bcf	    LED
	bcf	    LEDcounter0,0
	return
	
	
DisplayBlackWhiteIR1
	
	movlw	b'11000101'	;movlw	b'11000101' selects the clock and ra0 as the reader pin
	movwf	ADCON0	
		
	call		BWScanModule1	
	return
	
	
DisplayBlackWhiteIR2
	
	movlw	b'11001101'	;movlw	b'11000101' selects the clock and ra as the reader pin
	movwf	ADCON0	
	
	call		BWScanModule1	
	return

	
StdRotation1
	
	btfsc	    LEDcounter1,0
	goto	    $+4
	bcf		Std1
	bsf	    LEDcounter1,0
	return
	
	bsf		Std1
	bcf	    LEDcounter1,0
	return
	
StdRotation2

	btfsc	    LEDcounter2,0
	goto	    $+4
	
	bcf		Std2
	bsf	    LEDcounter2,0
	return
	
	btfss	    SwitchWhite
	bsf	    Std2
	
	bsf		Std2
	bcf	    LEDcounter2,0

	return
		
StdRotation1Backwards
	
	btfsc	    LEDcounter4,0
	goto	    $+4
	bsf		Std1Backwards
	bsf	    LEDcounter4,0
	return
	
	bcf		Std1Backwards
	bcf	    LEDcounter4,0
	return
	
StdRotation2Backwards

	btfsc	    LEDcounter5,0
	goto	    $+4
	bsf		Std2Backwards
	bsf	    LEDcounter5,0
	return
	
	btfss	    SwitchWhite
	bsf	    Std2
	
	bcf		Std2Backwards
	bcf	    LEDcounter5,0
	return

PWMFWD	
	
    ;************ FIRST PWM - Negative one******************************		    To stop it, clear CCP1RL and/or CCPR2L 
    
    ;save   pr2 value so that you can zero it after
    
    btfsc	    LEDcounter7,0
    goto	    $+4
    clrf	    CCP1CON
    bsf		    LEDcounter7,0
    return
    
    BANKSEL TRISC
    BCF	    TRISC, 2		;set CCP1 as output		;CCP1 is RC2 and CCP2 is RC1
    
    MOVF     CCP1CON,W		;set CCP1 as PWM
    ANDLW    0xF0
    IORLW    0x0C
    MOVWF    CCP1CON

    movfw   PR2
    movwf   pr2temp
    
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
    MOVLF   B'01101111', CCPR1L			; previous was 01101
    BSF	    CCP1CON, 3
    BSF	    CCP1CON, 2
    
    BSF     T2CON, TMR2ON	;and start the timer running
 
    bcf		    LEDcounter7,0
    
    return
    
PWMBACK
    
    ;************ SECOND PWM - Positive one ******************************
    
    btfsc	    LEDcounter8,0
    goto	    $+4
    clrf	    CCP2CON
    ;bcf		    PORTC,1
    bsf		    LEDcounter8,0
    return
    
 
    
    BANKSEL TRISC
    BCF	    TRISC, 1		;set CCP2 as output		;CCP1 is RC2 and CCP2 is RC1
    
    MOVF     CCP2CON,W		;set CCP2 as PWM
    ANDLW    0xF0
    IORLW    0x0C
    MOVWF    CCP2CON
    
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
    MOVLW   B'01101111'				; good speed for moving the arm
    MOVWF   CCPR2L	
    BSF	    CCP2CON, 3
    BSF	    CCP2CON, 2
    
    BSF     T2CON, TMR2ON	;and start the timer running
    BCF	    STATUS,RP0
    
    ;bsf		    PORTC,1
    
    bcf		    LEDcounter8,0
    
    return
    
PWMSTOP
    
    clrf   CCP1CON
    clrf   CCP2CON
    movfw   pr2temp
    movwf   PR2
    
    return
	
	
;***************************************
; LCD control
;***************************************
Switch_Lines
		movlw	B'11000000'
		call	WR_INS
		return

Clear_Display
		movlw	B'00000001'
		call	WR_INS
		return

;***************************************
; Delay 0.5s
;***************************************
HalfS	
	local	HalfS_0
      movlw 0x88
      movwf COUNTH
      movlw 0xBD
      movwf COUNTM
      movlw 0x03
      movwf COUNTL

HalfS_0
      decfsz COUNTH, f
      goto   $+2
      decfsz COUNTM, f
      goto   $+2
      decfsz COUNTL, f
      goto   HalfS_0

      goto $+1
      nop
      nop
		return
		
;***************************************
;***************************************
; Delay = 0.6 seconds
; Clock frequency = 4 MHz
DELAY1
			;599996 cycles
	movlw	0xD1
	movwf	lcd_d1
	movlw	0x4F
	movwf	lcd_d2
	movlw	0x02
	movwf	lcd_d1_2
Delay_0
	decfsz	lcd_d1, f
	goto	$+2
	decfsz	lcd_d2, f
	goto	$+2
	decfsz	lcd_d1_2, f
	goto	Delay_0

			;4 cycles
	return
	

DELAY2		;95 secs		;94999994 cycles
	movlw	0x63
	movwf	lcd_d1
	movlw	0x16
	movwf	lcd_d2
	movlw	0xD0
	movwf	lcd_d1_2
Delay_01
	decfsz	lcd_d1, f
	goto	$+2
	decfsz	lcd_d2, f
	goto	$+2
	decfsz	lcd_d1_2, f
	goto	Delay_01

			;1 cycle
	nop
	
	return
	
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
    
;************************************
; RTC Sublabels
;************************************
    
show_RTC
;		;clear LCD screen
;		movlw	b'00000001'
;		call	WR_INS
;
;		;Get year
;		movlw	"2"				;First line shows 20**/**/**
;		call	WR_DATA
;		movlw	"0"
;		call	WR_DATA
;		rtc_read	0x06		;Read Address 0x06 from DS1307---year
;		movfw	0x77
;		call	WR_DATA
;		movfw	0x78
;		call	WR_DATA
;
;		movlw	"/"
;		call	WR_DATA
;
;		;Get month
;		rtc_read	0x05		;Read Address 0x05 from DS1307---month
;		movfw	0x77
;		call	WR_DATA
;		movfw	0x78
;		call	WR_DATA
;
;		movlw	"/"
;		call	WR_DATA
;
;		;Get day
;		rtc_read	0x04		;Read Address 0x04 from DS1307---day
;		movfw	0x77
;		call	WR_DATA
;		movfw	0x78
;		call	WR_DATA
;
;		movlw	B'11000000'		;Next line displays (hour):(min):(sec) **:**:**
;		call	WR_INS
;		ONLY GONNA DISPLAY THE TIME HAHA
		;Get hour
		rtc_read	0x02		;Read Address 0x02 from DS1307---hour
		movfw	0x77
		call	WR_DATA
		movfw	0x78
		call	WR_DATA
		movlw			":"
		call	WR_DATA

		;Get minute		
		rtc_read	0x01		;Read Address 0x01 from DS1307---min
		movfw	0x77
		call	WR_DATA
		movfw	0x78
		call	WR_DATA		
		movlw			":"
		call	WR_DATA
		
		;Get seconds
		rtc_read	0x00		;Read Address 0x00 from DS1307---seconds
		movfw	0x77
		call	WR_DATA
		movfw	0x78
		call	WR_DATA
		
		call	OneS			;Delay for exactly one seconds and read DS1307 again
		return

;***************************************
; Setup RTC with time defined by user
;***************************************
set_rtc_time

		;rtc_resetAll	;reset rtc
		;works up to here

		;rtc_set	0x00,	B'10000000'
		;works up to here
		
		;set time 
;		rtc_set	0x06,	B'00010110'		; Year
;		;works up to here
;		rtc_set	0x05,	B'00000011'		; Month
;		rtc_set	0x04,	B'00100000'		; Date
		;works up to here
		;rtc_set	0x03,	B'00100010'		; Day
		;stops working on this one^
		;seems like day is the one that crashes it
		rtc_set	0x02,	B'00000000'		; Hours
		;works on this one
		rtc_set	0x01,	B'00000000'		; Minutes
		;works on this one
		rtc_set	0x00,	B'00000000'		; Seconds
		;works on this one
		return

;***************************************
; Delay 1s
;***************************************
OneS
		local	OneS_0
      movlw 0x10
      movwf COUNTH
      movlw 0x7A
      movwf COUNTM
      movlw 0x06
      movwf COUNTL

OneS_0
      decfsz COUNTH, f
      goto   $+2
      decfsz COUNTM, f
      goto   $+2
      decfsz COUNTL, f
      goto   OneS_0

      goto $+1
      nop
      nop
		return
		
lcdLongDelay
    movlw d'20'
    movwf lcd_d2
LLD_LOOP
    LCD_DELAY
    decfsz lcd_d2,f
    goto LLD_LOOP
    return
    
;***************************************
; Look up table
;***************************************
    
Welcome_Msg
	
	;Change Page
	movwf	Temp
	movlw	HIGH Welcome_MsgEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW Welcome_MsgEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
        addwf    PCL,F
Welcome_MsgEntries
        dt        "1:Simulation|#/D:PWM|*/0:Std1&Std2|3:B/W", 0
	
Welcome_Msg2
	
	;Change Page
	movwf	Temp
	movlw	HIGH Welcome_Msg2Entries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW Welcome_Msg2Entries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
        addwf    PCL,F
Welcome_Msg2Entries
        dt        "5Distance 6Locations", 0
        
FinalMessage
	
	;Change Page
	movwf	 Temp
	movlw	HIGH FinalMessageEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW FinalMessageEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL
        addwf    PCL,F
FinalMessageEntries
        dt        "1Stickers 2Bins 3Locations",0
	
DectoChar
	
	;Change Page
	movwf	Temp
	movlw	HIGH DectoCharEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW DectoCharEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
        addwf    PCL,F
DectoCharEntries
        dt        "0123456789",  0
	
StickerColours
	
	;Change Page
	movwf	Temp
	movlw	HIGH StickerColoursEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW StickerColoursEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
        addwf    PCL,F
StickerColoursEntries
        dt        "BW",  0
	
FrontStickers

	;Change Page
	movwf	Temp
	movlw	HIGH FrontStickersEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW FrontStickersEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
        addwf    PCL,F
FrontStickersEntries
        dt        "Front",  0
	
BackStickers

	;Change Page
	movwf	Temp
	movlw	HIGH BackStickersEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW BackStickersEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
        addwf    PCL,F
BackStickersEntries
        dt        "Back",  0
	
Spacee

	;Change Page
	movwf	Temp
	movlw	HIGH SpaceeEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW SpaceeEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
        addwf    PCL,F
SpaceeEntries
        dt        " ",  0
	
Colon

	;Change Page
	movwf	Temp
	movlw	HIGH ColonEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW ColonEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
        addwf    PCL,F
ColonEntries
        dt        ":",  0
	
Distances

	;Change Page
	movwf	Temp
	movlw	HIGH DistancesEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW DistancesEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
        addwf    PCL,F
DistancesEntries
        dt        "Dist (cm)",  0
	    
	END
