;*******************************************************************
; Keypad/LCD Test Code
; Assembler : mpasm.exe
; Linker    : mplink.exe
; Written By : Kevin Lam
;*******************************************************************

      list p=16f877                ; list directive to define processor
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
		Location1
		Location2
		Location3
		Location4
		Location5
		Location6
		Location7
		Front1		;51 PC
		Front2
		Front3
		Front4
		Front5
		Front6
		Front7
		Back1		;58 PC
		Back2
		Back3
		Back4
		Back5
		Back6
		Back7
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
		NumOfBins
		delay3
	endc	

	;Declare constants for pin assignments (LCD on PORTD)
		#define	RS 	PORTD,2
		#define	E 	PORTD,3
		#define	IR1	PORTA,0
		#define	IR2	PORTA,1
		#define	US	PORTA,2
		#define backwardsDCMotor	PORTC,5
		#define	DCMotor	PORTC,6
		#define	ColMotor PORTC,7
		#define	UST1	PORTC,2
		#define	US1E1	PORTA,4
		#define	UST2	PORTC,3
		#define	US1E2	PORTA,5
		
		
;distanceMoved	equ	b'0'

         ORG       0x0000     ;RESET vector must always be at 0x00
         goto      init       ;Just jump to the main code section.
	 
	 ;ORG	   0x100	;this command is sketchy
	 
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
; Number to Colour
;***************************************
PrintCol macro	    number
	movfw	    number
	call	    StickerColours
	call	    WR_DATA
	endm	

PrintYN macro	    number
	movfw	    number
	call	    ColumnValues
	call	    WR_DATA
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
;endm

;***************************************
; Initialize
;***************************************
init
         clrf      INTCON         ; No interrupts

         bsf       STATUS,RP0     ; select bank 1
         clrf      TRISA          ; All port A is output
         movlw     b'11110010'    ; Set required keypad inputs
         movwf     TRISB
         clrf      TRISC          ; All port C is output
         clrf      TRISD          ; All port D is output
	 
	          ;Set SDA and SCL to high-Z first as required for I2C
		 bsf	   TRISC,4		  
		 bsf	   TRISC,3

         bcf       STATUS,RP0     ; select bank 0
         clrf      PORTA
         clrf      PORTB
         clrf      PORTC
         clrf      PORTD
	 
	;Set up I2C for communication
		 
	call 	   i2c_common_setup
;*******************************************************************************
	; UNCOMMENT IF YOU WANT TO CHANGE THE TIME
	;rtc_resetAll
	;call set_rtc_time
;*******************************************************************************
		 
		 ;Used to set up time in RTC, load to the PIC when RTC is used for the first time
		 ;call	   set_rtc_time
          
         call      InitLCD  	  ;Initialize the LCD (code in lcd.asm; imported by lcd.inc)
	  ;A/D converter attempt
	 
	 bcf	STATUS,RP1
	 bsf	STATUS,RP0	;Select bank 1
	 
	 bsf	IR1		;makes RA0 an input
	 bcf	STATUS,RP0	; go back to bank 0
	 
	;bsf	PORTC,2		; gives RC2 5 volts
	 
	; Set ADCON1 to use RA0 as analog input
	
	
	bsf	STATUS,RP0	;Select bank 1
	movlw	b'00000110'	;left justified, all inputs digital
	movwf	ADCON1			;All digital input, reference voltage Vdd Vss

	;ADCON0
	bcf	STATUS,RP0
	movlw	b'11000101'	;movlw	b'11000101' selects the clock and ra0 as the reader pin
	movwf	ADCON0
	
;***************************************************
; Initialize variables and Displays
;****************************************************	
	Display		Welcome_Msg
	call		Switch_Lines
	Display		Welcome_Msg2
	bsf	    PORTD,0
	movlw		0X00
	movwf		NumOfBins   ;shivs bin counter init
	


;*************************************
;	Keypad and LCD forms	    
;*************************************
	
	
	
KeypadandLCD	btfss		PORTB,1     ;Wait until data is available from the keypad
		goto		$-1 

		swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
		andlw		0x0F
		movwf		temp
		;Key	0x00, OperationDisplay
		Key	0x00, StartInit
		Key	0x01, pollColumnSensor		;B/W
		Key	0x02, ReadBWIR1
		Rotation	0x03
		Key	0x04, DisplayUSSensor1
	    	Key	0x05, AddBin
		Key	0x06, Stickers
		Key	0x07, LCDConversionFront
		Key	0x08, pollColumnSensor
		;Key	0x09, 
		Key	0x0E, PWMSTART
		Key	0x0F, PWMSTOP
		btfsc		PORTB,1     ;Wait until key is released
	        goto		$-1
		goto		KeypadandLCD
		
goback
		return	
	

;**************************************************************************************************************************************************
;								  MAIN CODE
;----------------------------------------------------------------------------------------------------------------------------------------------
StartInit
		
; WAIT FOR INITIAL INPUT
	movlw	b'00'
	movwf	binCounter	;initialize bincounter with 0
	movwf	distanceMoved	;initialize distance moved with 0
	movlw		0X00
	movwf		NumOfBins   ;shivs bin counter init

;    goto    getTimeRTC	    ; gets time from RTC clock

StartMove
;   MOTOR ON
    
    bsf	    DCMotor
    ADDL    distanceMoved, distanceMoved, b'1'	    ;increase distance moved
    
    movlw		0x4			;checking whether distance moved is 4
    subwf		distanceMoved
    movlw		0x0
    btfsc		STATUS,C
    movlw		0x1
    btfsc		W,0
;    btfsc		distanceMoved,3		;count seven
    call		MoveBackwards

;    bsf	    DCMotor		    ; begin forward movement, set appropriate bit to 1
;    movlw   b'1'
;    addwf  distanceMoved,f     ; increments distanceMoved by 1 (will experimentally find binary value for 4 m)
    
    
    
    
;;   CHECK COLUMN

    ;call    ChooseIR1		;selects IR1 for data conversion
    call    DisplayUSSensor1	;checks to see if column present
    btfss   isColumnThere,0
    goto    StartMove
    bcf	    DCMotor		;stops motor from moving forward
    call    HalfS
    bsf	    ColMotor		;starts motor to move arm
    call    Clear_Display
    Display OperationTime
    call    HalfS
    call    HalfS
    call    HalfS		;emulates arm movement
    bcf	    ColMotor		;turns off motor to move arm
    goto    StartMove
    
;   CHECK IF AT THE END OF BUCKET Line
    

    
    
;    call    pollUSsensor	; call ultrasonic sensor
;    btfsc   nothingAhead,0	;   checks if there is nothing ahead
;    goto    Finalize
    
;;   CHECK IF CONTAINER IS THERE
;    
;    btfss   withinDetectionRange,0  ; checks whether container ahead is within detection range
;    goto    Main
;   call start arm motor
;   call other motor
;    call    frontscanContainer	    ; label to begin container scanning
;    call    backscanContainer	    ; label to begin container scanning
;    movlw  b'1'
;    addwf   frontbinCounter,f	    ;increment the bin counters
;    addwf   backbinCounter,f
    goto    Finalize
    
MoveBackwards
    
    bcf	    DCMotor
    bsf	    backwardsDCMotor
    
    goto    Finalize
    
    
    


		
;-----------------------------------------------------------------------------------------------------		

;*********************************************************
; A to D conversion with LCD display for IR sensor
;*********************************************************

LCDConversion
	;bsf	PORTC,2		; gives RC2 5 volts	
		
	bsf	ADCON0,2		;start conversion and wait for it to complete
	btfsc	ADCON0,2		;LCD CONVERSION MODULE
	goto	$-1
	
	movf	ADRESH, W
	movwf	NumH
	movf	ADRESL, W
	movwf	NumL
	
	call		bin16_BCD
	call		ClrLCD
	bcf		STATUS, IRP
	movlw		0x43
	movfw		FSR
	movlw		0x0
	decfsz		binCounter, W
	goto	$+2
	return
	
	addwf		FSR,F
	movlw		0x3
	subwf		TenK
	movlw		0x0
	btfsc		STATUS,C
	movlw		0x1
	movwf		INDF
	
	PrintNumber	TenK
	PrintNumber	Thou
	PrintNumber	Hund
	PrintNumber     Tens
	PrintNumber     Ones
	call		HalfS
	call		HalfS
	;bcf	PORTC,2		; gives RC2 5 volts
	return
	
;*** Shiv lcdconvfront ***
	
LCDConversionFront
	bcf	STATUS,RP0
	movlw	b'00010101'             ;RA0
	
	movwf	ADCON0			;RA1
	bsf	ADCON0,2		;start conversion and wait for it to complete
	btfsc	ADCON0,2		;LCD CONVERSION MODULE
	goto	$-1
	
	movf	ADRESH, W
	movwf	NumH
	movf	ADRESL, W
	movwf	NumL
	call	Clear_Display
	Call	bin16_BCD
	PrintNumber	TenK
	PrintNumber	Thou
	PrintNumber	Hund
	PrintNumber     Tens
	PrintNumber     Ones
	BCF     STATUS, IRP
	movlw	0x42
	movwf	FSR
	movlw	0X0
	decf	NumOfBins,W
	addwf	FSR,F
	movlw	0X3
	subwf	TenK
	movlw	0x0
	btfsc	STATUS, C
	movlw	0x1
	movwf	INDF
	return	

;****************************
;	Choose IR1
;****************************
	
ChooseIR1
	
	bsf	IR1		;makes RA0 an input
	movlw	b'11000101'	;movlw	b'11000101' selects the clock and ra0 as the reader pin
	movwf	ADCON0
	
	return
	
;****************************
;	Choose IR2
;****************************
	
ChooseIR2
	
	bsf	IR2		;makes RA0 an input
	movlw	b'11001101'	;movlw	b'11000101' selects the clock and ra0 as the reader pin
	movwf	ADCON0
	
	return
	
;******************************************
;	Sticker Print Modules
;*****************************************
	
ReadStickers
	bsf	PORTC,2		; gives RC2 5 volts
	
	PrintNumber	TenK
		
	bsf	ADCON0,2		;start conversion and wait for it to complete
	btfsc	ADCON0,2		;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 LCD CONVERSION MODULE
	goto	$-1
	
	movf	ADRESH, W
	movwf	NumH
	movf	ADRESL, W
	movwf	NumL
	
	call		Clear_Display
	call		bin16_BCD
	BCF		STATUS, IRP
	movlw		0x43
	movfw		FSR
	movlw		0x0
	decfsz		binCounter, W
	goto	$+3
	return
			
	addwf		FSR,F
	movlw		0x3
	subwf		TenK
	movlw		0x0
	btfsc		STATUS,C
	movlw		0x1
	movwf		INDF
	
	
	PrintNumber	Thou
	PrintNumber	Hund
	PrintNumber     Tens
	PrintNumber     Ones
	call		HalfS
	call		HalfS
	
	
	bcf	PORTC,2		; gives RC2 5 volts
	goto	    ReadStickers
	
ReadBWIR1
	
	;bsf	PORTC,2		; gives RC2 5 volts
	
	;movlw	b'11000101'	;movlw	b'11000101' selects the clock and ra0 as the reader pin
	;movwf	ADCON0
		
	bsf	ADCON0,2		;start conversion and wait for it to complete
	btfsc	ADCON0,2		;LCD CONVERSION MODULE
	goto	$-1
	
	movf	ADRESH, W
	movwf	NumH
	movf	ADRESL, W
	movwf	NumL
	
	call		Clear_Display
	call		bin16_BCD
	PrintNumber	TenK
	PrintNumber	Thou
	PrintNumber	Hund
	PrintNumber     Tens
	PrintNumber     Ones
	call		HalfS
	call		Clear_Display
	
;	BCF		STATUS, IRP
;	movlw		0x43
;	movfw		FSR
;	movlw		0x0
;	decfsz		binCounter, W
;	goto	$+3
;	return
	
;	addwf		FSR,F
	movlw		0x1
	subwf		TenK
	movlw		0x0
	btfsc		STATUS,C
	movlw		0x1
;	movwf		INDF
	
	goto		ReadBWIR1
	
ReadBWIR2
	
	;bsf	PORTC,2		; gives RC2 5 volts
	
	movlw	b'11001101'	;movlw	b'11000101' selects the clock and ra0 as the reader pin
	movwf	ADCON0
		
	bsf	ADCON0,2		;start conversion and wait for it to complete
	btfsc	ADCON0,2		;LCD CONVERSION MODULE
	goto	$-1
	
	movf	ADRESH, W
	movwf	NumH
	movf	ADRESL, W
	movwf	NumL
	
	call		Clear_Display
	call		bin16_BCD
	PrintNumber	TenK
	PrintNumber	Thou
	PrintNumber	Hund
	PrintNumber     Tens
	PrintNumber     Ones
	call		HalfS
	call		Clear_Display
	
;	BCF		STATUS, IRP
;	movlw		0x43
;	movfw		FSR
;	movlw		0x0
;	decfsz		binCounter, W
;	goto	$+3
;	return
	
;	addwf		FSR,F
	movlw		0x1
	subwf		TenK
	movlw		0x0
	btfsc		STATUS,C
	movlw		0x1
;	movwf		INDF
	
	goto		ReadBWIR2
	
ReadColumn			; not using anymore cus apparently column is ultrasonic
	
	bsf	PORTC,2		; gives RC2 5 volts
		
	bsf	ADCON0,2		;start conversion and wait for it to complete
	btfsc	ADCON0,2		;LCD CONVERSION MODULE
	goto	$-1
	
	movf	ADRESH, W
	movwf	NumH
	movf	ADRESL, W
	movwf	NumL
	
	call		Clear_Display
	call		bin16_BCD
	PrintNumber	TenK
	PrintNumber	Thou
	PrintNumber	Hund
	PrintNumber     Tens
	PrintNumber     Ones
	call		HalfS
	call		Clear_Display
	
	movlw		0x1
	subwf		TenK		    ;check tenK
	movlw		0x0
	btfsc		STATUS,C
	movlw		0x1
	
	return
	
;***************************************************
;	US Sensor Modules
;***************************************************
	
Read1_US
	 
	call		Read1_US1 
	 
	call		Clear_Display
	
	movf		TMR1H, W
	movwf		NumH
	movf		TMR1L, W
	movwf		NumL
	
	call		bin16_BCD
	
	PrintNumber	TenK
	PrintNumber	Thou
	PrintNumber	Hund
	PrintNumber	Tens
	PrintNumber	Ones
	
	call		HalfS
	
	call		Clear_Display
	
	movlw		0x1
	subwf		TenK
	movlw		0x1
	btfsc		STATUS,C
	movlw		0x0
	
	
	return
	
Read1_US1
		clrf	TMR1H
		clrf	TMR1L
		
		bcf	UST1		;make sure trigger is clear
		call	DELAY1
		;Delay_5ms
		
		bsf	UST1		;trigger high, bottom sensor
		call	delay10us		;10us delay
		bcf	UST1		;trigger low
		
		btfss	US1E1		;wait for echo to go high
		goto	$-1
		bsf	T1CON,TMR1ON	;turn on timer
		
		btfsc	US1E1		;wait for echo to go low
		goto	$-1
		
    		bcf	T1CON,TMR1ON	;turn off timer
		return
		
Read2_US
	 
	call		Read2_US1 
	 
	call		Clear_Display
	
	movf		TMR1H, W
	movwf		NumH
	movf		TMR1L, W
	movwf		NumL
	
	call		bin16_BCD
	
	PrintNumber	TenK
	PrintNumber	Thou
	PrintNumber	Hund
	PrintNumber	Tens
	PrintNumber	Ones
	
	call		HalfS
	
	call		Clear_Display
	
	movlw		0x1
	subwf		TenK
	movlw		0x1
	btfsc		STATUS,C
	movlw		0x0
	
	
	return
	
Read2_US1
		clrf	TMR1H
		clrf	TMR1L
		
		bcf	UST2		;make sure trigger is clear
		call	DELAY1
		;Delay_5ms
		
		bsf	UST2		;trigger high, bottom sensor
		call	delay10us		;10us delay
		bcf	UST2		;trigger low
		
		btfss	US1E2		;wait for echo to go high
		goto	$-1
		bsf	T1CON,TMR1ON	;turn on timer
		
		btfsc	US1E2		;wait for echo to go low
		goto	$-1
		
    		bcf	T1CON,TMR1ON	;turn off timer
		return
	

;*********************************************
; Keypad Modules
;*******************************************
	 

Finalize
        call        Clear_Display
        Display     WantResults
	call	    Switch_Lines
	Display	    WantRotate
	
	
	goto	    KeypadandLCD
	
ListDisplay
        call        Clear_Display
        Display	    Results1
	call	    Switch_Lines
	Display	    Results2
	
	return
		
BinsDisplay
        call        Clear_Display
        Display	    Bins
		
	return
	
BinLocationsDisplay
        call        Clear_Display
        Display	    BinLocations
		
	return

OperationTimeDisplay
        call        Clear_Display
        Display	    OperationTime
		
	return
	
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
	
StickerDisplay
	
	;movlw	    0x1
	;movwf	    Front1
	
	
	
	call	    Clear_Display
	movlw	    0x0
	movwf	    counter
	BCF	    STATUS,IRP
	movlw	    0x43				;0x43 is the front1
							;0x4A should be the back1	
	movwf	    FSR
	movfw	    binCounter
	movwf	    countdown
	PrintNumber	TenK
	call	    ReadStickers
	
AddBin
	Call Clear_Display
	incf	NumOfBins,F
	PrintNumber	NumOfBins
	Call	    HalfS
	movlw	0X7			;checks if max of 7 bins has been reached
	subwf	NumOfBins,W		
	btfsc	STATUS,Z
	goto	Finalize
	return
	
Stickers
	call 		Clear_Display
	movlw		0X0
	movwf		counter
	BCF             STATUS, IRP
	movlw		0x41
	movwf		FSR
	movfw		NumOfBins
	movwf		countdown
FrontLoop	
		INCF		FSR,1
		incf		counter	
		PrintNumber	counter
		call		HalfS
		movfw		INDF
		PrintCol	W
		decfsz		countdown,F
		goto		FrontLoop
Next		return
	
;FrontLoop
;	INCF	    FSR,1
;	incf	    counter
;	PrintNumber counter
;	call	    HalfS
;	movfw	    INDF
;
;	PrintCol    W
;	decfsz	    countdown,F
;	goto	    FrontLoop
;	return
	
DisplayBlackWhiteIR1
	call		ReadBWIR1	
	movwf		Front1
	PrintCol    	Front1
	call		HalfS
	
	return
	
DisplayBlackWhiteIR2
	call		ReadBWIR2	
	movwf		Front1
	PrintCol    	Front1
	call		HalfS
	
	return
	
pollColumnSensor	;checks to see if column present
 
	call		ReadColumn
	movwf		isColumnThere		    ;sets the column bit 1 or 0
	PrintYN		isColumnThere
	call		HalfS
	call		HalfS
	
	return
	
DisplayUSSensor1
	
	call		Read1_US
	call		ClrLCD
	movwf		isBinThere		    ;sets the bin bit 1 or 0
	PrintYN		isBinThere
	call		HalfS
	call		HalfS
	
	return
	
DisplayUSSensor2
	
	call		Read2_US
	call		ClrLCD
	movwf		isColumnThere		    ;sets the bin bit 1 or 0
	PrintYN		isColumnThere
	call		HalfS
	call		HalfS
	
	return
	
	
PWMSTART	
	
	;************ FIRST PWM ******************************		    To stop it, clear CCP1RL and/or CCPR2L 
    
    BANKSEL TRISC
    BCF	    TRISC, 2		;set CCP1 as output		;CCP1 is RC2 and CCP2 is RC1
    
    MOVF     CCP1CON,W		;set CCP1 as PWM
    ANDLW    0xF0
    IORLW    0x0C
    MOVWF    CCP1CON
    
    ;save   pr2 value so that you can zero it after
    
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
    MOVLW   B'01100001'
    MOVWF   CCPR2L	
    BSF	    CCP2CON, 3
    BSF	    CCP2CON, 2
    
    BSF     T2CON, TMR2ON	;and start the timer running
    BCF	    STATUS,RP0
    
    return
    
PWMSTOP
    
    clrf   CCPR1L
    clrf  CCPR2L
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
; Delay: ~ 5ms
;***************************************
DELAY1 
				;4998 cycles
	movlw	0xE7
	movwf	lcd_d1
	movlw	0x04
	movwf	lcd_d2
Delay_0
	decfsz	lcd_d1, f
	goto	$+2
	decfsz	lcd_d2, f
	goto	Delay_0

			;2 cycles
	goto	$+1
	return
	
;***************************************
; Delay: ~10us
;***************************************
delay10us
    movlw	d'20'
    movwf	delay3
Delay10usLoop
    decfsz	delay3, f
    goto	Delay10usLoop
    return

    
;************************************
; RTC Sublabels
;************************************
    
show_RTC
		;clear LCD screen
		movlw	b'00000001'
		call	WR_INS

		;Get year
		movlw	"2"				;First line shows 20**/**/**
		call	WR_DATA
		movlw	"0"
		call	WR_DATA
		rtc_read	0x06		;Read Address 0x06 from DS1307---year
		movfw	0x77
		call	WR_DATA
		movfw	0x78
		call	WR_DATA

		movlw	"/"
		call	WR_DATA

		;Get month
		rtc_read	0x05		;Read Address 0x05 from DS1307---month
		movfw	0x77
		call	WR_DATA
		movfw	0x78
		call	WR_DATA

		movlw	"/"
		call	WR_DATA

		;Get day
		rtc_read	0x04		;Read Address 0x04 from DS1307---day
		movfw	0x77
		call	WR_DATA
		movfw	0x78
		call	WR_DATA

		movlw	B'11000000'		;Next line displays (hour):(min):(sec) **:**:**
		call	WR_INS

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
		;goto	show_RTC		;use return instead to check for key press
		return

;***************************************
; Setup RTC with time defined by user
;***************************************
set_rtc_time

		rtc_resetAll	;reset rtc

		rtc_set	0x00,	B'10000000'

		;set time 
		rtc_set	0x06,	B'00010110'		; Year
		rtc_set	0x05,	B'00000010'		; Month
		rtc_set	0x04,	B'00010110'		; Date
		rtc_set	0x03,	B'00100001'		; Day
		rtc_set	0x02,	B'00000001'		; Hours
		rtc_set	0x01,	B'00110101'		; Minutes
		rtc_set	0x00,	B'01010000'		; Seconds
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

;***************************************
; Print sticker values
;***************************************
PrintBlackSticker
	movlw	    b'0'
	call	    StickerColours
	call	    WR_DATA
	return

PrintWhiteSticker
	movlw	    b'1'
	call	    StickerColours
	call	    WR_DATA
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
        dt        "1:Start|#/D:PWM|2:Col|3:B/W", 0
	
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
        dt        "5:AddBin|6:Stickers|7:B/Wstore", 0
        
Operation
	
	;Change Page
	movwf	 Temp
	movlw	HIGH OperationEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW OperationEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL
        addwf    PCL,F
OperationEntries
        dt        "Operating...",0

WantResults
	
	;Change Page
	movwf	 Temp
	movlw	HIGH WantResultsEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW WantResultsEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL
        addwf    PCL,F
WantResultsEntries
        dt        "Press 2 to display results",0


WantRotate
		
	;Change Page
	movwf	 Temp
	movlw	HIGH WantRotateEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW WantRotateEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL
	addwf	 PCL,F
WantRotateEntries
	dt	  "Press A to rotate screen",0
	
Results1 
		
	;Change Page
	movwf	 Temp
	movlw	HIGH Results1Entries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW Results1Entries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL
        addwf    PCL,F
Results1Entries	
        dt        "3: PWM test, 4: Sensor Test",0
	
Results2
	
	;Change Page
	movwf	 Temp
	movlw	HIGH Results2Entries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW Results2Entries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
        addwf    PCL,F
Results2Entries	
        dt        "6: bin locations, 8: operation time",0
	
Bins
	;Change Page
	movwf	 Temp
	movlw	HIGH BinsEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW BinsEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
	addwf	 PCL,F
BinsEntries
	dt	  "4 bins in total",0
	
StickerStatus
	
	;Change Page
	movwf	 Temp
	movlw	HIGH StickerStatusEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW StickerStatusEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
	addwf	 PCL,F
StickerStatusEntries
	dt	  "BBWBWWWB",0
	
BinLocations
	;Change Page
	movwf	 Temp
	movlw	HIGH BinLocationsEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW BinLocationsEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
	addwf	 PCL,F
BinLocationsEntries
	dt	  "0.35,0.56,0.79",0
	
OperationTime
	;Change Page
	movwf	 Temp
	movlw	HIGH OperationTimeEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW OperationTimeEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
	addwf	 PCL,F
OperationTimeEntries
	dt	  "Moving Arm",0
	
ReceivedInput
	;Change Page
	movwf	 Temp
	movlw	HIGH ReceivedInputEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW ReceivedInputEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
	addwf	 PCL,F
ReceivedInputEntries
	dt	  "takin in large value",0
	
SmallValueInput
	;Change Page
	movwf	 Temp
	movlw	HIGH SmallValueInputEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW SmallValueInputEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
	addwf	 PCL,F
SmallValueInputEntries
	dt	  "small value",0
	
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
	
ColumnValues
	
	;Change Page
	movwf	Temp
	movlw	HIGH ColumnValuesEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW ColumnValuesEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
        addwf    PCL,F
ColumnValuesEntries
        dt        "NY",  0

    
	END
