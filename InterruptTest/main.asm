;*******************************************************************
; Keypad/LCD Test Code
; Assembler : mpasm.exe
; Linker    : mplink.exe
; Written By : Kevin Lam
;*******************************************************************

      list p=16f877                 ; list directive to define processor
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
		NumH
		NumL
		TenK
		Thou
		Hund
		Tens
		Ones
		distanceMoved
		isColumnThere
		frontstickerValues
		backstickerValues
		frontbinCounter
		backbinCounter
		binCounter
	endc	

	;Declare constants for pin assignments (LCD on PORTD)
		#define	RS 	PORTD,2
		#define	E 	PORTD,3
		#define	IR1	PORTA,0
		#define	IR2	PORTA,1
		#define	US	PORTA,2
		#define	DCMotor	PORTA,3
		
;distanceMoved	equ	b'0'
		

 

	ORG       0x0000     ;RESET vector must always be at 0x00
        goto      init       ;Just jump to the main code section.
	 

           ORG 000H

	    GOTO KeypadandLCD

	    ORG 004H

	    GOTO INT_SERV
	    
;***************************************
; Number to Printable
;***************************************
PrintNumber macro	number
	;movf	    number ,W
	movfw	    number
	call	    DectoChar
	call	    WR_DATA
	endm

; white and black sticker labels at bottom
	    
INT_SERV:
    
     btfsc INTCON, INTF
     PrintNumber    0x2
     BCF INTCON, INTF    ; clear the appropriate flag
     
     RETFIE
     
     
	 ;ORG	   0x100	;this command is sketchy
         

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
         movlw     b'11110011'    ; Set required keypad inputs
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
	 
	bsf	PORTC,2		; gives RC2 5 volts
	 
	; Set ADCON1 to use RA0 as analog input
	
	
	bsf	STATUS,RP0	;Select bank 1
	movlw	b'00000110'	;left justified, all inputs digital
	movwf	ADCON1			;All digital input, reference voltage Vdd Vss

	;ADCON0
	bcf	STATUS,RP0
	movlw	b'11000101'	;movlw	b'11000101' selects the clock and ra0 as the reader pin
	movwf	ADCON0
	
	      
	
     BSF OPTION_REG, INTEDG   ; interrupt on positive
     BCF INTCON, INTF    ; clear interrupt flag
     BSF INTCON, INTE    ; mask for external interrupts
     BSF INTCON, GIE     ; enable interrupts
	
;***************************************************
; Initialize variables and Displays
;****************************************************	
	Display		Welcome_Msg
	
	
	movlw	b'0'
	movwf	frontbinCounter		;initialize binCounter with zero
	movlw	b'0'
	movwf	backbinCounter		;initialize binCounter with zero
	

	

;**************************************************************************************************************************************************
;								  MAIN CODE
;----------------------------------------------------------------------------------------------------------------------------------------------
MAIN	

; WAIT FOR INITIAL INPUT
    goto    KeypadandLCD     ;calls initial form of keypad and LCD, where  robot is waiting for input to begin 
	
    
    movlw   b'0000'
    movwf   frontbinCounter	    ;testing with value of 5 bins
    movwf   backbinCounter    
    
    
    
    
    
;    goto    getTimeRTC	    ; gets time from RTC clock
;	
;;   MOTOR ON
;	
;    bsf	DCMotor		    ; begin forward movement, set appropriate bit to 1	
;    movlw   b'1'
;    addwf  distanceMoved,f     ; increments distanceMoved by 1 (will experimentally find binary value for 4 m)
;    
;;   CHECK COLUMN
;    
;    call    ChooseIR1		;selects IR1 for data conversion
;    call    pollColumnSensor	;checks to see if column present
;    btfss   isColumnThere,0
;    goto    Main
;    
;;   CHECK IF AT THE END OF BUCKET LINE
;    
;    call    pollUSsensor	; call ultrasonic sensor
;    btfsc   nothingAhead,0	;   checks if there is nothing ahead
;    goto    Finalize
;    
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
    goto    MAIN


		
;-----------------------------------------------------------------------------------------------------		

;*************************************
;	Keypad and LCD forms	    
;*************************************
	
	
	
KeypadandLCD	btfss		PORTB,1     ;Wait until data is available from the keypad
		goto		$-1 

		swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
		andlw		0x0F
		movwf		temp
		bsf		PORTA,4
		Key	0x00, OperationDisplay
		;Key	0x00, goback
		Key	0x01, LCDConversion
		Rotation	0x03
		Key	0x04, BinsDisplay
	    	Key	0x05, StickerDisplay
		Key	0x06, BinLocationsDisplay
		Key	0x08, OperationTimeDisplay
		Key	0x09, RTCDisplay
		btfsc		PORTB,1     ;Wait until key is released
	        goto		$-1
		goto		KeypadandLCD
		
goback
		return	
		
		
;*********************************************************
; A to D conversion with LCD display for IR sensor
;*********************************************************

LCDConversion
	bsf	PORTC,2		; gives RC2 5 volts	
		
	bsf	ADCON0,2		;start conversion and wait for it to complete
	btfsc	ADCON0,2		;LCD CONVERSION MODULE
	goto	$-1
	
	movf	ADRESH, W
	movwf	NumH
	movf	ADRESL, W
	movwf	NumL
		
	call		bin16_BCD
	call		ClrLCD
	PrintNumber	TenK
	PrintNumber	Thou
	PrintNumber	Hund
	PrintNumber     Tens
	PrintNumber     Ones
	call		HalfS
	call		HalfS
	bcf	PORTC,2		; gives RC2 5 volts
	goto	LCDConversion

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
	
	
;****************************************************
;   ScanFrontContainer
;****************************************************
	
ScanFrontContainer
	; A to d conversion
	
	bsf	ADCON0,2		;start conversion and wait for it to complete
	btfsc	ADCON0,2		;LCD CONVERSION MODULE
	goto	$-1
	
	movf	ADRESH, W
	movwf	NumH
	movf	ADRESL, W
	movwf	NumL
	
;	btfss	ADRESH,2		;test bit 2 of ADRESH, the bit that we believe will distinguish black and white (need to experiment)
;	bsf	frontstickerValues,frontbinCounter		;if is set, means that colour is white
;	btfsc	ADRESH,2
;	bcf	frontstickerValues,frontbinCounter    ; modifies the bits in the stickervalues
	
	;display on screen for troubleshooting purposes
	
;	call		bin16_BCD
;	call		ClrLCD
;	PrintNumber	TenK
;	PrintNumber	Thou
;	PrintNumber	Hund
;	PrintNumber     Tens
;	PrintNumber     Ones	
	call		HalfS
	call		HalfS
	
	return
	
;****************************************************
;   ScanBackContainer
;****************************************************
	
ScanBackContainer
	; A to d conversion
	
	bsf	ADCON0,2		;start conversion and wait for it to complete
	btfsc	ADCON0,2		;LCD CONVERSION MODULE
	goto	$-1
	
	movf	ADRESH, W
	movwf	NumH
	movf	ADRESL, W
	movwf	NumL
	
;	btfss	ADRESH,2		;test bit 2 of ADRESH, the bit that we believe will distinguish black and white (need to experiment)
;	bsf	backstickerValues,backbinCounter		;if is set, means that colour is white
;	btfsc	ADRESH,2
;	bcf	backstickerValues,backbinCounter    ; modifies the bits in the stickervalues
	
	;display on screen for troubleshooting purposes
	
;	call		bin16_BCD
;	call		ClrLCD
;	PrintNumber	TenK
;	PrintNumber	Thou
;	PrintNumber	Hund
;	PrintNumber     Tens
;	PrintNumber     Ones	
	call		HalfS
	call		HalfS
	
	return

;*********************************************
; Keypad Modules
;*******************************************
	 

OperationDisplay
	call        Clear_Display
	
	
	
	call	    ScanFrontContainer
	call	    ScanBackContainer
	call	    ScanFrontContainer
	call	    ScanBackContainer
	call	    ScanFrontContainer
	call	    ScanBackContainer
	call	    ScanFrontContainer
	call	    ScanBackContainer
	
        call        Clear_Display
        Display     WantResults
	call	    Switch_Lines
	Display	    WantRotate
	
	return
	
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
	
StickerDisplay
        call        Clear_Display

;	movlw	    b'0000000'
;	movwf	    stickerValues
;	movlw	    b'1111111'
;	movwf	    backstickerValues
	
	movlw   b'0000'
	movwf   frontbinCounter	    ;testing with value of 5 bins
	movwf   backbinCounter 
	
	call	    ScanFrontContainer
	btfss	    frontstickerValues,0	; check the ith bucket value and if it is clear,
	call	    PrintBlackSticker	; print a black sticker on the thing
	btfsc	    frontstickerValues,0
	call	    PrintWhiteSticker
	;call	    Switch_Lines
	call	    ScanBackContainer
	btfss	    backstickerValues,0	; check the ith bucket value and if it is clear,
	call	    PrintBlackSticker	; print a black sticker on the thing
	btfsc	    backstickerValues,0
	call	    PrintWhiteSticker
	;call	    Switch_Lines
	
;	decfsz	    frontbinCounter		;decrements bin counter to know how much to print
;	goto	    $+2
;	return
	
	movlw   b'0001'
	movwf   frontbinCounter	    ;testing with value of 5 bins
	movwf   backbinCounter
	
	
	
	call	    ScanFrontContainer
	btfss	    frontstickerValues,1	; check the ith bucket value and if it is clear,
	call	    PrintBlackSticker	; print a black sticker on the thing
	btfsc	    frontstickerValues,1
	call	    PrintWhiteSticker
	;call	    Switch_Lines
	call	    ScanBackContainer
	btfss	    backstickerValues,1	; check the ith bucket value and if it is clear,
	call	    PrintBlackSticker	; print a black sticker on the thing
	btfsc	    backstickerValues,1
	call	    PrintWhiteSticker
	;call	    Switch_Lines
	
;	decfsz	    frontbinCounter		;decrements bin counter to know how much to print
;	goto	    $+2
;	return
	
	movlw   b'0010'
	movwf   frontbinCounter	    ;testing with value of 5 bins
	movwf   backbinCounter
	
	
	call	    ScanFrontContainer
	btfss	    frontstickerValues,2	; check the ith bucket value and if it is clear,
	call	    PrintBlackSticker	; print a black sticker on the thing
	btfsc	    frontstickerValues,2
	call	    PrintWhiteSticker
	;call	    Switch_Lines
	call	    ScanBackContainer
	btfss	    backstickerValues,2	; check the ith bucket value and if it is clear,
	call	    PrintBlackSticker	; print a black sticker on the thing
	btfsc	    backstickerValues,2
	call	    PrintWhiteSticker
	;call	    Switch_Lines
	
;	movlw   b'0011'
;	movwf   frontbinCounter	    ;testing with value of 5 bins
;	movwf   backbinCounter
;	
;;	decfsz	    frontbinCounter		;decrements bin counter to know how much to print
;;	goto	    $+2
;;	return
;;	
;	call	    ScanFrontContainer
;	btfss	    frontstickerValues,3	; check the ith bucket value and if it is clear,
;	call	    PrintBlackSticker	; print a black sticker on the thing
;	btfsc	    frontstickerValues,3
;	call	    PrintWhiteSticker
;	;call	    Switch_Lines
;	call	    ScanBackContainer
;	btfss	    backstickerValues,3	; check the ith bucket value and if it is clear,
;	call	    PrintBlackSticker	; print a black sticker on the thing
;	btfsc	    backstickerValues,3
;	call	    PrintWhiteSticker
;	;call	    Switch_Lines
;		
;	movlw   b'0100'
;	movwf   frontbinCounter	    ;testing with value of 5 bins
;	movwf   backbinCounter
;	
;;	
;;	decfsz	    frontbinCounter		;decrements bin counter to know how much to print
;;	goto	    $+2
;;	return
;;	
;	btfss	    frontstickerValues,4	; check the ith bucket value and if it is clear,
;	call	    PrintBlackSticker	; print a black sticker on the thing
;	btfsc	    frontstickerValues,4
;	call	    PrintWhiteSticker
;	;call	    Switch_Lines
;	btfss	    backstickerValues,4	; check the ith bucket value and if it is clear,
;	call	    PrintBlackSticker	; print a black sticker on the thing
;	btfsc	    backstickerValues,4
;	call	    PrintWhiteSticker
;	;call	    Switch_Lines

	
;	decfsz	    frontbinCounter		;decrements bin counter to know how much to print
;	goto	    $+2
;	return
;	
;	btfss	    frontstickerValues,5	; check the ith bucket value and if it is clear,
;	call	    PrintBlackSticker	; print a black sticker on the thing
;	btfsc	    frontstickerValues,5
;	call	    PrintWhiteSticker
;	;call	    Switch_Lines
;	btfss	    backstickerValues,5	; check the ith bucket value and if it is clear,
;	call	    PrintBlackSticker	; print a black sticker on the thing
;	btfsc	    backstickerValues,5
;	call	    PrintWhiteSticker
;	;call	    Switch_Lines
;	
;	decfsz	    frontbinCounter		;decrements bin counter to know how much to print
;	goto	    $+2
;	return
;	
;	btfss	    frontstickerValues,6	; check the ith bucket value and if it is clear,
;	call	    PrintBlackSticker	; print a black sticker on the thing
;	btfsc	    frontstickerValues,6
;	call	    PrintWhiteSticker
;	;call	    Switch_Lines
;	btfss	    backstickerValues,6	; check the ith bucket value and if it is clear,
;	call	    PrintBlackSticker	; print a black sticker on the thing
;	btfsc	    backstickerValues,6
;	call	    PrintWhiteSticker
;	;call	    Switch_Lines
;	
;	decfsz	    frontbinCounter		;decrements bin counter to know how much to print
;	goto	    $+2
;	return
	
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

	goto MAIN
	
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
        dt        "Press 1 to start robot", 0
        
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
        dt        "4: number of bins, 5: sticker status",0
	
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
        dt        "6: bin locations, 7: operation time",0
	
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
	dt	  "Not receiving anything",0
	
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
	dt	  "TAKING IN 5 VOLTS",0
	
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
; Number to Colour
;***************************************
PrintCol macro	number
	movlw	    0x80
	call	    WR_INS
	call	    lcdLongDelay
	movf	    number ,W
	call	    BinCol
	call	    WR_DATA
	endm		

    
	END