;*******************************************************************
; Keypad/LCD Test Code
; Assembler : mpasm.exe
; Linker    : mplink.exe
; Written By : Kevin Lam
;*******************************************************************

      list p=16f877                 ; list directive to define processor
      #include <p16f877.inc>        ; processor specific variable definitions
      __CONFIG _CP_OFF & _WDT_OFF & _BODEN_ON & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _CPD_OFF & _LVP_OFF

 
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
		Front1 
		Front2
		Front3
		Front4
		Front5
		Front6
		Front7
		Back1
		Back2
		Back3
		Back4
		Back5
		Back6
		Back7
		counter	
		countdown
		NumH
		NumL
		TenKOld
		TenK
		Zero
		Thou
		Hund
		Tens
		Ones
		NumOfBins
		
	endc	

	;Declare constants for pin assignments (LCD on PORTD)
		#define	RS 	PORTD,2
		#define	E 	PORTD,3

         ORG       0x0000     ;RESET vector must always be at 0x00
         goto      init       ;Just jump to the main code section.
         

Key	 macro	value, subroutine
 	 swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
	 andlw		0x0F
	 xorlw		value
	 btfsc		STATUS, Z
	 call		subroutine
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

         bcf       STATUS,RP0     ; select bank 0
         clrf      PORTA
         clrf      PORTB
         clrf      PORTC
         clrf      PORTD
          
         call      InitLCD  	  ;Initialize the LCD (code in lcd.asm; imported by lcd.inc)
	  ;A/D converter attempt
	 
	 bcf	STATUS,RP1
	 bsf	STATUS,RP0	;Select bank 1
	 
	 bsf	TRISA,0
	 bsf	TRISA,1
	 bsf	TRISA,2	;makes RA0 an input
	 bcf	STATUS,RP0	; go back to bank 0
	 bsf	PORTC,2		; gives RC1 5 volts
	 
	; Set ADCON1 to use RA0 as analog input
	
	
	bsf	STATUS,RP0	;Select bank 1
	movlw	b'00001001'	;currently left justified CHANGE AFTER ASSESSMENT
	movwf	ADCON1		;All analog input accept RA7 and RA6, reference voltage Vdd Vss

	;ADCON0
	bcf	STATUS,RP0
	movlw	b'00000101'	;RA0 is selected analog 
	movwf	ADCON0
;***************************************
; Number to Printable
;***************************************
PrintNumber macro	number
	movfw	    number
	call	    DectoChar
	call	    WR_DATA
	endm
	
;***************************************
; Number to Colour
;***************************************
PrintCol macro	number
	movfw	    number
	call	    BinCol
	call	    WR_DATA
	endm
;***************************************
; Main code
;***************************************
Main	Display		Welcome_Msg
	movlw		0X00
	movwf		NumOfBins
	


StartofRun      btfss		PORTB,1     ;Wait until data is available from the keypad
		goto		$-1 

		swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
		andlw		0x0F
		movwf		temp
		Key		0x00, Operation
		btfsc		PORTB,1     ;Wait until key is released
	        goto		$-1
		goto		StartofRun

Operation	
		;bsf	PORTC,0			; gives RC2 5 volts
		bcf	STATUS,RP0
		movlw	b'00010101'             ;RA2
		movwf	ADCON0
Convert		bsf	ADCON0,2		;start conversion and wait for it to complete
		btfsc	ADCON0,2		;LCD CONVERSION MODULE
		goto	$-1
		movf	ADRESH, W
		movwf	NumH
		movf	ADRESL, W
		movwf	NumL
		call	Clear_Display
		PrintNumber TenK
		call	HalfS
		Call	bin16_BCD
		movlw	0X3
		subwf	TenK
		;bcf	PORTC,0	
		;btfsc	STATUS, C
		;goto AddBin
		goto Convert
		
		
;Operation
;		bsf	PORTC,0			; gives RC2 5 volts
;		bcf	STATUS,RP0
;		movlw	b'00000101'             ;RA0
;		movwf	ADCON0
;Convert2		bsf	ADCON0,2		;start conversion and wait for it to complete
;		btfsc	ADCON0,2		;LCD CONVERSION MODULE
;		goto	$-1
;		movf	ADRESH, W
;		movwf	NumH
;		movf	ADRESL, W
;		movwf	NumL
;		call	Clear_Display
;		Call		bin16_BCD
;		PrintNumber	TenK
;		PrintNumber	Thou
;		PrintNumber	Hund
;		PrintNumber     Tens
;		PrintNumber     Ones
;		call HalfS
;		goto Convert2 
;		bcf	PORTC,0
;		call LCDConversionFront
		;goto 		RunningCode
		
	
RunningCode
		goto 		EndOfRun

EndOfRun	Call 		Clear_Display
		Display		Results1
		call 		Switch_Lines
		Display 	Results2
starting	btfss		PORTB,1     ;Wait until data is available from the keypad
		goto		$-1 

		swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
		andlw		0x0F
		movwf		temp
		Key		0x01, Rotating
		Key		0x02, RunTime
		Key		0x03, BinLocations
		Key		0x06, AddBin
		Key		0x04, Stickers
		Key		0x05, GoBack
		Key		0x07, LCDConversionFront
		btfsc		PORTB,1     ;Wait until key is released
	        goto		$-1
		goto		starting


AddBin
		Call Clear_Display
		incf	NumOfBins,F
		PrintNumber	NumOfBins
		Call	    HalfS
		movlw	0X3
		subwf	NumOfBins
		btfsc	STATUS,Z
		gotoEndOfRun
		return
		
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
	call	HalfS
	call	HalfS
	return
		
Rotating
		call		ShiftDisplayLeft
		return    
RunTime
		Call 		Clear_Display
		Display 	OperationTime
		Call 		Switch_Lines
		Display		Rotate2
		return

GoBack
		Call 		Clear_Display
		Display		Results1
		call 		Switch_Lines
		Display 	Results2
		return
BinLocations
		Call 		Clear_Display
		Display 	LocationsInMemory
		movfw		Zero
		movwf		NumH
		movfw		Location1
		movwf		NumL
		Call		bin16_BCD
		PrintNumber	TenK
		PrintNumber	Thou
		PrintNumber	Hund
		PrintNumber     Tens
		PrintNumber     Ones
		call		HalfS
		call		HalfS
		Call 		Switch_Lines
		Display		Rotate2
		return
		
Stickers
		Call 		Clear_Display
		movlw		0X0
		movwf		counter
		Display 	Front
		BCF             STATUS, IRP
		movlw		0x41
		movwf		FSR
		movfw		NumOfBins
		movwf		countdown
FrontLoop	
		INCF		FSR,1
		Display		Spacee
		incf		counter	
		PrintNumber	counter
		Display 	StickerStates
		call		HalfS
		movfw		INDF
		PrintCol	W
		Display		Spacee
		decfsz		countdown,F
		goto		FrontLoop
Next		Call 		Switch_Lines
		Display		Rotate2
		return
ReadytoDisplay
		
		call		WR_INS
		movlw		"S"
		call		WR_DATA
		
		call		lcdLongDelay

ShiftDisplayLeft
Left		movlw		b'00011000'		;Move to the left
		call		WR_INS
		return

    goto Main
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
        
Operating
	
	;Change Page
	movwf	 Temp
	movlw	HIGH OperatingEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW OperatingEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL
        addwf    PCL,F
OperatingEntries
        dt        "Operating...",0

Front
	
	;Change Page
	movwf	 Temp
	movlw	HIGH FrontEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW FrontEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL
        addwf    PCL,F
FrontEntries
        dt        "Front",0
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
        dt        "Press 3 for runtime, A for bin location",0

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
        dt        "Press 4 for sticker conditions",0

Spacee
	
	;Change Page
	movwf	 Temp
	movlw	HIGH SpaceeEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW SpaceeEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL
        addwf    PCL,F
SpaceeEntries
        dt        " ",0
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
        addwf    PCL,F
OperationTimeEntries
        dt        "The RunTime is",0

Rotate
	
	;Change Page
	movwf	 Temp
	movlw	HIGH RotateEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW RotateEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL
        addwf    PCL,F
RotateEntries
        dt        "Press 2 to rotate left",0

Rotate2
	
	;Change Page
	movwf	 Temp
	movlw	HIGH Rotate2Entries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW Rotate2Entries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL
        addwf    PCL,F
Rotate2Entries
        dt        "Press 2 to rotate left, 5 to go back",0
LocationsofBins
	
	;Change Page
	movwf	 Temp
	movlw	HIGH LocationsofBinsEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW LocationsofBinsEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL
        addwf    PCL,F
LocationsofBinsEntries
        dt        "Bin locations here",0

StickerStates
	
	;Change Page
	movwf	 Temp
	movlw	HIGH StickerStatesEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW StickerStatesEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL
        addwf    PCL,F
StickerStatesEntries
        dt        ":",0

LocationsInMemory
	
	;Change Page
	movwf	Temp
	movlw	HIGH LocationsInMemoryEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW LocationsInMemoryEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
        addwf    PCL,F
LocationsInMemoryEntries
        dt        "1:",  0
	
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
	
BinCol
	
	;Change Page
	movwf	Temp
	movlw	HIGH BinColEntries
	movwf	PCLATH
	movf	Temp,w
	addlw	LOW BinColEntries
	btfsc	STATUS,C
	    incf    PCLATH,f
	movwf	PCL	
        addwf    PCL,F
BinColEntries
        dt        "BW ",  0

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


;******* LCD-related subroutines *******


    ;***********************************
InitLCD
	bcf STATUS,RP0
	bsf E     ;E default high
	
	;Wait for LCD POR to finish (~15ms)
	call lcdLongDelay
	call lcdLongDelay
	call lcdLongDelay

	;Ensure 8-bit mode first (no way to immediately guarantee 4-bit mode)
	; -> Send b'0011' 3 times
	movlw	b'00110011'
	call	WR_INS
	call lcdLongDelay
	call lcdLongDelay
	movlw	b'00110010'
	call	WR_INS
	call lcdLongDelay
	call lcdLongDelay

	; 4 bits, 2 lines, 5x7 dots
	movlw	b'00101000'
	call	WR_INS
	call lcdLongDelay
	call lcdLongDelay

	; display on/off
	movlw	b'00001100'
	call	WR_INS
	call lcdLongDelay
	call lcdLongDelay
	
	; Entry mode
	movlw	b'00000110'
	call	WR_INS
	call lcdLongDelay
	call lcdLongDelay

	; Clear ram
	movlw	b'00000001'
	call	WR_INS
	call lcdLongDelay
	call lcdLongDelay
	return
    ;************************************

    ;ClrLCD: Clear the LCD display
ClrLCD
	movlw	B'00000001'
	call	WR_INS
    return

    ;****************************************
    ; Write command to LCD - Input : W , output : -
    ;****************************************
WR_INS
	bcf		RS				;clear RS
	movwf	com				;W --> com
	andlw	0xF0			;mask 4 bits MSB w = X0
	movwf	PORTD			;Send 4 bits MSB
	bsf		E				;
	call	lcdLongDelay	;__    __
	bcf		E				;  |__|
	swapf	com,w
	andlw	0xF0			;1111 0010
	movwf	PORTD			;send 4 bits LSB
	bsf		E				;
	call	lcdLongDelay	;__    __
	bcf		E				;  |__|
	call	lcdLongDelay
	return

    ;****************************************
    ; Write data to LCD - Input : W , output : -
    ;****************************************
WR_DATA
	bsf		RS				
	movwf	dat
	movf	dat,w
	andlw	0xF0		
	addlw	4
	movwf	PORTD		
	bsf		E				;
	call	lcdLongDelay	;__    __
	bcf		E				;  |__|
	swapf	dat,w
	andlw	0xF0		
	addlw	4
	movwf	PORTD		
	bsf		E				;
	call	lcdLongDelay	;__    __
	bcf		E				;  |__|
	return

lcdLongDelay
    movlw d'20'
    movwf lcd_d2
LLD_LOOP
    LCD_DELAY
    decfsz lcd_d2,f
    goto LLD_LOOP
    return
    
    
	END