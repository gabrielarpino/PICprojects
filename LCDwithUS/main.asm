      list p=16f877                 ; list directive to define processor
      #include <p16f877.inc>        ; processor specific variable definitions
      __CONFIG _CP_OFF & _WDT_OFF & _BODEN_ON & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _CPD_OFF & _LVP_OFF


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
		NumH
		NumL
		TenK
		Thou
		Hund
		Tens
		Ones
		Temp
		delay3
	endc	

	;Declare constants for pin assignments (LCD on PORTD)
		#define	RS 	PORTD,2
		#define	E 	PORTD,3
		#define	UST	PORTC,2
		#define	US1E	PORTA,4

         ORG       0x0000     ;RESET vector must always be at 0x00
         goto      init       ;Just jump to the main code section.
         

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
; Number to Printable
;***************************************
PrintNumber macro	number
	;movf	    number ,W
	movfw	    number
	call	    DectoChar
	call	    WR_DATA
	endm

; white and black sticker labels at bottom

;***************************************
; Initialize LCD
;***************************************
init
         clrf      INTCON         ; No interrupts

         bsf       STATUS,RP0     ; select bank 1
	 movlw	    0xFF
         movwf      TRISA          ; All port A is input
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

;***************************************
; Main code
;***************************************
Main	Display		Welcome_Msg
	 
Main2
	 
	;Display		Alphabet
	 
	call		Read_US1 
	
	;bsf	    PORTA,0	    ; for testing purposes to power sensor
	 
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
	
	goto		Main2
	


;SwtichLine
;		call		Switch_Lines
;		Display		Welcome_Msg

;ChangeToQuestionMark
;		movlw		b'11001011'
;		call		WR_INS
;		movlw		"?"
;		call		WR_DATA

;ShiftDisplayLeft
;		call		Clear_Display
;
;		Display		Alphabet
;
;Left	movlw		b'00011000'		;Move to the left
;		call		WR_INS
;		call		HalfS
;		goto		Left			;repeat operation
;		
		goto	$

		
;***************************************
; read US
;**************************************
		
Read_US1
		clrf	TMR1H
		clrf	TMR1L
		
		bcf	UST		;make sure trigger is clear
		call	DELAY1
		;Delay_5ms
		
		bsf	UST		;trigger high, bottom sensor
		call	delay10us		;10us delay
		bcf	UST		;trigger low
		
		btfss	US1E		;wait for echo to go high
		goto	$-1
		bsf	T1CON,TMR1ON	;turn on timer
		
		btfsc	US1E		;wait for echo to go low
		goto	$-1
		
    		bcf	T1CON,TMR1ON	;turn off timer
		return
;*******************************************************
;******             BCD		        ****************
;*******************************************************
	
;---------------- Binary (16-bit) to BCD -----------------------
;
;bin8_BCD:	; --- Takes Binary.number in      NumL
;bin16_BCD:	; --- Takes Binary.number in NumH:NumL 
		; --> Returns decimal.form  in TenK:Thou:Hund:Tens:Ones
;		btfsc	US1E		;wait for echo to go low

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
; Look up table
;***************************************

Welcome_Msg	
		addwf	PCL,F
		dt		"Hello World!", 0

Alphabet
		addwf	PCL,F
		dt		"ABCDEFGHIJKLMNOPQRSTUVWXYZ",0
	
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
