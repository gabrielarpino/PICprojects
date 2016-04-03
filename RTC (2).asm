       list p=16f877                 ; list directive to define processor
      #include <p16f877.inc>        ; processor specific variable definitions
      __CONFIG _CP_OFF & _WDT_OFF & _BODEN_ON & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _CPD_OFF & _LVP_OFF
      Errorlevel -302		    ; switches off message [302] Register in operand not in bank 0
      Errorlevel -305

      #include <lcd.inc>			   ;Import LCD control functions from lcd.asm
	#include <rtc_macros.inc>

	cblock	0x20
		temp		    ;holder variables
		multiply_temp	    ;holder from multiplication
		multiply_argument
		holder1
		holder2
		startsec
		endsec
		startmin	    ;registers for storing the start and end time
		endmin
		runtimemin
		runtimesec
		runtimeones
		runtimetens
		runtimehundreds
		COUNTH
		COUNTM
		COUNTL
	endc


  RESET  code      0x0000     ;RESET vector must always be at 0x00
         goto      init       ;Just jump to the main code section.


         ORG	   0x100
;***************************************
; Initilization of both LCD and RTC
;***************************************
init
         clrf      INTCON         ; No interrupts

         bsf       STATUS,RP0     ; select bank 1
         clrf      TRISA          ; All port A is output
         clrf	   TRISB	    ; All port B is output
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


PrintNumber macro	number
	movfw	    number
	addlw	    0x30
	call	    WR_DATA
	endm
		
		 
;***************************************
;macro multiply value1*value2, store in storeresult
;***************************************
multiply macro value1, value2, storeResult

	 movfw value2	;put value2 in the holder
	 movwf multiply_temp

	 movfw value1	;the working reg holds value1
	 movwf storeResult ;the result holds 1x value1
	 decf value2

	 movfw value1	;esnure the working register holds value1

	 addwf storeResult, F	;add value 1 to the working register
	 decfsz value2, F	;decrement value2, skip the next instruction if its zero
	 goto $-3
	 ;goto hitmeloop

	 movfw multiply_temp	;restore value2
	 movwf value2
			;only storeResult changes in this macro
	endm
;***************************************
;macro regtobcd
;***************************************	
regtobcd macro originalReg, bcdTens 
	;adds the number of tens from originalReg to bcdTems
 
	movlw 0x0A ;the working register holds 10
	incf bcdTens, F
	subwf originalReg, F   ;for everytime we subtract ten from the original
	btfsc STATUS,C	    ;register, we can add one to bcdTens
	goto $-4
	decf bcdTens, F
	movlw 0x0A
	addwf originalReg, F
	
	endm
	
 
	

;************************************************************
; RTC modules, to get start and end time, and get total elapsed time
;**********************************************************
;the robot will only be used between 1 and 6 pm in lab.

GetTimeMin macro inserttime
    rtc_read	0x01
    movfw	0x77
    movwf   temp  ;temp holds the tensmins
    movlw   0x30    ;convert from ascii to binary
    subwf   temp, F
    movlw   d'10'
    movwf   multiply_argument ;hold me 1 is 10
    multiply multiply_argument,temp, inserttime
    clrf multiply_argument
    clrf temp
    ;inserttime now holds tensmins, in mins

    rtc_read	0x01
    movfw	0x78
    movwf   temp
    movlw   0x30    ;convert from ascii to binary
    subwf   temp, F
    movfw   temp
    addwf   inserttime, F ;inserttimeholds the mins now
endm
    
GetTimeSec macro inserttime
    rtc_read	0x00
    movfw	0x77
    movwf   temp  ;temp holds the tensmins
    movlw   0x30    ;convert from ascii to binary
    subwf   temp, F
    movlw   0xA
    movwf   multiply_argument ;hold me 1 is 10
    multiply multiply_argument,temp, inserttime
    clrf multiply_argument
    clrf temp
    ;inserttime now holds tensmins, in mins

    rtc_read	0x00
    movfw	0x78
    movwf   temp
    movlw   0x30    ;convert from ascii to binary
    subwf   temp, F
    movfw   temp
    addwf   inserttime, F ;inserttimeholds the mins now
endm


;TotalTimeCalculator macro randomassvariables
;		    ;first find the number of hours elapsed
;		    movfw	    starthour
;		    subwf	    endhour, W
;		    movwf	    runtimehour	;runtimehour holds the total number of hours that have passed, rounded up
;		    movfw	    startmin
;		    subwf	    endmin, W
;		    movwf	    runtimemin
;		    btfsc	    STATUS, C
;		    goto	    secondcalc  ;this means that the run passed the hour
;
;
;	negativemin
;						     ;runtime minute holds a negative value
;		    movlw	    0x3C	    ;put 60 into thw working register
;		    addwf	    runtimemin, F   ;add it to runtim minute to get the correct number of minutes past
;		    movlw	    0x01
;		    subwf	    runtimehour, F  ;subtract one from the number of hours past
;
;	secondcalc
;						    ;runtime minute holds a negative value
;		    movfw	    startsec
;		    subwf	    endsec, W
;		    movwf	    runtimesec
;		    btfsc	    STATUS, C
;		    goto	    conversion	    ;this means that the runtimesec value is a negative
;							 ;if it is positive, keep it in place
;
;	negativesec
;		    movwf	    runtimesec   ;runtimesec is a negative value
;		    movlw	    0x3C
;		    addwf	    runtimesec, F   ;so add 60
;		    movlw	    0x01
;		    subwf	    runtimemin, F   ;and subtract a minute
;	conversion
;		    movfw	    runtimesec	;move runtimesec into holder
;		    movwf	    holder
;		    movlw	    0x00	;reset runtimetens to zero
;		    movwf	    runtimetens
;
;	conversionloop
;		    incf	    runtimetens, F  ; increase runtimetens
;		    movlw	    0x0A	;the working register holds 10
;		    subwf	    holder, F	;subtract 10 from the runtimesec
;		    btfss	    STATUS, C	;if it become negative, add 10, and correct variables
;		    goto	    conversionfix
;		    goto	    conversionloop
;	conversionfix
;		    decf	    runtimetens, F
;		    movlw	    0x0A    ;ensure the working register holds 10
;		    addwf	    holder, F	;the working register should hold the time in ones, positively
;		    movfw	    holder
;		    movwf	    runtimeones
;	printit						;print the runtime to the screen
;		    ;hours
;		    movlw	"0"		;the tens hours of the run is zero
;		    call	WR_DATA
;		    movfw	runtimehour ;converting to ascii
;		    movwf	holder
;		    movlw	0x30
;		    iorwf	holder
;		    movfw	holder
;		    movwf	runtimehour
;		    movfw	runtimehour
;		    call	WR_DATA
;		    movlw	":"
;		    call	WR_DATA
;
;		    ;minutes
;		    movlw	"0"		;the tens minutes of the run is zero
;		    call	WR_DATA
;		    movfw	runtimemin  ;converting to ascii
;		    movwf	holder
;		    movlw	0x30
;		    iorwf	holder
;		    movfw	holder
;		    movwf	runtimemin
;		    movfw	runtimemin	;write the ones minutes of the run
;		    call	WR_DATA
;		    movlw	":"
;		    call	WR_DATA
;
;		    ;seconds
;		    movfw	runtimetens	;first print the runtimetens digit
;		    movwf	holder
;		    movlw	0x30
;		    iorwf	holder
;		    movfw	holder
;		    movwf	runtimetens
;		    movfw	runtimetens
;		    call	WR_DATA
;
;		    movfw	runtimeones	;first print the runtimeone digit
;		    movwf	holder
;		    movlw	0x30
;		    iorwf	holder
;		    movfw	holder
;		    movwf	runtimeones
;		    movfw	runtimeones
;		    call	WR_DATA
;
;endm
;*******************************************************************************
		 ; UNCOMMENT IF YOU WANT TO CHANGE THE TIME
		 ;rtc_resetAll
		 ;call set_rtc_time
;*******************************************************************************

		 ;Used to set up time in RTC, load to the PIC when RTC is used for the first time
		 ;call	   set_rtc_time

         call      InitLCD    ;Initialize the LCD (code in lcd.asm; imported by lcd.inc)

main
	 
	 GetTimeMin startmin
	 GetTimeSec startsec
	 
	 call OneS
	 call OneS
	 call OneS
	 call OneS
	 call OneS
	 call OneS
	 call OneS
	 call OneS
	 call OneS

	 call OneS
	 call OneS
	 call OneS

	 call OneS
	 call OneS
	 call OneS

	 call OneS
	 call OneS
	 call OneS
	 
	 call OneS
	 call OneS
	 call OneS
	 
	 call OneS
	 call OneS
	 call OneS
	 
	 call OneS
	 call OneS
	 call OneS

	 
	 GetTimeMin endmin
	 GetTimeSec endsec

	 movlw 0

	movfw	    startmin 
	subwf	    endmin, W
	movwf	    runtimemin	;runtime min holds the minute difference
	btfsc	    STATUS, C	;skip the next line if runtimemin is negative
	goto	    secondcalc  ;this means that the run did not pass the hour
	
negativemin
	    ;runtime minute holds a negative value
	movlw	    d'60'	    ;put 60 into thw working register
	addwf	    runtimemin, F   ;add it to runtim minute to get the correct number of minutes past

secondcalc
		    ;runtime minute holds a positive value
	
	movfw	    startsec
	subwf	    endsec, W
	movwf	    runtimesec
	btfsc	    STATUS, C	    ;skip the next line if runtimesec is negative
	goto	    conversion	    ;this means that the runtimesec value is a positive
	
negativesec
				    ;runtimesec is a negative value
	movlw	    d'60'
	addwf	    runtimesec, F   ;so add 60
	movlw	    0x01
	subwf	    runtimemin, F   ;and subtract a minute
	
conversion
	clrf runtimehundreds                   ;to convert runtimesec into ones and tens
	clrf runtimetens	;reset runtimetens and runtimeones to zero
	clrf runtimeones
	
	movlw d'60'
	movwf multiply_argument
	multiply runtimemin, multiply_argument, runtimeones
	
	movfw runtimesec
	addwf runtimeones, F
	
	;runttimeones holds the total number of seconds
	
	regtobcd runtimeones, runtimetens
	regtobcd runtimetens, runtimehundreds
	
	call ClrLCD
	PrintNumber runtimehundreds
	PrintNumber runtimetens
	PrintNumber runtimeones
	
	goto main

hold
	   goto hold


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
		goto	show_RTC

;***************************************
; Setup RTC with time defined by user
;***************************************
set_rtc_time

		rtc_resetAll	;reset rtc

		rtc_set	0x00,	B'10000000'

		;set time
		rtc_set	0x06,	B'00010110'		; Year
		rtc_set	0x05,	B'00000010'		; Month
		rtc_set	0x04,	B'00100011'		; Date
		rtc_set	0x03,	B'00001011'		; Day
		rtc_set	0x02,	B'00000001'		; Hours
		rtc_set	0x01,	B'00101000'		; Minutes
		rtc_set	0x00,	B'00000000'		; Seconds
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

END
