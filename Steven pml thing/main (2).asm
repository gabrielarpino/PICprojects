;------------------------------
; Program Name: Demo Display
; Program Description: This code demonstrates the functionality of the Microcontroller Board
; Written For: Aero-Design Lab Board Display
; Date: May-August 2012 
;------------------------------

    list p=16f877					; Define the processor
    #include <p16f877.inc>       	; Processor specific variable definitions
	#include <lcd.inc>				; Import LCD control functions
	#include <rtc_macros.inc>		; Import RTC control functions

	__CONFIG _CP_OFF & _WDT_OFF & _BODEN_OFF & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _CPD_OFF & _LVP_OFF	
									; Initial standard configurations

	udata_shr
	empty		res 1				; Empty slot
	hold		res 1				; Variables used for storage 
	eepromAdd	res	1

	udata		0x25
	counth		res 1				; Constants used in delay
	countm		res	1		
	countl		res 1
	counter.	res 1				; Contsant used in displaying
	temp		res 1				; Variable used for storage
	workTemp1	res	1				; Variables used in interrupts
	statusTemp1	res 1
	countmTemp	res 1
	countlTemp	res 1
	
	counter1	res 1				; Variables used in time measurement
	counter2	res 1
	counter3	res 1

	stateFlags	res 1				; State indicators   (0:DCsPIC     , 1:DCsSens
									;					  2:ServoPIC   , 3:ServoSens
									; 					  4:StepperPIC , 5:StepperSens)
	navigation	res 1				; Naigation variable (0:First      , 1:Second)
	onOff  		res 1				; ON/OFF indicator   (0:Servo      , 1:Stepper)
	stepSpeed   res 1				; Stepper speed 	 (0:Slow	   , 1:Medium
									;					  2: fast)


;------------------------------
; Display Macro
;------------------------------
Display		macro	Message			; Macro name is Display, input is Message
	local		loop.				; Define local variables
	local 		end.
	
	clrw							; Clear WORK and counter registers
	clrf		counter.

loop.
	movf		counter., W			; counter -> WORK	
	
	call		Message
	xorlw		b'00000000'			; Check WORK to see if 0 is returned
	btfsc		STATUS, Z			; If Z of STATUS is set
		goto		end.			; End
	call		WR_DATA				; Else if Z is clear, send data to LCD 
	incf		counter.			; counter = counter + 1 
	goto		loop.				; Repeat the loop

end.
	endm							; End of macro 



;------------------------------
; Change Page
;------------------------------
PageChange	macro	tableIndex
	movwf		temp				; Save the index
	movlw		HIGH	tableIndex
	movwf		PCLATH				; Store it for next jump
	movf		temp, W
	addlw		LOW		tableIndex	; Compute its Offset
	btfsc		STATUS, C			
		incf	PCLATH, F			; If in next, PCLATH = PCLATH + 1
	movwf		PCL					; Write the correct address to PC

	endm							; End of macro



;------------------------------
; Clock Change
;------------------------------
CLKChange		macro	output
	local		Continue

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00000000'			; Check if 1 was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 
		goto		$+4	
	movlw		b'00000000'			; If 1 is pressed, use 0% duty cycle
	movwf		output				; Change the PWM duty cycle on RC1/2 
	goto		Continue			; move on

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'
	xorlw		b'00000001'			; Check if 2 was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 
		goto		$+4	
	movlw		b'10010101'			; If 2 is pressed, use 60% duty cycle 
	movwf		output				; Change the PWM duty cycle on RC1/2 
	goto		Continue			; move on

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'
	xorlw		b'00000010'			; Check if 3 was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 
		goto		$+4	
	movlw		b'11000111'			; If 3 is pressed, use 80% duty cycle 
	movwf		output				; Change the PWM duty cycle on RC1/2 
	goto		Continue			; move on

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'
	xorlw		b'00000100'			; Check if 4 was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 
		goto		$+4	
	movlw		b'11111111'			; If 4 is pressed, use 100% duty cycle 
	movwf		output				; Change the PWM duty cycle on RC1/2 
	goto		Continue			; move on

Continue
	endm							; End of macro



;------------------------------
; Start line
;------------------------------
	ORG			0x0000				; Reset Vector	
	clrf		PCLATH				; Ensure the page bits are clear 
	goto		Main				; Go to the main part 



;------------------------------
; Interrupt Vector Location 
;------------------------------
	ORG			0x004				; Interrupt Vector



;------------------------------
; Interrupt Service Routine
;------------------------------
ISR

	; Identify source of the interrupt, and handle is accordingly
	btfsc		INTCON,RBIF
		goto		KeyPadInterrupt 
	btfsc		INTCON,T0IF
		goto		TimerInterrupt


KeyPadInterrupt
	bcf			INTCON,RBIE			; Disable Port Change 
	bcf			INTCON,RBIF 		; Clear Port Change Flag
	movwf		workTemp1			; Save W temporarily
	movf		STATUS,W			; Copy STATUS into W
	movwf		statusTemp1			; Save STATUS temporarily

	btfss		PORTB,1
		goto		DoNothing 

	; Identify time of the interrupt, and handle it accordingly
	btfsc		stateFlags,0
		goto		DCsPICInterrupt 
	btfsc		stateFlags,1
		goto		DCsSensInterrupt
	btfsc		stateFlags,2
		goto		ServoPICInterrupt
	btfsc		stateFlags,3
		goto		ServoSensInterrupt
	btfsc		stateFlags,4
		goto		StepperPICInterrupt
	btfsc		stateFlags,5
		goto		StepperSensInterrupt

DCsPICInterrupt
	bcf			STATUS,RP0			; Bank0
	bcf			STATUS,RP1

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00001111'			; Check if D was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 

		goto		$+4
	bcf			navigation,0
	bsf			navigation,1
	goto		Done

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00001101'			; Check if 0 was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 

		goto		$+4
	bsf			navigation,0
	bcf			navigation,1
	goto		Done

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'
	xorlw		b'00000011'			; Check if A was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 
		goto		$+3
	; If A is pressed, go forward
	bsf			PORTC,0				; Switch the direction
	goto		Done
	
	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'
	xorlw		b'00000111'			; Check if B was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 
		goto		$+3
	; If B is pressed, go backward
	bcf			PORTC,0				; Switch the direction
	goto		Done
	
	; Check and change the speed 
	CLKChange 	CCPR2L
	goto		Done


DCsSensInterrupt
	bcf			STATUS,RP0			; Bank0
	bcf			STATUS,RP1

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00001110'			; Check if # was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 

		goto		$+4
	bcf			navigation,0
	bsf			navigation,1
	goto		Done

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00001101'			; Check if 0 was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 

		goto		$+4
	bsf			navigation,0
	bcf			navigation,1
	goto		Done	

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00001011'			; Check if C was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 

		goto		$+5
	bcf			navigation,0
	bcf			navigation,1
	bsf			navigation,2
	goto		Done

	goto		Done



ServoPICInterrupt
	bcf			STATUS,RP0			; Bank0
	bcf			STATUS,RP1

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00001111'			; Check if D was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 

		goto		$+4
	bcf			navigation,0
	bsf			navigation,1
	goto		Done

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00001101'			; Check if 0 was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 

		goto		$+4
	bsf			navigation,0
	bcf			navigation,1
	goto		Done


ServoSensInterrupt
	bcf			STATUS,RP0			; Bank0
	bcf			STATUS,RP1

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00001110'			; Check if # was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 

		goto		$+4
	bcf			navigation,0
	bsf			navigation,1
	goto		Done

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00001101'			; Check if 0 was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 

		goto		$+4
	bsf			navigation,0
	bcf			navigation,1
	goto		Done

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00001011'			; Check if C was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 

		goto		$+5
	bcf			navigation,0
	bcf			navigation,1
	bsf			navigation,2
	goto		Done	

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'
	xorlw		b'00000011'			; Check if A was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 
		goto		$+3
	; If A is pressed, put on ON 
	bsf			onOff,0				; Switch to ON
	goto		Done
	
	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'
	xorlw		b'00000111'			; Check if B was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 
		goto		$+2
	; If B is pressed, put on OFF
	bcf			onOff,0				; Switch to OFF
	goto		Done



StepperPICInterrupt
	bcf			STATUS,RP0			; Bank0
	bcf			STATUS,RP1

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00001111'			; Check if D was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 

		goto		$+4
	bcf			navigation,0
	bsf			navigation,1
	goto		Done

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00001101'			; Check if 0 was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 

		goto		$+4
	bsf			navigation,0
	bcf			navigation,1
	goto		Done

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'
	xorlw		b'00000011'			; Check if A was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 
		goto		$+4
	; If A is pressed, go forward
	bsf			PORTC,5				; Switch the direction
	bcf			PORTC,6
	goto		Done
	
	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'
	xorlw		b'00000111'			; Check if B was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 
		goto		$+4
	; If B is pressed, go backward
	bcf			PORTC,5				; Switch the direction
	bsf			PORTC,6
	goto		Done

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'
	xorlw		b'00000000'			; Check if 1 was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 
		goto		$+3
	; If 1 is pressed, put on OFF 
	bcf			onOff,1				; Switch to OFF
	goto		Done
	
	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'
	xorlw		b'00000001'			; Check if 2 was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 
		goto		$+5
	; If 2 is pressed, put on ON, with speed 1
	bsf			onOff,1				; Switch to ON
	movlw		b'00000001'			; Speed 1
	movwf		stepSpeed
	goto		Done

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'
	xorlw		b'00000010'			; Check if 3 was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 
		goto		$+5
	; If 3 is pressed, put on ON, with speed 2
	bsf			onOff,1				; Switch to ON
	movlw		b'00000010'			; Speed 2
	movwf		stepSpeed
	goto		Done		

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'
	xorlw		b'00000100'			; Check if 4 was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 
		goto		$+5
	; If 4 is pressed, put on ON, with speed 3
	bsf			onOff,1				; Switch to ON
	movlw		b'00000100'			; Speed 3
	movwf		stepSpeed
	goto		Done

	goto 		Done



StepperSensInterrupt
	bcf			STATUS,RP0			; Bank0
	bcf			STATUS,RP1

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00001110'			; Check if # was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 

		goto		$+4
	bcf			navigation,0
	bsf			navigation,1
	goto		Done

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00001101'			; Check if 0 was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 

		goto		$+4
	bsf			navigation,0
	bcf			navigation,1
	goto		Done

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'						
	xorlw		b'00001011'			; Check if C was pressed
	btfss		STATUS,Z			; Check if Z of STATUS is set 

		goto		$+5
	bcf			navigation,0
	bcf			navigation,1
	bsf			navigation,2	
	goto		Done

	goto		Done



Done
	movf		statusTemp1, W		; Copy back pre-ISR STATUS 
	movwf		STATUS				
	swapf		workTemp1, F		; Copy back pre-ISR	W
	swapf		workTemp1, W	
DoNothing
	clrf		PORTB
	bsf			INTCON,RBIE			; Enable Port Change again 
	goto		InterruptOver		; Finish ISR




TimerInterrupt
	bcf			INTCON,T0IE			; Disable Timer0 Interrupt
	movwf		workTemp1			; Save W temporarily
	movf		STATUS,W			; Copy STATUS into W
	movwf		statusTemp1			; Save STATUS temporarily

	; Identify time of the interrupt, and handle it accordingly
	btfsc		stateFlags,1
		goto		DCsSensTimer
	btfsc		stateFlags,2
		goto		ServoOnOffTimer	
	btfsc		stateFlags,3
		goto		ServoSensTimer
	btfsc		stateFlags,4
		goto		StepperOnOffTimer
	btfsc		stateFlags,5
		goto		StepperSensTimer

DCsSensTimer
	btfsc		PORTE,0				; Every 20 ms, check sensor 1 status
		goto		$+3    			; If it's high, run the motor
	clrf		CCPR2L				; Else, stop the motor
	goto		TimerOver
	movlw		b'11111111'	
	movwf		CCPR2L				; Use 100% duty cycle
	goto		TimerOver			; Go to Timer Over

ServoSensTimer
	btfsc		PORTE,1				; Every 20 ms, check sensor 1 status
		goto		$+3    			; If it's high, turn ON
	bcf			onOff,0				; Else, turn OFF
	goto		ServoOnOffTimer
	bsf			onOff,0
	goto		ServoOnOffTimer		; Go to ON/OFF Timer
ServoOnOffTimer
	btfss		onOff,0				; If motor is set on ON, pulse it (50 Hz, 10%)
		goto		TimerOver		; Else, do nothing 
	movf		countm,W			; Every 20 ms, pulse the motor 
	movwf		countmTemp			; Backup delay counters
	movf		countl,W
	movwf		countlTemp
	bsf			PORTB,0				; Pulse the motor
	call		DelayTwoMilliSec	; Delay for 2 milli seconds
	bcf			PORTB,0				
	movf		countmTemp,W		; Copy back delay counters
	movwf		countm
	movf		countlTemp,W
	movwf		countl

	movlw		d'60'				; Reset TMR0 for correct frequency
	movwf		TMR0

	goto		TimerOver


StepperSensTimer
	btfsc		PORTE,2				; Every 20 ms, check sensor 3 status
		goto		$+3    			; If it's high, run the motor
	bcf			onOff,1
	goto		StepperOnOffTimer
	bsf			onOff,1
	movlw		b'00000100'			; Run with maximum speed
	movwf		stepSpeed
StepperOnOffTimer
	btfss		onOff,1				; If motor is set on ON, pulse it (50%)  
		goto		TimerOver		; Else, do nothing 

	; Swap the state of high/low pulse
	btfsc		PORTC,2				; If it was low, set it high
		goto		$+3				; Else if it was high, set it low
	bsf			PORTC,2
	goto		$+2
	bcf			PORTC,2

	; Identify speed chosen, and handle it accordingly
	btfsc		stepSpeed,0
		goto		Speed1
	btfsc		stepSpeed,1
		goto		Speed2	
	btfsc		stepSpeed,2
		goto		Speed3

Speed1
	movlw		d'0'				; Set frequency to 38.15 Hz
	movwf		TMR0
	goto		TimerOver			; Go to Timer Over 

Speed2
	movlw		d'100'				; Set frequency to 62.60 Hz	
	movwf		TMR0
	goto		TimerOver			; Go to Timer Over 

Speed3
	movlw		d'200'				; Set frequency to 174.39 Hz
	movwf		TMR0		
	goto		TimerOver			; Go to Timer Over 
	
		
TimerOver		
	bcf			INTCON,T0IF			; Clear Timer0 Interrupt Flag 

	movf		statusTemp1,W		; Copy back pre-ISR STATUS 
	movwf		STATUS				
	swapf		workTemp1,F			; Copy back pre-ISR	W
	swapf		workTemp1,W	
	
	bsf			INTCON,T0IE			; Enable Timer0 Interrupt
	goto		InterruptOver		; Finish ISR
	


InterruptOver
	retfie							; Return



;------------------------------
; Main Code
;------------------------------
Main

	; Set up Input/Output port directions
	bsf			STATUS,RP0			; Bank1	
	movlw		b'00000000'
	movwf		TRISA
	movlw		b'11110010'
	movwf		TRISB				
	movlw		b'00011000'
	movwf		TRISC
	movlw		b'00000000'
	movwf		TRISD
	movlw		b'00000111'
	movwf		TRISE		

	; Set up analog/digital state of the pins  
    movlw 	    0x0F           		;Turn off A/D conversion
    movwf   	ADCON1 
	
	; Initialize the ports 
	bcf			STATUS,RP0			; Bank0
	clrf		PORTA				; Clear all of the PORTs 
	clrf		PORTB
	clrf 		PORTC
	clrf		PORTD
	clrf		PORTE

	; Initialize variables
	clrf		stateFlags

	; Set up the general timer (Timer0)
	bcf			STATUS,RP0			; Bank0
	bcf			STATUS,RP0
	movlw		d'60'
	movwf		TMR0				; Set initial value of 60 in TMR0 to produce 50 Hz
	bsf			STATUS,RP0			; Bank3 
	bsf			STATUS,RP1
	movlw		b'10000111' 		; Set to internal clock, falling edge, TMR0, and prescaler 1:256
	movwf		OPTION_REG
	bcf			STATUS,RP0			; Bank0
	bcf			STATUS,RP1
	bcf			INTCON,T0IF			; Clear Timer0 Interrupt Flag 

	; Set up Pulse Width Modulation (PWM)
	bsf			STATUS,RP0			; Bank1
	movlw		b'11111001'			; Configure PR2 with 10 kHz
	movwf		PR2
	bcf			STATUS,RP0			; Bank0
	movlw		b'00001111'			; Configure RC1 and RC2 as PWM outputs 
	movwf		CCP2CON				; RC1
	movlw		b'00000100'			; Configure Timer2
	movwf		T2CON				; Set to prescaler 1:1, postscaler 1:1 , enabled

	; Initialize motor variable
	clrf		CCPR2L				; Set RC1 to 0% duty cycle
	bcf			PORTB,0
	bcf			PORTC,0
	bcf			PORTC,2
	bsf			PORTC,5
	bcf			PORTC,6

	; Set up I2C for communication
	call 	   	i2c_common_setup
;	rtc_resetAll					; Reset all of the RTC values

	; Set up RTC for a specific time 
; 	call	   	SetRTCTime

	; Initialize the LCD
	call		InitLCD

	Display 	Table1				; "Welcome, "
	clrf		PCLATH
	call		DelayOne
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table2				; "Dear User!"
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds

	call 		ClearDisplay
	Display 	Table3				; "This is PML."
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds	

MotorList
	; Turn off all motors
	clrf		CCPR2L
	bcf			PORTB,0
	bcf			PORTC,2

	call		ClearDisplay
	call		DelayOne			; Delay for 1 second
	Display 	Table4				; "Please choose "
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table5				; "a motor:"
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds

	call 		ClearDisplay
	Display 	Table6				; "1)DC   2)Servo"
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table7				; "3)Uni-P Stepper"
	clrf		PCLATH

PollMotors1
	btfss		PORTB,1				; Poll until key pressed
	goto		$-1

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'				
	xorlw		b'00000000'			; Check if 1 was pressed
	btfsc		STATUS,Z			
		goto		DCsMan			; If 1 is chosen, go to DCs Manual
	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'				
	clrf		PORTB				; Clear the port
	xorlw		b'00000001'			; Else, check if 2 was pressed
	btfsc		STATUS,Z
		goto		ServoMan		; If 2 is chosen, go to Servo Manual
	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'				
	clrf		PORTB				; Clear the port
	xorlw		b'00000010'			; Else, check if 3 was pressed
	btfss		STATUS,Z
		goto		PollMotors1		; If none pressed yet, wait
	goto		StepperMan			; If 3 is chosen, go to Stepper Manual



; Controlling the DC motor
DCsMan
	; Turn off all motors
	clrf		CCPR2L
	bcf			PORTB,0
	bcf			PORTC,2
	; No interrupt
	clrf		INTCON
	call 		ClearDisplay
	call		DelayOne			; Delay for 1 second
	Display 	Table8				; "Manual Control."
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table9				; "#:PIC   D:Sensor"
	clrf		PCLATH
PollDCs1
	btfss		PORTB,1				; Poll until key pressed
	goto		$-1

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'				
	xorlw		b'00001110'			; Check if # was pressed
	btfsc		STATUS,Z			
		goto		DCsPIC			; If # is chosen, go to DCs PIC
	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'				
	clrf		PORTB				; Clear the port
	xorlw		b'00001111'			; Else, check if D was pressed
	btfsc		STATUS,Z
		goto		DCsSens 		; If D is chosen, go to DCs Sensor
	goto		PollDCs1			; Else, repeat

DCsPIC
	; Turn off all motors
	clrf		CCPR2L
	bcf			PORTB,0
	bcf			PORTC,2
	clrf		stateFlags
	bsf			stateFlags,0		; Adjust the state flags
	clrf		navigation			; Clear navigation
	; Set up the port change interrupt
	clrf		INTCON
	bcf			INTCON,RBIF			; Clear Port Change Flag
	bsf			INTCON,RBIE 		; Enable Port Change Interrupts
	bsf			INTCON,GIE			; Enable Global Interrupts
PollDCs2
	call 		ClearDisplay
	call		DelayOne			; Delay for 1 second
	Display 	Table10				; "PIC Control."
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table11				; "0:Man.  D:Sensor"
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds 
	btfsc		navigation,0
		goto		DCsMan
	btfsc		navigation,1
		goto		DCsSens
	call 		ClearDisplay
	Display 	Table11				; "0:Man.  D:Sensor"
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table12				; "A/B:Direction"
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds 
	btfsc		navigation,0
		goto		DCsMan
	btfsc		navigation,1
		goto		DCsSens
	call 		ClearDisplay
	Display 	Table12				; "A/B:Direction"
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table13				; "1/2/3/4:Speed"
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds
	btfsc		navigation,0
		goto		DCsMan
	btfsc		navigation,1
		goto		DCsSens
	goto		PollDCs2	

DCsSens
	; Turn off all motors
	clrf		CCPR2L
	bcf			PORTB,0
	bcf			PORTC,2
	clrf		stateFlags
	bsf			stateFlags,1		; Adjust the state flags
	clrf		navigation			; Clear navigation
	; Set up the port change interrupt
	clrf		INTCON
	bcf			INTCON,RBIF			; Clear Port Change Flag
	bsf			INTCON,RBIE 		; Enable Port Change Interrupts
	bsf			INTCON,T0IE			; Enable Timer Interrupts
	bsf			INTCON,GIE			; Enable Global Interrupts
PollDCs3
	btfsc		PORTE,0
		bsf			PORTB,1
	call 		ClearDisplay
	call		DelayOne			; Delay for 1 second
	Display 	Table14				; "Sensor Control."
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table15				; "0:Man.     #:PIC"
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds 
	btfsc		navigation,0
		goto		DCsMan
	btfsc		navigation,1
		goto		DCsPIC
	btfsc		navigation,2
		goto		MotorList
	call 		ClearDisplay
	Display 	Table15				; "0:Man.     #:PIC"
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table16				; "C:Switch motor"
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds
	btfsc		navigation,0
		goto		DCsMan
	btfsc		navigation,1
		goto		DCsPIC
	btfsc		navigation,2
		goto		MotorList
	goto		PollDCs3



; Controlling the Servo motor
ServoMan
	; Turn off all motors
	clrf		CCPR2L
	bcf			PORTB,0
	bcf			PORTC,2
	; No interrupt
	clrf		INTCON
	call 		ClearDisplay
	call		DelayOne			; Delay for 1 second
	Display 	Table8				; "Manual Control."
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table9				; "#:PIC   D:Sensor"
	clrf		PCLATH
PollServo1
	btfss		PORTB,1				; Poll until key pressed
	goto		$-1

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'				
	xorlw		b'00001110'			; Check if # was pressed
	btfsc		STATUS,Z			
		goto		ServoPIC		; If # is chosen, go to DCs PIC
	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'				
	clrf		PORTB				; Clear the port
	xorlw		b'00001111'			; Else, check if D was pressed
	btfsc		STATUS,Z
		goto		ServoSens		; If D is chosen, go to DCs Sensor
	goto		PollServo1			; Else, repeat

ServoPIC
	; Turn off all motors
	clrf		CCPR2L
	bcf			PORTB,0
	clrf		stateFlags
	bsf			stateFlags,2		; Adjust the state flags
	clrf		navigation			; Clear navigation
	clrf		onOff				; Clear ON/OFF indicator 
	; Set up the port change and timer interrupts
	clrf		INTCON
	bcf			INTCON,RBIF			; Clear Port Change Flag
	bsf			INTCON,RBIE 		; Enable Port Change Interrupts
	bsf			INTCON,T0IE			; Enable Timer Interrupts
	bsf			INTCON,GIE			; Enable Global Interrupts
PollServo2
	call 		ClearDisplay
	call		DelayOne			; Delay for 1 second
	Display 	Table10				; "PIC Control."
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table11				; "0:Man.  D:Sensor"
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds 
	btfsc		navigation,0
		goto		ServoMan
	btfsc		navigation,1
		goto		ServoSens
	call 		ClearDisplay
	Display 	Table11				; "0:Man.  D:Sensor"
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table17				; "A/B:ON/OFF"
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds
	btfsc		navigation,0
		goto		ServoMan
	btfsc		navigation,1
		goto		ServoSens
	goto		PollServo2	

ServoSens
	; Turn off all motors
	clrf		CCPR2L
	bcf			PORTB,0
	bcf			PORTC,2
	clrf		stateFlags
	bsf			stateFlags,3		; Adjust the state flags
	clrf		onOff				; Clear ON/OFF indicator 
	clrf		navigation			; Clear navigation
	; Set up the port change interrupt
	clrf		INTCON
	bcf			INTCON,RBIF			; Clear Port Change Flag
	bsf			INTCON,RBIE 		; Enable Port Change Interrupts
	bsf			INTCON,T0IE			; Enable Timer Interrupts
	bsf			INTCON,GIE			; Enable Global Interrupts
PollServo3
	call 		ClearDisplay
	call		DelayOne			; Delay for 1 second
	Display 	Table14				; "Sensor Control."
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table15				; "0:Man.     #:PIC"
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds 
	btfsc		navigation,0
		goto		ServoMan
	btfsc		navigation,1
		goto		ServoPIC
	btfsc		navigation,2
		goto		MotorList
	call 		ClearDisplay
	Display 	Table15				; "0:Man.     #:PIC"
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table16				; "C:Switch motor"
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds
	btfsc		navigation,0
		goto		ServoMan
	btfsc		navigation,1
		goto		ServoPIC
	btfsc		navigation,2
		goto		MotorList
	goto		PollServo3



; Controlling the Stepper motor
StepperMan
	; Turn off all motors
	clrf		CCPR2L
	bcf			PORTB,0
	bcf			PORTC,2
	; No interrupt
	clrf		INTCON
	call 		ClearDisplay
	call		DelayOne			; Delay for 1 second
	Display 	Table8				; "Manual Control."
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table9				; "#:PIC   D:Sensor"
	clrf		PCLATH
PollStepper1
	btfss		PORTB,1				; Poll until key pressed
	goto		$-1

	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'				
	xorlw		b'00001110'			; Check if # was pressed
	btfsc		STATUS,Z			
		goto		StepperPIC		; If # is chosen, go to Stepper PIC
	swapf		PORTB, W			; Copy RB4-7 into W0-3
	andlw		h'0F'				
	clrf		PORTB				; Clear the port
	xorlw		b'00001111'			; Else, check if D was pressed
	btfsc		STATUS,Z
		goto		StepperSens		; If D is chosen, go to Stepper Sensor
	goto		PollStepper1		; Else, repeat

StepperPIC
	; Turn off all motors
	clrf		CCPR2L
	bcf			PORTB,0
	bcf			PORTC,2
	; Reset the motor
	bsf			PORTC,5
	bsf			PORTC,6
	clrf		stateFlags
	bsf			stateFlags,4		; Adjust the state flags
	clrf		navigation			; Clear navigation
	clrf		onOff				; Clear ON/OFF indicator 
	clrf		stepSpeed			; Clear speed indicator
	; Set up the port change and timer interrupts
	clrf		INTCON
	bcf			INTCON,RBIF			; Clear Port Change Flag
	bsf			INTCON,RBIE 		; Enable Port Change Interrupts
	bsf			INTCON,T0IE			; Enable Timer Interrupts
	bsf			INTCON,GIE			; Enable Global Interrupts
PollStepper2
	call 		ClearDisplay
	call		DelayOne			; Delay for 1 second
	Display 	Table10				; "PIC Control."
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table11				; "0:Man.  D:Sensor"
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds 

	btfsc		navigation,0
		goto		StepperMan
	btfsc		navigation,1
		goto		StepperSens
	call 		ClearDisplay
	Display 	Table11				; "0:Man.  D:Sensor"
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table12				; "A/B:Direction"
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds 

	btfsc		navigation,0
		goto		StepperMan
	btfsc		navigation,1
		goto		StepperSens

	call 		ClearDisplay
	Display 	Table12				; "A/B:Direction"
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table13				; "1/2/3/4:Speed"
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds

	btfsc		navigation,0
		goto		StepperMan
	btfsc		navigation,1
		goto		StepperSens

	goto		PollStepper2	

StepperSens
	; Turn off all motors
	clrf		CCPR2L
	bcf			PORTB,0
	bcf			PORTC,2
	bsf			PORTC,5
	bcf			PORTC,6
	clrf		stateFlags
	bsf			stateFlags,5		; Adjust the state flags
	clrf		navigation			; Clear navigation
	clrf		onOff				; Clear ON/OFF indicator 
	clrf		stepSpeed			; Clear speed indicator
	; Set up the port change and timer interrupts
	clrf		INTCON
	bcf			INTCON,RBIF			; Clear Port Change Flag
	bsf			INTCON,RBIE 		; Enable Port Change Interrupts
	bsf			INTCON,T0IE			; Enable Timer Interrupts
	bsf			INTCON,GIE			; Enable Global Interrupts
PollStepper3
	call 		ClearDisplay
	call		DelayOne			; Delay for 1 second
	Display 	Table14				; "Sensor Control."
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	Display 	Table15				; "0:Man.     #:PIC"
	clrf		PCLATH
	call		DelayTwo			; Delay for 2 seconds 
	btfsc		navigation,0
		goto		StepperMan
	btfsc		navigation,1
		goto		StepperPIC
	btfsc		navigation,2
		goto		MotorList
	call 		ClearDisplay
	Display 	Table15				; "0:Man.     #:PIC"
	clrf		PCLATH
	movlw		b'11000000'			; Go to next line
	call 		WR_INS
	btfsc		navigation,0
		goto		StepperMan
	btfsc		navigation,1
		goto		StepperPIC
	btfsc		navigation,2
		goto		MotorList	
	goto		PollStepper3



;------------------------------
; Loop Up Table
;------------------------------

Table1
PageChange		Statement1
Statement1
	dt			"Welcome,", 0

Table2
PageChange		Statement2
Statement2
	dt			"Dear User!", 0

Table3
PageChange		Statement3
Statement3
	dt			"This is PML. ", 0

Table4
PageChange		Statement4
Statement4
	dt			"Please choose ", 0

Table5
PageChange		Statement5
Statement5
	dt			"a motor:", 0

Table6
PageChange		Statement6
Statement6
	dt			"1)DC   2)Servo", 0 

Table7
PageChange		Statement7
Statement7
	dt			"3)Uni-P Stepper", 0

Table8
PageChange		Statement8	
Statement8
	dt			"Manual Control.", 0

Table9
PageChange		Statement9
Statement9
	dt			"#:PIC   D:Sensor", 0

Table10
PageChange		Statement10	
Statement10
	dt			"PIC Control.",0

Table11
PageChange		Statement11
Statement11
	dt			"0:Man.  D:Sensor",0

Table12
PageChange		Statement12
Statement12
	dt			"A/B:Direction", 0

Table13
PageChange		Statement13
Statement13
	dt			"1/2/3/4:Speed", 0

Table14
PageChange		Statement14
Statement14
	dt			"Sensor Control.", 0

Table15
PageChange		Statement15
Statement15
	dt			"0:Man.     #:PIC", 0

Table16
PageChange		Statement16
Statement16
	dt			"C:Switch motor", 0

Table17
PageChange		Statement17
Statement17
	dt			"A/B:ON/OFF", 0



;------------------------------
; Set RTC Subroutine
;------------------------------
SetRTCTime
	rtc_resetAll					;reset RTC
	rtc_set		0x00, b'10000000'

; Set the time 
	rtc_set		0x06, b'00010010'	; Year
	rtc_set		0x05, b'00000101'	; Month
	rtc_set		0x04, b'00100010'	; Date
	rtc_set		0x03, b'00100010'	; Day
	rtc_set		0x02, b'00010111'	; Hours
	rtc_set		0x01, b'00110100'	; Minutes
	rtc_set		0x00, b'00000000'	; Seconds

	return



;------------------------------
; Clear Display Subroutine
;------------------------------
ClearDisplay
	movlw		b'00000001'			; Send clearing instruction
	call		WR_INS
	
	return



;------------------------------
; 0.002 Second Delay Subroutine
;------------------------------
DelayTwoMilliSec
	local		TwoMilliS_0
	movlw		0xE7
	movwf		countm
	movlw		0x04
	movwf		countl

TwoMilliS_0
	decfsz		countm, F
	goto		$+2
	decfsz		countl, F
	goto		TwoMilliS_0

	goto		$+1

	return



;------------------------------
; 1 Second Delay Subroutine
;------------------------------	
DelayOne
	local		OneS_0				; Load the counter sections 
    movlw 		0x16
    movwf 		counth
    movlw 		0x74
    movwf 		countm
    movlw 		0x06
    movwf 		countl

OneS_0
    decfsz 		counth, F			; Kill time by looping down the entire counter
    goto   		$+2
    decfsz 		countm, F
    goto   		$+2
    decfsz 		countl, F
    goto   		OneS_0				; Repeat until 1 second has passed

    goto 		$+1
    nop
    nop

	return



;------------------------------
; 2 Second Delay Subroutine
;------------------------------	
DelayTwo
	local		TwoS_0				; Load the counter sections 
    movlw 		0x2D
    movwf 		counth
    movlw 		0xE7
    movwf 		countm
    movlw 		0x0B
    movwf 		countl

TwoS_0
    decfsz 		counth, F			; Kill time by looping down the entire counter
    goto   		$+2
    decfsz 		countm, F
    goto   		$+2
    decfsz 		countl, F
    goto   		TwoS_0				; Repeat until 1 second has passed

    goto 		$+1
    nop
    nop

	return


	END
