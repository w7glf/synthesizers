; Try to improve Mike interrrupt routine

; FEB 22 2016 Changed "pins" TO GO DIRECT TO freq0 removed other freq
; so there are no choices.
; program to load Silicon Labs synthesizer Si4112 from Microchip PIC 16F688
; FEB 17 2016 
; uses WPUA and RAPU (weak pullup and RA pullup enable) for RA0, RA1 & RA4 
; frequency select using interrupts.  3 pins are used for 8 selections
;
; internally, frequencies are counted from zero to seven but since there is no
; way to flash the LED zero times, the led displays one to eight flashes
;
; initialize PIC, assign registers, load RF registers with 0,
; read pin status, load hex to H M & L registers of IF synth
; enable interrupts start main loop and turn off LED sleep
; wait for interrupt on RA2.
; On interrupt change frequency to a different frequency then read pins
; to get original or new frequency. Either way, whether a new frequency
; or a lock event, the remedy is the same. However in this case there is no
; frequency change.
;
; LDTEB LOW without 10 MHz REF IN,  high when locked. INTF used to wake
; PIC and generate interrupt on rising edge. Lock detect with the Si4112
; is uncertain and not a great indicator ... and can be very slow responding
; with this code, but it's the best I have done so far. 
;
;       PORTS 1 = INPUT 0 = OUTPUT
;     -----------   PORT A    ----------------------------------------
;			RA5	RA4	RA3	RA2	RA1	RA0
;	I/O	0	I	0	INT	I	I	0x17 010111
;	pins 	2	3	4	11	12	13
;	TRISA	0	1	0	1	1	1	0x17 010111
;	IOCA	0	1	0	0	1	1	Ox13 010011	
;	WPUA	0	1	0	0	1	1	0x13 010011
;
; RA0 RA1 AND RA4 ARE SWITCH IN, RA2 IS LDETB IN,RA3(MCLR) AND RA5 ARE UNUSED.
;
;     -----------   PORT C    ----------------------------------------
;	I/O	OUT	O	O	SEN	CLK	DATA	O
;	pins		5	6	7	8	9	10
;			RC5	RC4	RC3	RC2  	RC1 	RC0
;	TRISC		1	0	0	0	0	0	0x20
;
;  RC1 RC2 AND RC3 ARE DATA OUT TO SYNTH, RC4 DRIVES THE LED, RC0 AND RC5 ARE UNUSED
;
;************************************************************************
; 16F688 16F688 16F688 16F688 16F688 16F688 16F688 16F688 16F688 16F688 
;
P=16F688
		include P16F688.inc
		errorlevel  -302        ; suppress message 302 from list file
	    				; RC internal clock
	         __config 0x33E4   	; watchdog timer off 
				    	; power-up timer on
	                            	; code protect   off
	                            	; BOD enabled in run
	                            	; MCLR EXTERNAL
	         radix    hex 
;			 
; 16F688 16F688 16F688 16F688 16F688 16F688 16F688 16F688 16F688 16F688 
;
LED_ON	MACRO
		BSF		PORTC,4		; Turn on LED
		ENDM

LED_OFF	MACRO
		BCF		PORTC,4		; Turn off LED
		ENDM

	cblock	0x20
	reg_H		; High bits of register
	reg_M		; Mid bits of register
	reg_L		; Low bits of register
	regC		; delay register
	regD		; delay register
	regE		; delay register
;
	w_temp		;store a copy of W register while int
	status_temp	;store a copy of Status Reg while int
	endc
;
;************************************************************************
;
	ORG     0x00          	; startup address = 0000
	goto	start
;************************************************************************
;  BEGIN INTERRUPT SERVICE ROUTINE
;
;			store W and STATUS
;
	ORG     0x04       	; interrupt vector location
	MOVWF 	w_temp 		;copy W to temp register,could be in either bank
	SWAPF 	STATUS,W	;swap status to be saved into W
	BCF	STATUS,RP0 	;change to bank 0 regardless of current bank
	MOVWF	status_temp 	;save status to bank 0 register
;
; do stuff
;
;	BTFSS	INTCON, INTF	; skip next if lock lost
;    GOTO	exitint
	LED_ON					; set pin 6 high LED OFF	
retry
    CALL    PowerDownSynth
	CALL	pins		; LED flashes 1 - 8
    CALL    PowerUpSynth
	CALL	dly100
	BTFSS	PORTA, 2	; Are we locked?
	LED_OFF				; set pin 6 high LED ON	
	BTFSC	PORTA, 2	; Loop if no 10 MHz
	GOTO	retry		; if the LED stays on, we are still looping
;;				; it takes some time for PORTA, 2 to change low
exitint
	BCF	INTCON, INTF	; Clear the INT interrupt flag	
 	BCF	INTCON, RAIF 	; Clear the RA interrupt flag.
;
;--------------------------------------------------------------------
;   restore W and Status
;
	SWAPF 	status_temp,W	; swap STATUS_TEMP register into
				; W, sets bank to original state
	MOVWF	STATUS 		; move W into STATUS register
	SWAPF	w_temp,F 	; swap W_TEMP with F
	SWAPF	w_temp,W 	; swap W_TEMP into W
	BSF		INTCON, INTE
	RETFIE 			; Return from interrupt
;
;   END INTERRUPT SERVICE ROUTINES
;*************************************************************************	
; --------------------------------------------------------------------
;
;  Power Synth Routines
;
; --------------------------------------------------------------------
PowerDownSynth
		BCF		PORTC,0
		return

PowerUpSynth
		BSF		PORTC,0
		return

;
;			PORTC INIT
;
start	
	CLRF 	STATUS 		; Bank0
	CLRF 	PORTC 		;   Initialize PORTC by clearing output data latches
	MOVLW	0x3F		;   disable analog inputs
	MOVWF	CMCON0 		;   digital I/O
	BSF 	STATUS, RP0 	; Bank1
	CLRF	ANSEL		;   clear the analog register pins for digital i/o
	BCF	PCON,ULPWUE 	;   DISable ULP Wake-up
	MOVLW 	0x20		;   Value used to initialize data direction
	MOVWF 	TRISC 		;   ALL outputs except RC5 is input
	CLRF 	STATUS 		; Bank0
	BSF	PORTC, 3	;   HOLD SEN ENABLE HIGH UNTIL WRITING
;
;			PORTA INIT
;
	BANKSEL	PORTA 		;
	CLRF	PORTA		;Init PORTA
	MOVLW	07h 		;Set RA<2:0> to
	BANKSEL ANSEL 		;
	CLRF	ANSEL		;digital I/O
	MOVLW 	0x17 		;   10111
	MOVWF 	TRISA 		;   make some inputs See notes
	BCF	PCON,ULPWUE 	;   DISable ULP Wake-up
	CLRF	CMCON1		;   clear the register
	CLRF	PIR1		;   DISABLE int flags in the PIR REGISTER
	BSF 	STATUS, RP0 	; Bank1
	CLRF	PCON		;   deny that any resets occurred
	CLRF	PIE1		;   disable tmr1 - other stuff we don't need
	MOVLW	0x13		;   IOCA register (is in bank1) 010011
	MOVWF 	IOCA		;   set interrupt-on-change  1:0	OPTION_REG, 7	;   CLEAR RA pull ups enable
	BCF	OPTION_REG, 7	;   enable global wpua (0 enable, 1 disable)
	MOVLW	0x13		;   enable weak p/u on RA 0 1  & 4
	MOVWF	WPUA		;   do it (enabled = 1)
	CLRF 	STATUS		; Bank0
	CLRF	INTCON		;  Disable ALL interrupts and clear ALL flags
;
;	
;*************************************************************************
	CALL	dly100		; wait for 4112 to settle 
	CALL	ldzero		; load RF registers with zero
	CALL	pins		; get frequency from reading pins
	CALL	FLandINT	; enable interrupts, go to sleep wait for interrupt
	BSF	PORTC, 0	; Si4112 power up
;
;*************************************************************************
;
;  BEGIN MAIN LOOP
;
mnlp
	SLEEP 
  	nop
	goto	mnlp
;
;  END MAIN LOOP
;
;*************************************************************************
;
;subroutines here
;
FLandINT			; FLags and INTerrupts
	BCF	INTCON, 0	; RAIF - Clear Flag Bit Just In Case
	BCF	INTCON, 1	; Clear INT flag before enabling GIE AND INT
	BCF	INTCON, 3	; do not enable RA int (RAIE)
	BSF	INTCON, 4	; DEBUG INTE - external Interrupt Enable (Must be enabled before sleep)
	BSF	INTCON, 7	; GIE – Global interrupt enable (1=enable)
	RETURN
;*************************************************************************
; pins read RA4,RA1 and RA0, program freq
;
;------RA0 is pin 13 of chip, pin 4 of ICSP-----
;------RA1 is pin 12 of chip, pin 5 of ICSP--------
;------RA4 is pin  3 of chip --------
;
pins
	BTFSC	PORTA, 4	; Which freq?
	GOTO	freq0		; Do 112.235 as default
	GOTO	freq1		; 
;-----------------------------------------------
;		112.235  (897.88/8) Comparison Freq 40 kHz
freq0					; 
;  load registers reg_H reg_M and reg_L 
;	
; Load the MAIN register (0) 0x3C04 // divide by 8
; MSB first
;
	movlw	0x03			; 
	movwf	reg_H			;
	movlw	0xC0			; 		
	movwf	reg_M			;
	movlw	0x40			; 
	movwf	reg_L			;
	CALL	sendreg			; Send data to REGISTERS
;
; Load the phase det register (1) 0x4
; MSB first
;
	movlw	0x00			; 
	movwf	reg_H			;
	movlw	0x00			;
	movwf	reg_M			;
	movlw	0x41			; 
	movwf	reg_L			;
	CALL	sendreg			; Send data to REGISTERS
;
; Load the power down register (2) 0x2 
; MSB first
;
	movlw	0x00			; 
	movwf	reg_H			;
	movlw	0x00			;				  
	movwf	reg_M			;
	movlw	0x22			; 
	movwf	reg_L			;
	CALL	sendreg			; Send data to REGISTERS
;
; Load the IF N register (5) 0x57AF	bits 0-16
;
	movlw	0x05			; 
	movwf	reg_H			;
	movlw	0x7A			; 
	movwf	reg_M			;
	movlw	0xF5			; 22447
	movwf	reg_L			;
	CALL	sendreg			; Send data to REGISTERS
;
; Load the IF R register (8) 0xFA	bits 0-13
; MSB first
; 
	movlw	0x00			; 
	movwf	reg_H			;
	movlw	0x0F			; 
	movwf	reg_M			;
	movlw	0xA8			; 250
	movwf	reg_L			;
	CALL	sendreg			; Send data to REGISTERS
;
	CALL	PowerUpSynth
	RETURN				; DEBUG if called by pins in ISR
;-----------------------------------------------
;		112.2375  (897.9/8) Comparison Freq 100 kHz
freq1					; 
;  load registers reg_H reg_M and reg_L 
;	
; Load the MAIN register (0) 0x3C04 // divide by 8
; MSB first
;
	movlw	0x03			; 
	movwf	reg_H			;
	movlw	0xC0			; 		
	movwf	reg_M			;
	movlw	0x40			; 
	movwf	reg_L			;
	CALL	sendreg			; Send data to REGISTERS
;
; Load the phase det register (1) 0x4
; MSB first
;
	movlw	0x00			; 
	movwf	reg_H			;
	movlw	0x00			;
	movwf	reg_M			;
	movlw	0x41			; 
	movwf	reg_L			;
	CALL	sendreg			; Send data to REGISTERS
;
; Load the power down register (2) 0x2 
; MSB first
;
	movlw	0x00			; 
	movwf	reg_H			;
	movlw	0x00			;				  
	movwf	reg_M			;
	movlw	0x22			; 
	movwf	reg_L			;
	CALL	sendreg			; Send data to REGISTERS
;
; Load the IF N register (5) 0x2313	bits 0-16
;
	movlw	0x02			; 
	movwf	reg_H			;
	movlw	0x31			; 
	movwf	reg_M			;
	movlw	0x35			; 8979
	movwf	reg_L			;
	CALL	sendreg			; Send data to REGISTERS
;
; Load the IF R register (8) 0x64	bits 0-13
; MSB first
; 
	movlw	0x00			; 
	movwf	reg_H			;
	movlw	0x06			; 
	movwf	reg_M			;
	movlw	0x48			; 100
	movwf	reg_L			;
	CALL	sendreg			; Send data to REGISTERS
;
	CALL	PowerUpSynth
	RETURN				; DEBUG if called by pins in ISR
;*********************************************************************
change5
; use with ISR to force a retune of synthesizer LDETB and also gets
; old or new frequency when ISR reads back the old or new frequency
;
; Load the IF N register (5)
; MSB first
;
	movlw	0x03			;
	movwf	reg_H			;
	movlw	0x0E			;
	movwf	reg_M			;
	movlw	0x95			;
	movwf	reg_L			;
	CALL	sendreg			; Send data to REGISTERS
	RETURN
;***********************************************************************
; The RF registers still exist even though there are no RF synthesizers.
;
; Load the RF1 N register (3)
; MSB first
;
ldzero
	movlw	0x00			
	movwf	reg_H			
	movlw	0x00			
	movwf	reg_M			
	movlw	0x03			
	movwf	reg_L			
	CALL	sendreg			; Send data to REGISTERS
;
; Load the RF2 N register (4)
; MSB first
;
	movlw	0x04			
	movwf	reg_L			;
	CALL	sendreg			; Send data to REGISTERS
;
; Load the RF1 R register (6)
; MSB first
;
	movlw	0x06			
	movwf	reg_L			;
	CALL	sendreg			; Send data to REGISTERS

;
; Load the RF2 R register (7)
; MSB first
;
	movlw	0x07			
	movwf	reg_L			;
	CALL	sendreg			; Send data to REGISTERS
	RETURN
;********************************************************************
;
; loads data from the H M and L PIC registers into the synthesizer registers
;
sendreg				;DEBUG stack level 2
	bcf   PORTC, 3  	; sen low to enable writing to reg
	NOP
	NOP
	NOP
	NOP
	btfss reg_H,5		; Bit 22
	call 	zero		; 0
	btfsc reg_H,5		;
	call 	one		; 1
	btfss reg_H,4		; Bit 21
	call 	zero		; 0
	btfsc reg_H,4		;
	call 	one		; 1
	btfss reg_H,3		; Bit 20
	call 	zero		; 0
	btfsc reg_H,3		;
	call 	one		; 1
	btfss reg_H,2		; Bit 19
	call 	zero		; 0
	btfsc reg_H,2		;
	call 	one		; 1
	btfss reg_H,1		; Bit 18
	call 	zero		; 0
	btfsc reg_H,1		;
	call 	one		; 1
	btfss reg_H,0		; Bit 17
	call 	zero		; 0
	btfsc reg_H,0		;
	call 	one		; 1
	btfss reg_M,7		; Bit 16
	call 	zero		; 0
	btfsc reg_M,7		;
	call 	one		; 1
	btfss reg_M,6		; Bit 15
	call 	zero		; 0
	btfsc reg_M,6		;
	call 	one		; 1
	btfss reg_M,5		; Bit 14
	call 	zero		; 0
	btfsc reg_M,5		;
	call 	one		; 1
	btfss reg_M,4		; Bit 13
	call 	zero		; 0
	btfsc reg_M,4		;
	call 	one		; 1
	btfss reg_M,3		; Bit 12
	call 	zero		; 0
	btfsc reg_M,3		;
	call 	one		; 1
	btfss reg_M,2		; Bit 11
	call 	zero		; 0
	btfsc reg_M,2		;
	call 	one		; 1
	btfss reg_M,1		; Bit 10
	call 	zero		; 0
	btfsc reg_M,1		;
	call 	one		; 1
	btfss reg_M,0		; Bit 9
	call 	zero		; 0
	btfsc reg_M,0		;
	call 	one		; 1
	btfss reg_L,7		; Bit 8
	call 	zero		; 0
	btfsc reg_L,7		;
	call 	one		; 1
	btfss reg_L,6		; Bit 7
	call 	zero		; 0
	btfsc reg_L,6		;
	call 	one		; 1
	btfss reg_L,5		; Bit 6
	call 	zero		; 0
	btfsc reg_L,5		;
	call 	one		; 1
	btfss reg_L,4		; Bit 5
	call 	zero		; 0
	btfsc reg_L,4		;
	call 	one		; 1
	btfss reg_L,3		; Bit 4
	call 	zero		; 0
	btfsc reg_L,3		;
	call 	one		; 1
	btfss reg_L,2		; Bit 3
	call 	zero		; 0
	btfsc reg_L,2		;
	call 	one		; 1
	btfss reg_L,1		; Bit 2
	call 	zero		; 0
	btfsc reg_L,1		;
	call 	one		; 1
	btfss reg_L,0		; Bit 1
	call 	zero		; 0
	btfsc reg_L,0		;
	call 	one		; 1
;
	bsf   PORTC, 3		; make sen high to enter DATA AND CLOSE REG
	nop
	nop
	nop
	nop
	nop
	RETURN			; DEBUG stack level 2
;******************************************************************
; Subroutines to send 0 and 1
; sen is low while writing
; data clocked in on rising edge
; 	synth		io1		pic
; sen   24		11		7	portc, bit 3
; data  2		13		9	portc, bit 1
; clock 1		15		8	portc, bit 2
;------------------------------------------------------------------									    
zero    
	bcf   PORTC, 1		; RC1 data LOW
	bsf   PORTC, 2		; set clock RC0 HIGH
	nop
	nop
	nop
	bcf	PORTC, 2	; set clock low
				; all port C outputs are now low
				; sen was low and is still low
	RETURN
;*******************************************************************
one     
	bsf	PORTC, 1  	; RC1 data high
	bsf	PORTC, 2 	; set clock bit RC0 HIGH
	nop
	nop
	nop
	bcf	PORTC, 2	; set clock RC0 LOW
	bcf	PORTC, 1	; all port C outputs are now low
				; sen was low and is still low
	RETURN
;*******************************************************************
;	
BLINK0
	LED_ON				;turn on LED
	CALL	dly200
	LED_OFF				;turn off LED
	CALL	dly200
BLINK9
	LED_ON				;turn on LED
	CALL	dly200
	LED_OFF				;turn off LED
	CALL	dly200
BLINK8
	LED_ON				;turn on LED
	CALL	dly200
	LED_OFF				;turn off LED
	CALL	dly200
BLINK7
	LED_ON				;turn on LED
	CALL	dly200
	LED_OFF				;turn off LED
	CALL	dly200
BLINK6
	LED_ON				;turn on LED
	CALL	dly200
	LED_OFF				;turn off LED
	CALL	dly200
BLINK5
	LED_ON				;turn on LED
	CALL	dly200
	LED_OFF				;turn off LED
	CALL	dly200
BLINK4
	LED_ON				;turn on LED
	CALL	dly200
	LED_OFF				;turn off LED
	CALL	dly200
BLINK3
	LED_ON				;turn on LED
	CALL	dly200
	LED_OFF				;turn off LED
	CALL	dly200
BLINK2
	LED_ON				;turn on LED
	CALL	dly200
	LED_OFF				;turn off LED
	CALL	dly200
BLINK1
	LED_ON				;turn on LED
	CALL	dly200
	LED_OFF				;turn off LED
	CALL	dly200
	RETURN
;*****************************************
; 100  TO 500 MSEC DELAY ROUTINE 
;
dly500	MOVLW   0x82        	 
        MOVWF   regD		
wait5   DECFSZ  regC, F		     	
        GOTO    wait5		
        DECFSZ  regD, F      	
        GOTO    wait5
dly400	MOVLW   0x82        	 		       	
        MOVWF   regD
wait4   DECFSZ  regC, F     	
        GOTO    wait4
        DECFSZ  regD, F      	
        GOTO    wait4
dly300	MOVLW   0x82        	        	
        MOVWF   regD
wait3   DECFSZ  regC, F     	
        GOTO    wait3
        DECFSZ  regD, F      	
        GOTO    wait3
dly200	MOVLW   0x82        	       	
        MOVWF   regD
wait2   DECFSZ  regC, F     	
        GOTO    wait2
        DECFSZ  regD, F      	
        GOTO    wait2
dly100	MOVLW   0x82        	       	
        MOVWF   regD
wait1   DECFSZ  regC, F     	
        GOTO    wait1
        DECFSZ  regD, F      	
        GOTO    wait1
        RETURN
;
;**************************
;  for debug use
LITEON	
	BSF	PORTC, 4
	RETURN
;**************************
LITEOFF
	BCF	PORTC, 4
	RETURN
;**************************
;
        end
