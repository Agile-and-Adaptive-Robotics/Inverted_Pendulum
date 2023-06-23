; This code should be able to: 
; 	Take in a PWM Value (1 byte) and should be programmable with SPI
;	Send digital outputs (for a transistor) that emulate a PWM signal


			.include “m88adef.inc”

			.def	counter			=	r16			; counter value will be stored in r16
			.def	onTotal			=	r17			; onTotal value will be stored in r17
			.def	offTotal		=	r18			; offTotal value will be stored in r18
			.def	currentLevel	=	r19
			.def	on 				=	r20
			.def	mask			=	r21
          	
			.def	delayCounterO	=	r22
			.def	delayCounterI	=	r23
          
			.equ	levels			=	63

			.cseg
			.org	0x00

			ldi		mask,			(1 << PINB0)
			out		DDRB,			mask


start:		in		currentLevel,	SPDR
		
			mov		onTotal,		currentLevel
			ldi		offTotal,		levels
			sub		offTotal,		currentLevel

			inc		counter							; increment counter by 1


			cpi		on,				0				; compare on to 0
			breq	else							; branch to else if on == 0

			; if on != 0

			cp		counter,		onTotal			; compare counter to onTotal
			brlo	finish							; branch to finish if counter < onTotal

			; if counter >= onTotal

			ldi		counter,		0				; load 0 to counter
			ldi		on,				0				; load 0 to on

			rjmp	finish							; jump to finish to skip else

			; if on == 0

else:		cp		counter,		offTotal		; compare counter to offTotal
			brlo	finish							; branch to finish if counter < offTotal

			; if counter >= offTotal

			ldi		counter,		0				; load 0 to counter
			ldi		on,				1				; load 1 to on
		

finish:		out		PORTB,			on				; load on to PORTB


			; DELAY

			ldi		delayCounterO,	255				; load 255 to outer counter

delayOuter:	dec		delayCounterO					; decrement outer counter by 1
		
		
			ldi		delayCounterI,	255				; load 255 to inner counter

delayInner:	dec		delayCounterI					; decrement inner counter by 1

			cpi		delayCounterI,	0				; compare inner counter to 0
			brne	delayInner						; branch to delayInner if inner counter != 0

			cpi		delayCounterO,	0				; compare outer counter to 0
			brne	delayOuter						; branch to delayOuter if inner counter != 0
		

			rjmp	start


