#include <avr/io.h> // calls header file for specific Atmel controller
#include <stdio.h> // to use serial port
#include <avr/interrupt.h> // include interrupts
#include <stdint.h>

// Math functions
#include <math.h>
#include <stdlib.h>

#define F_CPU 32000000UL  // 32 MHz  CPU Clock Frequency
#include <util/delay.h>  // include the built in delay fuction

//// macros for baudrate (this fast baud rate causes noise in the toggle pin porta-pin0 in while loop)
//#define BAUD 921600
//#define f_PER 32000000
//#define BSCALE 	-6
//#define BSEL 	75

//Baudrate macros
#define BAUD 460800
#define f_PER 32000000
#define BSCALE 	-5
#define BSEL 	107

/***** Function Prototypes *****/
void clk_init(void);
void timer_init(uint16_t topCount);
void pwmtimer_init(void);
void encoder_init(void);
void usart_init(void);
void pwmtoMotor1(float voltSet1);

//USART declarations, writing only
static int put_char(char c, FILE *stream);
static FILE usart_output = FDEV_SETUP_STREAM(put_char,NULL,_FDEV_SETUP_WRITE); //(A1 - not needed; required for A1U)

/***** Control Parameters *****/
//volatile float 	Ts = 0.008;		//Sample Time set by period of TCC1. Max of 0.00819 with Clk/4 (0.008 = 125 Hz, 0.004 = 250 Hz, 0.001 = 1000 Hz)
//volatile float 	Ts = 0.10;		//Sample Time set by period of TCC1. Max of 0.131 with Clk/64 (0.131 = 7.6 Hz, 0.1 = 10 Hz)
volatile float	Ts = 0;		//Max of 0.524 with Clk/256 (0.524 = 1.9Hz, 0.5 = 2 Hz, 0.2 = 5 Hz, 0.1 = 10 Hz, 0.05 = 20 Hz, 0.01 = 100 Hz) (18hz is Ts = 0.055)
//Max Ts is based on 16 bit timer counter (2^16 = 65536), so Ts(max) = 65536/(clock/clockdiv)

/***** Variable Initialization *****/
volatile uint16_t	topCount = 0;			//TOP value for interrupt timer
volatile float		voltSet1 = 0;			//Starting set voltage motor 1
volatile float		digiHB1 = 0;			//Starting digital H Bridge 1 value
volatile float	maxVoltage = 12;					//global max voltage
volatile int 	encCount1;				//Encoder 1 count at beginning of control loop
volatile int	encCount1_first = 0;		//Encoder count at first loop (offset)
volatile int	encCount1_m1 = 0;
volatile int	encCount1_m2 = 0;
volatile float	position1_now = 0;			//Position of motor 1 in current loop (deg)
volatile float	position1_m1 = 0;			//Position of motor 1 in last loop (deg)
//volatile int	encCount2 = 0;				//Encoder 2 count at beginning of control loop
//volatile int	pos2_now = 0;				//Encoder 2 overflow compare variable
//volatile int	pos2_m1 = 0;
//volatile int	encCount2_m1 = 0;
//volatile int	encCount2_m2 = 0;
volatile uint8_t overFlag1 = 0;				//Flag to mark first overflow interrupt for encoder 1
volatile uint8_t overFlag2 = 0;				//Flag to mark first overflow interrupt for encoder 2
volatile float		revCnt1 = 0;			//Integer revolutions of motor 1
volatile float		revCnt1_m1 = 0;			//Rev count minus 1
volatile float		revCnt2 = 0;			//Integer revolutions of motor 2
volatile float 	deltapos1_ang = 0;			//Wheel 1 change in angular position
volatile float	deltapos1_ang_m1 = 0;
volatile float	velocity1_ang = 0;			//Wheel 1 angular velocity
//volatile float	deltapos2_ang = 0;			//Wheel 2 change in angular position
//volatile float	deltapos2_ang_m1 = 0;

volatile float ctrlOut = 0;					//output for control equation
volatile float ctrlOut_m1 = 0;
volatile float errorNow = 0;			//error for control equation
volatile float error_m1 = 0;


/***** Parameters to Change ******/
/** Control Parameters **/
volatile float Kp = 0.1;					//Proportional Gain
volatile float Ki = 0.05;					//Integral control constant

/** Input Parameters **/
//For sine wave input (starting at horizontal), gen form theta = A*sin(2pi*f*t)
//velocity (analog to voltage) is given by the first derivative omega = A*2pi*f*cos(2pi*f*t)
volatile float amplitude = 45;			//Input amplitude in degrees (as measured from horizontal, not peak to peak)
volatile float frequency = 0.5;			//Input frequency in Hz
volatile uint8_t numberofCycles = 2;	//Number of cycles desires
volatile float timeNow;					//Current time will be calculated in program
volatile float stopTime;				//Total time of data collection (in s) will be calculated in program
volatile float neutralPositionCounter;			//Ending position will be calculated in program as near to neutral as possible
volatile float inputVelocity;			//Desired velocity as calculated from inputs
volatile float commandPosition;			//Desired position as calculated from inputs
volatile float fs = 0;					//Sample frequency calculated in program
volatile uint8_t inputMode = 2;			//for Step or Oscillatory inputs
//volatile float stepInputPosition = 0;	//command position in degrees if Step Input mode is active

#define STEP 1
#define OSCILLATION 2

int main (void)
{
	clk_init();     // Initialize the system clock to 32 MHz
	_delay_ms(200);	// Delay is necessary to let the clock stabilize, otherwise encoder offset problem occurs
	
	//USART setup
	usart_init(); //Initialize serial output
	stdout = &usart_output;	//required for fprint (A1 - not needed; required for A1U)
	
	//H bridge hardware setup and PWM initialization
	PORTF.DIRSET = PIN2_bm|PIN3_bm;	//set output pins for H bridge communication (Pin 2 = INA ("clockwise" input), Pin 3 = INB ("CCW" input))
	
	pwmtimer_init();	//initialize timer type 1 on Port F pin 4 for PWM signal out

	PORTA.DIRCLR = PIN5_bm | PIN6_bm | PIN7_bm;	//set input pins for "manual positioning mode" buttons
	PORTA.PIN5CTRL = PORT_OPC_PULLDOWN_gc;	//Use Pull down resistor
	PORTA.PIN6CTRL = PORT_OPC_PULLDOWN_gc;
	PORTA.PIN7CTRL = PORT_OPC_PULLDOWN_gc;

	if((inputMode == OSCILLATION) & (frequency <= 0.4))
	{
		fs = 20.;
	}
	else
	{
		fs = 50.;
	}

	Ts = 1./fs;

	//Interrupt timer setup
	topCount = (uint16_t)(Ts*125000.);		//Computed TOP value for TCC1 interrupt timer (500000 for div64, 125000 for div256)
	timer_init(topCount);

	while(!(PORTA.IN & PIN7_bm))
	{
		if(PORTA.IN & PIN6_bm)
		{
			pwmtoMotor1(3);
		}
		else if(PORTA.IN & PIN5_bm)
		{
			pwmtoMotor1(-3);
		}
		else
		{
			pwmtoMotor1(0);
		}
	}

	//revCnt1 = 0;	//the
	//cli();	//disable global interrupts (order seems to matter, setting pmic.ctrl register then enabling interrupts, so have to clear before we can reset otherwise we get bursts of voltage to the motor)

	//Set timing pins as output
	PORTD.DIRSET = PIN7_bm;	//set D7 as output for timing pin (peripheral clock 32Mhz)
	PORTD.DIRSET = PIN6_bm;	//set D6 as output for interrupt timing pin (1000 Hz)

	if(inputMode == OSCILLATION)
	{
		stopTime = numberofCycles/frequency;	//Stop time in seconds
	}
	else if(inputMode == STEP)
	{
		stopTime = 10.0;
	}

	encoder_init();	//encoder must be initialized AFTER manual mode, otherwise the medium level interrupt misfires and loads a value into revCnt variable causing voltage burst to motor

	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;	//enable interrupts medium and high levels
	sei();	//enable global interrupts
	
	while(1)
	{
		PORTD.OUTTGL = (1<<7);	//Toggle Pin D7 for oscillator clock timing

		if((timeNow >= stopTime) & !(PORTA.IN & PIN7_bm))	//after program is over if manual override switch is turned back to ground, manual button positioning is enabled
		{
			cli();						//disable interrupts to prevent interruption between CCP end and reset
			CCP = 0xD8;					//configuration change protection: allow protected IO register write
			RST.CTRL = RST_SWRST_bm;	//software reset at end of program
		}
	}

}

ISR(TCC1_OVF_vect)
{
	
	PORTD.OUTTGL = PIN6_bm;	//Toggle Pin D6 for interrupt timing

	if((inputMode == OSCILLATION) & (timeNow <= stopTime))
	//if(neutralPositionCounter != (numberofCycles*2))
	{
		
		commandPosition = amplitude*sin(2*M_PI*frequency*timeNow);	//command position of platform (degrees)
		inputVelocity = amplitude*2*M_PI*frequency*cos(2*M_PI*frequency*timeNow);	//calculated velocity value (dps)
	}
	else if((inputMode == STEP) & (timeNow <= stopTime))
	{
		commandPosition = amplitude;	//Input position in degrees
	}
	else
	{
		voltSet1 = 0;
	}

	////Read encoder 1 and calculate position and velocity
	if(timeNow == 0)
	{
		encCount1_first = TCD1.CNT;
		encCount1_first = encCount1_first;
	}

	encCount1 = TCD1.CNT;
	
	if(encCount1 < 0) encCount1 = 0;		//encoder 1 out of bounds check
	if(encCount1 > 8191) encCount1 = 8191;	//encoder 1 out of bounds check
	
	position1_now = (revCnt1 + (encCount1-encCount1_first)/8191.)*360.;	//Position in degrees
	
	deltapos1_ang = (position1_now - position1_m1);			//calculate change in angular position of motor 1 for current loop (degrees/count = (360 deg/rev)/(8192 counts/rev))
	velocity1_ang = (deltapos1_ang)/(Ts);					//calculate angular velocity of motor 1 for last loop (deg/s)
	
	errorNow = commandPosition - position1_now;
	
	//Controller
	
	//ctrlOut = Kp * errorNow;	//Proportional Controller

	ctrlOut = ((-Kp) + 1./2.*Ki*Ts)*error_m1 + (Kp + 1./2.*Ki*Ts)*errorNow + ctrlOut_m1;	//PI Controller
	//ctrlOut = ctrlOut_m1 + 10*errorNow - 10*error_m1; //not so sure

	voltSet1 = ctrlOut;

	//if(revCnt1 != revCnt1_m1)
	//{
	//neutralPositionCounter += 1;
	//}

	position1_m1 = position1_now;			//store last position of motor 1
	revCnt1_m1 = revCnt1;					//store last rev counter
	error_m1 = errorNow;					//store last error value for motor 1
	ctrlOut_m1 = ctrlOut;					//store last control value for motor 1
	
	//printf("%f \t %f \t %f \t %f \t %f \t %f \n", timeNow, voltSet1, commandPosition, position1_now, inputVelocity, velocity1_ang);
	//printf("%f \t %f \t %f \t %f \t %f \t %f \n", timeNow, voltSet1, commandPosition, position1_now, revCnt1, stopPosition);

	if(abs(voltSet1) >= maxVoltage)
	{
		//printf("\nBumped back in: %f",voltSet1);
		voltSet1 = copysign(maxVoltage,voltSet1);
	} //if greater than max, set value to max with original sign

	pwmtoMotor1(voltSet1);
	
	printf("%f \t %f \t %f \t %f \n", timeNow, voltSet1, commandPosition, position1_now);

	timeNow += Ts;	//Calculate time in program
}

void pwmtoMotor1(float voltSet1)
{
	digiHB1 = voltSet1 / 12.0*1599;	//Convert to digital PWM compare value for HB1
	
	//Motor 1
	if(digiHB1 < 0)
	{
		//PORTF.OUTCLR = PIN2_bm;	//pull HB1 direction pin low for reverse direction
		PORTF.OUTSET = PIN3_bm;	//set INB high for CCW direction
		PORTF.OUTCLR = PIN2_bm;	//set INA
		digiHB1 = -digiHB1;	//make digital HB value positive
		if(digiHB1 >= 1599){digiHB1 = 1599;}	//PWM overflow error check (pulse width cannot exceed top count)
		TCF1.CCA = digiHB1; //Set duty cycle on compare channel A (HB1)
	}
	else
	{
		//PORTF.OUTSET = PIN2_bm;	//pull HB1 direction pin high for forward direction
		PORTF.OUTSET = PIN2_bm;	//set INA high for CW direction
		PORTF.OUTCLR = PIN3_bm;	//set INB low
		if(digiHB1 >= 1599){digiHB1 = 1599;}	//PWM overflow error check (pulse width cannot exceed top count)
		TCF1.CCA = digiHB1;
	}
}

ISR(TCD1_OVF_vect)
{
	if (TCD1.CTRLFSET & TC1_DIR_bm)	//if DIR pin is 0, encoder direction is positive (counting up); if DIR pin is 1, enoder direction is negative (counting down)
	{
		--revCnt1;		//decrement RevCnt1 variable if overflow in negative direction
		overFlag1 = 0;	//the overFlag variable is solely intended to make the double firing problem visible in the positive direction
		//printf("revCnt1 is %d \n\n", revCnt1);
	}
	else if (overFlag1 == 0)
	{
		++revCnt1;		//increment RevCnt1 variable if overflow in positive direction
		overFlag1 = 1;	//when positive overflow from TOP to 0, set flag to 1
		//printf("revCnt1 is %d \n\n", revCnt1);
	}
	else if (overFlag1 == 1)
	overFlag1 = 0;		//when overflow is positive, and flag is set a second time (in error) from 0 to 1, clear the flag
}

void clk_init(void)
{
	OSC.CTRL |= OSC_RC32MEN_bm;					//enable 32Mhz RC Osc
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));		//wait for Osc to be stable
	CCP = CCP_IOREG_gc;							//disables IO protection for 4 clock cycles to permit selection of system clock
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;			//set 32Mhz RC Osc as system clock
}


void timer_init(uint16_t topCount)	//initialization of Interrupt timer
{
	TCC1.CTRLB |= TC_WGMODE_NORMAL_gc;		//Normal mode on Port C, Output Compare pins disconnected
	TCC1.INTCTRLA |= TC_OVFINTLVL_LO_gc;	//Enable overflow interrupt
	TCC1.PER = topCount;					//Set Period
	TCC1.CTRLA |= TC_CLKSEL_DIV256_gc;		//Start at Clk/256
}


void usart_init(void) //(Port C)
{

	PORTC.DIRSET = PIN3_bm;		//Set Tx pin as output
	PORTC.DIRCLR = PIN2_bm;		//Set Rx pin as input (this should be default)
	PORTC.OUTSET = PIN3_bm;		//Set Tx pin high

	//Set mode, baud rate and frame format
	USARTC0.CTRLC |= USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc;
	USARTC0.BAUDCTRLA = (uint8_t)BSEL;
	USARTC0.BAUDCTRLB = (BSCALE<<USART_BSCALE0_bp) | (BSEL>>8);

	//enable Tx (Rx not enabled)
	USARTC0.CTRLB |= USART_TXEN_bm;

	// setup printf to use serial port (A1; not needed for A1U)
	//fdevopen(&put_char,NULL);

}


static int put_char(char c, FILE *stream) //Serial transfer function
{
	if (c == '\n') put_char('\r',stream);	//add return to newline character for term

	while(!(USARTC0.STATUS & USART_DREIF_bm)); //loop until Tx is ready
	USARTC0.DATA = c;
	return 0;
}


void pwmtimer_init(void) //initialization of PWM timer (come back to period of timer)
{
	TCF1.CTRLB = TC_WGMODE_SS_gc | TC1_CCAEN_bm;		//Singleslope PWM mode, enable compare on Channel A
	TCF1.PER = 1599;									//Set Period at 20k Hz (20k = 32e6/(1(top+1))), top = 1599, res = 10.64
	TCF1.CTRLA |= TC_CLKSEL_DIV1_gc;					//Start at Clk/1
	PORTF.DIRSET = PIN4_bm;								//Set Port F channel as output, ChA on pin4
	TCF1.CCA = 0;										//Set starting duty cycle on compare channel A
}


void encoder_init(void)
{
	//set up 2 encoders, do not use index
	PORTD.DIRCLR = PIN0_bm | PIN1_bm | PIN3_bm | PIN4_bm; //set encoder pins as input (Enc1 pins 0-2, Enc2, pins 3-5)
	PORTD.PIN0CTRL |= PORT_ISC_LEVEL_gc;		//set Pin 0 to low level sensing (encoder 1 phase A)
	PORTD.PIN1CTRL |= PORT_ISC_LEVEL_gc;		//set Pin 1 to low level sensing (encoder 1 phase B)
	PORTD.PIN3CTRL |= PORT_ISC_LEVEL_gc;		//set Pin 3 to low level sensing (encoder 2 phase A)
	PORTD.PIN4CTRL |= PORT_ISC_LEVEL_gc;		//set Pin 4 to low level sensing (encoder 2 phase B)

	//setup event system
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTD_PIN0_gc;	//route PIND0 (encoder 1 phase A) to Event channel 0
	EVSYS.CH2MUX = EVSYS_CHMUX_PORTD_PIN3_gc;	//route PIND3 (encoder 2 phase A) to Event channel 2
	EVSYS.CH0CTRL |= EVSYS_QDEN_bm 				//enable quadrature decode for encoder 1
	| EVSYS_DIGFILT_2SAMPLES_gc;				//set digital filter to 2 samples for encoder 1
	EVSYS.CH2CTRL |= EVSYS_QDEN_bm				//enable quadrature decode for encoder 2
	| EVSYS_DIGFILT_2SAMPLES_gc;				//set digital filter to 2 samples for encoder 2

	//setup timer for encoder 1
	TCD1.CTRLA |= TC_CLKSEL_DIV1_gc;			//enables timer type 1 on Port D, at peripheral clock speed
	TCD1.CTRLD |= TC_EVACT_QDEC_gc				//set event action of timer to quadrature decode
	| TC_EVSEL_CH0_gc;							//set EVCH0 as source (encoder 1)
	TCD1.PER = 8191;							//set period based on: (((pulses/rev)*1/1)*4 - 1) of WHEEL (This CUI encoder has 2048 pulses/rev, at 1:1 (gear out/gear in) scale for output shaft of Tiff's motor - mounted after gear box)
	TCD1.INTCTRLA |= TC_OVFINTLVL_MED_gc;		//set priority of timer overflow interrupt to medium

	//setup timer for encoder 2
	TCE1.CTRLA |= TC_CLKSEL_DIV1_gc;			//enables timer type 1 on Port E, at peripheral clock speed
	TCE1.CTRLD |= TC_EVACT_QDEC_gc				//set event action of timer to quadrature decode
	| TC_EVSEL_CH2_gc;							//set EVCH2 as source (encoder 2)
	TCE1.PER = 8191;
	TCE1.INTCTRLA |= TC_OVFINTLVL_MED_gc;
}