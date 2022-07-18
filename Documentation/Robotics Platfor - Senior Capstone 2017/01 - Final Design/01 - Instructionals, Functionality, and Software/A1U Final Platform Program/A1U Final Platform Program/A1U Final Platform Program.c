#include <avr/io.h> // calls header file for specific Atmel controller
#include <stdio.h> // to use serial port
#include <avr/interrupt.h> // include interrupts
#include <stdint.h>

// Math functions
#include <math.h>
#include <stdlib.h>

#define F_CPU 32000000UL  // 32 MHz  CPU Clock Frequency
#include <util/delay.h>  // include the built in delay fuction

//Baudrate macros (2nd fastest baud rate, first fastest caused noise in while loop toggle pin)
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
void spi_init(void);

//USART declarations, writing only
static int put_char(char c, FILE *stream);
static FILE usart_output = FDEV_SETUP_STREAM(put_char,NULL,_FDEV_SETUP_WRITE); 

/***** Control Parameters *****/
//volatile float 	Ts = 0.008;		//Sample Time set by period of TCC1. Max of 0.00819 with Clk/4 (0.008 = 125 Hz, 0.004 = 250 Hz, 0.001 = 1000 Hz)
//volatile float 	Ts = 0.10;		//Sample Time set by period of TCC1. Max of 0.131 with Clk/64 (0.131 = 7.6 Hz, 0.1 = 10 Hz)
//volatile float	Ts = 0.055;		//Max of 0.524 with Clk/256 (0.524 = 1.9Hz, 0.5 = 2 Hz, 0.2 = 5 Hz, 0.1 = 10 Hz, 0.05 = 20 Hz, 0.01 = 100 Hz) (18hz is Ts = 0.055)
//Max Ts is based on 16 bit timer counter (2^16 = 65536), so Ts(max) = 65536/(clock/clockdiv)

/***** Variable Initialization *****/
volatile uint16_t	topCount = 0;			//TOP value for interrupt timer
volatile float		voltSet1 = 0;			//Starting set voltage motor 1
volatile float		digiHB1 = 0;			//Starting digital H Bridge 1 value
volatile float		maxVoltage = 12;		//global max voltage
volatile int 		encCount1;				//Encoder 1 count at beginning of control loop
volatile int		encCount1_first = 0;	//Encoder count at first loop (offset)
volatile int		encCount1_m1 = 0;
volatile int		encCount1_m2 = 0;
volatile float		actualPosition1 = 0;	//Position of motor 1 in current loop (deg)
volatile uint8_t	actualPositionTag1 = 0;	//Tag byte for motor 1 actual position
volatile float		actualPosition1_flt = 0;	//variables for parsing for SPI transaction
volatile int8_t		actualPosition1_int = 0;
volatile int8_t		actualPosition1_dec = 0;
volatile float		position1_m1 = 0;		//Position of motor 1 in last loop (deg)
//volatile int	encCount2 = 0;				//Encoder 2 count at beginning of control loop
//volatile int	pos2_now = 0;				//Encoder 2 overflow compare variable
//volatile int	pos2_m1 = 0;
//volatile int	encCount2_m1 = 0;
//volatile int	encCount2_m2 = 0;
volatile uint8_t	overFlag1 = 0;			//Flag to mark first overflow interrupt for encoder 1
volatile uint8_t	overFlag2 = 0;			//Flag to mark first overflow interrupt for encoder 2
volatile float		revCnt1 = 0;			//Integer revolutions of motor 1
volatile float		revCnt1_m1 = 0;			//Rev count minus 1
volatile float		revCnt2 = 0;			//Integer revolutions of motor 2
volatile float 		deltapos1_ang = 0;		//Wheel 1 change in angular position
volatile float		deltapos1_ang_m1 = 0;
volatile float		velocity1_ang = 0;		//Wheel 1 angular velocity
//volatile float	deltapos2_ang = 0;		//Wheel 2 change in angular position
//volatile float	deltapos2_ang_m1 = 0;

//SPI Variables
volatile uint16_t	slaveRead = 0;
volatile uint8_t	slaveReadlowByte = 0;
volatile uint8_t	slaveReadhighByte;
volatile uint8_t	slaveWritelowByte = 0;
volatile uint8_t	slaveWritehighByte = 0;
volatile uint8_t	fshighbyte = 0b11110000;	//first hex = 0xf, to signal transmission of sample frequency
volatile uint8_t	fslowbyte;

/** Control Variables **/
volatile float ctrlOut1 = 0;				//output for control equation
volatile float ctrlOut1_m1 = 0;
volatile float errorNow1 = 0;				//error for control equation
volatile float error1_m1 = 0;


/***** Parameters to Change ******/
/** Control Parameters **/
volatile float		Kp = 0.;				//Proportional Gain
volatile float		Ki = 0.05;				//Integral control constant
volatile uint8_t	fs = 50;					//Sample frequency			/**(need to complete frequency bands in code)**/
volatile float		Ts = 0;					//Calculated in program - Max of 0.524 with Clk/256 (0.524 = 1.9Hz, 0.5 = 2 Hz, 0.2 = 5 Hz, 0.1 = 10 Hz, 0.05 = 20 Hz, 0.01 = 100 Hz) (18hz is Ts = 0.055)

/** Input Parameters **/
//For sine wave input (starting at horizontal), general form theta = A*sin(2pi*f*t)
//velocity (linearly related to voltage) is given by the first derivative omega = A*2pi*f*cos(2pi*f*t)
volatile uint8_t	amplitudeMotor1_input = 0;
volatile float		amplitudeMotor1 = 0;
volatile uint8_t	amplitudeMotor2_input = 0;
volatile float		amplitudeMotor2 = 0;
volatile uint8_t	frequencyMotor1_input = 0;
volatile float		frequencyMotor1 = 0;
volatile uint8_t	frequencyMotor2_input = 0;
volatile float		frequencyMotor2 = 0;
volatile uint8_t	numberofCycles = 0;			//Number of cycles desired (needed for both motors?)
volatile uint8_t	inputMode = 0;				//step or oscillatory input (not activated - needed for both motors?)
volatile float		timeNow;					//Current time will be calculated in program
volatile float		stopTime;					//Total time of data collection (in s) will be calculated in program
volatile float		neutralPositionCounter;		//Ending position will be calculated in program as near to neutral as possible
volatile float		inputVelocity1;				//Desired velocity as calculated from inputs
volatile float		commandPosition1;			//Desired position as calculated from inputs
volatile uint8_t	commandPositionTag1 = 0;	//Tag byte for motor 1 command position SPI transaction
volatile float		commandPosition1_flt = 0;	//variables for parsing for SPI transaction
volatile int8_t		commandPosition1_int = 0;
volatile int8_t		commandPosition1_dec = 0;

#define STEP 1
#define OSCILLATION 0

int main (void)
{
	clk_init();     // Initialize the system clock to 32 MHz
	_delay_ms(200);	// Delay is necessary to let the clock stabilize, otherwise encoder offset problem occurs
	
	//USART setup
	usart_init(); //Initialize serial output
	stdout = &usart_output;	//required for fprint
	
	//H bridge hardware setup and PWM initialization
	PORTF.DIRSET = PIN2_bm|PIN3_bm;	//set output pins for H bridge communication (Pin 2 = INA ("clockwise" input), Pin 3 = INB ("CCW" input))
	
	pwmtimer_init();	//initialize timer type 1 on Port F pin 4 for PWM signal out

	PORTA.DIRCLR = PIN5_bm | PIN6_bm | PIN7_bm;	//set input pins for "manual positioning mode" buttons
	PORTA.PIN5CTRL = PORT_OPC_PULLDOWN_gc;	//Use Pull down resistor
	PORTA.PIN6CTRL = PORT_OPC_PULLDOWN_gc;
	PORTA.PIN7CTRL = PORT_OPC_PULLDOWN_gc;

	//Interrupt timer setup
	Ts = 0.02;
	Ts = 1./fs;
	topCount = (uint16_t)(Ts*125000.);		//Computed TOP value for TCC1 interrupt timer (500000 for div64, 125000 for div256)
	timer_init(topCount);

	//Enter manual positioning mode, flip switch when done with manual positioning
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

	spi_init();

	//16-bit SPI: transfer command bytes from Raspberry Pi
	for(int ii = 0; ii < 7; ii++)
	{
		if(ii < 6)
		{
			SPIC.DATA = 0;
			PORTA.OUTCLR = PIN3_bm;		//signal RPi that data is ready on first data ready pin

			while(!(SPIC.STATUS & SPI_IF_bm));	//wait until data tx from Rpi is done and flag is set
			PORTA.OUTSET = PIN3_bm;				//set data ready pin high
			slaveReadhighByte = SPIC.DATA;		//Grab byte received from Master
			SPIC.DATA = 0;						//Send zero low byte to Master

			while(!(SPIC.STATUS & SPI_IF_bm)); //wait until data tx from BB is done and flag is set
			slaveReadlowByte = SPIC.DATA; //Grab byte received from Master
		}
		else
		{
			if((inputMode == OSCILLATION) & (frequencyMotor1 <= 0.3))
			{
				fs = 20;		//sample frequency in Hz for high friction low frequency band
			}

			Ts = 1./fs;
			fslowbyte = fs;		//load sample frequency into fs lowbyte

			SPIC.DATA = fshighbyte;
			PORTA.OUTCLR = PIN3_bm;		//signal RPi that data is ready on first data ready pin

			while(!(SPIC.STATUS & SPI_IF_bm));	//wait until data tx from Rpi is done and flag is set
			PORTA.OUTSET = PIN3_bm;				//set data ready pin high
			slaveReadhighByte = SPIC.DATA;		//Grab byte received from Master
			SPIC.DATA = fslowbyte;				//Send zero low byte to Master

			while(!(SPIC.STATUS & SPI_IF_bm)); //wait until data tx from BB is done and flag is set
			slaveReadlowByte = SPIC.DATA; //Grab byte received from Master
		}

		//slaveRead = ((slaveReadhighByte & 0x00ff) << 8) | slaveReadlowByte; //Combine two bytes into 16 bit number

		if(slaveReadhighByte == 0b00000001)	//store inputs (slaveReadlowByte) from RPi into appropriate variables according to command bits (slaveReadhighByte)
		{
			amplitudeMotor1_input = slaveReadlowByte;
			amplitudeMotor1 = amplitudeMotor1_input/10.0;
			//printf("amplitudeMotor1 = %f \n",amplitudeMotor1);
		}
		else if(slaveReadhighByte == 0b00000010)
		{
			amplitudeMotor2_input = slaveReadlowByte;
			amplitudeMotor2 = amplitudeMotor2_input/10.0;
		}
		else if(slaveReadhighByte == 0b00000011)
		{
			frequencyMotor1_input = slaveReadlowByte;
			frequencyMotor1 = frequencyMotor1_input/10.0;
		}
		else if(slaveReadhighByte == 0b00000100)
		{
			frequencyMotor2_input = slaveReadlowByte;
			frequencyMotor2 = frequencyMotor2_input/10.0;
		}
		else if(slaveReadhighByte == 0b00000101)
		{
			numberofCycles = slaveReadlowByte;	//do we need this for each motor?
		}
		else if(slaveReadhighByte == 0b00000110)
		{
			inputMode = slaveReadlowByte;		//input mode refers to oscillatory or step input, once enabled
		}
		else
		{
			slaveReadhighByte = 0;
			slaveReadlowByte = 0;
		}

		//printf("numLoop = %f \t\t commandByte = %u \t\t valueByte = %u \t ii = %i \n", numberLoop_flt, slaveReadhighByte, slaveReadlowByte, ii);
		
	}

	if((inputMode == OSCILLATION) & (frequencyMotor1 <= 0.3))
	{
		Kp = 2.0;
	}
	else
	{
		Kp = 1.0;
	}

	printf("A1 = %f \t A2 = %f \t f1 = %f \t f2 = %f \t nC = %u \t iM = %u Kp = %f \n", amplitudeMotor1,amplitudeMotor2,frequencyMotor1,frequencyMotor2,numberofCycles,inputMode,Kp);

	//Ts = 1./fs;
	topCount = (uint16_t)(Ts*125000.);		//Computed TOP value for TCC1 interrupt timer (500000 for div64, 125000 for div256)
	TCC1.PER = topCount;					//Set Period


	//Set timing pins as output
	PORTD.DIRSET = PIN7_bm;	//set D7 as output for while loop timing pin
	PORTD.DIRSET = PIN6_bm;	//set D6 as output for interrupt timing pin

	if((inputMode == OSCILLATION) & (frequencyMotor1 != 0))	//Currently only one motor may be active at a time
	{
		stopTime = numberofCycles/frequencyMotor1;	//Stop time in seconds (based on motor 1 inputs)
	}
	else if((inputMode == OSCILLATION) & (frequencyMotor2 != 0))
	{
		stopTime = numberofCycles/frequencyMotor2;	//Stop time based on motor 2 inputs
	}
	else if(inputMode == STEP)
	{
		stopTime = 3;	//3 second stop time for step inputs, do not switch to auto reset before 3 seconds is reached or software reset will not work
	}

	printf("inputMode = %u \t stopTime = %f \n",inputMode,stopTime);

	encoder_init();	//encoder must be initialized AFTER manual mode, otherwise the medium level interrupt misfires and loads a value into revCnt variable causing voltage burst to motor

	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;	//enable interrupts medium and high levels
	sei();	//enable global interrupts
	
	while(1)
	{
		PORTD.OUTTGL = (1<<7);	//Toggle Pin D7 for oscillator clock timing

		if((timeNow >= stopTime) & !(PORTA.IN & PIN7_bm))	//after program is over if manual override switch is turned back to ground, initiate software reset to start of program
		{
			cli();						//disable interrupts to prevent interruption
			CCP = 0xD8;					//configuration change protection: allow protected IO register write
			RST.CTRL = RST_SWRST_bm;	//software reset at end of program
		}
	}

}

ISR(TCC1_OVF_vect)
{
	
	PORTD.OUTTGL = PIN6_bm;	//Toggle Pin D6 for interrupt timing
	if(timeNow <= stopTime)
	{

		if(inputMode == OSCILLATION)
		{
			commandPosition1 = amplitudeMotor1*sin(2*M_PI*frequencyMotor1*timeNow);	//command position of platform (degrees)
			//inputVelocity1 = amplitudeMotor1*2*M_PI*frequencyMotor1*cos(2*M_PI*frequencyMotor1*timeNow);	//calculated velocity value (dps)
			//inputVelocity = amplitude*sin(2*M_PI*frequency*timeNow);	//command velocity for open loop input (dps)
		}
		else if(inputMode == STEP)
		{
			commandPosition1 = amplitudeMotor1;	//Input position in degrees for step input
			//printf("commandPositionStep = %f \t amplitudeMotor1 = %f \n",commandPosition1,amplitudeMotor1);
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
		
		actualPosition1 = (revCnt1 + (encCount1-encCount1_first)/8191.)*360.;	//Position in degrees
		
		deltapos1_ang = (actualPosition1 - position1_m1);			//calculate change in angular position of motor 1 for current loop (degrees/count = (360 deg/rev)/(8192 counts/rev))
		velocity1_ang = (deltapos1_ang)/(Ts);					//calculate angular velocity of motor 1 for last loop (deg/s)
		
		errorNow1 = commandPosition1 - actualPosition1;

		//Controller
		//ctrlOut1 = Kp * errorNow1;	//Proportional Controller

		ctrlOut1 = ((-Kp) + 1./2.*Ki*Ts)*error1_m1 + (Kp + 1./2.*Ki*Ts)*errorNow1 + ctrlOut1_m1;	//PI Controller

		voltSet1 = ctrlOut1;

		if(revCnt1 != revCnt1_m1)
		{
			neutralPositionCounter += 1;
		}

		//Set up data for SPI transactions
		if(commandPosition1 >= 0)
		{
			commandPositionTag1 = 0b00000000;
		}
		else if(commandPosition1 < 0)
		{
			commandPositionTag1 = 0b10000000;
		}

		if(actualPosition1 >= 0)
		{
			actualPositionTag1 = 0b00010000;
		}
		else if(actualPosition1 < 0)
		{
			actualPositionTag1 = 0b10010000;
		}

		commandPosition1_flt = fabs(commandPosition1);
		commandPosition1_int = floor(commandPosition1_flt);
		commandPosition1_dec = floor((commandPosition1_flt - commandPosition1_int)*100);

		actualPosition1_flt = fabs(actualPosition1);
		actualPosition1_int = floor(actualPosition1_flt);
		actualPosition1_dec = floor((actualPosition1_flt - actualPosition1_int)*100);
		
		for(int jj = 0; jj < 2; jj++)
		{
			if(jj < 1)
			{
				slaveWritehighByte = (commandPositionTag1 | commandPosition1_int);
				slaveWritelowByte = commandPosition1_dec;
				//printf("commandPos = %f \t\t highByte = %u \t\t lowByte = %u \t\t commandPosTag = %u \t\t int = %i \t\t dec = %i \t\t jj = %i\n",commandPosition1,slaveWritehighByte,slaveWritelowByte,commandPositionTag1,commandPosition1_int,commandPosition1_dec,jj);
			}
			else
			{
				slaveWritehighByte = (actualPositionTag1 | actualPosition1_int);
				slaveWritelowByte = actualPosition1_dec;
				//printf("actualPos = %f \t\t highByte = %u \t\t lowByte = %u \t\t actualPosTag = %u \t\t int = %i \t\t dec = %i \t\t jj = %i\n",actualPosition1,slaveWritehighByte,slaveWritelowByte,actualPositionTag1,actualPosition1_int,actualPosition1_dec,jj);
			}
			
			SPIC.DATA = slaveWritehighByte;
			//_delay_ms(2);
			PORTA.OUTCLR = PIN4_bm;		//signal RPi that data is ready on first data ready pin

			while(!(SPIC.STATUS & SPI_IF_bm));	//wait until data tx from Rpi is done and flag is set
			//_delay_ms(100);
			PORTA.OUTSET = PIN4_bm;				//set data ready pin high
			slaveReadhighByte = SPIC.DATA;		//Grab byte received from Master
			SPIC.DATA = slaveWritelowByte;						//Send zero low byte to Master

			while(!(SPIC.STATUS & SPI_IF_bm)); //wait until data tx from BB is done and flag is set
			
			slaveReadlowByte = SPIC.DATA; //Grab byte received from Master

		}

		position1_m1 = actualPosition1;			//store last position of motor 1
		revCnt1_m1 = revCnt1;					//store last rev counter
		error1_m1 = errorNow1;					//store last error value for motor 1
		ctrlOut1_m1 = ctrlOut1;					//store last control value for motor 1
	}
	else
	{
		voltSet1 = 0;
	}

	if(abs(voltSet1) >= maxVoltage)
	{
		voltSet1 = copysign(maxVoltage,voltSet1);
	} //if greater than max, set value to max with original sign

	pwmtoMotor1(voltSet1);

	timeNow += Ts;	//Calculate time in program
}

void pwmtoMotor1(float voltSet1)
{
	digiHB1 = voltSet1 / 12.0*1599;	//Convert to digital PWM compare value for HB1
	
	//Motor 1
	if(digiHB1 < 0)
	{
		PORTF.OUTSET = PIN3_bm;	//set INB high for CCW direction
		PORTF.OUTCLR = PIN2_bm;	//set INA
		digiHB1 = -digiHB1;	//make digital HB value positive
		if(digiHB1 >= 1599){digiHB1 = 1599;}	//PWM overflow error check (pulse width cannot exceed top count)
		TCF1.CCA = digiHB1; //Set duty cycle on compare channel A (HB1)
	}
	else
	{
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
	}
	else if (overFlag1 == 0)
	{
		++revCnt1;		//increment RevCnt1 variable if overflow in positive direction
		overFlag1 = 1;	//when positive overflow from TOP to 0, set flag to 1
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

void spi_init()
{
	//SPI Slave configuration on Port C
	PORTC.DIRSET = PIN6_bm;  	// Set Output Port for the SPI Interface (MISO (O), others input by default)
	PORTC.OUTCLR = PIN6_bm;		//Set MISO low
	PORTC.PIN4CTRL = PORT_OPC_PULLUP_gc;	//Pull up chip select pin
	PORTC.PIN5CTRL = PORT_OPC_PULLDOWN_gc;	//Pull down MOSI
	PORTC.PIN7CTRL = PORT_OPC_PULLDOWN_gc;	//Pull down Clock
	_delay_ms(200);
	SPIC.CTRL = 0x40;          // spi slave, spi mode 0
	PORTA.DIRSET = PIN3_bm|PIN4_bm;		//Set output ports for manual data ready pin to Master
	PORTA.OUTSET = PIN3_bm|PIN4_bm;		//Set data ready pins high to signal Master not to send data

	/* Flush slave receive buffer */
	while(SPIC.STATUS & SPI_IF_bm)
	{
		slaveReadhighByte = SPIC.DATA;   // flush spi receive buffer
	}
	slaveReadhighByte = 0;
}