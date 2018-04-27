/*
 * Scott Bossard
 * CSCI 5143
 * Motor Project
 * 3/7/2018


 Error = Reference - Measured
 P = Error
 D = Change in error over change in time (dt)
 Pr = Position reference (goal input from the user)
 Pm = Position measured (from the encoder reading in the ISR)
 T = torque (or motor drive). This should be in range of -TOP to TOP
 T = kP(P) + kD( delta(P) / dt )
 T = kP(Pr-Pm) + kD(delta(Pr-Pm) / dt )
 kP = Proportional gain (user defined value)
 kD = Derivative gain (user defined value)
 delta( Pr - Pm ) / dt = ( Pr - Pm(i) ) - ( Pr - Pm(i-1) ) / dt
 = ( Pr - Pr - Pm(i) + Pm(i-1) ) / dt
 = - Velocity
 T = kP(Pr-Pm) - kd(Velocity)

 To Quit Screen Process if lost
 $ screen -ls
 $ screen -X -S [session # you want to kill] quit

 Added: '-Wl,-u,vfprintf -lprintf_flt -lm' to makefile to allow printing of floats
 */

/* Start Initialize*/
/* ****************************************************** */

#define F_CPU 16000000ul
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//Length of input buffers used to store user configuration option
#define RECEIVE_BUFFER_LENGTH 50
//Buffer for first user input to decide what task to run
static volatile char recv_buffer[RECEIVE_BUFFER_LENGTH];
volatile uint8_t recv_buffer_ptr = 0;
//Buffer for second user input to input data like reference position
static volatile char recv_buffer2[RECEIVE_BUFFER_LENGTH];
volatile uint8_t recv_buffer_ptr2 = 0;
volatile unsigned int user_command_ready = 0;
volatile unsigned int *user_token;
volatile unsigned int user_command_ready2 = 0;
volatile unsigned int userCommand = 0;
volatile unsigned int prevUserCommand = 0;
volatile unsigned int *user_token2;

#define clearBit( port, pin ) port &= ~(1 << pin );
#define setBit( port, pin ) port |= (1 << pin );
#define toggleBit( port, pin ) port ^= (1 << pin );
#define bitMask( bit ) (1 << bit );

void setupMotor2(void);
void setupEncoder(void);
// For setting frequency of timer. Freq calculation based on TOP+1
#define TOP_4kHz 3999
#define BOTTOM_4kHz -3999
//Led ports
#define yellowPort PORTC
#define yellowPin PORTC7
#define greenPort PORTD
#define greenPin PORTD5
// Motor 2 PWM signal generated on Port B Pin 6
#define m2_port PORTB
#define m2Pin PORTB6
#define m2Control DDRB
// The direction of motor 2 is controlled on Port E pin 2
#define m2DirControl DDRE
#define m2DirPin DDE2
#define m2DirOutput PORTE
// Set the direction (arbitrarily) to 0.
// Whether this generates positive or negative encoder counts depends on how motor is plugged in.
#define motorForward() clearBit( m2DirOutput, m2DirPin )
#define motorBackward() setBit( m2DirOutput, m2DirPin )
// Turn motor on and off by changing direction of signal to output (set) or input (clear)
#define OnMotor2() setBit( m2Control, m2Pin )
#define OffMotor2() clearBit( m2Control, m2Pin )
// Encoder channels (yellow and white motor wires) plugged in to PB4 and PB5
#define chA_control DDRB
#define chA_pin DDB4
#define chB_control DDRB
#define chB_pin DDB5
// Interrupt #s for encoder are based on where they are plugged in.
#define chA_INT PCINT4
#define chB_INT PCINT5
// Powering the encoder through general I/O. Plug power (blue motor wire) in to Port D Pin 1
#define enc_power_control DDRD
#define enc_power_pin DDD1
#define enc_power_output PORTD

//Set up global variables
volatile int16_t globalCountsM2;
volatile int16_t refCountsM2;
volatile int16_t msTicks = 0;
volatile int8_t globalErrorM2;
volatile int16_t globalLastM2A;
volatile int16_t globalLastM2B;
int secondInput = 0; //used for secondary inputs like ref position
int trajectory = 0; //used for secondary inputs like ref position
int trajectoryPointer = 0; //used for secondary inputs like ref position
volatile int16_t trajectoryArray[100];
uint16_t duty = 100; //% of duty cycle initial=100%
int16_t refPos = 0; //user defined reference position (degrees)
int16_t refPosCounts = 0; //user defined reference position converted to ticks per revolution
int16_t currentRefPos = 0; //user defined reference position converted to ticks per revolution
float propGain = 1.0; //kp
float derGain = 1.0; //kd
float velocity = 0.0; //kd
float torque = 0; //torque used in pdController
float der = 0; //derivative used in pdController
uint16_t pos = 0; //proportional used in pdController
uint16_t prevPos = 0; //previous proportional used to calculate derrivative
int16_t prevMsTick; //previous msTicks used to calculate derrivative
volatile long dummy = 0;
int count = 0;

/* ****************************************************** */
/* End Initialize*/

/* Start Motor code */
/* ****************************************************** */
//Set pins for Motor
void setupMotor2(void) {
	// Make sure motor is not going to start until all set up
	OffMotor2();
	// Configure the motor direction pin to output
	setBit( m2DirControl, m2DirPin );
	// start off going "forward" with the intent of increased encoder counts
	motorForward();
	// Motor 2 is connected to Timer0 Channel B
	//Clear the registers
	TCCR1A = 0;
	TCCR1B = 0;
	ICR1 = 0;
	// Clearing on match for non-inverted signal (i.e. larger match value produces larger duty cycle)
	TCCR1A |= (1 << COM1B1);
	/* want mode 14 (1110), ICR1 is top, OCR1B is match */
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12) | (1 << WGM13);
	// Using 1 as prescaler (001)
	/* 4kHz counter frequency, period of 1/4000 = .25ms period */
	TCCR1B |= (1 << CS10);
	ICR1 = TOP_4kHz;
}
//Set up the Motor encoder
void setupEncoder(void) {
	// Set the encoders as input
	clearBit( chA_control, chA_pin );
	clearBit( chB_control, chB_pin );
	// Enable the interrupts for the 2 encoder channels
	setBit( PCMSK0, chA_INT );
	setBit( PCMSK0, chB_INT );
	// enable PCINT interrupts
	setBit( PCICR, PCIE0 );
	// Powering the encoder through general I/O. This sets signal high to provide power to device.
	setBit( enc_power_control, enc_power_pin );
	setBit( enc_power_output , enc_power_pin );
}
//ISR for Motor Encoder
ISR(PCINT0_vect){
	// Make a copy of the current reading from the encoders
	uint8_t tmpB = PINB;
	// Get value of each channel, making it either a 0 or 1 valued integer
	uint8_t m2A = (tmpB & (1 << chA_pin )) >> chA_pin;
	uint8_t m2B = (tmpB & (1 << chB_pin )) >> chB_pin;
	// Adding or subtracting counts is determined by how these change between interrupts
	int8_t plusM2 = m2A ^ globalLastM2B;
	int8_t minusM2 = m2B ^ globalLastM2A;
	// Add or subtract encoder count as appropriate
	if(plusM2) {
		globalCountsM2 += 1;
		refCountsM2 += 1;
	}
	if(minusM2) {
		 globalCountsM2 -= 1;
		 refCountsM2 -= 1;
	 }
	// If both values changed, something went wrong - probably missed a reading
	if(m2A != globalLastM2A && m2B != globalLastM2B) {
		globalErrorM2 = 1;
	}
	// Save for next interrupt
	globalLastM2A = m2A;
	globalLastM2B = m2B;

  msTicks += 1;
}

/* ****************************************************** */
/* End Motor code */


/* Start Serial Comm code */
/* ****************************************************** */
void handleInput(); //prepare to handle user input

//Function that takes care of configurating the serial communication
void setupUART(void) {
	UBRR1 = ((F_CPU/(16*38400)) - 1);

	UCSR1C |= (1 << UCSZ11) | (1 << UCSZ10);		// 8 bit char size
	UCSR1B |= (1 << TXEN1);		// enable transmit

	UCSR1B |= (1 << RXEN1);     // enable receive
	UCSR1B |= (1 << RXCIE1);     // enable interrupt on data receipt
}

//ISR that is called whenever there is data to receive
ISR(USART1_RX_vect) {
		// BE CAREFUL with this. It is an ISR and will interfere with your
		// PCINT if it takes too long.
    uint8_t ch = UDR1;
		//If secondary input (reference position) then secondInput is > 0
		//Else run user task input
		if(secondInput > 0){
			//If user pressed 'Enter' it means that the user has finished typing the command
			if (ch == 13 || (recv_buffer_ptr2 >= RECEIVE_BUFFER_LENGTH)) {
					user_command_ready2 = 1;
			}

			//User deletes a character
			else if (ch == 8) {
					if (recv_buffer_ptr2 >= 1){
						recv_buffer_ptr2 -= 1;
					}
					recv_buffer2[recv_buffer_ptr2] = 0;
			}
			else if (ch == 'e'){
				trajectory = 1;
			}
			//Only store numeric symbols
			else if
			( ((ch >= '0') && (ch <= '9')) || (ch == '-') || (ch == '.')) {
					recv_buffer2[recv_buffer_ptr2] = ch;
					recv_buffer_ptr2 += 1;
			}
		}
		else{
			//If user pressed 'Enter' it means that the user has finished typing the command
			if (ch == 13 || (recv_buffer_ptr >= RECEIVE_BUFFER_LENGTH)) {
					user_command_ready = 1;
			}
			//User deletes a character
			else if (ch == 8) {
					if (recv_buffer_ptr >= 1){
						recv_buffer_ptr -= 1;
					}
					recv_buffer[recv_buffer_ptr] = 0;
			}
			//Only store alphanumeric symbols, space, the dot, plus and minus sign
			else if
			( (ch == ' ') || (ch == '.') || (ch == '+') || (ch == '-') ||
			((ch >= '0') && (ch <= '9')) ||
			((ch >= 'A') && (ch <= 'Z')) ||
			((ch >= 'a') && (ch <= 'z')) ) {
					recv_buffer[recv_buffer_ptr] = ch;
					recv_buffer_ptr += 1;
			}
		}
}

void sendChar(char c) {
	while((UCSR1A & (1<<UDRE1)) == 0);	// wait while data register is NOT empty
	UDR1 = c;
}

void sendString(char *s) {
	while(*s != 0x00) {
		sendChar(*s);
		s++;
	}
	sendChar('\r');
}

/* ****************************************************** */
/* End Serial Comm code */

/* Start User Tasks code */
/* ****************************************************** */

void zeroEncoder(){
	globalCountsM2 = 0;
}

void setRefPos(){
	secondInput = 1; //ready to accept 2nd input
	while(user_command_ready2 == 0){
			dummy++; //wait for user input
	}
	recv_buffer2[recv_buffer_ptr2 + 1] = '/0';//add 0 to help convert to int
	sscanf(recv_buffer2,"%d", &refPos); //convert to int
	if(refPos >= -360 && refPos <= 360){
		//only allow this range so torque can stay between -TOP and TOP
		refPosCounts = refPos*6.24; //6.24 = 1 degree
		sendString(recv_buffer2);
	}
	else{
		sendString("Please enter between -360 & +360 degrees): \r\n");
	}
	secondInput = 0; //leave second input. allow first input layer
	// reset the buffer
	recv_buffer_ptr2 = 0;
	memset(recv_buffer2, 0, RECEIVE_BUFFER_LENGTH);

	// set flag that ready for more
	user_command_ready2 = 0;
}

void setUpTrajectory(){
	secondInput = 1; //ready to accept 2nd input
	trajectory = 0;
  sendString("Press e + Enter to finish. \r\n");
	while(user_command_ready2 == 0){
			dummy++; //wait for user input
	}
	if(trajectory == 0){
		recv_buffer2[recv_buffer_ptr2 + 1] = '/0'; //add 0 to help convert to int
		sscanf(recv_buffer2,"%d", &refPos);//convert to int
		if(refPos >= -360 && refPos <= 360){
			//only allow this range so torque can stay between -TOP and TOP
			trajectoryArray[trajectoryPointer] = refPos*6.24; //6.24 = 1 degree
			trajectoryPointer += 1; //increase pointer
		}
		else{
			sendString("Please enter between -360 & +360 degrees): \r\n");
		}
	}
	secondInput = 0; //leave second input. allow first input layer

	// reset the buffer
	recv_buffer_ptr2 = 0;
	memset(recv_buffer2, 0, RECEIVE_BUFFER_LENGTH);

	// set flag that ready for more
	user_command_ready2 = 0;
}

void runMotor(int16_t reference){
	OnMotor2();
	int counter = 0;
	//If a trajectory then use the parameter
	//This method is repeated for each value in the trajectory
	//else the user is just setting the reference position for a 1 time run
	if(trajectory == 1){
		currentRefPos = reference;
	}
	else{
		currentRefPos = refPosCounts;
	}
	//PORTC |= ( 1 << PORTC7 );
	setBit(yellowPort , yellowPin);// Yellow LED toggle on
	refCountsM2 = 0; //reset to go 360 degrees forward from current position
	pdController();
	if(currentRefPos > 0){
		//initial user reference position is forwards
		//Going forwards
		while (torque > 0){
			counter++;
			prevPos = pos; //previous position used for calculattion in controller
			prevMsTick = msTicks; //previous time used for calculattion in controller
			motorForward();
			pdController(); //Call controller
			//Counter is set to exit to not allow motor to continulsy run
			//Tested and found that position is reached by counter = 1000
			if(counter > 1000){
				break;
			}
		}
		//Going backwards
		while (torque < 0){
			counter++;
			prevPos = pos; //previous position used for calculattion in controller
			prevMsTick = msTicks; //previous time used for calculattion in controller
			motorBackward();
			pdController(); //Call controller
			//Counter is set to exit to not allow motor to continulsy run
			//Tested and found that position is reached by counter = 1000
			if(counter > 1000){
				break;
			}
		}
	}
	else{
		//initial user reference position is backwards
		//Going backwards
		while (torque > 0){
			counter++;
			prevPos = pos; //previous position used for calculattion in controller
			prevMsTick = msTicks; //previous time used for calculattion in controller
			motorBackward();
			pdController(); //Call controller
			//Counter is set to exit to not allow motor to continulsy run
			//Tested and found that position is reached by counter = 1000
			if(counter > 1000){
				break;
			}
		}
		//Going forwards
		while (torque < 0){
			counter++;
			prevPos = pos; //previous position used for calculattion in controller
			prevMsTick = msTicks; //previous time used for calculattion in controller
			motorForward();
			pdController(); //Call controller
			//Counter is set to exit to not allow motor to continulsy run
			//Tested and found that position is reached by counter = 1000
			if(counter > 1000){
				break;
			}
		}
	}
	OffMotor2();

	//reset90
	pos = 0;
	torque = 0;
	counter = 0;
	clearBit(yellowPort , yellowPin);
}

/* ****************************************************** */
/* End User Tasks code */

/* Start Controller Code */
/* ****************************************************** */
void pdController(){
	count += 1;
	pos = currentRefPos - refCountsM2; //curent position equals user defined position - calculated position from motor ISR (P int PD controller)
	velocity = refCountsM2 / msTicks;
	int16_t changeInTime = (msTicks-prevMsTick);
	if ( changeInTime > 0){
		der = (pos - prevPos)/(msTicks-prevMsTick); //derivative calaulate (D int PD controller)
	}
	else{
		//avoid divide by 0
		der = 0;
	}
	torque = (propGain*pos) + (derGain*der); // T = kP(P) + kD( delta(P) / dt )

  OCR1B = (uint16_t) ((abs(torque) + 1.0) * (duty / 100.0 )) - 1;

	//debug
	//if (count < 10){
		//char posBuff[50];
		//sprintf(posBuff, "%f\n",currentRefPos);
		//sendString("ref: \n");
		//sendString(posBuff);
		//memset(posBuff, 0, 50);
		//char derBuff[50];
		//sprintf(derBuff, "%f\n",refCountsM2);
		//sendString("meas: \n");
		//sendString(derBuff);
		//memset(derBuff, 0, 50);
		//char torBuff[50];
		//sprintf(torBuff, "%f\n",torque);
		//sendString("tor: \n");
		//sendString(torBuff);
		//memset(torBuff, 0, 50);
	//}
	sendString("...running...");
}
/* ****************************************************** */
/* End Controller Code */

/* Start Potentiometer Code */
/* ****************************************************** */
//set duty cycle, duty is a percent of full speed
void setDuty(uint16_t d){
	//d is adc found from potentiometer
	//change speed in intervals of 15
	//adc is in range of 300-350
	if (d <= 300 ) {
		duty = 0;
	}
	if (d >= 300 && d < 310) {
		duty = 20;
	}
	if (d >= 310 && d < 320) {
		duty = 40;
	}
	if (d >= 320 && d < 330) {
		duty = 60;
	}
	if (d >= 330 && d < 340) {
		duty = 80;
	}
	if (d >= 340) {
		duty = 100;
	}
}

// initialize adc
void adc_init() {
    // AREF = AVcc
    ADMUX = (1<<REFS0);

    // ADC Enable and prescaler of 128
    // 16000000/128 = 125000
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

    // initialize power supply
    DDRD |= ( 1 << DDD6 );
    PORTD |= ( 1 << PORTD6 );
}

// read adc value
uint16_t adc_read(uint8_t ch) {
    // select the corresponding channel 0~15
    // ANDing with '8' will always keep the value
    // of 'ch' between 0 and 15
    ch &= 0b00001111;  // AND operation with 7
    ADMUX = (ADMUX & 0b11100000)|ch;     // clears the bottom 5 bits before ORing

    // start single conversion
    // write '1' to ADSC
    ADCSRA |= (1<<ADSC);

    // wait for conversion to complete
    // ADSC becomes '0' again
    // till then, run loop continuously
    while(ADCSRA & (1<<ADSC));

    return (ADC);
}

/* ****************************************************** */
/* End Potentiometer Code */

//Function that parses the commands the user inputs into the serial console.
void handleInput() {
		// This function is called from main, but "released" in the ISR.
		// The ISR sets user_command_ready=1, main watches for the flag.
		char command;
		int value;
		// only accepting a command character with optional value
		sscanf(recv_buffer,"%c %d",&command, &value);
		//iterate through each command given by the user
			switch(command) {
				case 'm':
					sendString("Running motor... \r\n");
					if(trajectory == 1){
						int i = 0;
						for(i; i < trajectoryPointer; i++){
							runMotor(trajectoryArray[i]);
						}
						trajectoryPointer == 0;
						trajectory = 0;
					}
					else{
						runMotor(refPosCounts); //single run
					}
					sendString("Motor stopped \r\n");
					printMenu();
					break;
				case 'r':
					sendString("Enter reference positon (-360 - +360 degrees): \r\n");
					setRefPos();
					printMenu();
					break;
				case 'z':
					sendString("Zeroing the encoder... \r\n");
					zeroEncoder();
					sendString("Done. \r\n");
					break;
				case 'v':
					sendString("Current values are: \r\n");
					printInfo();
					break;
				case 't':
					sendString("Setting up trajectory... \r\n");
					trajectoryPointer = 0;
					while(trajectory == 0){
						setUpTrajectory();
					}
					printMenu();
					break;
				case 'P':
					sendString("Increase Kp by .05: \r\n");
					propGain += .05;
					if(propGain > 2){
						propGain = 2;
						sendString("Can't increase anymore. \r\n");
					}
					else{
						sendString("Kp increased. \r\n");
					}
					break;
				case 'p':
					sendString("Decrease Kp by .05: \r\n");
					propGain -= .05;
					if(propGain < .05){
						propGain = .05;
						sendString("Can't decrease anymore. \r\n");
					}
					else{
						sendString("Kp decreased. \r\n");
					}
					break;
				case 'D':
					sendString("Increase Kd by .5: \r\n");
					derGain += .05;
					if(derGain > 2){
						derGain = 2;
						sendString("Can't increase anymore. \r\n");
					}
					else{
						sendString("Kd increased. \r\n");
					}
					break;
				case 'd':
					sendString("Decrease Kd by .5: \r\n");
					derGain -= .05;
					if(derGain < .05){
						derGain = .05;
						sendString("Can't decrease anymore. \r\n");
					}
					else{
						sendString("Kd decreased. \r\n");
					}
					break;
				default:
					printMenu();
				}
		// reset the buffer
    recv_buffer_ptr = 0;
    memset(recv_buffer, 0, RECEIVE_BUFFER_LENGTH);
		// set flag that ready for more
		user_command_ready = 0;
}

//Print current data
void printInfo(){
	char prBuff[100];
	char pmBuff[100];
	char kdBuff[100];
	char kpBuff[100];
	char dBuff[100];
	char tBuff[100];
	char vBuff[100];

	//print reference position
	sprintf(prBuff, "%d\n", refPos);
	sendString("Position reference (Pr in degrees): \n");
	sendString(prBuff);
	sendString("\n");

	//Print measured position
	int16_t measuredPos = globalCountsM2 / 6.24; //convert to  degrees
	sprintf(pmBuff, "%d\n", measuredPos);
	sendString("Position measured (Pm in degrees): \n");
	sendString(pmBuff);
	sendString("\n");

	//Print proportional gain
	sprintf(kpBuff, "%f\n", propGain);
	sendString("Proportional gain: \n");
	sendString(kpBuff);
	sendString("\n");

	//Print derivative gain
	sprintf(kdBuff, "%f\n", derGain);
	sendString("Derivative gain: \n");
	sendString(kdBuff);
	sendString("\n");

	//Print duty cycle
	sprintf(dBuff, "%d\n", duty);
	sendString("Duty cycle: \n");
	sendString(dBuff);
	sendString("\n");

	//Print velocity
	sprintf(vBuff, "%d\n", velocity);
	sendString("Velocity: \n");
	sendString(vBuff);
	sendString("\n");

	//Print torque
	sprintf(tBuff, "%f\n", torque);
	sendString("Torque: \n");
	sendString(tBuff);
	sendString("\n");

	//reset buffer
	memset(prBuff, 0, 100);
	memset(pmBuff, 0, 100);
	memset(kpBuff, 0, 100);
	memset(kdBuff, 0, 100);
	memset(dBuff, 0, 100);
	memset(tBuff, 0, 100);
	memset(vBuff, 0, 100);
}

//Print the options the user has
void printMenu(){
	//We are ready for take off
	sendString("Motor ready!\r\n ");
	sendString("m - Start motor.\r\n ");
	sendString("r - Set the reference position in degrees.\r\n ");
	sendString("z - Zero the encoder marking the current position as 0 degrees.\r\n ");
	sendString("v - View the current values Kd, Kp, Vm, Pr, Pm, and T\r\n ");
	sendString("t - Execute trajectory\r\n ");
	sendString("P - Increase Kp\r\n ");
	sendString("p - Decrease Kp\r\n ");
	sendString("D - Increase Kd\r\n ");
	sendString("d - Decrease Kd\r\n ");
}

int main(void) {
	USBCON = 0;
	//Set up motor
	setupMotor2();
	setupEncoder();
	// Perform sanity check.
	int i;
	for (i=0; i<3; i++) {
		setBit(yellowPort , yellowPin);
		_delay_ms(200);
		clearBit(yellowPort , yellowPin);// Yellow LED off
		setBit(greenPort , greenPin);// Green LED on
		_delay_ms(200);
		clearBit(greenPort , greenPin);// Green LED off
		_delay_ms(200);
	}
  setupUART(); //setup serial communication
  printMenu(); //print options
	sei(); //enable interrupts
	// initialize ADC
	uint16_t adc_results;
	adc_init();
  for(;;) {
		//Always watch for user input
		if (user_command_ready) {
			adc_results = adc_read(7);      // read adc value at PA7
			setDuty(adc_results);
			handleInput();
		}
	}
	return 0;
}
