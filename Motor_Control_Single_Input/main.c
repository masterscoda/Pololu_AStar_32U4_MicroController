/*
 * Scott Bossard
 * 2/21/2108
 * Motor control using single input
 */
#define F_CPU 16000000ul	// required for _delay_ms()
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//Length of output bufer used for serial communication
#define OUTPUT_BUFFER_LENGTH 300
//Length of input buffer used to store user configuration option
#define RECEIVE_BUFFER_LENGTH 50
//Global constant string specifying menu options:
const char *menu = "MENU\n\r";

//Serial communication output buffer
static volatile char output_buffer[OUTPUT_BUFFER_LENGTH];
static volatile char recv_buffer[RECEIVE_BUFFER_LENGTH];
volatile uint8_t recv_buffer_ptr = 0;
volatile unsigned int user_command_ready = 0;
volatile unsigned int *user_token;

#define clearBit( port, pin ) port &= ~(1 << pin );
#define setBit( port, pin ) port |= (1 << pin );
#define toggleBit( port, pin ) port ^= (1 << pin );
#define bitMask( bit ) (1 << bit );
#include "motor.c"

int speed = 10;
void handleInput();

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

    //Is user pressed 'Enter' it means that the user has finished typing the command
    if (ch == 13 || (recv_buffer_ptr >= RECEIVE_BUFFER_LENGTH)) {
        user_command_ready = 1;
    }

    //User deletes a character
    else if (ch == 8) {
        if (recv_buffer_ptr >= 1)
            --recv_buffer_ptr;
        recv_buffer[recv_buffer_ptr] = 0;
    }

    //Only store alphanumeric symbols, space, the dot, plus and minus sign
    else if
		( (ch == ' ') || (ch == '.') || (ch == '+') || (ch == '-') ||
		((ch >= '0') && (ch <= '9')) ||
		((ch >= 'A') && (ch <= 'Z')) ||
		((ch >= 'a') && (ch <= 'z')) ) {
        recv_buffer[recv_buffer_ptr] = ch;
        ++recv_buffer_ptr;
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

void setupMStimer(void) {
	TCCR3A = 0; // clear for good measure
	TCCR3B = 0x08;	// set CTC mode (clear-timer-on-compare)

	// apparently this writes to the HIGH and LOW capture regs
	OCR3A = 15999;  // 1ms period

	TIMSK3 = 0x02;  // OCIE1A compare match A interrupt Enable
	TCCR3B |= 0x01; // start timer with no prescaler
}

int main(void) {
	USBCON = 0;

	//Set up motor
	setupMotor2();
	setupEncoder();

	// Configure LED pins as output
  // DDRC: Data Direction Register for Port C
  // DDC7: Data Direction pin/bit mask of Port C, pin 7
  DDRC |= (1 << DDC7);
  // Configure Green LED
  DDRD |= ( 1 << DDD5 );
  // Configure Red LED
  DDRB |= (1 << DDB0 );

	// Perform sanity check.
	int i;
	for (i=0; i<3; i++) {
		PORTC |= ( 1 << PORTC7 );// Yellow LED on
		_delay_ms(200);
		PORTC &= ~( 1 << PORTC7 );// Yellow LED off
		PORTD &= ~( 1 << PORTD5 );// Green LED on
		_delay_ms(200);
		PORTD |= ( 1 << PORTD5 );// Green LED off
		_delay_ms(200);
	}

	setDutyCycle(speed);	// start motor with 10% duty cycle
	OnMotor2(); //turn motor 2 on
  setupUART(); //setup serial communication

	//We are ready for take off
	sendString("Here we go!\r\n ");
	sendString("w - forward!\r\n ");
	sendString("s - backward!\r\n ");
	sendString("d - increase speed!\r\n ");
	sendString("a - decrease speed!\r\n ");
	sei(); //enable interrupts

  for(;;) {
		//Always watch for user input
		if (user_command_ready) {
			handleInput();
		}
	}
	return 0;
}

#define printMenu() sendString("w, s, d, a \r\n");

//Function that parses the commands the user inputs into the serial console.
void handleInput() {
		// This function is called from main, but "released" in the ISR.
		// The ISR sets user_command_ready=1, main watches for the flag.
		char command;
		int value;
		char fGood = 1;

		// provides feedback to the terminal
		char outputBuffer[50] = "\r\n";

		// only accepting a command character with optional value
		sscanf(recv_buffer,"%c %d",&command, &value);

		// Make this into whatever makes sense for you
		switch(command) {
			case 'w':
				PORTC ^= ( 1 << PORTC7 );// Yellow LED toggle
				motorForward();
				sprintf( outputBuffer,"Motor forward.\r\n");
				break;
			case 's':
			  PORTD ^= ( 1 << PORTD5 );// Green LED toggle
				motorBackward();
				sprintf( outputBuffer,"Motor backward.\r\n");
				break;
			case 'd':
				PORTB ^= (1 << PORTB0); //Red LED toggle
				speed = speed + 10; //increment dudty cycle by 10%
				if(speed > 100){
					speed = 100; //Can't set duty cycle more than 100%
				}
				setDutyCycle(speed);
				sprintf( outputBuffer,"Increasing speed.\r\n");
				break;
			case 'a':
			  PORTB ^= (1 << PORTB0); //Red LED toggle
				speed = speed - 10; //decrement dudty cycle by 10%
				if(speed < 0){
					speed = 0; //Can't set duty cycle less than 0%
				}
				setDutyCycle(speed);
				sprintf( outputBuffer,"Decreasing speed.\r\n");
				break;
			default:
				fGood = 0;
				printMenu();
		}

		// reset the buffer
    recv_buffer_ptr = 0;
    memset(recv_buffer, 0, RECEIVE_BUFFER_LENGTH);

		// feedback to terminal
		if (fGood) sendString(outputBuffer);

		// set flag that ready for more
		user_command_ready = 0;
}
