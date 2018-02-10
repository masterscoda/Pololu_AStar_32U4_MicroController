/*
Scott Bossard

2/8/2018

Task 1: Use PWM to control the brightness. Allow user to press button A to increase brightness in external LED
Task 2: Use PMW to control blink rate. Allow user to press button C to control blink rate of external LED.
Task 3: Blink built in LED, at a rate of .5 Hz (ON at 1 sec, OFF at 1 sec, …) in main loop using interupts
Task 4: Blink built in LED, at a rate of 2 Hz (ON at 250 ms, OFF at 500 ms, ...) in ISR.
*/


#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

uint8_t brightness;
long counter_1s = 2000;
long counter_250ms = 500;
volatile long ms_ticks = 0;
int buttonPress = 0;

int main() {

  // This prevents the need to reset after flashing
  USBCON = 0;

  // Configures yellow LED pin as output
  // DDRC: Data Direction Register for Port C
  // DDC7: Data Direction pin/bit mask of Port C, pin 7
  DDRC |= (1 << DDC7);
  // Configure Green LED
  DDRD |= ( 1 << DDD5 );
  // Configure Red LED
  DDRB |= (1 << DDB0 );
  // Configure Pin B7 as output for external LED on breadboard
  DDRB |= (1 << DDB7);
  // Configure Pin C6 as output for external LED on breadboard
  DDRC |= (1 << DDC6);
  // Configure Pin B5 as output for external LED on breadboard
  DDRB |= (1 << DDB5);

  // Configure button A: Port B, pin 3 as input
  DDRB &= ~(1 << DDB3);
  // Configure button C: Port B, pin 0 as input
  DDRB &= ~(1 << DDB0 );

  // Enable Button pull-up resistor
  PORTB |= (1 << PORTB3)| (1<<PORTB0);

  // PCICR: Pin Change Interrupt Control Registe
  // PCIE0: Pin Change Interrupt Enable Bit:
  // Any change on any enabled PCINT7..0 can fire ISR.
  PCICR |= ( 1 << PCIE0 );

  // PCMSK0: Pin Change Mask for Interrupt0, which is for all pins 0 through 7
  // Enable interrupts on Button A (PCINT3) and Button C (PCINT0)
  PCMSK0 |= (1 << PCINT3)| ( 1 << PCINT0);

  // PWM mode
  TCCR0A |= (1<<WGM00)|(1<<COM0A1)|(1<<WGM01);
  TCCR0B |= (1 << CS01)|(1<<CS00); //Prescaler = 64 thus use bit (1 << CS01)|(1 << CS00)

  //Initialize counter_1s
  TCNT0 = 0;

  //Timer Interupt Mask Register
  TIMSK0 |= (1 << OCIE0B);

  //Used for innterupt on match (Used for task 3-4)
  OCR0B = 250;

  // PWM mode for Timer 3 (used for task 2)
  //TCCR3A |= (1<<WGM30)|(1<<COM3A1)|(1<<WGM31);
  //TCCR3B |= (1 << CS31)|(1<<CS30); //Prescaler = 64 thus use bit (1 << CS01)|(1 << CS00)
  
  //Dusty cycle is 50% for task 3
  //OCR3A = 128;


  // PWM mode
  TCCR1A |= (1<<COM1A1)|(1<<COM1A1)|(1<<WGM11);
  TCCR1B |= (1<<WGM12)|(1<<WGM13)|(1<<CS10); //Prescaler = 64 thus use bit (1 << CS01)|(1 << CS00)
  OCR1A = 0x7FFF;
  ICR1 = 0xFFFF;
    
  // Perform sanity check.
  // Are the LEDs configured properly?
  // Is the board reset and running main?

  // Yellow and Green ON then off quick for sanity check
  // Green is turned on by setting port low
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

  sei();
  while(1) {
    //If brighness reaches MAX then reset
    if (brightness == 255){
      brightness = 0;
    }
    //Increase OCR0A to increase brighness of LED
    OCR0A = brightness;

    //Change frequncy on button c press (task 2)
    if(buttonPress == 0){
      //First frequency requested by user (default)
      //Blink External LED at .5Hz (On 1 sec, off 1 sec)      
      if (ms_ticks >= 1000 && ms_ticks <= 2000){        
        ICR1 = 0xFFFF;
      }
      if (ms_ticks > 0 && ms_ticks <= 1000){
        //Turn LED lower for 1 second
        ICR1 = 0x0FFF;
      }
      if (ms_ticks >= 2000){
        //Reset counter_1s to repeat the 1 second off/ 1 sec on period has ended
        ms_ticks = 0;
      }
    }
    if(buttonPress == 1){
      //Next frequency requested by user
      //Blink External LED at 1 Hz      
      if (ms_ticks >= 500 && ms_ticks <= 1000){        
        ICR1 = 0xFFFF;
      }
      if (ms_ticks > 0 && ms_ticks <= 500){
        //Turn LED lower for 1 second
        ICR1 = 0x0FFF;
      }
      if (ms_ticks > 1000){
        //Reset counter_1s to repeat the 1 second off/ 1 sec on period has ended
        ms_ticks = 0;
      }
    }
    if(buttonPress == 2){
      //Next frequency requested by user
      //Blink External LED at 2 Hz      
      if (ms_ticks >= 250 && ms_ticks <= 500){        
        ICR1 = 0xFFFF;
      }
      if (ms_ticks > 0 && ms_ticks <= 250){
        //Turn LED lower for 1 second
        ICR1 = 0x0FFF;
      }
      if (ms_ticks > 500){
        //Reset counter_1s to repeat the 1 second off/ 1 sec on period has ended
        ms_ticks = 0;
      }
    }
    if(buttonPress == 3){
      //Next frequency requested by user
      //Blink External LED at       
      if (ms_ticks >= 125 && ms_ticks <= 250){        
        ICR1 = 0xFFFF;
      }
      if (ms_ticks > 0 && ms_ticks <= 125){
        //Turn LED lower for 1 second
        ICR1 = 0x0FFF;
      }
      if (ms_ticks > 250){
        //Reset counter_1s to repeat the 1 second off/ 1 sec on period has ended
        ms_ticks = 0;
      }
    }
    if(buttonPress == 4){
      //Reset
      buttonPress = 0;
    }
    
    //Blink Yellow LED at .5Hz (On 1 sec, off 1 sec)
    //counter_1s = 2000 and is being decremented
    if (counter_1s >= 1000 && counter_1s <= 2000){        
      //Turn on LED for 1 second
       PORTC |= ( 1 << PORTC7 );// Yellow LED on
    }
    if (counter_1s >= 0 && counter_1s <= 1000){
      //Turn off LED for 1 second
       PORTC &= ~( 1 << PORTC7 );// Yellow LED off
    }
    if (counter_1s <= 0){
      //Reset counter_1s to repeat the 1 second off/ 1 sec on period has ended
      counter_1s = 2000;
    }
  }
}

ISR(PCINT0_vect) {
  //Button C Press to change frequency of external
  if (0 == (PINB & (1 << 0))) {
    // Check again in case of noise on system
    _delay_ms(10);
    if (0 == (PINB & (1 << 0))) {
      buttonPress = buttonPress + 1; //Increment Brightness by 1 each button push
    }
  }
  
  // Button A press increases brightness each press on LED connected to breadboard (Pin B7)
  if (0 == (PINB & (1 << 3))) {
    // Check again in case of noise on system
    _delay_ms(10);
    if (0 == (PINB & (1 << 3))) {
      brightness = brightness + 1; //Increment Brightness by 1 each button push
    }
  }
}

//ISR that fires when TCNT0 and OCR0B match
ISR(TIMER0_COMPB_vect) { 
  ms_ticks++;
  counter_1s--; //decrement counter for 1 second switch in main loop
  counter_250ms--;
  //Blink Green LED at 2Hz (ON at 250 ms, OFF at 500 ms, ...).
  //counter_250ms = 500 and is being decremented
  if (counter_250ms >= 250 && counter_250ms <= 500){        
    //Turn on LED for 250 ms
     PORTD &= ~( 1 << PORTD5 );// Green LED on
  }
  if (counter_250ms >= 0 && counter_250ms <= 250){
    //Turn off LED for 250 ms
     PORTD |= ( 1 << PORTD5 );// Green LED on
  }
  if (counter_250ms <= 0){
    //Reset counter_250ms to repeat the 500ms period
    counter_250ms = 500;
   }
}
