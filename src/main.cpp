#include <Arduino.h>
// #include <TimerOne.h>
// #include <TimerThree.h>

bool NoPaso = true;
bool noBreake = false;
int DefSpeed = 180;
int DefDelay = 1500;
int DefAngle = 90;
const int pwmFrequency = 10; // 100Hz


// put function declarations here:
void setWaveforms(unsigned long, int);
// void accelerate(int speed, int delayTime);
// void breakMotor(int speed, int delayTime);

// This code demonstrates how to generate two output signals
// with variable phase shift between them using an AVR Timer 
// The output shows up on Arduino pin 9, 10
// More AVR Timer Tricks at http://josh.com

void setup() {
  pinMode( A3 , INPUT_PULLUP); // Arduino Pin A3 = Button Break
  pinMode(A5, INPUT_PULLUP);   // Arduino Pin A5 = Button Acelerate
  pinMode(A4, INPUT_PULLUP);   // Arduino Pin A4 = Button Hi Speed
  digitalWrite(A7, LOW);       // Set OC0A to low to enable Button

  // Configure Timer 0 for Phase-Correct PWM
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  TCCR0A = 0b00100001; // Phase-Correct PWM mode
  // TCCR0B = 0b00000000; // No clock source (timer stopped)
  // TCCR0B = 0b00000010; // Clock divided by 8
  // TCCR0B = 0b00000011; // Clock divided by 64
  // TCCR0B = 0b00000100; // Clock divided by 256
  TCCR0B = 0b00000101; // Clock divided by 1024
  TCCR0A = 0b11100000 | (TCCR0A & 0b00001111) ; 
  // Bit 7 (COM2A1) 0: OC2A disconnected. 1: Toggle OC2A on Compare Match.
  // Bit 6 (COM2A0) 0: OC2A disconnected. 1: Clear OC2A on Compare Match (non-inverted mode).
  // Bit 5 (COM2B1) 0: OC2B disconnected. 1: Toggle OC2B on Compare Match.
  // Bit 4 (COM2B0) 0: OC2B disconnected. 1: Clear OC2B on Compare Match (non-inverted mode).
  // Bit 3 (RESERVED): This bit is reserved and should be written as 0.
  // Bit 2 (WGM21): Waveform Generation Mode 0: Normal mode. 1: CTC (Clear Timer on Compare Match) mode. Timer 2 counts up to the value specified in OCR2A, then resets.
  // Bit 1 (WGM20): Waveform Generation Mode 0: Normal mode. 1: CTC (Clear Timer on Compare Match) mode. Timer 2 counts up to the value specified in OCR2A, then resets.
  // Bit 0 (FOC2A): Force Output Compare A 0: Normal operation. This bit is not used in PWM modes. 1: Force OC2A to logic high.
  OCR0A = 142;
  OCR0B = 112;

  
  // Configure Timer 2 for Phase-Correct PWM
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  // Set the initial counter value for Timer 2
  TCNT2 = 128;
  TCCR2A = 0b00100001; // Phase-Correct PWM mode
  // TCCR2B = 0b00000000; // No clock source (timer stopped)
  // TCCR2B = 0b00000001; // Clock with no prescaling (full clock speed)
  // TCCR2B = 0b00000010; // Clock divided by 8
  // TCCR2B = 0b00000011; // Clock divided by 32
  // TCCR2B = 0b00000100; // Clock divided by 64
  // TCCR2B = 0b00000101; // Clock divided by 128
  // TCCR2B = 0b00000110; // Clock divided by 256
  TCCR2B = 0b00000111; // Clock divided by 1024
  TCCR2A = 0b11100000 | (TCCR2A & 0b00001111) ;
  OCR2A = 142;
  OCR2B = 112;
}

// prescaler of 1 will get us 8MHz - 488Hz
// User a higher prescaler for lower freqncies

#define PRESCALER 4
#define PRESCALER_BITS 0x01

#define CLK 16000000UL    // Default clock speed is 16MHz on Arduino Uno

// Output phase shifted wave forms on Arduino Pins 9 & 10
// freq = freqnecy in Hertz (  122 < freq <8000000 )
// shift = phase shift in degrees ( 0 <= shift < 180 )
// Do do shifts 180-360 degrees, you could invert the OCR1B by doing an extra toggle using FOC
/// Note phase shifts will be rounded down to the next neared possible value so the higher the frequency, the less phase shift resolution you get. At 8Mhz, 
/// you can only have 0 or 180 degrees because there are only 2 clock ticks per cycle.  

// Demo by cycling through some phase shifts at 50Khz  

void loop() {
  if (digitalRead(A4) == HIGH) {
    DefSpeed = 50;
  } else {
    DefSpeed = 180;
  }
  
  if (digitalRead(A5) == LOW) {
    digitalWrite(LED_BUILTIN, HIGH);
    if (NoPaso) {
//      accelerate(DefSpeed, DefDelay);
      NoPaso = false;
    }
//    setWaveforms( DefSpeed , DefAngle );
    noBreake = true;
  }
  else {
    // Turn off timer
    TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10));
    digitalWrite(LED_BUILTIN, LOW);
    NoPaso = true;
  }

  if (digitalRead(A3) == LOW) {
    digitalWrite(LED_BUILTIN, HIGH);
    if (noBreake) {
//      breakMotor(DefSpeed, DefDelay);
      noBreake = false;
    }
    // Turn off timer
    TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10));
    digitalWrite(LED_BUILTIN, LOW);
  }   
}

// put function definitions here:
void setWaveforms( unsigned long freq , int shift ) {

  // This assumes prescaler = 1. For lower freqnecies, use a larger prescaler.
  
  // Calculate the number of clock cycles per toggle
  unsigned long clocks_per_toggle = (CLK / (freq * PRESCALER)) / 4;    // /2 becuase it takes 2 toggles to make a full wave

  ICR1 = clocks_per_toggle;

  unsigned long offset_clocks = (clocks_per_toggle * shift) / 180UL; // Do mult first to save precision

  OCR1B= offset_clocks;

  // Turn on timer now if is was not already on
  // Clock source = clkio/1 (no prescaling)
  // Note: you could use a prescaller here for lower freqnencies
  TCCR1B |= _BV( CS11 ); 

  // Prescale to obtain 50hz range
  // TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10));


  // Prescale to obtain 180hz range
  // TCCR1B |= _BV( CS11 );
}

// void accelerate(int speed, int delayTime) {
//   int intdelay = delayTime / (speed - 30);
//   for (int i = 30; i <= speed; i++) {
//     setWaveforms( i , DefAngle );
//     delay(intdelay);
//   }
// }

// void breakMotor(int speed, int delayTime) {
//   int intdelay = delayTime / (speed - 30);
//   for (int i = speed; i >= 30; i--) {
//     setWaveforms( i , DefAngle );
//     delay(intdelay);
//   }
// }