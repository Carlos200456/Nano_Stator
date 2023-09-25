#include <Arduino.h>

bool NoPaso = true;
bool noBreake = false;
int DefSpeed = 180;
int DefDelay = 1500;
int DefAngle = 90;


// put function declarations here:
void Frec30hz(void);
void Frec120hz(void);
void setWaveforms(unsigned long, int);
// void accelerate(int speed, int delayTime);
// void breakMotor(int speed, int delayTime);

// This code demonstrates how to generate two output signals
// with variable phase shift between them using an AVR Timer 
// The output shows up on Arduino pin 9, 10
// More AVR Timer Tricks at http://josh.com

void setup() {
  pinMode( A0 , INPUT_PULLUP); // Arduino Pin A0 = Button 30Hz.
  pinMode(A1, INPUT_PULLUP);   // Arduino Pin A1 = Button 120Hz.
  pinMode(A2, INPUT_PULLUP);   // Arduino Pin A2 = Button Accelerate.
  
  // Configure Timer 0 for Phase-Correct PWM
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  // Configure Timer 2 for Phase-Correct PWM
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
}

void loop() {
  if (!digitalRead(A0)) {
    if (NoPaso) {
      Frec30hz();
      NoPaso = false;
    }
    noBreake = true;
  }
  else {
    if(digitalRead(A1)){
      // Turn off timer0 and timer2
      TCCR0B = 0b00000000; // No clock source (timer stopped)
      TCCR2B = 0b00000000; // No clock source (timer stopped)
      digitalWrite(LED_BUILTIN, LOW);
      NoPaso = true;
    }
  }

  if (!digitalRead(A1)) {
    if (NoPaso) {
      Frec120hz();
      NoPaso = false;
    }
    noBreake = true;
  }
  else {
    if (digitalRead(A0)){
      // Turn off timer0 and timer2
      TCCR0B = 0b00000000; // No clock source (timer stopped)
      TCCR2B = 0b00000000; // No clock source (timer stopped)
      digitalWrite(LED_BUILTIN, LOW);
      NoPaso = true;
    }
  }
}

// put function definitions here:
void setWaveforms( unsigned long freq , int shift ) {
  // Set the initial counter value for Timers

}

void Frec30hz(void) {
  digitalWrite(LED_BUILTIN, HIGH);
  TCNT0 = 0;
  // Prescale to obtain 30hz range
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
  OCR0A = 132;
  OCR0B = 124;
 
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
  OCR2A = 132;
  OCR2B = 124;
}

void Frec120hz(void) {
  digitalWrite(LED_BUILTIN, HIGH);
  TCNT0 = 0;
  // Prescale to obtain 120hz range
  TCCR0A = 0b00100001; // Phase-Correct PWM mode
  // TCCR0B = 0b00000000; // No clock source (timer stopped)
  // TCCR0B = 0b00000010; // Clock divided by 8
  // TCCR0B = 0b00000011; // Clock divided by 64
  TCCR0B = 0b00000100; // Clock divided by 256
  // TCCR0B = 0b00000101; // Clock divided by 1024
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

  
  // Set the initial counter value for Timer 2
  TCNT2 = 128;
  TCCR2A = 0b00100001; // Phase-Correct PWM mode
  // TCCR2B = 0b00000000; // No clock source (timer stopped)
  // TCCR2B = 0b00000001; // Clock with no prescaling (full clock speed)
  // TCCR2B = 0b00000010; // Clock divided by 8
  // TCCR2B = 0b00000011; // Clock divided by 32
  // TCCR2B = 0b00000100; // Clock divided by 64
  // TCCR2B = 0b00000101; // Clock divided by 128
  TCCR2B = 0b00000110; // Clock divided by 256
  // TCCR2B = 0b00000111; // Clock divided by 1024
  TCCR2A = 0b11100000 | (TCCR2A & 0b00001111) ;
  OCR2A = 142;
  OCR2B = 112;
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