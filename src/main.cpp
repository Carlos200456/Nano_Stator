#include <Arduino.h>

bool NoPaso = true;
bool noBreake = false;
boolean toggle1 = 0;
boolean toggle2 = 0;
boolean Enable = false;
byte savePrescale;
int DefSpeed = 0;
int RealSpeed = 0;
int DefDelay = 1500;
int DefAngle = 90;


// put function declarations here:
void setWaveforms(unsigned long, int);
void accelerate(int speed, int delayTime);
void breakMotor(int speed, int delayTime);

// prescaler of 1 will get us 8MHz - 488Hz
// User a higher prescaler for lower freqncies
#define PRESCALER 64
#define PRESCALER_BITS 0x01
#define CLK 16000000UL    // Default clock speed is 16MHz on Arduino Uno


void setup() {
  pinMode( 0 , INPUT_PULLUP);  // Arduino Pin 0 = Button Accelerate.
  pinMode(1, INPUT_PULLUP);    // Arduino Pin 1 = Button Break
  pinMode(A3, INPUT_PULLUP);   // Arduino Pin A3 = Button High Speed
  
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);

  // Configure Timer 1 for Frequency Generation
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 and CS10 bits for 64 prescaler
  TCCR1B |= (1 << CS11) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  TIMSK1 |= (1 << OCIE1B);
  // Turn off Timer 1 clock source
  savePrescale = TCCR1B & (0b111<<CS10);
  TCCR1B &= ~(0b111<<CS10); // Stop Timer1 Clock
  digitalWrite(3,LOW);
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
  digitalWrite(11,LOW);
  NoPaso = true;
}

ISR(TIMER1_COMPA_vect){   // Timer1 interrupt A toggles pin 5 and 6
  if (toggle1 && Enable){
    digitalWrite(6,LOW);
    delayMicroseconds(1);
    digitalWrite(5,HIGH);
    toggle1 = 0;
  }
  else{
    digitalWrite(5,LOW);
    delayMicroseconds(1);
    digitalWrite(6,HIGH);
    toggle1 = 1;
  }
}

ISR(TIMER1_COMPB_vect){   // Timer1 interrupt B toggles pin 11 and 3
  if (toggle2 && Enable){
    digitalWrite(11,LOW);
    delayMicroseconds(1);
    digitalWrite(3,HIGH);
    toggle2 = 0;
  }
  else{
    digitalWrite(3,LOW);
    delayMicroseconds(1);
    digitalWrite(11,HIGH);
    toggle2 = 1;
  }
}

void loop() {
  if (!digitalRead(A3)) {   // High Speed
    DefSpeed = 200; 
  } else {
    DefSpeed = 50;
  }

  if (!digitalRead(0)) {    // Accelerate
    if (DefSpeed > RealSpeed) {
      NoPaso = true;
    }
    if (NoPaso) {
      NoPaso = false;
      // Turn on timer now if is was not already on
      TCCR1B |= savePrescale;  // Restart Timer1 clock.
      setWaveforms( 30 , DefAngle );
      Enable = true;
      accelerate(DefSpeed, DefDelay);
      setWaveforms( DefSpeed , DefAngle );
      RealSpeed = DefSpeed;
    }
    noBreake = true;
  }

  if (!digitalRead(1)) {    // Break
    if (noBreake) {
      noBreake = false;
      while(digitalRead(3));  // Wait for Output 3 to go LOW
      breakMotor(RealSpeed, DefDelay);
      Enable = false;
      TCCR1B &= ~(0b111<<CS10); // Stop Timer1 Clock
      digitalWrite(3,LOW);
      digitalWrite(5,LOW);
      digitalWrite(6,LOW);
      digitalWrite(11,LOW);
      RealSpeed = 0;
      NoPaso = true;
    }
  }
}

void setWaveforms( unsigned long freq , int shift ) {
  while (TCNT1 > 5);   // Wait for Timer1 to be in range
  TCNT1  = 0;//initialize counter value to 0
  // Calculate the number of clock cycles per toggle
  unsigned long clocks_per_toggle = (CLK / (freq * PRESCALER)) / 2;    // /2 becuase it takes 2 toggles to make a full wave
  // set compare match register for Frequency Generation
  OCR1A = clocks_per_toggle;
  unsigned long offset_clocks = (clocks_per_toggle * shift) / 180UL; // Do mult first to save precision
  // set compare match register for Angle Shift
  OCR1B= offset_clocks;
}

void accelerate(int speed, int delayTime) {
  if (RealSpeed == 0) RealSpeed = 30;
  int intdelay = (delayTime / 2) / (speed - RealSpeed);
  for (int i = RealSpeed; i <= speed; i++) {
    setWaveforms( i , DefAngle );
    delay(intdelay);
  }
}

void breakMotor(int speed, int delayTime) {
  int intdelay = (delayTime / 2) / (speed - 30);
  for (int i = speed; i >= 30; i--) {
    setWaveforms( i , DefAngle );
    delay(intdelay);
  }
}
