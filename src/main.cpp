#include <Arduino.h>

bool NoPaso = true;
bool noBreake = false;
boolean toggle1 = 0;
boolean toggle2 = 0;
int DefSpeed = 180;
int DefDelay = 1500;
int DefAngle = 90;


// put function declarations here:
void Frec30hz(void);
void Frec120hz(void);
void setWaveforms(unsigned long, int);
void accelerate(int speed, int delayTime);
void breakMotor(int speed, int delayTime);

// prescaler of 1 will get us 8MHz - 488Hz
// User a higher prescaler for lower freqncies
#define PRESCALER 4
#define PRESCALER_BITS 0x01
#define CLK 16000000UL    // Default clock speed is 16MHz on Arduino Uno


void setup() {
  pinMode( 0 , INPUT_PULLUP); // Arduino Pin A0 = Button 30Hz.
  pinMode(1, INPUT_PULLUP);   // Arduino Pin A1 = Button 120Hz.
  pinMode(A2, INPUT_PULLUP);   // Arduino Pin A2 = Button Accelerate.
  
  // Configure Timer 0 for Phase-Correct PWM
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  // Configure Timer 2 for Phase-Correct PWM
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  // initialize Timer1
  cli();          // disable global interrupts
//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register
  OCR1A = 2500;  // = (16*10^6) / (50*256) - 1 (must be <65536)
  // set compare match register for 1hz increments
  OCR1B = 1250;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 and CS10 bits for 64 prescaler
  TCCR1B |= (1 << CS11) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  TIMSK1 |= (1 << OCIE1B);
  sei();          // enable global interrupts   
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  if (toggle1){
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

ISR(TIMER1_COMPB_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  if (toggle2){
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
  if (!digitalRead(0)) {
    if (NoPaso) {
      // accelerate(DefSpeed, DefDelay);
      Frec30hz();
      NoPaso = false;
    }
    // setWaveforms( DefSpeed , DefAngle );
    noBreake = true;
  }
  else {
    if(digitalRead(1)){
      // Turn off timer0 and timer2
      TCCR0B = 0b00000000; // No clock source (timer stopped)
      TCCR2B = 0b00000000; // No clock source (timer stopped)
      digitalWrite(LED_BUILTIN, LOW);
      NoPaso = true;
    }
  }

  if (!digitalRead(1)) {
    if (NoPaso) {
      Frec120hz();
      NoPaso = false;
    }
    noBreake = true;
  }
  else {
    if (digitalRead(0)){
      // Turn off timer0 and timer2
      TCCR0B = 0b00000000; // No clock source (timer stopped)
      TCCR2B = 0b00000000; // No clock source (timer stopped)
      digitalWrite(LED_BUILTIN, LOW);
      NoPaso = true;
    }
  }
}

void setWaveforms( unsigned long freq , int shift ) {
  // Calculate the number of clock cycles per toggle
  unsigned long clocks_per_toggle = (CLK / (freq * PRESCALER)) / 4;    // /2 becuase it takes 2 toggles to make a full wave
  ICR1 = clocks_per_toggle;
  unsigned long offset_clocks = (clocks_per_toggle * shift) / 180UL; // Do mult first to save precision
  OCR1B= offset_clocks;
  // Turn on timer now if is was not already on
  TCCR1B |= _BV( CS11 ); 
}

void accelerate(int speed, int delayTime) {
  int intdelay = delayTime / (speed - 30);
  for (int i = 30; i <= speed; i++) {
    setWaveforms( i , DefAngle );
    delay(intdelay);
  }
}

// void breakMotor(int speed, int delayTime) {
//   int intdelay = delayTime / (speed - 30);
//   for (int i = speed; i >= 30; i--) {
//     setWaveforms( i , DefAngle );
//     delay(intdelay);
//   }
// }

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
