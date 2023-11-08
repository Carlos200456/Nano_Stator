#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

#define OLED          // Con Display OLED

#ifdef OLED
// OLED 0.96"
// U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
// OLED 1.3"
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
#endif

// Define the pin for the frequency measurement
#define INPUT_PIN 2

// Variable to store the frequency
volatile unsigned long frequency;

// Variable to store the time of the last pulse
volatile unsigned long lastPulseTime, previousPulseTime;

uint8_t saveTCCMode;
uint8_t saveTIMMode;

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
int Steps = 0;


// put function declarations here:
void setWaveforms(unsigned long, int);
void accelerate(int speed, int delayTime);
void breakMotor(int speed, int delayTime);
void countPulse(void);
void configTimerForPulse(void);
void configTimerForMeasure(void);

// prescaler of 1 will get us 8MHz - 488Hz
// User a higher prescaler for lower freqncies
#define PRESCALER 64
#define CLK 16000000UL    // Default clock speed is 16MHz on Arduino Uno


void setup() {
  pinMode( 0, INPUT_PULLUP);  // Arduino Pin 0 = Button Accelerate.
  pinMode( 1, INPUT_PULLUP);  // Arduino Pin 1 = Button Break
  pinMode( 2, INPUT_PULLUP);  // Arduino Pin 2 = Frequency Feedback
  pinMode(A3, INPUT_PULLUP);  // Arduino Pin A3 = Button High Speed
  
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(13, OUTPUT);

  #ifdef OLED
  u8x8.begin();  // initialize with the I2C
  u8x8.setPowerSave(0);
  // init done
  u8x8.setFont(u8x8_font_pxplustandynewtv_r);  // u8x8_font_torussansbold8_r, u8x8_font_chroma48medium8_r, u8x8_font_victoriamedium8_r, u8x8_font_pxplustandynewtv_r
  u8x8.draw1x2String(0,0,"Pimax S.R.L.");
  u8x8.setCursor(0,2);             // Column, Row
  u8x8.print("Nano Starter");
  u8x8.setCursor(0,3);             // Column, Row
  u8x8.print("Version 1.Git");     // SOFTWARE VERSION ---------------------------<<<<<<<<<<<<<<<<
  delay(2000);
  u8x8.clearDisplay();
  #endif

  // Save TCC mode
  saveTCCMode = TCCR1B;
  // Save Timer 1 interrupt mask
  saveTIMMode = TIMSK1;
  digitalWrite(3,LOW);
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
  digitalWrite(11,LOW);
  NoPaso = true;

  // Write a menu on display

  #ifdef OLED
  u8x8.setCursor(0,0);             // Column, Row
  u8x8.print("Speed : ");
  u8x8.setCursor(0,1);             // Column, Row
  u8x8.print("Angle : ");
  u8x8.setCursor(0,2);             // Column, Row
  u8x8.print("Delay : ");
  u8x8.setCursor(0,3);             // Column, Row
  u8x8.print("Status: ");
  u8x8.setCursor(0,4);             // Column, Row
  u8x8.print("Real F: ");
  #endif

  // Attach the interrupt
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), countPulse, RISING);
}

void loop() {
  if (!digitalRead(A3)) {   // High Speed
    DefSpeed = 200; 
  } else {
    DefSpeed = 50;
  }

  if (previousPulseTime != lastPulseTime) {
    // Calculate frequency in Hz
    frequency = 1000000 / (lastPulseTime - previousPulseTime);
    previousPulseTime = lastPulseTime;
  }

  u8x8.setCursor(8,0);             // Column, Row
  u8x8.print(DefSpeed);
  u8x8.print("  ");
  u8x8.setCursor(8,1);             // Column, Row
  u8x8.print(DefAngle);
  u8x8.print("  ");
  u8x8.setCursor(8,2);             // Column, Row
  u8x8.print(DefDelay);
  u8x8.print("  ");
  if (Steps == 490){
    u8x8.setCursor(8,4);             // Column, Row
    u8x8.print(frequency);
    u8x8.print("  ");
  }

  if (Steps == 100){
    if (RealSpeed != 0){
      u8x8.setCursor(8,3);             // Column, Row
      u8x8.print("Mantain");
      u8x8.print(" ");
      Enable = false;
    }
  }

  if (Steps == 400){
    if (RealSpeed != 0){
      configTimerForMeasure();
      u8x8.setCursor(8,3);             // Column, Row
      u8x8.print("Measure");
      u8x8.print(" ");
    }
  }

  if (Steps == 500){
    Steps = 0;
    if (RealSpeed == 0) {
      configTimerForMeasure();
      u8x8.setCursor(8,3);             // Column, Row
      u8x8.print("Stopped");
      u8x8.print(" ");
    } else {
      configTimerForPulse();
      setWaveforms( RealSpeed , DefAngle );
      u8x8.setCursor(8,3);             // Column, Row
      u8x8.print("Running");
      u8x8.print(" ");
      Enable = true;
    }
  }
  
  Steps++;
  
  if (!digitalRead(0)) {    // Accelerate
    if (DefSpeed > RealSpeed) {
      NoPaso = true;
    }
    if (NoPaso) {
      NoPaso = false;
      u8x8.setCursor(8,3);             // Column, Row
      u8x8.print("Accelera");
      configTimerForPulse();
      setWaveforms( 30 , DefAngle );
      Enable = true;
      accelerate(DefSpeed, DefDelay);
      setWaveforms( DefSpeed , DefAngle );
      RealSpeed = DefSpeed;
      Steps = 0;
    }
    noBreake = true;
  }

  if (!digitalRead(1)) {    // Break
    if (noBreake) {
      u8x8.setCursor(8,3);             // Column, Row
      u8x8.print("Break");
      u8x8.print("   ");
      noBreake = false;
      while(digitalRead(3));  // Wait for Output 3 to go LOW
      breakMotor(RealSpeed, DefDelay);
      Enable = false;
      configTimerForMeasure();
      RealSpeed = 0;
      NoPaso = true;
    }
  }
}

void configTimerForMeasure(void){
  // Configure Timer 1 for Frequency Measurement
  TCCR1B = saveTCCMode;// same for TCCR1B
  TIMSK1 = saveTIMMode;
}

void configTimerForPulse(void){
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
}

ISR(TIMER1_COMPA_vect){   // Timer1 interrupt A toggles pin 5 and 6
  if (Enable) {
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
  else {
    digitalWrite(5,LOW);
    digitalWrite(6,LOW);
  }
}

ISR(TIMER1_COMPB_vect){   // Timer1 interrupt B toggles pin 11 and 3
  if (Enable) {
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
  else {
    digitalWrite(3,LOW);
    digitalWrite(11,LOW);
  }
}

// Interrupt service routine to count the pulse
void countPulse() {
  previousPulseTime = lastPulseTime;
  lastPulseTime = micros();
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
