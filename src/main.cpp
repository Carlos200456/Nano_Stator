#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define OLED          // Con Display OLED

#ifdef OLED
// OLED 0.96"
// U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
// OLED 1.3"                R0 = Rotate 0°,  R2 = Rotate 180°
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);
// Full Graphic Smart Controller
// U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ A5, /* data=*/ A4, /* CS=*/ 10, /* reset=*/ U8X8_PIN_NONE);
#endif

// Define the pin for the frequency measurement
#define INPUT_PIN 2

// Variable to store the frequency
volatile unsigned long frequency;

// Variable to store the time of the last pulse
volatile unsigned long lastPulseTime, previousPulseTime;

uint8_t saveTCCMode;
uint8_t saveTIMMode;
uint8_t G1 = 5;
uint8_t G2 = 6;
uint8_t G3 = 3;
uint8_t G4 = 11;

bool NoPaso = true;
bool noBreake = false;
boolean toggle1 = 0;
boolean toggle2 = 0;
boolean Enable_I = false;
boolean Enable_C = false;
byte savePrescale;
int DefSpeed = 0;
int RealSpeed = 0;
int DefDelay = 1500;
int DefAngle = 0;
unsigned long DefInterval = 0;
int DefSpeedPrev = 0;
unsigned long Init_Time = 0;

// put function declarations here:
void setWaveforms(unsigned long, int);
void accelerate(int speed, int delayTime);
void breakMotor(int speed, int delayTime);
void countPulse(void);
void configTimerForPulse(void);
void configTimerForMeasure(void);
void PrintStatus(String status);
void Reverse(void);
void Fordward(void);

// prescaler of 1 will get us 8MHz - 488Hz
// User a higher prescaler for lower freqncies
#define PRESCALER 64
#define CLK 16000000UL    // Default clock speed is 16MHz on Arduino Uno


void setup() {
  // Set the pin as an input
  pinMode( 13, INPUT_PULLUP); // Arduino Pin 13 = Button Accelerate.
  pinMode( 1, INPUT_PULLUP);  // Arduino Pin 1 = Button Break
  pinMode( 2, INPUT_PULLUP);  // Arduino Pin 2 = Frequency Feedback
  pinMode(A3, INPUT_PULLUP);  // Arduino Pin A3 = Button High Speed
  pinMode( 12, INPUT_PULLUP); // Arduino Pin 12 = Reverse Jumper
  pinMode( 7, INPUT_PULLUP);  // Arduino Pin 7 = Sin Cap Jumper
  pinMode(A7, INPUT);         // Arduino Pin A7 = Desfasage
  pinMode(A6, INPUT);         // Arduino Pin A6 = Max Speed
  pinMode(A1, INPUT);         // Arduino Pin A1 = Speed Up Time
  pinMode(A0, INPUT);         // Arduino Pin A0 = Energize Interval

  // Set the Gx pin at LOW state
  digitalWrite(G1,LOW);
  digitalWrite(G2,LOW);
  digitalWrite(G3,LOW);
  digitalWrite(G4,LOW);

  // Set the pin as an output
  pinMode(G1, OUTPUT);
  pinMode(G2, OUTPUT);
  pinMode(G3, OUTPUT);
  pinMode(G4, OUTPUT);

  #ifdef OLED
  u8g2.begin();  // initialize with the I2C
  u8g2.setPowerSave(0);
  // init done
  // Set a small footprint font
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setFontDirection(0);
  u8g2.clearBuffer();
  u8g2.setCursor(0, 10);
  u8g2.print("Pimax S.R.L.");
  u8g2.setCursor(0,20);             // Column, Row
  u8g2.print("Nano Starter");
  u8g2.setCursor(0,30);             // Column, Row
  u8g2.print("Version 1.Git");     // SOFTWARE VERSION ---------------------------<<<<<<<<<<<<<<<<
  u8g2.sendBuffer();
  delay(2000);
  u8g2.clearDisplay();
  #endif

  // Save TCC mode
  saveTCCMode = TCCR1B;
  // Save Timer 1 interrupt mask
  saveTIMMode = TIMSK1;
  NoPaso = true;

  // Attach the interrupt
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), countPulse, RISING);
}

void loop() {
  if (Init_Time == 0) Init_Time = millis();
  // Read the High Speed button
  if (!digitalRead(A3)) {
    DefSpeed = map(analogRead(A6), 0, 1023, 120, 220);
  } else {
    DefSpeed = 60;
  }

  DefAngle = map(analogRead(A7), 0, 1023, 60, 120);
  DefDelay = map(analogRead(A1), 0, 1023, 800, 2500);
  DefInterval = map(analogRead(A0), 0, 1023, 2000, 5000);

  // Read the Reverse jumper
  if (!digitalRead(12)) {
    Reverse();
  } else {
    Fordward();
  }

  // Read the Sin Cap jumper
  if (!digitalRead(7)) {
    Enable_C = false;
  } else {
    // Enable_C = true;
  }

  if(DefSpeedPrev != DefSpeed){
    DefSpeedPrev = DefSpeed;
    #ifdef OLED 
      PrintStatus("Speed Changed");
    #endif
    Init_Time = millis();
  }

  if (((millis() - Init_Time) > 400) && Enable_I){
    if (RealSpeed != 0){
      #ifdef OLED 
        PrintStatus("Keep Speed");
      #endif
      Enable_I = false;
      delay(100);
    }
  }

  if (((millis() - Init_Time) > 1400) && !Enable_I){
    if (RealSpeed != 0){
      configTimerForMeasure();
      #ifdef OLED 
        PrintStatus("Measuring");
      #endif
      if (previousPulseTime != lastPulseTime) {
        // Calculate frequency in Hz
        frequency = 1000000 / (lastPulseTime - previousPulseTime);
        previousPulseTime = lastPulseTime;
      }
    }
  }

  if ((millis() - Init_Time) > DefInterval){    // Energize Interval
    Init_Time = 0;
    if (RealSpeed == 0) {
      configTimerForMeasure();
      #ifdef OLED 
        PrintStatus("Stopped");
      #endif
    } else {
      configTimerForPulse();
      #ifdef OLED 
        PrintStatus("Energized");
      #endif
      setWaveforms( RealSpeed , DefAngle );
      Enable_I = true;
      Enable_C = false;
    }
  }
  
  if (!digitalRead(13)) {    // Accelerate
    Init_Time = 0;
    if (DefSpeed > RealSpeed) {
      NoPaso = true;
    }
    if (NoPaso) {
      NoPaso = false;
      #ifdef OLED 
        PrintStatus("Accelerating");
      #endif
      configTimerForPulse();
      setWaveforms( 30 , DefAngle );
      Enable_I = true;
      Enable_C = true;
      accelerate(DefSpeed, DefDelay);
      setWaveforms( DefSpeed , DefAngle );
      RealSpeed = DefSpeed;
      Init_Time = 0;
    }
    noBreake = true;
  }

  if (!digitalRead(1)) {    // Break
    if (noBreake) {
      #ifdef OLED 
        PrintStatus("Breaking");
      #endif
      Enable_I = true;
      Enable_C = true;
      noBreake = false;
      while(digitalRead(G1));  // Wait for Output 1 to go LOW
      configTimerForPulse();
      breakMotor(RealSpeed, DefDelay);
      Enable_I = false;
      delay(100);
      configTimerForMeasure();
      RealSpeed = 0;
      NoPaso = true;
      Init_Time = millis();
      Init_Time -= 2000;
    }
  }

  
}
#ifdef OLED 
void PrintStatus(String status){
  // Write a menu on display
  u8g2.clearBuffer();
  u8g2.setCursor(0,10);             // Column, Row
  u8g2.print("Speed : ");
  u8g2.setCursor(0,20);             // Column, Row
  u8g2.print("Fase  : ");
  u8g2.setCursor(0,30);             // Column, Row
  u8g2.print("T Arr : ");
  u8g2.setCursor(0,40);             // Column, Row
  u8g2.print("I Mant: ");
  u8g2.setCursor(0,50);             // Column, Row
  u8g2.print("Giro R: ");
  u8g2.setCursor(0,60);             // Column, Row
  u8g2.print("Status: ");
  u8g2.setCursor(44,10);             // Column, Row
  if (RealSpeed > DefSpeed) u8g2.print(RealSpeed); else u8g2.print(DefSpeed);
  u8g2.print("  ");
  u8g2.setCursor(44,20);             // Column, Row
  u8g2.print(DefAngle);
  u8g2.print("  ");
  u8g2.setCursor(44,30);             // Column, Row
  u8g2.print(DefDelay);
  u8g2.print("  ");
  u8g2.setCursor(44,40);             // Column, Row
  u8g2.print(DefInterval);
  u8g2.print("  ");
  u8g2.setCursor(44,50);             // Column, Row
  u8g2.print(frequency);
  u8g2.print("  ");
  u8g2.setCursor(44,60);             // Column, Row
  u8g2.print(status);
  u8g2.sendBuffer();
}
#endif


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
  toggle1 = 0;
  toggle2 = 0;
  OCR1B = 0;
  OCR1A = 0;
}

ISR(TIMER1_COMPA_vect){   // Timer1 interrupt A toggles pin 5 and 6
  if (Enable_I) {
    if (toggle1){
      digitalWrite(G2,LOW);
      delayMicroseconds(1);
      digitalWrite(G1,HIGH);
      toggle1 = 0;
    }
    else{
      digitalWrite(G1,LOW);
      delayMicroseconds(1);
      digitalWrite(G2,HIGH);
      toggle1 = 1;
    }
  }
  else {
    digitalWrite(G1,LOW);
    digitalWrite(G2,LOW);
  }
}

ISR(TIMER1_COMPB_vect){   // Timer1 interrupt B toggles pin 11 and 3
  if (Enable_I && Enable_C) {
    if (toggle2){
      digitalWrite(G4,LOW);
      delayMicroseconds(1);
      digitalWrite(G3,HIGH);
      toggle2 = 0;
    }
    else{
      digitalWrite(G3,LOW);
      delayMicroseconds(1);
      digitalWrite(G4,HIGH);
      toggle2 = 1;
    }
  }
  else {
    digitalWrite(G3,LOW);
    digitalWrite(G4,LOW);
  }
}

// Interrupt service routine to count the pulse
void countPulse() {
  previousPulseTime = lastPulseTime;
  lastPulseTime = micros();
}

void setWaveforms( unsigned long freq , int shift ) {
  // TODO: Verify if the rigth value is (TCNT1 < 5)
  while (TCNT1 < 5);   // Wait for Timer1 to be in range
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
  if (RealSpeed == 0) RealSpeed = 20;
  int intdelay = (delayTime / 2) / (speed - RealSpeed);
  for (int i = RealSpeed; i <= speed; i++) {
    setWaveforms( i , DefAngle );
    delay(intdelay);
  }
}

void breakMotor(int speed, int delayTime) {
  int intdelay = (delayTime / 2) / (speed - 20);
  for (int i = speed; i >= 20; i--) {
    setWaveforms( i , DefAngle );
    delay(intdelay);
  }
}

void Reverse(void){
  G1 = 3;
  G2 = 11;
  G3 = 5;
  G4 = 6;
}

void Fordward(void){
  G1 = 5;
  G2 = 6;
  G3 = 3;
  G4 = 11;
}