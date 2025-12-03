#include <Arduino.h>
#include <Wire.h>
#include <TimerOne.h>

// Device addresses
#define ADC1_PIN 8  // 28
#define ADC2_PIN 10 // 30
#define ADC3_PIN 4  // 25
#define ADC4_PIN A0  // 36

#define MENB1_PIN 9  // 29
#define MENB2_PIN 13 // 32
#define MENB3_PIN 6  // 27

#define LED1 A2 // 38

// LMP91000 registers
#define LMP91000_ADDR 0b1001000
// #define LMP91000_REG_STATUS 0x00
#define LMP91000_REG_LOCK 0x01
#define LMP91000_REG_TIACN 0x10
#define LMP91000_REG_REFCN 0x11
#define LMP91000_REG_MODECN 0x12

// ADC
#define ADC_SAMPLE_RATE_HZ 20
#define ADC_BUFFER_SIZE 32
volatile uint16_t adcBuffer1[ADC_BUFFER_SIZE];
volatile uint16_t adcBuffer2[ADC_BUFFER_SIZE];
volatile uint16_t adcBuffer3[ADC_BUFFER_SIZE];
volatile uint16_t adcBuffer4[ADC_BUFFER_SIZE];
volatile uint16_t adcHead = 0;
volatile uint16_t adcTail = 0;

// Function declarations
void LMP91000Configure(uint8_t menbPin);
bool LMP91000WriteRegister(uint8_t menbPin, uint8_t reg, uint8_t value);
uint8_t LMP91000ReadRegister(uint8_t menbPin, uint8_t reg);
void handleSerialInput();
void processCommand(String cmd);
void onADCTimer();

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  Serial.println("Initializing...");

  Wire.begin();
  Wire.setClock(100000); // Set I2C clock to 100kHz

  pinMode(MENB1_PIN, OUTPUT);
  pinMode(MENB2_PIN, OUTPUT);
  pinMode(MENB3_PIN, OUTPUT);
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);

  LMP91000Configure(MENB1_PIN);
  LMP91000Configure(MENB2_PIN);
  LMP91000Configure(MENB3_PIN);

  // Setup Timer1 for ADC sampling
  Timer1.initialize(1000000UL / ADC_SAMPLE_RATE_HZ); // Set period in microseconds
  Timer1.attachInterrupt(onADCTimer);

  Serial.println("Initialization complete");
  Serial.print("Sampling rate: ");
  Serial.print(ADC_SAMPLE_RATE_HZ);
  Serial.println(" Hz");
}

void loop() {
  handleSerialInput();

  while (adcTail != adcHead) {
    noInterrupts(); 
    uint16_t value1 = adcBuffer1[adcTail];
    uint16_t value2 = adcBuffer2[adcTail];
    uint16_t value3 = adcBuffer3[adcTail];
    uint16_t value4 = adcBuffer4[adcTail];
    adcTail = (adcTail + 1) % ADC_BUFFER_SIZE;
    interrupts();

    Serial.print("ADC ");
    Serial.print(value1);
    Serial.print(" ");
    Serial.print(value2);
    Serial.print(" ");
    Serial.print(value3);
    Serial.print(" ");
    Serial.println(value4);
  }
}

void onADCTimer() {
  uint16_t value1 = analogRead(ADC1_PIN);
  uint16_t value2 = analogRead(ADC2_PIN);
  uint16_t value3 = analogRead(ADC3_PIN);
  uint16_t value4 = analogRead(ADC4_PIN);

  uint16_t next = (adcHead + 1) % ADC_BUFFER_SIZE;
  if (next != adcTail) { // Check for buffer overflow
    adcBuffer1[next] = value1;
    adcBuffer2[next] = value2;
    adcBuffer3[next] = value3;
    adcBuffer4[next] = value4;
    adcHead = next;
  }
}

void LMP91000Configure(uint8_t menbPin) {
  LMP91000WriteRegister(menbPin, LMP91000_REG_LOCK, 0b0); // Disable the write protection

  Serial.print("Configured LMP91000 on pin ");
  Serial.println(menbPin);
}

bool LMP91000WriteRegister(uint8_t menbPin, uint8_t reg, uint8_t value) {
  digitalWrite(menbPin, LOW); // Enable the device

  Wire.beginTransmission(LMP91000_ADDR);
  Wire.write(reg);
  Wire.write(value);
  uint8_t status = Wire.endTransmission();
  
  digitalWrite(menbPin, HIGH); // Disable the device
  return (status == 0);
}

uint8_t LMP91000ReadRegister(uint8_t menbPin, uint8_t reg) {
  uint8_t value = 0;
  digitalWrite(menbPin, LOW); // Enable the device

  Wire.beginTransmission(LMP91000_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(LMP91000_ADDR, 1); // Request 1 byte
  if (Wire.available()) {
    value = Wire.read();
  }

  digitalWrite(menbPin, HIGH); // Disable the device
  return value;
}

void handleSerialInput() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.startsWith("WRITE")) {
    // WRITE <CHIP> <REG> <VALUE>
    unsigned int chip, reg, value;
    if (sscanf(cmd.c_str(), "WRITE %u %u %u", &chip, &reg, &value) != 3) {
      Serial.println("Syntax: WRITE <CHIP> <REG> <VALUE>");
    } else {
      bool ok;
      uint8_t pin;
      if (chip == 1) {
        pin = MENB1_PIN;
      } else if (chip == 2) {
        pin = MENB2_PIN;
      } else {
        Serial.println("WRITE ERROR: Invalid chip number (1 or 2)");
      }
      ok = LMP91000WriteRegister(pin, (uint8_t)reg, (uint8_t)value);
      if (ok)
        Serial.println("WRITE OK");
      else
        Serial.println("WRITE FAIL: I2C error");
    }
  } 
  else {
    Serial.println("Unknown command");
  }
}