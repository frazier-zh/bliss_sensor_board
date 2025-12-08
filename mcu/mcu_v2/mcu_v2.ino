/*
    Project: BLISS
    Application: BLISS Sensor Board User Interface
    File: mcu_v2.ino
    Description: Arduino MCU control code (ATMEGA32U4)
    Author: Fang Zihang (Dr.)
    Email: fang.zh@nus.edu.sg
    Affiliation: National University of Singapore
*/

#include <Arduino.h>
#include <Wire.h>
#include <TimerOne.h>

// ADC addresses
#define ADC1 0b100011   // ADC11
#define ADC2 0b100101   // ADC13
#define ADC3 0b100000   // ADC8
//#define ADC4 0b000111   // ADC7
#define ADC4_1x 0b010111   // P: ADC7, N: ADC1, Gain: 1x
#define ADC4_10x 0b101111   // P: ADC7, N: ADC1, Gain: 10x
#define ADC4_40x 0b110111   // P: ADC7, N: ADC1, Gain: 40x
#define ADC4_200x 0b111111   // P: ADC7, N: ADC1, Gain: 200x

#define MENB1_PIN 9   // 29
#define MENB2_PIN 13  // 32
#define MENB3_PIN 6   // 27

#define LED1 A2  // 38

// LMP91000 registers
#define LMP91000_ADDR 0b1001000
// #define LMP91000_REG_STATUS 0x00
#define LMP91000_REG_LOCK 0x01
#define LMP91000_REG_TIACN 0x10
#define LMP91000_REG_REFCN 0x11
#define LMP91000_REG_MODECN 0x12

// External gain, 100ohm load
#define LMP91000_TIACN_DEFAULT 0b00000011
// External reference, 50%, positive bias, 0V
#define LMP91000_REFCN_DEFAULT 0b10110000
// FET short disabled, deep sleep mode
#define LMP91000_MODECN_DEFAULT 0b00000000

// ADC
volatile uint16_t adc4 = ADC4_1x;

#define ADC_SAMPLE_RATE_HZ 20
#define ADC_BUFFER_SIZE 64
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
uint16_t ADCRead(uint16_t pin);
uint16_t ADCDifferentialRead(uint16_t pin);
void ADC4SelectGain(int gain);

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB
  }

  Serial.println("Initializing...");

  Wire.begin();
  Wire.setClock(100000);  // Set I2C clock to 100kHz

  pinMode(MENB1_PIN, OUTPUT);
  pinMode(MENB2_PIN, OUTPUT);
  pinMode(MENB3_PIN, OUTPUT);
  pinMode(LED1, OUTPUT);

  LMP91000Configure(MENB1_PIN);
  LMP91000Configure(MENB2_PIN);
  LMP91000Configure(MENB3_PIN);
  digitalWrite(LED1, HIGH);

  // Setup Timer1 for ADC sampling
  Timer1.initialize(1000000UL / ADC_SAMPLE_RATE_HZ);  // Set period in microseconds
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
  uint16_t value1 = ADCRead(ADC1);
  uint16_t value2 = ADCRead(ADC2);
  uint16_t value3 = ADCRead(ADC3);
  uint16_t value4 = ADCDifferentialRead(adc4);

  uint16_t next = (adcHead + 1) % ADC_BUFFER_SIZE;
  if (next != adcTail) {  // Check for buffer overflow
    adcBuffer1[next] = value1;
    adcBuffer2[next] = value2;
    adcBuffer3[next] = value3;
    adcBuffer4[next] = value4;
    adcHead = next;
  }
}

uint16_t ADCRead(uint16_t pin) {
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 5) & 0x01) << MUX5);
  ADMUX = (DEFAULT << REFS0) | (pin & 0x1F);
	// start the conversion
  ADCSRA |= (1 << ADSC);
	// ADSC is cleared when the conversion finishes
	while (ADCSRA & (1 << ADSC));
	// ADC macro takes care of reading ADC register.
	// avr-gcc implements the proper reading order: ADCL is read first.
	return ADC;
}

uint16_t ADCDifferentialRead(uint16_t pin) {
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 5) & 0x01) << MUX5);
  ADMUX = (DEFAULT << REFS0) | (pin & 0x1F);
	// start the conversion
  ADCSRA |= (1 << ADSC);
	// ADSC is cleared when the conversion finishes
	while (ADCSRA & (1 << ADSC));
  // return ADC; // Discard the first result
	// start the conversion
  ADCSRA |= (1 << ADSC);
	// ADSC is cleared when the conversion finishes
	while (ADCSRA & (1 << ADSC));
	// ADC macro takes care of reading ADC register.
	// avr-gcc implements the proper reading order: ADCL is read first.
	return ADC;
}

void ADC4SelectGain(int gain) {
  switch (gain) {
    case 0:
      adc4 = ADC4_1x;
      break;
    case 1:
      adc4 = ADC4_10x;
      break;
    case 2:
      adc4 = ADC4_40x;
      break;
    case 3:
      adc4 = ADC4_200x;
      break;
    default:
      adc4 = ADC4_1x;
  }
}

void LMP91000Configure(uint8_t menbPin) {
  LMP91000WriteRegister(menbPin, LMP91000_REG_LOCK, 0b0);  // Disable the write protection
  LMP91000WriteRegister(menbPin, LMP91000_REG_TIACN, LMP91000_TIACN_DEFAULT);
  LMP91000WriteRegister(menbPin, LMP91000_REG_REFCN, LMP91000_REFCN_DEFAULT);
  LMP91000WriteRegister(menbPin, LMP91000_REG_MODECN, LMP91000_MODECN_DEFAULT);

  Serial.print("Configured LMP91000 on pin ");
  Serial.println(menbPin);
}

bool LMP91000WriteRegister(uint8_t menbPin, uint8_t reg, uint8_t value) {
  digitalWrite(menbPin, LOW);  // Enable the device

  Wire.beginTransmission(LMP91000_ADDR);
  Wire.write(reg);
  Wire.write(value);
  uint8_t status = Wire.endTransmission();

  digitalWrite(menbPin, HIGH);  // Disable the device
  return (status == 0);
}

uint8_t LMP91000ReadRegister(uint8_t menbPin, uint8_t reg) {
  uint8_t value = 0;
  digitalWrite(menbPin, LOW);  // Enable the device

  Wire.beginTransmission(LMP91000_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(LMP91000_ADDR, 1);  // Request 1 byte
  if (Wire.available()) {
    value = Wire.read();
  }

  digitalWrite(menbPin, HIGH);  // Disable the device
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
    // WRITE <ID> <REG> <VALUE>
    unsigned int id, reg, value;
    if (sscanf(cmd.c_str(), "WRITE %u %u %u", &id, &reg, &value) != 3) {
      Serial.println("Syntax: WRITE <CHIP> <REG> <VALUE>");
    } else {
      bool ok = false;
      uint8_t pin;
      switch (id) {
        case 1:
          ok = LMP91000WriteRegister(MENB1_PIN, (uint8_t)reg, (uint8_t)value);
          break;
        case 2:
          ok = LMP91000WriteRegister(MENB2_PIN, (uint8_t)reg, (uint8_t)value);
          break;
        case 3:
          ok = LMP91000WriteRegister(MENB3_PIN, (uint8_t)reg, (uint8_t)value);
          break;
        case 4:
          ADC4SelectGain(value);
          ok = true;
          break;
        default:
          Serial.println("WRITE ERROR: Invalid id number [1-4]");
      }
      if (ok)
        Serial.println("WRITE OK");
      else
        Serial.println("WRITE FAIL: I2C error");
    }
  } else {
    Serial.println("Unknown command");
  }
}