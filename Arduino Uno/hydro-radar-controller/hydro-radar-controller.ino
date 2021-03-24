#include <SPI.h>
#define BYTES_PER_REGISTER 4

void transferRegister(const byte registerName[]);

void trigger();

// SPI pinout
const int loadEnablePin = 8;
// Data is transferred from the shift register to one of eight latches on the rising edge of loadEnablePin.
const int chipEnablePin = 9; // Power-on ADF4158
const int chipSelectNegPin = 10; // Must be set as OUTPUT, also if not used.
const int clockPin = 13;
const int serialOutPin = 11;

/*
 * DSO external triggering:
 * Several transitions of clockPin occurs before the SPI initialization.
 * This additional signal is used to trigger the DSO (one shot) when you want.
 */
const int externalTriggerPin = 7;

// BGT24MTR11 programming words.
// const byte MSB = B00000000;     //LNA Gain Red off -          Not Used       -         Not Used       - disable TX pwr on -        AMUX2 0     -   Test Bit  -   Test Bit  -   AMUX1 0
// const byte LSB = B00001000;     //    AMUX0 0      - disable 64k divider off - disable 16 divider off - Hi LO buffer off  - High TX buffer off - TW pwr red2 - TW pwr red1 - TW pwr red0
// TX ON:
const byte communication_MSB = B00000000;
// TX OFF:
// const byte communication_MSB = B00010000;

// Full TX power:
// const byte communication_LSB = B00001000;
// TX power reduced of 9dB:
 const byte communication_LSB = B00001111;
// TX power reduced of 6dB:
// const byte communication_LSB = B00001110;
// TX power reduced of 4dB:
// const byte communication_LSB = B00001101;

// ADF4158 programming words.
// Register values (obtained using ADIsimPLL and ADF4158 evaluation software):
const byte R0[] =       { 0x80, 0x24, 0xB8, 0x00};
const byte R1[] =       { 0x00, 0x00, 0x00, 0x01};
const byte R2[] =       { 0x02, 0x10, 0xFF, 0xFA};
const byte R3[] =       { 0x00, 0x00, 0x00, 0x43};
//const byte R4[] =       { 0x00, 0x1F, 0xFF, 0x84}; // 0.8s per step.
const byte R4[] =       { 0x00, 0x18, 0xF4, 0x04}; // 0.1s per step.
const byte R5_load1[] = { 0x00, 0x4B, 0xFF, 0xFD};
const byte R5_load2[] = { 0x00, 0x80, 0x00, 0x7D};
const byte R6_load1[] = { 0x00, 0x00, 0x00, 0x56};
const byte R6_load2[] = { 0x00, 0x80, 0x00, 0x06};
byte R7[] =             { 0x00, 0x02, 0xFF, 0xFF};

void setup() {
  pinMode(chipSelectNegPin,OUTPUT);
  pinMode(loadEnablePin,OUTPUT);
  digitalWrite(loadEnablePin,LOW);
  pinMode(externalTriggerPin,OUTPUT);
  digitalWrite(externalTriggerPin,LOW);
  pinMode(chipEnablePin,OUTPUT);
  pinMode(clockPin,OUTPUT);
  pinMode(serialOutPin,OUTPUT);
  
  Serial.begin(9600);

  // BGT24MTR11 programming.
  Serial.println("BGT24MTR11 programming started...");
  SPI.beginTransaction(SPISettings(125000,MSBFIRST,SPI_MODE0)); // min. 125kHz; strange signals on clockPin line due to this line.

//  trigger();
  
  digitalWrite(chipSelectNegPin,HIGH);
  delayMicroseconds(20);
  digitalWrite(chipSelectNegPin,LOW);
  delayMicroseconds(20);
  SPI.transfer(communication_MSB);
  SPI.transfer(communication_LSB);
  delayMicroseconds(20);
  digitalWrite(chipSelectNegPin,HIGH);
  delayMicroseconds(20);
  SPI.endTransaction();
  Serial.println("BGT24MTR11 programming ended.");
  
  // ADF4158 programming.
  Serial.println("ADF4158 programming started...");
  SPI.beginTransaction(SPISettings(125000,MSBFIRST,SPI_MODE0)); // min. 125kHz; strange signals on clockPin line due to this line.
  
//  trigger();
  
  digitalWrite(chipEnablePin,LOW);
  delayMicroseconds(20);
  digitalWrite(chipEnablePin,HIGH);
  delay(200); // Waiting time for ADF4158 power-on.

  transferRegister(R7);
  transferRegister(R6_load1);
  transferRegister(R6_load2);
  transferRegister(R5_load1);
  transferRegister(R5_load2);
  transferRegister(R4);
  transferRegister(R3);
  transferRegister(R2);
  transferRegister(R1);
  transferRegister(R0); //last one to be loaded (double-buffered).

  SPI.endTransaction();

  Serial.println("ADF4158 programming ended.");

  }

  
void loop() {
  
}

void transferRegister(const byte registerName[]){
  trigger();
  // Data transfer into shift register:
  for (int index=0; index<BYTES_PER_REGISTER; index++){
    SPI.transfer(registerName[index]);
  }
  // Load into the specific 32-bits latch:
  delayMicroseconds(20);
  digitalWrite(loadEnablePin,HIGH);
  delayMicroseconds(100);
  digitalWrite(loadEnablePin,LOW);
  delay(80); // On EVB programming is 80ms.
  return;
}

void trigger(){
  digitalWrite(externalTriggerPin,HIGH);
  delayMicroseconds(20);
  digitalWrite(externalTriggerPin,LOW);
  return;
}

/* Oscilloscope settings:
 *  channel 1: serialOutPin or chipEnablePin
 *  channel 2: clockPin
 *  trigger: EXT, level:1.2V, mode:edge, slope:positive, sweep:single
 *  vertical scale: 2V for both channels
 *  horizontal scale: 200us, trigger anticipated of 1.184ms
 */
