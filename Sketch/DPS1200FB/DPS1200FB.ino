#define TINY  // Specify build for attiny

/*
 * ATTINY85 Wiring
 *       ----------
 * /RST--|1      8|--  VCC
 * A3  --|2      7|--  SCL
 * A2  --|3      6|--  1 (NC)
 * GND --|4      5|--  SDA
 *       ----------
 * 
 * AVRDude
 * avrdude -c usbtiny -P usb -p attiny85
 * avrdude -c usbtiny -P usb -p attiny85 -e -U lfuse:w:0xE2:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m -U lock:w:0xFF:m
 * avrdude -c usbtiny -P usb -p attiny85 -e -U flash:w:firmware.hex:a
 * Fuses
 * Internal RC OSC to 8M, do not divide to keep at 8M
 * http://eleccelerator.com/fusecalc/fusecalc.php?chip=attiny85&LOW=E2&HIGH=DF&EXTENDED=FF&LOCKBIT=FF
 * -U lfuse:w:0xE2:m
 * -U hfuse:w:0xDF:m
 * -U efuse:w:0xFF:m 
 * -U lock:w:0xFF:m
 * 
 * See Repo for schematic
 * 
 * ATTINY PINS
 * 1 - /RST -> Pulled high w/ 10K Resistor
 * 2 - A3 - to 3-pin jumper for analog pot
 * 3 - A2 - Pulled high internally (via code INPUT_PULLUP) - Jump to gnd to disable A3
 * 4 - GND
 * 5 - SDA - Connected to SDA Pin 32 on PSU
 * 6 - Digital Pin 1 - NC
 * 7 - SCL - Connected to SCL Pin 31 on PSU
 * 8 - VCC - 5V from reg
 * 
 * PSU PINS    
 * (Bottom)
 * 30 - GND
 * 31 - SCL
 * 32 - SDA
 * 
 * (Top)
 * 33 - /ENABLE (Connected to Present with 1K Resistor)
 * 34 - Current Monitor (NC)
 * 35 - PSU Status (NC)
 * 36 - Present (Connected to /Enable with 1K Resistor)
 * 37 - +12v Standby (Input to regulator)
 * 38 - PSU Alarm (NC)
 */

#ifdef TINY
/* https://github.com/lucullusTheOnly/TinyWire */
// Add as zip lib
#include <twi.h>
#include <TinyWire.h>
#else
#include <Wire.h>
#endif

/* Enable Serial to print debug messages */
#ifndef TINY
  #define SERIALEN
#endif

/* Enable I2C - Enable / Disable messages going to PSU - useful for debugging */
#define I2CEN
/* Address of PIC MCU in PSU (0x57 -> EEPROM, 0x5F -> PIC MCU) - DOESN'T ACCOUNT FOR ADDRESS SELECT PINS */
#define I2CADDR 0x5F 

#define STATIC_RPM 8000

/* Analog Control - adjust PSU RPM Fan via Analog input - uses map function */
#define AIN_PIN   A3  // Works for both ATTINY85 and Pro Micro
#define ACTRL_PIN A2  // Works for both ATTINY85 and Pro Micro

/* Map values for analog control */
#define MAP_INIT_LOW 0 
#define MAP_INIT_HIGH 1023
#define MAP_END_LOW 3300
#define MAP_END_HIGH 16000

#define LOOP_DELAY 1000

// analog input val
uint16_t ain_val = MAP_INIT_LOW;

// vals for i2c write
uint8_t  reg = 0x40;
uint8_t  valLSB = 0;
uint8_t  valMSB = 0;
uint16_t rpm_value = STATIC_RPM;  // rpm
uint16_t cs = 0;
uint8_t  regCS = 0;
uint8_t  data[4] = {0,0,0,0};
uint8_t  analog_ctrl_state = HIGH;

void setup() {
  // put your setup code here, to run once:
  #ifdef I2CEN
    #ifndef TINY
      Wire.begin();
    #else
      TinyWire.begin();
    #endif
  #endif

  pinMode(AIN_PIN, INPUT);
  pinMode(ACTRL_PIN, INPUT_PULLUP);
  analog_ctrl_state = digitalRead(ACTRL_PIN);

  #ifdef SERIALEN
    Serial.begin(9600);
    while (!Serial); // Leonardo: wait for serial monitor
  #endif
}

void loop() {
  analog_ctrl_state = digitalRead(ACTRL_PIN);
  if(analog_ctrl_state == HIGH) {
    // Jumper not installed
    // read analog value
    ain_val = analogRead(AIN_PIN);
    rpm_value = map(ain_val, MAP_INIT_LOW, MAP_INIT_HIGH, MAP_END_LOW, MAP_END_HIGH);
  } else {
    // Jumper installed
    rpm_value = STATIC_RPM;
  } 
  
  #ifdef SERIALEN
    Serial.print(ain_val);
    Serial.print(" : ");
    Serial.println(rpm_value);
  #endif

  #ifdef I2CEN
    // Most of the following is taken from the dump work from this github repo
    // https://github.com/raplin/DPS-1200FB/blob/master/DPS-1200FB.py
  
    // Extract MSB and LSB from RPM
    valLSB=rpm_value&0xff;
    valMSB=rpm_value>>8;
    // calculate checksum - the checksum is the 'secret sauce'
    cs=(I2CADDR<<1)+reg+valLSB+valMSB;
    regCS=((0xff-cs)+1)&0xff;
   
    // pack the data
    data[0] = reg;
    data[1] = valLSB;
    data[2] = valMSB;
    data[3] = regCS;
  
    // write to psu
    #ifndef TINY
      Wire.beginTransmission(I2CADDR);
      Wire.write(data, 4);
      Wire.endTransmission();
    #else
      TinyWire.beginTransmission(I2CADDR);
      for(uint8_t i=0; i<4; i++) {
        TinyWire.send(data[i]);
      }
      TinyWire.endTransmission();
    #endif
  #endif

  // basic loop delay - resend i2c message every LOOP_DELAY ms
  delay(LOOP_DELAY);
}
