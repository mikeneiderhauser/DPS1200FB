// Include TinyWireM Lib for ATTINY I2C over USI
#include <TinyWireM.h>
#include <USI_TWI_Master.h>

/*
 * ATTINY85 Wiring
 *       ----------
 * /RST--|1      8|--  VCC
 * A3  --|2      7|--  SCL
 *  2  --|3      6|--  1 - LED
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
 * 1 - /RST - Pulled high w/ 10K Resistor
 * 2 - A3 - to 3-pin jumper for analog pot
 * 3 -  2 - Pulled high internally (via code INPUT_PULLUP) - Jump to gnd to disable A3
 * 4 - GND
 * 5 - SDA - Connected to SDA Pin 32 on PSU
 * 6 -  1  - LED with 1K resistor to GND
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

/* Address of PIC MCU in PSU (0x57 -> EEPROM, 0x5F -> PIC MCU) - DOESN'T ACCOUNT FOR ADDRESS SELECT PINS */
#define I2CADDR 0x5F 
#define STATIC_RPM 8000

/* Analog Control - adjust PSU RPM Fan via Analog input - uses map function */
#define AIN_PIN   A3  // Works for both ATTINY85 and Pro Micro
#define LED_PIN 1     // Pin for blink code LED
#define ACTRL_PIN  2  // Works for both ATTINY85 and Pro Micro

/* Map values for analog control */
#define MAP_INIT_LOW 0 
#define MAP_INIT_HIGH 1023
#define MAP_END_LOW 3300
#define MAP_END_HIGH 16000

/* Common Delays */
#define LOOP_DELAY 1000
#define BLINK_TIME 100

// vals for i2c write
uint8_t  reg = 0x40;
uint8_t  valLSB = 0;
uint8_t  valMSB = 0;
uint16_t rpm_value = STATIC_RPM;  // rpm
uint16_t cs = 0;
uint8_t  regCS = 0;
uint8_t  data[4] = {0,0,0,0};
uint8_t  analog_ctrl_state = HIGH;

#define ENBLINKCODES // Enable / disable led blink codes.

void blinkCode(uint8_t ct){
  #ifdef ENBLINKCODES
  for(uint8_t i = 0; i<ct; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(BLINK_TIME);
    digitalWrite(LED_PIN, LOW);
    delay(BLINK_TIME);
  }
  // end blink timeout
  delay(LOOP_DELAY);
  #endif
}

void setup() {
  pinMode(LED_PIN, OUTPUT);          // LED Pin as output for blink codes
  digitalWrite(LED_PIN, LOW);        // Initial state of LED is off
  pinMode(AIN_PIN, INPUT);           // Analog input pin for potentiometer 
  pinMode(ACTRL_PIN, INPUT_PULLUP);  // Control Pin -> Jumper on for static RPM, Jumper off for potentiometer ctrl

  // Start I2C
  TinyWireM.begin();

  // read control state
  analog_ctrl_state = digitalRead(ACTRL_PIN);
}

void loop() {
  blinkCode(1);

  // re-read control state
  analog_ctrl_state = digitalRead(ACTRL_PIN);
  
  if(analog_ctrl_state == HIGH) {
    // Jumper not installed
    // read analog value and map to RPM
    rpm_value = map((uint16_t)analogRead(AIN_PIN), MAP_INIT_LOW, MAP_INIT_HIGH, MAP_END_LOW, MAP_END_HIGH);
  } else { // LOW
    // Jumper installed
    rpm_value = STATIC_RPM;
  } 
  
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
  blinkCode(2);
  // I2C Transaction
  TinyWireM.beginTransmission(I2CADDR);
  TinyWireM.write(data,4);
  TinyWireM.endTransmission();
  blinkCode(3);
  
  // basic loop delay - resend i2c message every LOOP_DELAY ms (ignoring time for blinks)
  delay(LOOP_DELAY);
}
