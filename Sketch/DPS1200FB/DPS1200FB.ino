/*
 * ATTINY85 Wiring
 *       ----------
 * /RST--|1      8|--  VCC
 * A3  --|2      7|--  SCL
 * A2  --|3      6|--  1 - LED
 * GND --|4      5|--  SDA
 *       ----------
 * 
 * AVRDude
 * ATTINY85
 * avrdude -c usbtiny -P usb -p attiny85
 * avrdude -c usbtiny -P usb -p attiny85 -e -U lfuse:w:0xE2:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m -U lock:w:0xFF:m
 * avrdude -c usbtiny -P usb -p attiny85 -e -U flash:w:firmware.hex:a
 * 
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
 * 3 - A2 - External pullup Jump to gnd to disable A3
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

//#define ATTINY85
#define ATMEGA

#ifdef ATTINY85
  // Include TinyWireM Lib for ATTINY I2C over USI
  #include <TinyWireM.h>
  #include <USI_TWI_Master.h>
  
  /* Analog Control - adjust PSU RPM Fan via Analog input - uses map function */
  #define AIN_PIN   A3
  #define LED_PIN   1
  #define ACTRL_PIN A2
  // Pin 5 is SDA
  // Pin 7 is SCL
  #define MYWIRE TinyWireM
#endif

#ifdef ATMEGA
  // Include Wire Lib for ATMEGA I2C
  #include <Wire.h>
  
  /* Analog Control - adjust PSU RPM Fan via Analog input - uses map function */
  #define AIN_PIN   A3
  #define LED_PIN   13
  #define ACTRL_PIN A2

  // Pin 2 is SDA
  // Pin 3 is SDL
  #define MYWIRE Wire
  #define SERIALEN
#endif

/* Map values for analog control */
#define MAP_INIT_LOW 0 
#define MAP_INIT_HIGH 1023

/* Common Delays */
#define LOOP_DELAY 2000
#define BLINK_TIME 100

// RPM vals
#define RPM_MODE_DYNAMIC  LOW  // Jumper installed, pulled low
#define RPM_MODE_ANALOG_IN HIGH // Jumper removed, pulled high
uint8_t rpm_mode = RPM_MODE_DYNAMIC;

//#define TMP_MIN 40
float TMP_MIN = 20;
#define TMP_MAX 90
#define TMP_MIN_SETPOINT_LOW 0
#define TMP_MIN_SETPOINT_HIGH 40

#define RPM_MIN 4000  // 3600
#define RPM_MAX 14000 //18000

 // alias for heartbeat
#define HEARTBEAT blinkCode(1)

// Enable / Disable defines
#define ENBLINKCODES // LED Blink codes
#define ENTMPSETPOINT
#define ENI2C        // I2C
#define ENADD_STATS  // Additional Statistics

/* Address of PIC MCU in PSU (0x57 -> EEPROM, 0x5F -> PIC MCU) - DOESN'T ACCOUNT FOR ADDRESS SELECT PINS */
#define I2CADDR 0x5F 

// I2C Registers
#define REG_RPM_WRITE (0x40)
#define REG_TMP_INTAKE_READ (0x0d<<1)   // SCALE 32
#define REG_TMP_INTERNAL_READ (0x0e<<1) // SCALE 32
#define REG_RPM_READ (0x0f<<1)          // SCALE 1

#define REG_VOLT_IN (0x04<<1)   // Scale 32
#define REG_AMP_IN (0x05<<1)    // Scale 128
#define REG_WATT_IN (0x06<<1)   // Scale 2
#define REG_VOLT_OUT (0x07<<1)  // Scale 254.5
#define REG_AMP_OUT (0x08<<1)  // Scale 128
#define REG_WATT_OUT (0x09<<1) // Scale 2

uint16_t ain_value = 0;  // value of pot on A3
uint16_t rpm_value = 0;
uint16_t rpm_read = 0;

#define ADJUST_TMP_F 18 // TODO best way to calibrate?
float intake_tmp_c = 0.0;
float internal_tmp_c = 0.0;

#ifdef ENADD_STATS
// additional vals to read
float volt_in = 0.0;
float amp_in = 0.0;
float watt_in = 0.0;
float volt_out = 0.0;
float amp_out = 0.0;
float watt_out = 0.0;
#endif

void blinkCode(uint8_t ct){
  #ifdef ENBLINKCODES
  for(uint8_t i = 0; i<ct; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(BLINK_TIME);
    digitalWrite(LED_PIN, LOW);
    delay(BLINK_TIME);
  }
  // end blink timeout
  #endif
  delay(BLINK_TIME*5); // Still want delay if blink codes disabled
}

void setup() {
  pinMode(LED_PIN, OUTPUT);    // LED Pin as output for blink codes
  digitalWrite(LED_PIN, LOW);  // Initial state of LED is off
  pinMode(AIN_PIN, INPUT);     // Analog input pin for potentiometer 

  // RPM Mode selection.  Use ACTRL_PIN to determine what mode - Static RPM (Jumper installed, LOW) - Variable POT - (Jumper removed, HIGH) + POT
  pinMode(ACTRL_PIN, INPUT);
  delay(500);
  // read control state - only allow control state change on reboot
  //rpm_mode = digitalRead(ACTRL_PIN);

  // Start I2C
  #ifdef ENI2C
  MYWIRE.begin();
  #endif

  #ifdef SERIALEN
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Starting prog");
  #endif
}

void writePSU(uint8_t reg, uint16_t val) {
  uint8_t  valLSB = 0;
  uint8_t  valMSB = 0;
  uint16_t cs = 0;
  uint8_t  regCS = 0;
  uint8_t  data[4] = {0,0,0,0};

  // Most of the following is taken from the dump work from this github repo
  // https://github.com/raplin/DPS-1200FB/blob/master/DPS-1200FB.py

  // Extract MSB and LSB from RPM
  valLSB=val&0xff;
  valMSB=val>>8;
  // calculate checksum - the checksum is the 'secret sauce'
  cs=(I2CADDR<<1)+reg+valLSB+valMSB;
  regCS=((0xff-cs)+1)&0xff;
 
  // pack the data
  data[0] = reg;
  data[1] = valLSB;
  data[2] = valMSB;
  data[3] = regCS;

  // write to psu
  //blinkCode(2);
  // I2C Transaction
  #ifdef ENI2C
  MYWIRE.beginTransmission(I2CADDR);
  MYWIRE.write(data,4);
  MYWIRE.endTransmission();
  #endif
  //blinkCode(3);
}

uint16_t readPSU(uint16_t reg) {
  uint16_t val = 0;

  uint16_t cs = 0;
  uint8_t  regCS = 0;
  uint8_t  data[3] = {0,0,0};
  
  cs=reg+(I2CADDR<<1);
  regCS=((0xff-cs)+1)&0xff;  //#this is the 'secret sauce' - if you don't add the checksum byte when reading a register the PSU will play dumb
  data[0] = reg;
  data[1] = regCS;
  
  #ifdef ENI2C
  // Send read request
  MYWIRE.beginTransmission(I2CADDR);
  MYWIRE.write(data,2);
  MYWIRE.endTransmission();

  // Read data  
  MYWIRE.requestFrom(I2CADDR, 3);
  while (!MYWIRE.available()); // wait for data
  data[0] = MYWIRE.read(); // LSB
  data[1] = MYWIRE.read(); // MSB
  data[2] = MYWIRE.read(); // Checksum -> ignoring
  #endif

  return ((uint16_t)data[1] << 8) | (uint16_t)data[0];
}

float f2c(uint16_t temp) {
  return (temp- 32) *.5556;
}

void loop() {
  // inherit loop delay via heartbeat
  HEARTBEAT; // heartbeat

  // read info from psu
  intake_tmp_c = f2c((readPSU(REG_TMP_INTAKE_READ) / 32) + ADJUST_TMP_F);
  internal_tmp_c = f2c((readPSU(REG_TMP_INTERNAL_READ) / 32) + ADJUST_TMP_F);
  rpm_read = readPSU(REG_RPM_READ);


  #ifdef ENADD_STATS
  // TODO read additonal fields.. voltage, amps, etc
  volt_in = readPSU(REG_VOLT_IN ) / 32;
  amp_in = readPSU(REG_AMP_IN) / 128;
  watt_in = readPSU(REG_WATT_IN) / 2;

  volt_out = readPSU(REG_VOLT_OUT ) / 254.5;
  amp_out = readPSU(REG_AMP_OUT) / 128;
  watt_out = readPSU(REG_WATT_OUT) / 2;
  #endif


  ain_value = analogRead(AIN_PIN);
  if ( rpm_mode == RPM_MODE_ANALOG_IN ) {
    rpm_value = map(ain_value, MAP_INIT_LOW, MAP_INIT_HIGH, RPM_MIN, RPM_MAX);
  }
  else if (rpm_mode == RPM_MODE_DYNAMIC) {
    // TODO add ability to use AIN_PIN as offset for Dynamic RPM Mode
    // maybe tmp min to keep fan off, but once it reaches the setpoint, it will throttle up faster
    #ifdef ENTMPSETPOINT
      TMP_MIN = map(ain_value, MAP_INIT_LOW, MAP_INIT_HIGH, TMP_MIN_SETPOINT_LOW, TMP_MIN_SETPOINT_HIGH);
    #endif
    
    if ( internal_tmp_c > TMP_MAX) {
      // handle above tmp max
      rpm_value = RPM_MAX;
    }
    else if (internal_tmp_c < TMP_MIN) {
      // handle below tmp min
      rpm_value = RPM_MIN;
    }
    else
    {
      // dynamically control rpm based on temp scale
      rpm_value = map(internal_tmp_c, TMP_MIN, TMP_MAX, RPM_MIN, RPM_MAX);
    }
  }

  // write rpm to PSU
  writePSU(REG_RPM_WRITE, rpm_value);
  
  
  #ifdef SERIALEN
    Serial.print("ETMP: ");
    Serial.print(intake_tmp_c);
    Serial.print(", ITMP: ");
    Serial.print(internal_tmp_c);
    Serial.print(", RPM: ");
    Serial.print(rpm_read);
    Serial.print(", AVG: ");
    Serial.print((intake_tmp_c+internal_tmp_c)/2);
    Serial.print(", PROP: ");
    Serial.println(rpm_value);

    #ifdef ENADD_STATS
      Serial.print("  V_IN: ");
      Serial.print(volt_in);
      Serial.print(", A_IN: ");
      Serial.print(amp_in);
      Serial.print(", W_IN: ");
      Serial.println(watt_in);
      Serial.print("  V_OUT: ");
      Serial.print(volt_out);
      Serial.print(", A_OUT: ");
      Serial.print(amp_out);
      Serial.print(", W_OUT: ");
      Serial.println(watt_out);
    #endif
  #endif

  delay(LOOP_DELAY);
}
