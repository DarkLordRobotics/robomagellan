//======================================================================
// MagellanBot_3_teensy_sensor_debug w/Teensy 3.6
//======================================================================
#define code_version "220223_0850"

// to do:

// update libraries for BNO055 IMU

// create subset test code to debug sensors, use IMU for tilt
// where does tilt = 3' range reading on lidar?

// test libs for I2C, serial (wire.h ok?)
// sensors on I2C don't work other than IMU - wtf??
// obstacle avoidance is a wreck - test options to fix
//   Lidarlite, sonar, VL53, ??
//   add spinning lidar?
//   add & test mini-LIDAR (?)
//   LIDAR on servo (code?) to sweep ground ahead
//     collect 4 sets of readings & calc rolling average?

// Obs avoid, test VL53, PulsedLight LiDAR, add sonar?? 

// add bump sensors to LCD?  bumperHit_L, bumperHit_R
// add bump sensor down low between front tires?

// SD card for logging - test Teensy SDIO
// read IMU accelerometer to see if tilted (Lidar, VL53xx, compass) or tipped over (stop)

// IMU cal is a roll of the dice!
//   only read compass when stopped? doesn't work right now....
// add kalman filter to combine IMU & solar compass headings to produce better result?

/* -------------------------------------
// PIN MAP:
Teensy 3.6 PINS: (0-39 on top side, 40+ underneath)

0,1,31,32 SPI1: (old) PixyCam uses SPI_0 interface (11,12,13) CS on various pins...

3-4: WIRE2_PINS   I2C_PINS_3_4     (Wire 2)

6: Encoder (INT)
11,12,13 SPI0: (old) PixyCam uses SPI_0 interface (11,12,13) CS on various pins...

16: Bump (R) sensor on INT (unused)
17: Bump (L) sensor on INT (one for now...)

18-19: WIRE_PINS    I2C_PINS_18_19   (Wire 0)

21: Camera pan servo (PWM)
22: PWM for throttle
23: PWM for steering servo
24: LED pin (R) for obs R detect
25: LED pin (Y) wp arrival or deadman
26: LED pin (G) for obs l detect
27: LED pin (B) cone detect
28: PN2222_PIN enable (high) to open collector output for 555, Siren. 555 for LED lighting (not implemented yet)
      Control siren from deadman. 30 mA. 2N2222 on siren GND leg, Pos to battery direct

31-32: softSerial(31,32); // RX, TX
33: RC deadman detect input

37-38: WIRE1_PINS   I2C_PINS_37_38   (Wire 1)
// bottom of 3.6 pcb:
56-57: WIRE3_PINS   I2C_PINS_56_57  // bottom of 3.6 pcb    (Wire 3)

// -------------------------------------
I2C addresses:

0x62 LIDAR-Lite pulsedlight3d
0x52 VL53L1X I2C device address (0x29?) V53L1X (two) 0101001b = 0x29 plus r/w, so 0x52 or 0x53
//0x72 OpenLCD (using serial1 instead)
0x28 BNO055 IMU: default is 0x28. ADR: if connected to 3V, the address will be 0x29

VL53L0X sensor2(&Wire);
VL53L0X sensor3(&Wire1);

// -------------------------------------
INT Pins: (Teensy: all digital pins have INT capability)

Bump sensors on INT
Encoder INT

*/


// ---------------------------------------------------------------------------
// code starts here....
// ---------------------------------------------------------------------------

#include <SoftwareSerial.h>
SoftwareSerial mySerial(31,32); // RX, TX

int incomingByte = 0; // for incoming serial data

// To use interrupts, you must include the AVR interrupt header
#include <avr/io.h>
#include <avr/interrupt.h>


// I2C -------------------------------------
#include <Wire.h>  // Arduino Wire Library, see pins below except no wire3
//#include <i2c_t3.h>  // Teensy I2C lib (don't use this...)

// Teensy LC & 3.0-3.6 require pullup resistors to 3.3 V.
// The on-chip pullups are not used. 4.7k resistors are recommended for most applications.

// default pin setting SCL/SDA for teensy 3.5/3.6
#define WIRE_PINS    I2C_PINS_18_19
#define WIRE1_PINS   I2C_PINS_37_38
#define WIRE2_PINS   I2C_PINS_3_4
//#define WIRE3_PINS   I2C_PINS_56_57  // not supported on Arduino wire.h

// RH is this needed? how many I2C masters needed?
// Adafruit TCA9548A 1-to-8 I2C Multiplexer Breakout
//#define TCAADDR 0x70   //   tcaselect(0);
// -------------------------------------


// -------------------------------------
// OpenLCD is an LCD with Serial/I2C/SPI interfaces fr SparkFun Electronics
int lineNum, charNum;
int lineNumMap[4] = {0,64,20,84};  // to map line numbers

// -------------------------------------
// Sensors
#define BUMP_L_PIN  17  // Bump (L) sensor on INT
#define BUMP_R_PIN  16  // Bump (R) sensor on INT

int bumperHit_L = 0;  // 1 indicates hit
int bumperHit_R = 0;  // 1 indicates hit

int dAvoid0 = 13;  // distance to trigger obstacle avoidance in cm
int dAvoid1 = 30;  // distance to trigger obstacle avoidance in cm
int dAvoid2 = 60;  // distance to trigger obstacle avoidance in cm
int dAvoid3 = 90;  // distance to trigger obstacle avoidance in cm

/*
// to hold average sensor values (needed?)
int avgIndexSensor;
int avgSensorReady;  // to get past startup issue
int avgdVL53L0X;   // dVL53L1X_left dVL53L1X_right
int avgdLIDARlite;
int dVL53L0X[4];
int dLIDARlite[4] = {500, 500, 500, 500};     // placeholder value to avoid issues until LIDAR is working (fix by taking 4 readings at once? too slow?)
*/

// VL53L1X I2C device address: 0x52
//#include "SparkFun_VL53L1X_Arduino_Library.h"  // what's wrong with this one again?
#include <VL53L1X.h>
//#include <VL53L1X.h>  // Pololu library
//VL53L0X sensor2(&Wire);
//VL53L0X sensor3(&Wire1);

VL53L1X VL53L1X_left();  // 3-4: WIRE2_PINS SCL(3)/SDA(4)
VL53L1X_left.setBus(&Wire2);
int dVL53L1X_left;

VL53L1X VL53L1X_right();  // 37-38: WIRE1_PINS SCL(37)/SDA(38)
VL53L1X_right.setBus(&Wire1);
int dVL53L1X_right;

//18-19: WIRE_PINS (wire0, SCL(19), SDA(18))

// LIDAR-Lite  // http://pulsedlight3d.com
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

int dLIDARlite = 500;     // placeholder value to avoid issues until LIDAR is working


// -------------------------------------
// BNO055 IMU, I2C address 0x28

  // Board layout:
  //       |   | RST   PITCH  ROLL  HEADING
  //   ADR |   | SCL
  //   INT |   | SDA     ^            /->
  //   PS1 |   | GND     |            |
  //   PS0 |   | 3VO     Y    Z-->    \-X
  //       |   | VIN
  //
/*
BNO055 Calbration reuse?
I discovered in the data-sheet (Chapter 3.11.4) that the Operation Mode have to be set to Config Mode also when READING calibration data: 
"Reading Calibration profile
The calibration profile includes sensor offsets and sensor radius. Host system can read the
offsets and radius only after a full calibration is achieved and the operation mode is switched
to CONFIG_MODE. Refer to sensor offsets and sensor radius registers."
*/

//#include <Wire.h>  // included above
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// IMU: Set the delay between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55); // ?? this seems to work...
//Adafruit_BNO055 bno = Adafruit_BNO055();  // ??
//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28,&Wire2);

int sysCal   = 0; //sys;
int gyroCal  = 0; //gyro;
int accelCal = 0; //accel;
int magCal   = 0; //mag

//float headingRaw;
int compassHeading;
//int compassHeadingCal;  // calibrated heading from bench measured errors
//RH:
// 0:+22, 270: +15, 180: +20, 90: +25

//float declination = 0;  // use magnetic North only
float declination = 13.5;  // san jose mag Morth is 13 deg, 28 min East, of true North
int avgHeading;
int headingCurrent;
int headingError;

int headingInitial;  // reading from solar compass during setup

int newCourse;  // new course at next wp; north is 0°, east is 90°, south is 180°, and west is 270°.

// -------------------------------------
// Misc Stuff

#define shutDownPinRC 33  // for RC switch
// use with INT only: (don't use INT, PITA)
volatile int shutDownRC = 0;   // active low, 0 = stop. If throttle on RC controller not engaged (1500), bot must stop!
//int shutDownRC = 0;   // 0 = stop

// LED pins
#define LED_RED 24  // turn on if obstacle detected (OBS) right
#define LED_YEL 25  // turn on if obstacle detected (OBS) center, or deadmnan disengaged
#define LED_GRN 26  // turn on if obstacle detected (OBS) left
#define LED_BLU 27  // turn on if cone detected (cone)
#define PN2222_PIN 28  // controlled by deadman, drive high to turn on PN2222, enable 555, siren

int success = 0;   // 1 = success/done

// debug
unsigned long time;
unsigned long time_last, time_now; 

#define benchTest 0  // add delay for compass testing (what does this do? IMU testing?)

#define compassTest 1  // mode to test solar compass, another bench test mode
int loop_count = 0;  // to slow down LCD updates

#define debug 1  // use to turn on debug code if set to 1, also time readouts if set to 2


////////////////////////////////////////////////////////////////////////////////
void setup() {
////////////////////////////////////////////////////////////////////////////////

  if (debug >= 1) {
    Serial.begin(115200);  // no line ending in terminal
    delay(250);  // teensy needs time to get ready
    Serial.println();
    Serial.println("Dark Lord Robotics:"); 
    Serial.println("One Bot to Rule Them All, and in the Darkness, Bind Them..."); 
  }

  Serial1.begin(9600);  // Begin communication with Serial1 for OpenLCD
  //delay(250);
  delay(1000);
  LCD_clear();  // forces the cursor to the beginning of the display
  //delay(250);
  delay(1000);
  LCD_setCursor(0,0);
  Serial1.print("Dark Lord Robotics:"); 
  LCD_setCursor(1,0);
  Serial1.print("One Bot to Rule Them"); 
  LCD_setCursor(2,0);
  Serial1.print("All, and in the"); 
  LCD_setCursor(3,0);
  Serial1.print("Darkness Bind Them"); 
  //delay(1250);
  delay(2500);

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);

  // setup pins for various IO functions
  pinMode(PN2222_PIN, OUTPUT);     // pin 48
  digitalWrite(PN2222_PIN, HIGH);  // siren on
  pinMode(LED_RED, OUTPUT);      // pin 38
  pinMode(LED_YEL, OUTPUT);      // pin 40
  pinMode(LED_GRN, OUTPUT);      // pin 42
  pinMode(LED_BLU, OUTPUT);      // pin 42
  // LED test
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_YEL, HIGH);
  digitalWrite(LED_GRN, HIGH);
  digitalWrite(LED_BLU, HIGH);
  delay(1000);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YEL, LOW);   
  digitalWrite(LED_GRN, LOW);
  digitalWrite(LED_BLU, LOW);
  // end LED test
  digitalWrite(PN2222_PIN, LOW);   // siren off

  // show code_version, Calibrating Throttle
  LCD_clear();
  LCD_setCursor(0,0);
  Serial1.print("Blackbeard says arrg"); 
  LCD_setCursor(1,0);
  Serial1.print("version: ");
  LCD_setCursor(1,9);
  Serial1.print(code_version);
  LCD_setCursor(2,0);
  Serial1.print("Calibrating Throttle"); 
  LCD_setCursor(3,0);
  Serial1.print("shutDownPinRC: ");
  LCD_setCursor(3,14);
  Serial1.print(digitalRead(shutDownPinRC));

  if (debug >= 1) {
    Serial.println("Blackbeard says arrg!"); 
    Serial.print("version: ");
    Serial.println(code_version);
    Serial.println("Calibrating Throttle..."); 
    Serial.print("shutDownPinRC: ");
    Serial.println(digitalRead(shutDownPinRC));
    Serial.println();
  }
  
  LCD_setCursor(3,0);  // go to start of 4th line
  Serial1.print("Calibration Complete"); 
  delay(1000);

  LCD_clear();
  LCD_setCursor(0,0);
  Serial1.print("Blackbeard says arrg"); 
  LCD_setCursor(1,0);
  Serial1.print("version: ");
  LCD_setCursor(1,9);
  Serial1.print(code_version);
  LCD_setCursor(2,0);
  Serial1.print("initialize pixycam"); 

  if (debug >= 1) {
    Serial.println();
    Serial.println("initialize pixycam"); 
  }


// I2C -------------------------------------
  // -------------------------------------
  // I2C setup
  // Arduino I2C library
  Wire.begin();
  Wire.setClock(400000);   // Increase I2C bus speed to 400 kHz
  Wire1.begin();
  Wire1.setClock(400000);  // Increase I2C bus speed to 400 kHz
  Wire2.begin();
  Wire2.setClock(400000);  // Increase I2C bus speed to 400 kHz
  //Wire3.begin();           // not supported on Arduino wire.h
  //Wire3.setClock(400000);  // Increase I2C bus speed to 400 kHz

/*
  // Teensy Setup for Master mode, all buses, external pullups, 400 kHz, 10 ms default timeout
  Wire.begin(I2C_MASTER, 0x00, WIRE_PINS, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(10000); // 10ms
  Wire1.begin(I2C_MASTER, 0x00, WIRE1_PINS, I2C_PULLUP_EXT, 400000);
  Wire1.setDefaultTimeout(10000); // 10ms
  Wire2.begin(I2C_MASTER, 0x00, WIRE2_PINS, I2C_PULLUP_EXT, 400000);
  Wire2.setDefaultTimeout(10000); // 10ms
  //Wire3.begin(I2C_MASTER, 0x00, WIRE3_PINS, I2C_PULLUP_EXT, 400000);
  //Wire3.setDefaultTimeout(10000); // 10ms
*/
  // Teensy Setup for Master mode, pins 18/19, external pullups, 400 kHz, 200 ms default timeout
  //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  //Wire.setDefaultTimeout(200000); // 200ms


// I2C -------------------------------------
//RH does this work with Teensy?
//RH remove? How to use the various ports on the Teensy?
  // read LIDARLite
  //tcaselect(0);  // Adafruit TCA9548A

//RH ??  debug for lidar, fix lidar!
  readLIDARLite();
  LCD_clear();
  LCD_setCursor(3,0); // go to start of 4th line
  Serial1.print("dLIDARlite: "); 
  LCD_setCursor(3,12);
  Serial1.print(dLIDARlite);  
  delay(2000);

//RH init VL53s here, not in loop
//RH sensors need to be tested, get this working...

// I2C -------------------------------------
  // Pololu VL53L1X Initialize the left sensor

//RH remove? How to use the various ports on the Teensy?
  //tcaselect(2);  // Adafruit TCA9548A

  VL53L1X_left.setTimeout(500);
  if (!VL53L1X_left.init()) {
    Serial.println("Failed to detect and initialize VL53L1X_left!");
    while (1);
  }
  VL53L1X_left.setDistanceMode(VL53L1X::Long);
  VL53L1X_left.setMeasurementTimingBudget(50000);
  VL53L1X_left.startContinuous(50);
  VL53L1X_left.read();
  dVL53L1X_left = VL53L1X_left.ranging_data.range_mm / 10;  // convert to cm

  //LCD_clear();
  //Serial1.print("range: ");
  //Serial1.print(VL53L1X_left.ranging_data.range_mm);

  LCD_setCursor(2,0); // go to start of 3rd line
  Serial1.print("Ll "); 
  LCD_setCursor(2,4);
  Serial1.print(dVL53L1X_left);  

  delay (1000);

// I2C -------------------------------------
  // Pololu VL53L1X Initialize the right sensor

//RH remove? How to use the various ports on the Teensy?
  //tcaselect(1);  // Adafruit TCA9548A

  VL53L1X_right.setTimeout(500);
  if (!VL53L1X_right.init()) {
    Serial.println("Failed to detect and initialize VL53L1X_right!");
    while (1);
  }
  VL53L1X_right.setDistanceMode(VL53L1X::Long);
  VL53L1X_right.setMeasurementTimingBudget(50000);
  VL53L1X_right.startContinuous(50);
  VL53L1X_right.read();
  dVL53L1X_right = VL53L1X_right.ranging_data.range_mm / 10;  // convert to cm

  //LCD_clear();
  //LCD_setCursor(1,0);
  //Serial1.print("range: ");
  //Serial1.print(VL53L1X_right.ranging_data.range_mm);

  LCD_setCursor(2,7);
  Serial1.print("Lr "); 
  LCD_setCursor(2,10);
  Serial1.print(dVL53L1X_right);  
  delay (1000);
  delay (1000);


// I2C -------------------------------------
//RH remove? How to use the various ports on the Teensy?
//  tcaselect(7);


// debug RH
  if (compassTest == 0) {   // skip if testing the solar compass (to line 711)

  // BNO055 IMU initialise the sensor
  if(!bno.begin()) {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    Serial1.print("no BNO055 detected");
    while(1);
  }
  delay(100);
  // Use external crystal for better accuracy
  bno.setExtCrystalUse(true);

  // prompt for IMU calibration
  LCD_clear();
  Serial1.print("Calibrate the IMU..."); 
  LCD_setCursor(1,0);
  Serial1.print("Rotate left/right...");
  if (debug == 1) {
    Serial.println("Calibrating IMU, Rotate left/right until calibrated");
    Serial.println("Calibration status values: 0 = uncalibrated, 3 = fully calibrated");
  }
  readBNO055cal();

  while ((sysCal != 3) || (gyroCal != 3) || (accelCal != 3) || (magCal != 3)) {
  //while (magCal != 3) {
    readBNO055cal();
    //LCD_setCursor(2,0);
    //Serial1.print("Calibration = "); 
    //LCD_setCursor(2,16);
    //Serial1.print(magCal); 

    LCD_setCursor(2,0);
    Serial1.print("sysCal= "); 
    LCD_setCursor(2,8);
    Serial1.print(sysCal); 
    LCD_setCursor(2,10);
    Serial1.print("gyroCal= "); 
    LCD_setCursor(2,18);
    Serial1.print(gyroCal); 

    LCD_setCursor(3,0);
    Serial1.print("accCal= "); 
    LCD_setCursor(3,8);
    Serial1.print(accelCal); 
    LCD_setCursor(3,10);
    Serial1.print("magCal= "); 
    LCD_setCursor(3,18);
    Serial1.print(magCal); 

    delay(BNO055_SAMPLERATE_DELAY_MS);

    if (debug == 1) {
      Serial.println("Calibrating IMU, Rotate left/right until calibrated");
      Serial.println(magCal, DEC); // compass
    }
  }
  LCD_setCursor(2,0);
  Serial1.print("IMU calibrated !!   "); 
  if (debug == 1) {
    Serial.println("Calibration status: 3 = fully calibrated");
  }
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GRN, HIGH);
  digitalWrite(LED_BLU, HIGH);
  if(digitalRead(shutDownPinRC == 0)) {
    digitalWrite(LED_YEL, HIGH);
  }
  delay(4000);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GRN, LOW);
  digitalWrite(LED_BLU, LOW);


  }   // RH skip if debug of solar compass

  
  // -------------------------------------
  // Misc Stuff

  //avgIndexSensor = 0;  // for averaged sensor readings
  //avgSensorReady = 0;  // to get past startup issue

//RH - update... how to calibrate heading?
  if (mySerial.available() > 0) {
    headingInitial = mySerial.read();
    headingInitial = (headingInitial * 360) / 32;
    headingCurrent = headingInitial;
  }

  newCourse = headingCurrent;

  LCD_clear();  // forces the cursor to the beginning of the display
  updateLCD();  // debug

  if (debug >= 1) {
    Serial.print("headingCurrent: "); 
    Serial.println(headingCurrent);
  }

  //RH to read LCD and look at compass for debug...
  delay(2000);
  if (debug >= 1) {
    delay(2000);
  }


  // setup pins for bumper INT
  pinMode(BUMP_L_PIN, INPUT);  //Bump (L) sensor on INT
  pinMode(BUMP_R_PIN, INPUT);  //Bump (R) sensor on INT
  attachInterrupt(digitalPinToInterrupt(BUMP_L_PIN), bumpStop_L, FALLING);  // LOW?
  attachInterrupt(digitalPinToInterrupt(BUMP_R_PIN), bumpStop_R, FALLING);  // LOW?

  // setup pin for RC controller deadman switch INT
  pinMode(shutDownPinRC, INPUT);
  attachInterrupt(digitalPinToInterrupt(shutDownPinRC), readDeadman, FALLING);  // LOW?

  // -------------------------------------
  LCD_clear();
  Serial1.print("Blackbeard sez arrg!"); 
  LCD_setCursor(1,0);
  Serial1.print("ver: "); 
  LCD_setCursor(1,5);
  Serial1.print(code_version);
  LCD_setCursor(2,0);
  Serial1.print("Ready to set sail!!"); 
  LCD_setCursor(3,0);
  Serial1.print("Where be da wenches?"); 

  if (debug == 1) {
    Serial.println("Blackbeard says arrg!"); 
    Serial.println(code_version);
    Serial.println("Ready to set sail!!"); 
    Serial.println();
  }

  digitalWrite(LED_GRN, HIGH);

  digitalWrite(LED_GRN, LOW);
  delay(200);
  LCD_clear();  // forces the cursor to the beginning of the display
  updateLCD();  // debug
  
}  // end setup




////////////////////////////////////////////////////////////////////////////////
void loop() {
////////////////////////////////////////////////////////////////////////////////



  if (benchTest == 1) {
    //delay(500);
    Serial.println("\n");
    Serial.print("* benchTest *");
    Serial.println("\n");
  }

  if (compassTest == 1) {
    //delay(200);
    Serial.println("\n");
    Serial.print("* compassTest *");
    Serial.println("\n");
  }

  if (bumperHit_L == 1 || bumperHit_R == 1) {
    moveStop();
  }

  // startup: set waypoint distance, newCourse
  if (atStart == 1) {
    atStart        = 0;   // 1 = start of run, 0 = run already started
    newCourse      = wayPoint[wayPointIndex][1];
    updateLCD();
  }

  // read from solar compass
  if (mySerial.available() > 0) {
    headingCurrent = mySerial.read();
    headingCurrent = (headingCurrent * 360) / 32;
  }

/*
  readCompass();
  //headingCurrent = compassHeading;
  headingCurrent = avgHeading;
*/

/*
//RH??
  // read compass: average of four readings
  calcAvgCompassHeading();
  headingCurrent = avgHeading;
*/

  if (compassTest == 1) {
    if (loop_count == 2000) {
      updateLCD();  // keep this, main LCD update for the loop
      loop_count = 0;      
    }
    else loop_count++;
  }
  if (debug == 1) {
    Serial.print("loop_count: ");
    Serial.println(loop_count);
  }



//RH - fix this...
// I2C -------------------------------------

  // Obstacle detection via sensor readings
  //RH - replace all these sensors with spinning LIDAR?? (keep bump sensor, but redesign it)
  //RH - collect 4 sets of readings & calc rolling average?

  // LIDAR-Lite

// I2C -------------------------------------
  //tcaselect(0);
  //for (int i = 0; i < 4; i++) {   // to take an average? 40 ms * 4?
  //}

//RH
  readLIDARLite();

  if (debug == 1) {
    Serial.print("LIDAR-Lite: ");
    Serial.print(dLIDARlite);
    Serial.println(" cm ");
  }
  if (debug == 2) {
    Serial.print("Time LIDAR: ");
    Serial.println(millis());
  }  

// I2C -------------------------------------
  // Pololu VL53L1X left sensor

  //RH remove?
  //tcaselect(2);  // Adafruit TCA9548A

  //RH ?? do I need all this every time I want to read it?

  //VL53L1X_left.setTimeout(500);
  //if (!VL53L1X_left.init()) {
  //  Serial.println("Failed to detect and initialize VL53L1X_left!");
  //}
  //VL53L1X_left.setDistanceMode(VL53L1X::Long);
  //VL53L1X_left.setMeasurementTimingBudget(50000);
  //VL53L1X_left.startContinuous(50);

  //RH just keep this?
  VL53L1X_left.read();
  dVL53L1X_left = VL53L1X_left.ranging_data.range_mm / 10;  // convert to cm

  if (debug == 1) {
    Serial.print("VL53L1X_left: ");
    if (VL53L1X_left.timeoutOccurred()) { Serial.println(" TIMEOUT"); }
    else {
      Serial.print(dVL53L1X_left);
      Serial.println(" cm");       
    }
  }  
  if (debug == 2) {
    Serial.print("Time VL53L1X_left: ");
    Serial.println(millis());
  }


// I2C -------------------------------------
  // Pololu VL53L1X right sensor

  //RH remove?
  //tcaselect(1);  // Adafruit TCA9548A

  //RH ?? do I need all this every time I want to read it?

  //VL53L1X_right.setTimeout(500);
  //if (!VL53L1X_right.init()) {
  //  Serial.println("Failed to detect and initialize VL53L1X_right!");
  //}
  //VL53L1X_right.setDistanceMode(VL53L1X::Long);
  //VL53L1X_right.setMeasurementTimingBudget(50000);
  //VL53L1X_right.startContinuous(50);

  //RH just keep this?
  VL53L1X_right.read();
  dVL53L1X_right = VL53L1X_right.ranging_data.range_mm / 10;  // convert to cm

  if (debug == 1) {
    Serial.print("VL53L1X_right: ");
    if (VL53L1X_right.timeoutOccurred()) { Serial.println(" TIMEOUT"); }
    else {
      Serial.print(dVL53L1X_right);
      Serial.println(" cm");
    }       
  }  
  if (debug == 2) {
    Serial.print("Time VL53L1X_right: ");
    Serial.println(millis());
  }

  LCD_clear();  // forces the cursor to the beginning of the display
  updateLCD();  // debug

/*
// collect 4 sets of readings & calc rolling average  

  //RH this section needs work!
  //RH add LIDAR, dVL53L1X_left, dVL53L1X_right
  //RH how to avoid startup failure? use array with non-zero init values, or take all 4 readings at once.

  //RH move to Functions section?
  //void calcAvgSensors() {
  avgdVL53L0X    = 0;
  avgdLIDARlite  = 0;

  //RH when were these readings taken?
  for (int i = 0; i < 4; i++) {
    avgdVL53L0X   = avgdVL53L0X   + dVL53L0X[i];
    avgdLIDARlite = avgdLIDARlite + dLIDARlite[i];
  }
  avgdVL53L0X    = avgdVL53L0X    >> 2; // div by 4
  avgdLIDARlite  = avgdLIDARlite  >> 2; // div by 4

  avgIndexSensor = avgIndexSensor + 1;
  if (avgIndexSensor == 4) {
    avgIndexSensor = 0;
    avgSensorReady = 1;  // to get past startup issue
  }
  if (debug == 1) {
    Serial.print("avgdVL53L0X: ");
    Serial.println(avgdVL53L0X);
    Serial.print("avgdLIDARlite: ");
    Serial.println(avgdLIDARlite);
    Serial.println();
  }
  if (debug == 2) {
    Serial.print("TimecalcAvgSensors: ");
    Serial.println(millis());
    Serial.println();
  }
*/


  //-------------------------------------
  // Avoid any obstacles detected, take evasive action!

  // dAvoid0 = 13; dAvoid1 = 30; dAvoid2 = 60; int dAvoid3 = 90;  // distance to trigger obstacle avoidance in cm

  //if (bumperHit_L == 1 || bumperHit_R == 1 || dVL53L1X_left < dAvoid0 || dVL53L1X_right < dAvoid0 || ((dLIDARlite >= 10) && (dLIDARlite < dAvoid2))) {
  if (bumperHit_L == 1 || bumperHit_R == 1) {   // bump only

  // RH test code for average readings:
  //if (avgSensorReady == 1 && 
  //   (((dLIDARlite >= 10) && (dLIDARlite < dAvoid2))) || dVL53L1X_left < dAvoid0 || dVL53L1X_right < dAvoid0 || (bumperHit_L == 1 || bumperHit_R == 1)) {  // obstacle detected

//    moveStop();

// I2C -------------------------------------
    //if ((bumperHit_L == 1) || (dVL53L1X_left < dAvoid1)) {      // put back if I can get both V53 working
    if (bumperHit_L == 1) {
      digitalWrite(LED_GRN, HIGH);
    }
// I2C -------------------------------------
    //if ((bumperHit_R == 1) || (dVL53L1X_right < dAvoid1)) {      // put back if I can get both V53 working
    if (bumperHit_R == 1) {
      digitalWrite(LED_RED, HIGH);
    }
// I2C -------------------------------------
    if ((dLIDARlite >= 10) && (dLIDARlite < dAvoid2)) { 
      digitalWrite(LED_YEL, HIGH);
    }

    LCD_clear();
    LCD_setCursor(0,0);
    Serial1.print("Obstacle detected!"); 
    LCD_setCursor(2,0);
    Serial1.print("Ll "); 
    LCD_setCursor(2,4);
    Serial1.print(dVL53L1X_left);  
    LCD_setCursor(2,7);
    Serial1.print("Lr "); 
    LCD_setCursor(2,10);
    Serial1.print(dVL53L1X_right);  
    LCD_setCursor(2,12);
    Serial1.print("LD "); 
    LCD_setCursor(2,16);
    Serial1.print(dLIDARlite);  

    LCD_setCursor(3,0);
    Serial1.print("BMP_L ");
    LCD_setCursor(3,7);
    Serial1.print(bumperHit_L);  
    LCD_setCursor(3,10);
    Serial1.print("BMP_R ");
    LCD_setCursor(3,17);
    Serial1.print(bumperHit_R);  

    // RH debug test code: (remove later)
    delay(1500);  // to view readngs on LCD

    bumperHit_L = 0;
    bumperHit_R = 0;
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_YEL, LOW);
    digitalWrite(LED_GRN, LOW);

    //RH is this ok?
    //moveForward();  // check for new course and obstacles first
    LCD_clear();  // clear the display and go to start of 1st line
  } 
  else {
    readDeadman();  // stop until remote throttle closed
    moveForward();  // if > distanceAvoid
    bumperHit_L = 0;
    bumperHit_R = 0;
    digitalWrite(PN2222_PIN, HIGH);  // siren on
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_YEL, LOW);
    digitalWrite(LED_GRN, LOW);
  }
  // updateLCD display if needed
  // updateLCD();

  if (debug == 2) {
    Serial.print("Time loop: ");
    Serial.println(millis());
  }

}  // end void loop


////////////////////////////////////////////////////////////////////////////////
// functions
////////////////////////////////////////////////////////////////////////////////


// I2C -------------------------------------
// -------------------------------------
void tcaselect(uint8_t i) {
  //Adafruit TCA9548A 1-to-8 I2C Multiplexer Breakout
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

// I2C -------------------------------------
// -------------------------------------
void readBNO055cal() {
  // send calibration data for each sensor
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  sysCal   = sys;
  gyroCal  = gyro;
  accelCal = accel;
  magCal   = mag;
  if (debug == 1) {
    Serial.print(F("Calibration: "));
    Serial.print(sys, DEC);
    Serial.print(F(" "));
    Serial.print(gyro, DEC);
    Serial.print(F(" "));
    Serial.print(accel, DEC);
    Serial.print(F(" "));
    Serial.println(mag, DEC); // compass  
  }
}

// I2C -------------------------------------
// -------------------------------------
void readBNO055() {
  sensors_event_t event;
  bno.getEvent(&event);
  compassHeading = event.orientation.x;
  //compassHeading = event.orientation.x + declination;
  // data as roll, pitch, heading
  if (debug == 1) {
    Serial.print(F("Orientation: "));
    Serial.print((float)event.orientation.x); // compass
    Serial.print(F(" "));
    Serial.print((float)event.orientation.y);
    Serial.print(F(" "));
    Serial.print((float)event.orientation.z);
    Serial.println(F(""));
  }
}

// I2C -------------------------------------
// -------------------------------------
void A_readCompass() {
  // read IMU
  readBNO055();
  if (debug >= 1) {
    Serial.print("compassHeading= ");
    Serial.println(compassHeading);
    Serial.println();
  }
  if (debug == 2) {
    Serial.println("\n");
    Serial.print("Time readCompass: ");
    Serial.println(millis());
  }
}

// -------------------------------------
void readCompass() {
//void calcAvgCompassHeading() {
  // collect multiple compass readings & calc average heading

/*
//RH??  solar compass?
  if (mySerial.available() > 0) {
    headingCurrent = mySerial.read();
    headingCurrent = (headingCurrent * 360) / 32;
  }
*/
  avgHeading = 0;
  for (int i = 0; i < 16; i++) {
    A_readCompass();  // read BNO055 IMU
    avgHeading = avgHeading + compassHeading;
  }
  avgHeading = avgHeading >> 4; // div by 16
  //avgHeading = avgHeading >> 2; // div by 4

  if (debug >= 1) {
    Serial.print("avgCompassHeading: ");
    Serial.println(avgHeading);
    Serial.println();
  }
  if (debug == 2) {
    Serial.println("\n");
    Serial.print("Time calcAvgCompassHeading: ");
    Serial.println(millis());
  }
}

// -------------------------------------
void calcHeadingError() {
  // to compensate for courses near 360/0 deg, calc relative course error:
  // negative error = turn left, positive error = turn right

  headingError = newCourse - headingCurrent;
  // correct for nearest compass course
  if (headingError > 180) {
    headingError = headingError - 360;
  }
  else if (headingError < -180) {
    headingError = headingError + 360;
  }
  if (debug == 1) {
  //  Serial.println(" ");
    Serial.print("newCourse: ");
    Serial.println(newCourse);
    Serial.print("headingCurrent: ");
    Serial.println(headingCurrent);
    Serial.print("headingError: ");
    Serial.println(headingError);
  }
  if (debug == 2) {
    Serial.print("Time calcHeadingError: ");
    Serial.println(millis());
  }
}

// -------------------------------------
void steeringCorrection() {
  // correct steering to new course, adjustSteering = 300
  // valueSteering;  // 1200 to 1800, right to left, 1500 centered
  // headingError = newCourse - headingCurrent;

  if (headingError < 0) {      // turn left, increase valueSteering
    if (headingError <= -19) {  // turn left full (300/16=18.75)
    //if (headingError < -30) {   // turn left
      valueSteering = center + adjustSteering;
    }
    else {  // correct proportionally 
      valueSteering = center + adjustSteering/3;
      //valueSteering = center - (headingError);  // subtract since error is also negative
      //valueSteering = center + (abs(headingError) << 2);    // mult by 8  // shift destroys sign bit
      //valueSteering = center + (abs(headingError) << 4);    // mult by 16  // shift destroys sign bit
      //valueSteering = center - (10 * headingError);  // subtract since error is also negative
      //valueSteering = center - (adjustSteering * headingError)/30;  // subtract since error is also negative
    }
  }
  else if (headingError > 0) {  // turn right, decrease valueSteering
    if (headingError >= 19) {   // turn right full (300/16=18.75)
    //if (headingError > 30) {    // turn right
      valueSteering = center - adjustSteering;
    }
    else {  // correct proportionally 
      valueSteering = center - adjustSteering/3;
      //valueSteering = center - (headingError);
      //valueSteering = center - (abs(headingError)  << 2);  // mult by 8  // shift destroys sign bit
      //valueSteering = center - (abs(headingError)  << 4);  // mult by 16  // shift destroys sign bit
      //valueSteering = center - (10 * headingError);
      //valueSteering = center - (adjustSteering * headingError)/30;
    }
  }
  else {  // on course!
    valueSteering = center;
  }
  servoSteering.writeMicroseconds(valueSteering);

  if (debug == 1) {
    //Serial.println();
    Serial.print("headingError: ");
    Serial.println(headingError);
    Serial.print("valueSteering: ");
    Serial.println(valueSteering);
  }
  if (debug == 2) {
    Serial.print("Time steeringCorrection: ");
    Serial.println(millis());
  }
}

// -------------------------------------
void turnToNewCourse() {
  // manual turn based on timer

// do I need this?

//RH not working!! delay way too long??
  
  int courseChange = newCourse - wayPoint[wayPointIndex - 1][1];   // new - current
  // correct for nearest compass heading
  if (courseChange > 180) {
    courseChange = courseChange - 360;
  }
  else if (courseChange < -180) {
    courseChange = courseChange + 360;
  }

  //RH
  //int turnDelay = ((courseChange * 10)/(turnRate)) * 100; // ms = deg * ms/deg (or div by deg/ms) // what is turnRate?
  int turnDelay = 2000;

  if (headingError < -3) {  // turn left, increase valueSteering
    valueSteering = center + adjustSteering;
    }
  else if (headingError > 3) {  // turn right, decrease valueSteering
    valueSteering = center - adjustSteering;
    }
  else {  // on course!
    valueSteering = center;
    turnDelay = 0;
  }
  servoSteering.writeMicroseconds(valueSteering);
  moveForward();
  delay(turnDelay);
  moveStop();
  valueSteering = center;
  servoSteering.writeMicroseconds(valueSteering);

  if (debug == 1) {
    Serial.print("newCourse: ");
    Serial.println(newCourse);
    Serial.print("headingCurrent: ");
    Serial.println(wayPoint[wayPointIndex - 1][1]);
    Serial.print("courseChange: ");
    Serial.println(courseChange);
    Serial.print("valueSteering: ");
    Serial.println(valueSteering);
    Serial.print("turnDelay: ");
    Serial.println(turnDelay);
  }
}


// -------------------------------------
// bumper INT service routines

void bumpStop_L() {
  //valueThrottle = stopped;
  valueThrottle = rev;  // brake
  servoThrottle.writeMicroseconds(valueThrottle);  // immediate stop!
  bumperHit_L = 1;
}

void bumpStop_R() {
  //valueThrottle = stopped;
  valueThrottle = rev;  // brake
  servoThrottle.writeMicroseconds(valueThrottle);  // immediate stop!
  bumperHit_R = 1;
}


// -------------------------------------
void readDeadman() {  // RC deadman INT service routine
  // read RC throttle detector
  // stop when RC throttle released

  shutDownRC = digitalRead(shutDownPinRC);

  while (digitalRead(shutDownPinRC) == 0 || success == 1) {
    moveStop();
    digitalWrite(PN2222_PIN, LOW);  // siren off
    digitalWrite(LED_YEL, HIGH);
    readCompass();
    updateLCD();
    if (debug == 1) {
      Serial.println("readDeadman()");
      Serial.print("shutDownRC: ");
      Serial.println(digitalRead(shutDownPinRC));
      Serial.print("success: ");
      Serial.println(success);
      Serial.println(millis());
      Serial.println();
    }
    delay(500);  // for LCD display
  }
  digitalWrite(PN2222_PIN, HIGH);  // siren on
  digitalWrite(LED_YEL, LOW);
  if (debug == 1) {
    Serial.println("readDeadman()");
    Serial.print("shutDownRC: ");
    Serial.println(shutDownRC);
    Serial.print("success: ");
    Serial.println(success);
    Serial.println();
  }
}



// I2C -------------------------------------
// -------------------------------------
void readLIDARLite() {
  // using Arduino wire library
  // currently unused...

// I2C -------------------------------------
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)  
  Wire.write((int)MeasureValue); // sets register pointer to  (0x00)  
  Wire.endTransmission(); // stop transmitting
  delay(20); // Wait 20 ms for transmit

  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting
  delay(20); // Wait 20 ms for transmit

  Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite

  if (2 <= Wire.available()) {    // if two bytes were received...
    dLIDARlite = Wire.read();     // receive high byte (overwrites previous reading)
    dLIDARlite = dLIDARlite << 8; // shift high byte to be high 8 bits
    dLIDARlite |= Wire.read();    // receive low byte as lower 8 bits
  }
}

// -------------------------------------
void flashLEDs() {
  // for wp arrival, etc.
  for (int i = 0; i < 4; i++) {
    digitalWrite(PN2222_PIN, LOW);  // shut off siren
    digitalWrite(LED_RED, HIGH);   // OBS 
    digitalWrite(LED_YEL, HIGH);   // wp
    digitalWrite(LED_GRN, HIGH);   // cone
    LCD_clear();
    Serial1.print("wayPoint arrival!   ");
  LCD_setCursor(1,10);
  Serial1.print("HD");
  LCD_setCursor(1,13);
  Serial1.print(headingCurrent);
    delay(500);
    digitalWrite(PN2222_PIN, HIGH);  // siren on
    digitalWrite(LED_RED, LOW);   // OBS 
    digitalWrite(LED_YEL, LOW);   // wp
    digitalWrite(LED_GRN, LOW);   // cone
    LCD_clear();
  LCD_setCursor(1,10);
  Serial1.print("HD");
  LCD_setCursor(1,13);
  Serial1.print(headingCurrent);
    delay(500);
  }
}

// -------------------------------------
void LCD_clear() {
  // Send reset command - this forces the cursor to the beginning of the display
  //  LCD_clear();
  Serial1.write('|');  // Send setting character
  Serial1.write('-');  // Send clear display character
}

void LCD_setCursor(int lineNum, int charNum) {
  // Jump cursor to line, position
  //  LCD_setCursor(3,19);
  Serial1.write(254); // Send command character
  Serial1.write(128 + lineNumMap[lineNum] + charNum);  // 0, 64, 20, 84, 0-19
}

// -------------------------------------
void updateLCD() {
//RH??

  //RH replace with daylight readable display? OpenLCD ok? add sun shade? fixed?
  //RH update for OpenLCD Serial1

  // waypoint index & new course, distance
  // display distance traveled, target
  // display curr course, target course
  // display cone?
  // display obstacle range?

  LCD_clear();  // clear the display and go to start of 1st line

  // line 1:
  // display waypoint index, target distance, distance traveled
  LCD_setCursor(0,0); // go to start of 1st line
  Serial1.print("WP:");
  LCD_setCursor(0,4);
  Serial1.print(wayPointIndex);
  LCD_setCursor(0,6);     
  Serial1.print("D:"); 
  LCD_setCursor(0,8);     
  Serial1.print(targetDistance); 
  LCD_setCursor(0,13);     
  Serial1.print("ENC:"); 
  LCD_setCursor(0,17);
  Serial1.print(encoderCount); 

  // line 2:
  // display new target course, current course, calibrated compass heading, raw compass heading
  LCD_setCursor(1,0); // go to start of 2nd line
  Serial1.print("newC:"); 
  LCD_setCursor(1,6);
  Serial1.print(newCourse); // new target course   '02580' ??  RH
  LCD_setCursor(1,10);
  Serial1.print("HD");
  LCD_setCursor(1,13);
  Serial1.print(headingCurrent);
  //LCD_setCursor(1,11);
  //Serial1.print("c"); 
  //LCD_setCursor(1,12);
  //Serial1.print(compassHeadingCal); 
  //LCD_setCursor(1,16);
  //Serial1.print("r");
  //LCD_setCursor(1,17);
  //Serial1.print(int(headingRaw));

  // line 3:
  // display obstacle distance sensor readings 
  LCD_setCursor(2,0); // go to start of 3rd line
  Serial1.print("Ll "); 
  LCD_setCursor(2,3);   // 4?? RH
  Serial1.print(dVL53L1X_left);  
  LCD_setCursor(2,7);
  Serial1.print("Lr "); 
  LCD_setCursor(2,10);
  Serial1.print(dVL53L1X_right);  
  //RH does it fit? yes...
  LCD_setCursor(2,13);
  Serial1.print("LD "); 
  LCD_setCursor(2,16);
  Serial1.print(dLIDARlite);  

/*
  Serial1.print("S"); 
  LCD_setCursor(2,1);
  Serial1.print(int(dSonarPing1));  
  LCD_setCursor(2,5);
  Serial1.print("S"); 
  LCD_setCursor(2,6);
  Serial1.print(int(dSonarPing2));  
  LCD_setCursor(2,10);
  Serial1.print("I"); 
  LCD_setCursor(2,11);
  Serial1.print(int(dIrA710));  
  LCD_setCursor(2,15);
  Serial1.print("I"); 
  LCD_setCursor(2,16);
  Serial1.print(int(dIrA02Y));  
*/

  // line 4:
  // display LIDAR obstacle range, Cone: atCone/detected
  //RH ?? add bump sensors?  bumperHit_L, bumperHit_R            // RH ??
  LCD_setCursor(3,0); // go to start of 4th line
  Serial1.print("TH "); 
  LCD_setCursor(3,3);
  Serial1.print(valueThrottle);  
  LCD_setCursor(3,7);
  Serial1.print(" Vel ");
  LCD_setCursor(3,12);
  Serial1.print(velocity);  
  //LCD_setCursor(3,16);
  //Serial1.print("S ");
  //LCD_setCursor(3,18);
  //Serial1.print(valueSteering);  

  //Serial1.print("LD "); 
  //LCD_setCursor(3,4);
  //Serial1.print(dLIDARlite);  

  //Serial1.print("L");
  //LCD_setCursor(3,1);
  ////Serial1.print(dVL53L0X);
  //LCD_setCursor(3,5);
  //Serial1.print("L");
  //LCD_setCursor(3,6);
  //Serial1.print(dLIDARlite);
  //LCD_setCursor(3,8);
  //Serial1.print("Cdet ");
  //LCD_setCursor(3,13);
  //Serial1.print(coneDetected);
  //LCD_setCursor(3,15);
  //Serial1.print("@C ");
  //LCD_setCursor(3,18);
  //Serial1.print(atCone);

}  // updateLCD();


//RH fix LCD_setCursor values below (swap):

//debug
/*
  Serial1.print("PING1 "); 
  Serial1.print(dSonarPing1);  
  LCD_setCursor(10,0);
  Serial1.print("PING2 "); 
  Serial1.print(dSonarPing2);  
  LCD_setCursor(0,1);
  Serial1.print("A710 "); 
  Serial1.print(dIrA710);  
  LCD_setCursor(10,1);
  Serial1.print("A02Y "); 
  Serial1.print(dIrA02Y);  
  LCD_setCursor(0,2);
  Serial1.print("L0X  "); 
  Serial1.print(dVL53L0X);  
  LCD_setCursor(0,3);
  dLIDARlite

  Serial1.print("cone:");
  LCD_setCursor(5,3);
  Serial1.print(coneDetected);
  LCD_setCursor(10,3);
  Serial1.print("OBS:");
  LCD_setCursor(15,3);
  Serial1.print(dVL53L0X);  // dLIDARlite
  //Serial1.print(dLIDARlite);  // dVL53L0X
*/

//RH fix LCD_setCursor values below (swap):
/*
  // line 1:
  // display target, wp #?
  Serial1.clear();  // clear the display and home the cursor

  Serial1.print("WP:");
  LCD_setCursor(4,0);       // go to the 1st line, 4th character
  Serial1.print(wayPointIndex);
  LCD_setCursor(6,0);     
  Serial1.print("D:"); 
  LCD_setCursor(8,0);     
  Serial1.print(targetDistance); 
  LCD_setCursor(13,0);     
  Serial1.print("ENC:"); 
  LCD_setCursor(17,0);
  Serial1.print(encoderCount); 
*/
/*
  LCD_setCursor(7,0);     
  Serial1.print("target:"); 
  LCD_setCursor(15,0);     
  Serial1.print(targetDistance); 
*/
/*
  // line 2:
  // display distance traveled, current course, target course
  LCD_setCursor(0,1); // go to start of 2nd line
  Serial1.print("encoderCount"); 
  LCD_setCursor(15,1);     
  Serial1.print(encoderCount); 
*/
/*
  // line 3:
  // waypoint index & new course, or other messages, speed, obs distance sensor/reading, etc.
  LCD_setCursor(0,2); // go to start of 3rd line

  Serial1.print("newHD:"); 
  LCD_setCursor(6,2);  // go to the 3rd line, 6th character
  Serial1.print(newCourse); 
  LCD_setCursor(10,2);  // go to the 3rd line, 10th character
  Serial1.print("curHD:");
  LCD_setCursor(16,2);  // go to the 3rd line, 16th character
  Serial1.print(headingCurrent);

  // line 4:
  // display cone?
  // display obstacle range?
  LCD_setCursor(0,3); // go to start of 4th line
//debug
  Serial1.print("calHD:"); 
  LCD_setCursor(6,3);  // go to the 3rd line, 6th character
  Serial1.print(compassHeadingCal); 
  LCD_setCursor(10,3);  // go to the 3rd line, 10th character
  Serial1.print("rawHD:");
  LCD_setCursor(16,3);  // go to the 3rd line, 16th character
  Serial1.print(int(headingRaw));
*/
/*
  Serial1.print("PING1 "); 
  Serial1.print(dSonarPing1);  
  LCD_setCursor(10,0);       // go to 10th char, 1st line
  Serial1.print("PING2 "); 
  Serial1.print(dSonarPing2);  
  LCD_setCursor(0,1);        // go to the 2nd line
  Serial1.print("A710 "); 
  Serial1.print(dIrA710);  
  LCD_setCursor(10,1);       // go to the 10th char, 2nd line
  Serial1.print("A02Y "); 
  Serial1.print(dIrA02Y);  
  LCD_setCursor(0,2);        // go to the 3rd line
  Serial1.print("L0X  "); 
  Serial1.print(dVL53L0X);  
  LCD_setCursor(0,3);        // go to the 4th line
  dLIDARlite

  Serial1.print("cone:");
  LCD_setCursor(5,3);  // go to the 3rd line, 6th character
  Serial1.print(coneDetected);
  LCD_setCursor(10,3);  // go to the 4th line, 10th character
  Serial1.print("OBS:");
  LCD_setCursor(15,3);
  Serial1.print(dVL53L0X);  // dLIDARlite
  //Serial1.print(dLIDARlite);  // dVL53L0X
*/
