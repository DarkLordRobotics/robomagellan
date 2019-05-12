////////////////////////////////////////////////////////////////////////////////
// MagellanBot (Blackbeard) on NitroRCX MadBeast 8 chassis using Teensy 3.6
////////////////////////////////////////////////////////////////////////////////
#define code_version "190512_0900"

// obs avoid is a wreck, doesn't back up.
// sensors on I2C don't work other than IMU
// IMU cal is a roll of the dice, GPS?? solar compass?
// dead reckoning is no good on grass
// fix front wheel alignment
// pixy is a bit flaky, unreliable
// SD card for logging? Teensy
// solar compass?
// only read compass when stopped? doesn't work rigth now....
// bumpers: add more or flip around to cover gap, add zip ties or sticks?


// see gpsCourseCalc.ino for path planning help

/*
notes on RoboGames '18:

obs det sucked:
add spinning lidar?
add & test mini-LIDAR (?)
LIDAR on servo (code?) to sweep ground ahead
  collect 4 sets of readings & calc rolling average?

code needs a very careful review, esp for pixycam and cone detection
_add wp based code in pixycam function for > 1 cone

add code from floorbot for 'lane detect' to move between obstacles?

waterproofing? jar for pixy? acrylic cover? etc?
*/

/* -------------------------------------
FLOWCHART:

Init encoderCount, compass, etc.
**Save current course at start?? doesn't work?

Read first wp for distance goal, heading.
Move forward (distance), avoid obstacles.
at distance, if no cone, read next wp for new distance, turn to new course, fwd
repeat

if cone at wp, at distance minus pixyCamThreshold:
Look for cone: pan camera, track angle, adjust course to match if cone detected.
When close, slow down, touch.
**backup, turn, move fwd to avoid hitting cone

Read next wp, determine new distance, course, fwd
repeat until all targets done
*/

/* -------------------------------------
// NAVIGATION: Path Planning based on course layout

// from SRS: 
Bot may not move or knock over cones! tip is ok.
30 minutes to prep after course revealed.
ok to walk the course, use handheld GPS, iPhone, etc.

// Compass info:
San Jose CA
*Magnetic declination: +13° 28' (EAST)
SEATTLE WA
Magnetic declination: +15° 56' (EAST)
*/

/* -------------------------------------
// PIN MAP:
// -------------------------------------
Teensy PINS: (0 - 39)

6: Encoder (INT)

17: Bump (L) sensor on INT
16: Bump (R) sensor on INT

21: pixyCam pan servo (PWM)
22: PWM for throttle
23: PWM for steering servo

24: LED pin (R) for obs R detect
25: LED pin (Y) wp arrival or deadman
26: LED pin (G) for obs l detect
27: LED pin (B) cone detect

28: PN2222_PIN enable (high) to open collector output for 555, Siren
    control siren from deadman. current? 30 mA. 2N2222 on GND leg, Pos to battery direct
33: RC deadman detect input

SPI: PixyCam uses SPI_0 interface (11,12,13)

I2C addresses:

0x62 LIDAR
0x52 VL53L1X I2C device address (0x29?) V53L1X (two)
//0x72 OpenLCD (using serial1 instead)
0x28 BNO055 default is 0x28. ADR: if connected to 3V, the address will be 0x29

INT Pins: (all digital pins have INT capability)

Bump sensors on INT
Encoder INT

// old pins from Arduino:


*/


// ---------------------------------------------------------------------------
// code starts here....
// ---------------------------------------------------------------------------

//To use interrupts, you must include the AVR interrupt header
#include <avr/io.h>
#include <avr/interrupt.h>

#include <Wire.h>  // Arduino Wire Library
//#include <i2c_t3.h>  // Teensy I2C lib at https://github.com/nox771/i2c_t3
//default pin setting SCL/SDA:
//- Wire: 19/18
//- Wire1: 37/38 (3.5/3.6)
//- Wire2: 3/4 (3.5/3.6)
//- Wire3: 57/56 (3.6)

//Adafruit TCA9548A 1-to-8 I2C Multiplexer Breakout
#define TCAADDR 0x70   //   tcaselect(0);

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
int avgIndexSensor;
int avgSensorReady;  // to get past startup issue
int avgdVL53L0X;   // dVL53L1X_left dVL53L1X_right
int avgdLIDARlite;
int dVL53L0X[4];
int dLIDARlite[4] = {500, 500, 500, 500};     // placeholder value to avoid issues until LIDAR is working
*/

// VL53L1X I2C device address: 0x52
//#include "SparkFun_VL53L1X_Arduino_Library.h"
#include <VL53L1X.h>  // Pololu library

VL53L1X VL53L1X_left;
int dVL53L1X_left;

VL53L1X VL53L1X_right;
int dVL53L1X_right;

// LIDAR-Lite  // http://pulsedlight3d.com
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

int dLIDARlite = 500;     // placeholder value to avoid issues until LIDAR is working

// -------------------------------------
// IR line sensor for encoder
#define encoderPinA 6
volatile int encoderPosA = 0;
//volatile unsigned int encoderPos = 0; // removes sign bit, allows up to 64k, rolls over
int encoderCount;  // long?

// encoder calibration
// Boris/Blackbeard 2:
// 10 enc ticks = 3 wheel revs on Boris MadBeast 8, 6.75" tires, 63.6" total distance
//#define encoderTicksPerFoot 1.887  // RH calibration constant for later use
#define encoderTicksPerStep 3.97  // # ticks for one of Ralph's steps (16"), i.e. one leg forward, not a full pace

// 30': 57 (1.9), 206 (1.966), 263 (1.9) (1x,3x,4x)
// 30'x4: 235 (1.958)
// 25' grass rough: 4x = 192 (1.92)
// 25' grass plain: 4x = 192 (1.92)

// for MaxStone 10 from NitroRCX (aka Cujo)
//#define encoderTicksPerFoot 25  // RH calibration constant for later use
//#define encoderTicksPerStep 13.2  // 13.2 ticks for one of Ralph's steps, i.e. one leg forward, not a full pace

// -------------------------------------
// BNO055 IMU, 0x28
  // Board layout:
  //       |   | RST   PITCH  ROLL  HEADING
  //   ADR |   | SCL
  //   INT |   | SDA     ^            /->
  //   PS1 |   | GND     |            |
  //   PS0 |   | 3VO     Y    Z-->    \-X
  //       |   | VIN
  //

//#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Set the delay between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55); // ??
//Adafruit_BNO055 bno = Adafruit_BNO055();  // ??

int magCal = 0;
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


// -------------------------------------
// pixycam
#include <SPI.h>  
#include <Pixy.h>
#include <Servo.h> 

Pixy pixy;

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)       
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

Servo servoPanPixy;
// servoPanPixy.attach(21);  

// PanPixy servo:
int panleft   = 2000;
int pancenter = 1575; // mounting offset on Blackbeard
int panright  = 1000;
int valuePanPixy;  // 1000 to 2000, right to left, 1500 centered
int panError;
int panFactor = 150;  // how much pixy pan error to use

int coneDetected;  // 1 means cone detected

// -------------------------------------
// Throttle & Steeering Servo settings
//#include <Servo.h> 

Servo servoThrottle;
Servo servoSteering;

// Madbeast throttle servo:
int valueThrottle;  // 1000 to 2000, full rev to full fwd, 1500 stopped
int newThrottle;    // to update throttle setting
//int ThrottleStepRate = 5;  // one sec total delay

int fwd     = 1610; // 1600? // 1550? //2000;
int creep   = 1575;
int stopped = 1500;
int rev     = 1200; // 1350?  //1300;
// note: for reverse, must disable breaking by rev, stop, rev sequence

// Madbeast steering servo:
int valueSteering;  // 1200 to 1800, right to left, 1500 centered
int adjustSteering = 300;  // max adjustment on steering servo

int left   = 1800;
int center = 1380;  // 1500?
int right  = 1200;

// delays
int delayLeft     = 800;  // How long to delay when turning left in milliseconds
int delayRight    = 800;  // How long to delay when turning right in milliseconds
int delayBackward = 2000; // How long to backup in milliseconds
int delayPixycam  = 1000; // How long to move between camera searches

//RH ??
int turnRate = 30;  // RH set for manual turning: 30 deg/second; aka 33.33 msec/deg


// -------------------------------------
// mapping variables & waypoints

#include <math.h>  // RH needed?? trig, etc.

// wp should include distance & new course // waypoint[0] is starting point
int wayPointIndex;  // next waypoint destination
int wpCone[8] = {0,0,0,0,0,0,0,0};  // does wp have a cone? 1 = yes, 0 = no; set values in void setup();
int wayPoint[8][2] = {0,0};      // new course [1], distance (# ticks) to next wp [0], for path planning
// always clear last waypoint entry:
//  wayPoint[5][0] = 0;
//  wayPoint[5][1] = 0;

int newCourse;  // new course at next wp; north is 0°, east is 90°, south is 180°, and west is 270°.
int targetDistance; // distance (# ticks) to next wp
int atStart;  // 1 = start of run, 0 = run already started
int atCone;   // set to 1 when cone touched, set to 0 after wp incremented, new course plotted
int pixycamThreshold = encoderTicksPerStep * 4;  // number of ticks to subtract from target distance to turn on pixycam; 10 steps

// -------------------------------------
// Misc Stuff

//int shutDownRC = 0;   // 0 = stop, i.e. throttle on RC controller not engaged.
// use with INT only: (don't use INT, PITA)
volatile int shutDownRC = 0;   // active low, 0 = stop. if RC hand controller goes to 1500, bot must stop!
#define shutDownPinRC 33  // for RC switch

// LED pins
#define LED_RED 24  // turn on if obstacle detected (OBS) right
#define LED_YEL 25  // turn on if obstacle detected (OBS) center, or deadmnan disengaged
#define LED_GRN 26  // turn on if obstacle detected (OBS) left
#define LED_BLU 27  // turn on if cone detected (cone)
#define PN2222_PIN 28  // controlled by deadman, drive high to turn on PN2222, enable 555, siren

int success = 0;   // 1 = success/done

// debug
unsigned long time;
int debug = 0;  // 1 = turn on debug code. 2 = turn on timing code
int benchTest = 0;  // add delay for compass testing


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
  delay(250);
  LCD_clear();  // forces the cursor to the beginning of the display
  delay(250);
  LCD_setCursor(0,0);
  Serial1.print("Dark Lord Robotics:"); 
  LCD_setCursor(1,0);
  Serial1.print("One Bot to Rule Them"); 
  LCD_setCursor(2,0);
  Serial1.print("All, and in the"); 
  LCD_setCursor(3,0);
  Serial1.print("Darkness Bind Them"); 

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
  
  // assign pins to control throttle & steering
  servoThrottle.attach(22);
  servoSteering.attach(23);


//RH do I need to do this?
  // Calibrating Throttle to full range (does this work? seems to...)
  servoThrottle.writeMicroseconds(2000);
  delay(5);  // 100? 10?
  servoThrottle.writeMicroseconds(1000);
  delay(5);
  servoThrottle.writeMicroseconds(stopped);
  delay(100);


  // center throttle & steering
  valueThrottle = stopped;
  servoThrottle.writeMicroseconds(valueThrottle);
  valueSteering = center;
  servoSteering.writeMicroseconds(valueSteering);
  //delay(1000);

  LCD_setCursor(3,0);  // go to start of 4th line
  Serial1.print("Calibration Complete"); 

  // assign pins to control pixycam
  servoPanPixy.attach(21);  
  valuePanPixy = panleft;  // center pixycam
  servoPanPixy.writeMicroseconds(valuePanPixy);
  delay(1000);
  valuePanPixy = panright;  // center pixycam
  servoPanPixy.writeMicroseconds(valuePanPixy);
  delay(1000);
  valuePanPixy = pancenter;  // center pixycam
  servoPanPixy.writeMicroseconds(valuePanPixy);

  LCD_clear();
  LCD_setCursor(0,0);
  Serial1.print("initialize pixycam"); 
  if (debug >= 1) {
    Serial.println();
    Serial.println("initialize pixycam"); 
  }
  pixy.init();  // initialize pixycam
  LCD_setCursor(1,0);
  Serial1.print("pixycam initialized!"); 
  if (debug >= 1) {
    Serial.println();
    Serial.println("pixycam initialized!"); 
  }

  Wire.begin();  // join i2c bus
  Wire.setClock(400000);  // Increase I2C bus speed to 400 kHz

  // read LIDARLite
  tcaselect(0);  // Adafruit TCA9548A
  //readLIDARLite();
  LCD_setCursor(3,0);
  Serial1.print("dLIDARlite: "); 
  LCD_setCursor(3,4);
  Serial1.print(dLIDARlite);  

/*
//RH ??

  tcaselect(2);  // Adafruit TCA9548A

  // -------------------------------------
  // Pololu VL53L1X Initialize the left sensor

  // Pololu VL53L1X
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

  LCD_clear();
  //Serial1.print("range: ");
  //Serial1.print(VL53L1X_left.ranging_data.range_mm);

  LCD_setCursor(2,0); // go to start of 3rd line
  Serial1.print("Ll "); 
  LCD_setCursor(2,4);
  Serial1.print(dVL53L1X_left);  

  delay (1000);

  tcaselect(1);  // Adafruit TCA9548A

  // -------------------------------------
  // Pololu VL53L1X Initialize the right sensor

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
*/

//  tcaselect(7);

  // -------------------------------------
  // BNO055 IMU initialise the sensor
  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
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
  while (magCal != 3) {
    readBNO055cal();
    LCD_setCursor(2,0);
    Serial1.print("Calibration = "); 
    LCD_setCursor(2,16);
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
  digitalWrite(LED_GRN, HIGH);
  if(digitalRead(shutDownPinRC == 0)) {
    digitalWrite(LED_YEL, HIGH);
  }
  delay(4000);
  //digitalWrite(LED_GRN, LOW);

  // -------------------------------------
  // Misc Stuff

  atStart = 1;  // 1 = start of run, 0 = run already started
  encoderCount = 0;
  coneDetected = 0;  // 1 means cone detected
  atCone  = 0;  // set to 1 when cone touched, set to 0 after wp incremented, new course plotted
  success = 0;  // 1 = success/done

  //avgIndexSensor = 0;  // for averaged sensor readings
  //avgSensorReady = 0;  // to get past startup issue

  LCD_clear();
  LCD_setCursor(0,0);
  Serial1.print("atStart: "); 
  LCD_setCursor(0,10);
  Serial1.print(atStart); 
  LCD_setCursor(1,0);
  Serial1.print("encoderCount: ");
  LCD_setCursor(1,14);
  Serial1.print(encoderCount);
  LCD_setCursor(2,0);
  Serial1.print("coneDetected: "); 
  LCD_setCursor(2,14);
  Serial1.print(coneDetected); 
  LCD_setCursor(3,0);
  Serial1.print("pixycamThreshold: ");
  LCD_setCursor(3,18);
  Serial1.print(pixycamThreshold);
  delay(2000);

  // -------------------------------------
  // NAV waypoint initialization

  wayPointIndex = 0;  // wayPointIndex initialization

  // distance to next wp in ticks (3.97 * steps) Boris
  // distance to next wp in ticks (13.2 * steps) Cujo
  // new course magnetic (if true N from Google Maps, GPS, etc., add declination)
  // is there a cone at next wp?

  // example wayPoint entry:
  //wayPoint[0][0] = 528; // distance to next wp
  //wayPoint[0][1] = 20; // new course
  //wpCone[0] = 1;   // is there a cone at next wp?

  // stokes/Boris
  // 62 steps is 248 ticks
  // 25 steps is 100 ticks

/*  
  // stokes/Boris
  wayPoint[0][0] = 248; // distance to next wp
  wayPoint[0][1] = 215; // new course magnetic
  wpCone[0] = 0;   // is there a cone at next wp?
  
  // next wayPoint:
  wayPoint[1][0] = 100; // distance to next wp
  wayPoint[1][1] = 120; // new course
  wpCone[1] = 0;   // is there a cone at next wp?

  // next wayPoint:
  wayPoint[2][0] = 248; // distance to next wp
  wayPoint[2][1] = 25; // new course
  wpCone[3] = 1;   // is there a cone at next wp?

  // end wayPoint:
  wayPoint[4][0] = 0;
  wayPoint[4][1] = 0;
  wpCone[4] = 0;
*/
/*
  //stokes testing out & back Boris
  wayPoint[0][0] = 120; // distance to next wp in ticks (3.97 * steps)
  wayPoint[0][1] = 220; // new course magnetic
  wpCone[0] = 0;   // is there a cone at next wp?

  // next wayPoint:
  wayPoint[1][0] = 24; // distance to next wp in ticks (3.97 * steps)
  wayPoint[1][1] = 125; // new course magnetic
  wpCone[1] = 0;   // is there a cone at next wp?

  // next wayPoint:
  wayPoint[2][0] = 120; // distance to next wp in ticks (3.97 * steps)
  wayPoint[2][1] = 30; // new course magnetic
  wpCone[2] = 1;   // is there a cone at next wp?

  // end wayPoint:
  wayPoint[3][0] = 0;
  wayPoint[3][1] = 0;
  wpCone[3] = 0;
*/

/*
Baylands Park BotNic
all 4 cones:

37.4136111
-121.98750000

37.4136111
-121.98750000

37.41055556
-121.99250000


Start:
37.411724, -121.996022
newCourse: 262
distance: 110.07

cone 1:
37.411669, -121.996401
newCourse: 21
distance: 216.78

cone 2:
37.412239, -121.996177
newCourse: 256
distance: 69.53

Final cone:
37.412180, -121.996519

possible bonus cone?
37.412447, -121.996408
*/

/*
//Baylands Park BotNic
  wayPoint[0][0] = 116; // distance to next wp in ticks (3.97 * steps)
  wayPoint[0][1] = 250; // new course magnetic
  wpCone[0] = 1;   // is there a cone at next wp?

  // next wayPoint:
  wayPoint[1][0] = 380; // distance to next wp in ticks (3.97 * steps)
  wayPoint[1][1] = 346; // new course magnetic
  wpCone[1] = 1;   // is there a cone at next wp?

  // next wayPoint:
  wayPoint[2][0] = 132; // distance to next wp in ticks (3.97 * steps)
  wayPoint[2][1] = 228; // new course magnetic
  wpCone[2] = 1;   // is there a cone at next wp?

  // end wayPoint:
  wayPoint[3][0] = 0;
  wayPoint[3][1] = 0;
  wpCone[3] = 0;
*/

//Baylands Park BotNic
//final cone only

//waypoint: 0
wayPoint[0][0] = 20; // distance to next wp in feet
wayPoint[0][1] = 312; // new course true
wpCone[0] = 0;   // is there a cone at next wp?

//waypoint: 1
wayPoint[1][0] = 410; // distance to next wp in feet
wayPoint[1][1] = 312; // new course true
wpCone[1] = 1;   // is there a cone at next wp?

//waypoint: 2
wayPoint[2][0] = 0; // distance to next wp in feet
wayPoint[2][1] = 0; // new course true
wpCone[2] = 0;   // is there a cone at next wp?


  // -------------------------------------
  LCD_clear();
  Serial1.print("Blackbeard sez arrg!"); 
  LCD_setCursor(1,0);
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

  //sei();  // global INT enable, broken?

  //if(digitalRead(shutDownPinRC == 0)) {
  //  digitalWrite(LED_YEL, HIGH);   // OBS det  
  //}

  // setup pins for bumper INT
  pinMode(BUMP_L_PIN, INPUT);  //Bump (L) sensor on INT
  pinMode(BUMP_R_PIN, INPUT);  //Bump (R) sensor on INT
  attachInterrupt(digitalPinToInterrupt(BUMP_L_PIN), bumpStop_L, FALLING);  // LOW?
  attachInterrupt(digitalPinToInterrupt(BUMP_R_PIN), bumpStop_R, FALLING);  // LOW?

  // setup pin for encoder interface and INT
  pinMode(encoderPinA, INPUT);  // encoder sensor on INT
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, FALLING);

  // setup pin for RC controller deadman switch
  pinMode(shutDownPinRC, INPUT);
  attachInterrupt(digitalPinToInterrupt(shutDownPinRC), readDeadman, FALLING);  // LOW?

  delay(1000);
  digitalWrite(LED_GRN, LOW);
  readCompass();
  updateLCD();  // debug

  readDeadman();  // stop until remote throttle closed
  updateLCD();  // debug
  
}  // end setup




////////////////////////////////////////////////////////////////////////////////
void loop() {
////////////////////////////////////////////////////////////////////////////////

/*FLOWCHART:
Dead Reckoning Navigation:

at start? set current heading = new course
turn to new course
set distance to travel to next wp
Move forward (distance), avoid obstacles.
if obstacle, back & turn toward wp, fwd; 
read encoder, compass/IMU until wp reached
at wp, if no cone, wp++, encoderCount = 0, turn to new course

if cone at wp, after distance minus 12.5' (# ticks?), look for cone: 
pan camera, if cone detected, match bearings & approach
When close, slow down, touch.
disable pixy cam, increment wp
backup, turn, move fwd to avoid hitting cone?

Read next wp, determine new course
Move, repeat until # targets done
*/

// -------------------------------------
// code start

  if (debug == 2) {
    Serial.println("\n");
    Serial.print("Time void start: ");
    Serial.println(millis());
  }

  if (debug >= 1) {
    Serial.println("\n");
    Serial.print("*** void start ***");
    Serial.println("\n");
  }

  if (benchTest == 1) {
    delay(500);
    Serial.println("\n");
    Serial.print("* benchTest *");
    Serial.println("\n");
  }

  //readDeadman();  // stop until remote throttle closed
  if (bumperHit_L == 1 || bumperHit_R == 1) {
    moveStop();
  }

// -------------------------------------
// startup: set waypoint distance, newCourse

  if (atStart == 1) {
    atStart        = 0;   // 1 = start of run, 0 = run already started
    wayPointIndex  = 0;
    encoderCount   = 0;
    targetDistance = wayPoint[wayPointIndex][0];
    newCourse      = wayPoint[wayPointIndex][1];
    updateLCD();

    if (debug == 1) {
      Serial.print("At Start: wayPointIndex: ");
      Serial.println(wayPointIndex);
      Serial.print("atStart: ");
      Serial.println(atStart);
      Serial.print("newCourse: ");
      Serial.println(newCourse);
      Serial.print("targetDistance: ");
      Serial.println(targetDistance);
      Serial.print("encoderCount: ");
      Serial.println(encoderCount);
      Serial.println();
    }
  }

// -------------------------------------
// read encoder for targetDistance, waypoint update?
  readEncoder();

  // check range for pixycam
  if (wpCone[wayPointIndex] == 1) {
    if ((targetDistance - encoderCount) <= pixycamThreshold) {
      checkPixyCam();  // look for cones, touch if found
    }
    while (coneDetected == 1) {
      checkPixyCam();  // look for cones, touch if found
    }
  }
  // time for wayPointIndex update?
  else {
    if ((encoderCount >= targetDistance) || (atCone == 1 )) {   // wp or cone reached
      wayPointIndex = wayPointIndex + 1;
      moveStop();
      flashLEDs();  // 4 seconds
      // final waypoint arrival?
      if ((wayPoint[wayPointIndex][0] + wayPoint[wayPointIndex][1]) == 0) {
//RH??
        if (atCone == 1) {
          success = 1;
          victoryDance();  // course completed, shutdown
        }

        else {
          checkPixyCam();  // check for cone with pixycam
        }
      }
      // update course & distance for new wp, zero encoder
      else {
        targetDistance = wayPoint[wayPointIndex][0];  // distance to next wp in ticks
        newCourse      = wayPoint[wayPointIndex][1];
        encoderCount   = 0;
        atCone         = 0;
        coneDetected   = 0;
        //RH below not working! delay way too long??
        //turnToNewCourse();  // manual turn based on timer?
      }
      if (debug == 1) {
        Serial.print("targetDistance: ");
        Serial.println(targetDistance);
        Serial.print("newCourse: ");
        Serial.println(newCourse);
      }
    }
  }


// -------------------------------------
  readCompass();
  //headingCurrent = compassHeading;
  headingCurrent = avgHeading;
  if (debug == 2) {
    Serial.println("\n");
    Serial.print("Time readCompass: ");
    Serial.println(millis());
  }

/*
// -------------------------------------
// read compass: average of four readings
  calcAvgCompassHeading();
  headingCurrent = avgHeading;
  if (debug == 2) {
    Serial.println("\n");
    Serial.print("Time calcAvgCompassHeading: ");
    Serial.println(millis());
  }
*/

// -------------------------------------
// calcHeadingError: negative error = turn left, positive error = turn right
  calcHeadingError();
  if (debug == 2) {
    Serial.print("Time calcHeadingError: ");
    Serial.println(millis());
  }

// -------------------------------------
// adjust steering to new course
  steeringCorrection();
  if (debug == 2) {
    Serial.print("Time steeringCorrection: ");
    Serial.println(millis());
  }

  updateLCD();
  //readDeadman();  // stop until remote throttle closed



// ---------------------------------------------------------------------------
// Obstacle detection via sensor readings

//RH - replace all these sensors with spinning LIDAR?? (keep bump sensor, but redesign it)

// collect 4 sets of readings & calc rolling average?

/*
//tcaselect(0);
// -------------------------------------
// LIDAR-Lite
// dLIDARlite

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

  if(2 <= Wire.available()) {  // if two bytes were received
    dLIDARlite = Wire.read(); // receive high byte (overwrites previous reading)
    dLIDARlite = dLIDARlite << 8; // shift high byte to be high 8 bits
    dLIDARlite |= Wire.read(); // receive low byte as lower 8 bits
  }

  // debug
  if (debug == 1) {
    Serial.print("LIDAR-Lite: ");
    Serial.print(dLIDARlite);
    Serial.println(" cm ");
  }
  if (debug == 2) {
    Serial.print("Time LIDAR: ");
    //time = millis();
    Serial.println(millis());
  }  
*/


/*
  // -------------------------------------
  // Pololu VL53L1X left sensor

  tcaselect(2);  // Adafruit TCA9548A

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

  //LCD_setCursor(2,0); // go to start of 3rd line
  //Serial1.print("Ll "); 
  //LCD_setCursor(2,4);
  //Serial1.print(dVL53L1X_left);  

  if (debug == 1) {
    Serial.print("VL53L1X_left: ");
    Serial.print(dVL53L1X_left);
    if (VL53L1X_left.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    Serial.println(" cm");       
  }  
  if (debug == 2) {
    Serial.print("Time VL53L1X_left: ");
    Serial.println(millis());
  }

  // -------------------------------------
  // Pololu VL53L1X right sensor

  tcaselect(1);  // Adafruit TCA9548A

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

  //LCD_setCursor(2,7);
  //Serial1.print("Lr "); 
  //LCD_setCursor(2,10);
  //Serial1.print(dVL53L1X_right);  

  if (debug == 1) {
    Serial.print("VL53L1X_right: ");
    Serial.print(dVL53L1X_right);
    if (VL53L1X_right.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    Serial.println(" cm");       
  }  
  if (debug == 2) {
    Serial.print("Time VL53L1X_right: ");
    Serial.println(millis());
  }
*/


/*
// -------------------------------------
// collect 4 sets of readings & calc rolling average  
//RH how to avoid startup failure? all readings div by 4!

// RH add LIDAR

//void calcAvgSensors() {
  avgdVL53L0X    = 0;
  avgdLIDARlite  = 0;

  for (int i = 0; i < 4; i++) {
    avgdVL53L0X    = avgdVL53L0X    + dVL53L0X[i];
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
    Serial.println();  // to space out the data in the monitor for readability
  }
*/

// -------------------------------------
// avoid any obstacles detected, take evasive action!
// RH: don't turn while backing up!! (MaxStone aka Cujo)

  //int dAvoid0 = 13;  //int dAvoid1 = 30;  //int dAvoid2 = 60;  //int dAvoid3 = 90;  // distance to trigger obstacle avoidance in cm

  //if (bumperHit_L == 1 || bumperHit_R == 1 || dVL53L1X_left < dAvoid0 || dVL53L1X_right < dAvoid0 || ((dLIDARlite >= 10) && (dLIDARlite < dAvoid2))) {
  if (bumperHit_L == 1 || bumperHit_R == 1) {   // bump only

  // RH test code for average readings:
  //if (avgSensorReady == 1 && ( (avgdSonarPing1 >= 1) && (avgdSonarPing1 < dAvoid2) || (avgdSonarPing2 >= 1) && (avgdSonarPing2 < dAvoid2)
  // || avgdIrA710 < dAvoid3 || avgdIrA02Y < dAvoid3 || avgdVL53L0X < dAvoid1
  // || digitalRead(BUMP_L_PIN) == 0 || digitalRead(BUMP_R_PIN) == 0 ) {  // obstacle detected

    moveStop();

    if (bumperHit_L == 1) {   // || dVL53L1X_left < dAvoid1
      digitalWrite(LED_GRN, HIGH);
    }
    if (bumperHit_R == 1) {   //  || dVL53L1X_right < dAvoid1
      digitalWrite(LED_RED, HIGH);
    }
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

    //readDeadman();  // stop until remote throttle closed

    LCD_setCursor(1,0);
    Serial1.print("take evasive action!"); 

    moveTurnCenter();    // RH: don't turn while backing up!!
    moveStop();
    readEncoder();
    moveBackward();
    delay(delayBackward);
    moveStop();
    readEncoder();

    //readDeadman();  // stop until remote throttle closed
    //RH if only one bumper L/R is triggered, turn away from that bumper:

    if ((bumperHit_L == 1) && (bumperHit_R == 0)) {   // || dVL53L1X_left < dAvoid1, && dVL53L1X_right > dAvoid1
    //if ((bumperHit_L == 1 || dVL53L1X_left < dAvoid1) && (bumperHit_R == 0) && dVL53L1X_right > dAvoid1 && (dLIDARlite > dAvoid2)) { 
      // turn right
      moveTurnRight();
      // do trig to figure out proper course change. How far off due to turn? How far to target?
      newCourse = newCourse - 5;  
      moveForward();
      delay(delayRight);
    }
    else if ((bumperHit_R == 1) && (bumperHit_L == 0)) {   //  || dVL53L1X_right < dAvoid1, && dVL53L1X_left > dAvoid1
    //if ((bumperHit_R == 1 || dVL53L1X_right < dAvoid1) && (bumperHit_L == 0) && dVL53L1X_left > dAvoid1 && (dLIDARlite > dAvoid2)) {
      // turn left
      moveTurnLeft();
      // do trig to figure out proper course change. How far off due to turn? How far to target?
      newCourse = newCourse + 5;
      moveForward();
      delay(delayLeft);
    }
    // turn toward wp_relative based on courseError
    // headingError: negative error = turn left, positive error = turn right
    else if (headingError < 0) {  // turn left
      moveTurnLeft();
      // do trig to figure out proper course change. How far off due to turn? How far to target?
      newCourse = newCourse + 5;
      moveForward();
      delay(delayLeft);
    }
    else if (headingError >= 0) {  // turn right
      moveTurnRight();
      // do trig to figure out proper course change. How far off due to turn? How far to target?
      newCourse = newCourse - 5;  
      moveForward();
      delay(delayRight);
    }
    moveTurnCenter();

    bumperHit_L = 0;
    bumperHit_R = 0;
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_YEL, LOW);
    digitalWrite(LED_GRN, LOW);
    //moveForward();  // check for new course and obstacles first
  } 
  else {
    //readDeadman();  // stop until remote throttle closed
    moveForward();  // if > distanceAvoid
    bumperHit_L = 0;
    bumperHit_R = 0;
    digitalWrite(PN2222_PIN, HIGH);  // siren on
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_YEL, LOW);
    digitalWrite(LED_GRN, LOW);
  }

  //  updateLCD display if needed
  //  updateLCD();

  if (debug == 2) {
    Serial.print("Time loop: ");
    Serial.println(millis());
  }

}  // end void loop




////////////////////////////////////////////////////////////////////////////////
// functions
////////////////////////////////////////////////////////////////////////////////

// -------------------------------------
void tcaselect(uint8_t i) {
  //Adafruit TCA9548A 1-to-8 I2C Multiplexer Breakout
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

// -------------------------------------
void readBNO055cal() {
  // send calibration data for each sensor
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  magCal = mag;
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

// -------------------------------------
void A_readCompass() {
  readBNO055();
  if (debug >= 1) {
    Serial.print("compassHeading= ");
    Serial.println(compassHeading);
    Serial.println();
  }
}

// -------------------------------------
void readCompass() {
//void calcAvgCompassHeading() {
  // collect 4 compass readings & calc average heading
  avgHeading = 0;
  for (int i = 0; i < 16; i++) {
    A_readCompass();
    avgHeading = avgHeading + compassHeading;
  }
  avgHeading = avgHeading >> 4; // div by 16
  //avgHeading = avgHeading >> 2; // div by 4
  if (debug >= 1) {
    Serial.print("avgCompassHeading: ");
    Serial.println(avgHeading);
    Serial.println();
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
}

// -------------------------------------
void steeringCorrection() {
  // correct steering to new course, adjustSteering = 300
  // valueSteering;  // 1200 to 1800, right to left, 1500 centered
  // headingError = newCourse - headingCurrent;

  if (headingError < -0) {  // turn left, increase valueSteering
    if (headingError <= -19) {  // turn left full (300/16=18.75)
    //if (headingError < -30) {  // turn left
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
    if (headingError >= 19) {  // turn right full (300/16=18.75)
    //if (headingError > 30) {  // turn right
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
    Serial.print("headingError: ");
    Serial.println(headingError);
    Serial.print("valueSteering: ");
    Serial.println(valueSteering);
    Serial.print("\n");
  }
}

// -------------------------------------
void turnToNewCourse() {
  // manual turn based on timer
  //RH not working!! delay way too long??
  
  int courseChange = newCourse - wayPoint[wayPointIndex - 1][1];   // new - current
  // correct for nearest compass heading
  if (courseChange > 180) {
    courseChange = courseChange - 360;
  }
  else if (courseChange < -180) {
    courseChange = courseChange + 360;
  }
  int turnDelay = ((courseChange * 10)/(turnRate)) * 100; // ms = deg * ms/deg (or div by deg/ms) // what is turnRate?
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
void readEncoder() {
  // read encoder & update encoderCount
  if (newThrottle == rev) {
    encoderCount = encoderCount - encoderPosA;
  }
  else {
    encoderCount = encoderCount + encoderPosA;
  }
  if (debug == 1) {
    Serial.println("encoderPosA"); 
    Serial.println(encoderPosA);
    Serial.println("encoderCount"); 
    Serial.println(encoderCount);
    Serial.println();
  }
  encoderPosA = 0;
}

// -------------------------------------
// bumper INT service routines

void bumpStop_L() {
  valueThrottle = rev;  // brake
  servoThrottle.writeMicroseconds(valueThrottle);  // immediate stop!
  //valueThrottle = stopped;
  //servoThrottle.writeMicroseconds(valueThrottle);  // immediate stop!
  bumperHit_L = 1;
}

void bumpStop_R() {
  valueThrottle = rev;  // brake
  servoThrottle.writeMicroseconds(valueThrottle);  // immediate stop!
  //valueThrottle = stopped;
  //servoThrottle.writeMicroseconds(valueThrottle);  // immediate stop!
  bumperHit_R = 1;
}

// -------------------------------------
void doEncoderA() {
  // encoder INT service routine
  encoderPosA++;
}

// -------------------------------------
void readDeadman() {  
  // read RC throttle detector (use instead of INT?)
  // stop when RC throttle released
  shutDownRC = digitalRead(shutDownPinRC);
  moveStop();
  while (digitalRead(shutDownPinRC) == 0 || success == 1) {
    //moveStop();
    digitalWrite(PN2222_PIN, LOW);  // siren off
    digitalWrite(LED_YEL, HIGH);
    //tcaselect(7);
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

// -------------------------------------
void moveTurnCenter() {
  servoSteering.writeMicroseconds(center);  // 1500
}

void moveTurnLeft() {
  servoSteering.writeMicroseconds(left);  // 1800
}

void moveTurnRight() {
  servoSteering.writeMicroseconds(right);  // 1200
}

// -------------------------------------
void moveStop() {
  valueThrottle = rev;
  servoThrottle.writeMicroseconds(valueThrottle);
  //delay(100);
  //valueThrottle = stopped;
  //servoThrottle.writeMicroseconds(valueThrottle);
}

void moveForward() {
  valueThrottle = fwd;
  servoThrottle.writeMicroseconds(valueThrottle);
}

void moveBackward() {
  valueThrottle = stopped;
  servoThrottle.writeMicroseconds(valueThrottle);
  delay(200);
  valueThrottle = rev;
  servoThrottle.writeMicroseconds(valueThrottle);
}

// -------------------------------------
void checkPixyCam() {
  // RH review this carefully, is it right??

  // set course for cone, touch, set atCone = 1, back up, etc.
  // use VL53L1X or other ranging sensor for slowdown & touch
  // read limit switches for cone touch

  // cone detection, approach, touch, reverse
  // coneDetected? coneDetected = 1;
  // match steering to cam
  // measure distance, determine speed
  // slow down as we approach
  // when touch sensor triggers, stop!
  // then backup, turn to next wp

  //moveStop();

  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  blocks = pixy.getBlocks();
  if (blocks) {
    coneDetected = 1;
    // update LCD
    LCD_setCursor(3,8);
    Serial1.print("Cdet ");
    LCD_setCursor(3,13);
    Serial1.print(coneDetected);
    //LCD_setCursor(3,15);
    //Serial1.print("@C ");
    //LCD_setCursor(3,18);
    //Serial1.print(atCone);
    digitalWrite(LED_BLU, HIGH);   // set Cone det LED on

    if (debug >= 1) {
      Serial.println("");
      Serial.println("block detected");  // debug only RH
    }
    if (debug >= 1) {
      delay(5000); // debug    
    }
    steerPanError();
    if (debug >= 1) {
      delay(5000); // debug    
    }

/*
    panError = X_CENTER-pixy.blocks[0].x;
    LCD_setCursor(3,15);
    Serial1.print("PE ");
    LCD_setCursor(3,18);
    Serial1.print(panError);

    // set steering angle based on object direction from pixycam pan angle
    if (panError < -30) {
      valueSteering = 1200;
    }
    else if (panError > 30) {
      valueSteering = 1800;
    }
    else {
      valueSteering = map(panError, -30, 30, 0, 600) + 1200;  // steering servo
    }
    servoSteering.writeMicroseconds(valueSteering);
    valueThrottle = fwd;
    servoThrottle.writeMicroseconds(valueThrottle);
    delay(delayPixycam);
*/

    // cone touched?
    // added code here for > 1 cone:
    if (coneDetected == 1 && ((bumperHit_L == 1) || (bumperHit_R == 1))) {
    //if (coneDetected == 1 && (digitalRead(BUMP_L_PIN) == 1) || (digitalRead(BUMP_R_PIN) == 1)) {
      atCone  = 1;   // set to 1 when cone touched, set to 0 after wp incremented, new course plotted
      moveStop();
      digitalWrite(LED_RED, HIGH);   // set OBS det LED
      digitalWrite(LED_GRN, HIGH);   // set OBS det LED
      digitalWrite(LED_BLU, HIGH);   // set Cone det LED on
      LCD_clear();
      Serial1.print("CONE TOUCHED!!"); 
      LCD_setCursor(1,0);
      Serial1.print("atCone = ");
      LCD_setCursor(1,10);
      Serial1.print(atCone);

      // final waypoint arrival?
      if ((wayPoint[wayPointIndex + 1][0] + wayPoint[wayPointIndex + 1][1]) == 0) {
        success = 1;
        victoryDance();  // course completed, shutdown
      }
      else {
        moveTurnCenter();    // RH: don't turn while backing up!!
        moveBackward();
        delay(delayBackward);
        moveStop();
        digitalWrite(LED_RED, LOW);   // clear OBS det LED
        digitalWrite(LED_GRN, LOW);   // clear OBS det LED
        digitalWrite(LED_BLU, LOW);   // clear Cone det LED
        LCD_setCursor(3,0);
        Serial1.print("updating waypoint"); 
//RH    // see wp checking code near start for next steps
      }
    }
    if (debug >= 1) {
      Serial.print("panError: ");
      Serial.println(panError);
      Serial.print("steering: ");
      Serial.println(valueSteering);
      Serial.print("throttle: ");
      Serial.println(valueThrottle);
    }
    delay(100);  // why?
  }            // end if blocks detected
  else {     // stop and look around
    valueSteering = center;
    servoSteering.writeMicroseconds(valueSteering);
    moveStop();
    LCD_clear();
    updateLCD();
    LCD_setCursor(2,0);
    Serial1.print("looking for cone..."); 
    delay(100);

    // pan left
    valuePanPixy = panleft;
    servoPanPixy.writeMicroseconds(valuePanPixy);
    LCD_setCursor(3,0);
    Serial1.print("Looking Left..."); 
    if (debug >= 1) {
      Serial.print("Looking Left...");
    }
    delay(2000);
  
    blocks = pixy.getBlocks();
    if (blocks) {
      coneDetected = 1;
      digitalWrite(LED_BLU, HIGH);   // set Cone det LED on
      if (debug >= 1) {
        Serial.println("\n");
        Serial.println("block detected");  // debug only RH
      }
      //steerPanError();
      servoSteering.writeMicroseconds(left);
      if (debug >= 1) {
        delay(5000); // debug
      }
      valueThrottle = fwd;
      servoThrottle.writeMicroseconds(valueThrottle);
      delay(delayPixycam);
      moveStop();
    }
    else {
      // pan right
      valuePanPixy = panright;
      servoPanPixy.writeMicroseconds(valuePanPixy);
      LCD_setCursor(3,0);
      Serial1.print("Looking Right..."); 
      if (debug == 1) {
        Serial.print("Looking Right...");
      }
      delay(2000);

      blocks = pixy.getBlocks();
      if (blocks) {
        coneDetected = 1;
        digitalWrite(LED_BLU, HIGH);   // set Cone det LED on
        if (debug >= 1) {
          Serial.println("\n");
          Serial.println("block detected");  // debug only RH
        }
        //steerPanError();
        servoSteering.writeMicroseconds(right);
        if (debug >= 1) {
          delay(5000); // debug
        }
        valueThrottle = fwd;
        servoThrottle.writeMicroseconds(valueThrottle);
        delay(delayPixycam);
        moveStop();
      }
      else {
        coneDetected = 0;
        digitalWrite(LED_BLU, LOW);   // clear Cone det LED
        valuePanPixy = pancenter;
        servoPanPixy.writeMicroseconds(valuePanPixy);
        delay(300);
        // circle right
        if (encoderCount >= targetDistance) {   // wp reached
          valueSteering = right;
          servoSteering.writeMicroseconds(valueSteering);
          valueThrottle = fwd;
          servoThrottle.writeMicroseconds(valueThrottle);
          delay(delayPixycam);
          valueSteering = center;
          servoSteering.writeMicroseconds(valueSteering);
          moveStop();
        }
        else {   // move closer to wp
          valueSteering = center;
          servoSteering.writeMicroseconds(valueSteering);
          valueThrottle = fwd;
          servoThrottle.writeMicroseconds(valueThrottle);
          delay(delayPixycam);
          moveStop();
        }
      }
    }
  }
  // center camera
  valuePanPixy = pancenter;
  servoPanPixy.writeMicroseconds(valuePanPixy);
  delay(100);

}  // end checkPixyCam


// -------------------------------------
void steerPanError() {

    panError = X_CENTER-pixy.blocks[0].x;
    LCD_setCursor(3,15);
    Serial1.print("PE ");
    LCD_setCursor(3,18);
    Serial1.print(panError);

    // set steering angle based on object direction from pixycam pan angle
    if (panError < (panFactor*-1)) {
      valueSteering = 1200;
    }
    else if (panError > panFactor) {
      valueSteering = 1800;
    }
    else {
      valueSteering = map(panError, (panFactor*-1), panFactor, 0, 600) + 1200;  // steering servo
    }

    servoSteering.writeMicroseconds(valueSteering);
    LCD_setCursor(3,0);
    Serial1.print("Steering: ");
    LCD_setCursor(3,11);
    Serial1.print(valueSteering);

    if (debug >= 1) {
      moveStop();
      delay(5000); // debug
    }
    valueThrottle = fwd;
    servoThrottle.writeMicroseconds(valueThrottle);
    //delay(delayPixycam);

}


// -------------------------------------
void readLIDARLite() {
  // using Arduino wire library
  // currently unused...

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

  if (2 <= Wire.available()) {// if two bytes were received
    dLIDARlite = Wire.read(); // receive high byte (overwrites previous reading)
    dLIDARlite = dLIDARlite << 8; // shift high byte to be high 8 bits
    dLIDARlite |= Wire.read(); // receive low byte as lower 8 bits
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
void victoryDance() {
  // victoryDance: course completed!
  // do a little dance, make a little love....
  while (success == 1) {
    valueThrottle = stopped;
    servoThrottle.writeMicroseconds(valueThrottle);
    if (debug == 1) {
      Serial.print("success: ");
      Serial.println(success);
    }
    for (int i = 0; i < 1000; i++) {
      digitalWrite(PN2222_PIN, LOW);  // shut off siren
      digitalWrite(LED_RED, HIGH);   // OBS 
      digitalWrite(LED_YEL, HIGH);   // wp
      digitalWrite(LED_GRN, HIGH);   // cone
      LCD_clear();
      Serial1.print("success!!!");
      delay(1000);
      digitalWrite(PN2222_PIN, HIGH);  // siren on
      digitalWrite(LED_RED, LOW);   // OBS 
      digitalWrite(LED_YEL, LOW);   // wp
      digitalWrite(LED_GRN, LOW);   // cone
      LCD_clear();
      delay(1000);
    }
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

/*
  LCD_clear();
  LCD_setCursor(3,19);
*/

// -------------------------------------
void updateLCD() {
  //RH replace with daylight readable display? OpenLCD ok?
  //RH update for OpenLCD Serial1

  // waypoint index & new course, distance
  // display distance traveled, target
  // display curr course, target course
  // display cone?
  // display obstacle range?

  // line 1:
  // display waypoint index, target distance, distance traveled
  LCD_clear();  // clear the display and go to start of 1st line

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
  // display target course, current course, calibrated compass heading, raw compass heading
  LCD_setCursor(1,0); // go to start of 2nd line

  Serial1.print("newC:"); 
  LCD_setCursor(1,6);
  Serial1.print(newCourse); 
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
  // display obstacle ranges
  LCD_setCursor(2,0); // go to start of 3rd line
  Serial1.print("Ll "); 
  LCD_setCursor(2,4);
  Serial1.print(dVL53L1X_left);  
  LCD_setCursor(2,7);
  Serial1.print("Lr "); 
  LCD_setCursor(2,10);
  Serial1.print(dVL53L1X_right);  
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
  LCD_setCursor(3,0); // go to start of 4th line

  LCD_setCursor(3,0);
  Serial1.print("LD "); 
  LCD_setCursor(3,4);
  Serial1.print(dLIDARlite);  

//  Serial1.print("L");
//  LCD_setCursor(3,1);
//  //Serial1.print(dVL53L0X);
//  LCD_setCursor(3,5);
//  Serial1.print("L");
//  LCD_setCursor(3,6);
//  Serial1.print(dLIDARlite);
  LCD_setCursor(3,8);
  Serial1.print("Cdet ");
  LCD_setCursor(3,13);
  Serial1.print(coneDetected);
  LCD_setCursor(3,15);
  Serial1.print("@C ");
  LCD_setCursor(3,18);
  Serial1.print(atCone);

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


/*
//-----------------------------------------------
void calibrateAverageHeading() {
  // calibrate compass: headingCurrentCalibrated
  // correction factor for 0, 90, 180, 270, 0
  // A, B, C, D, A
  // headingCurrentCalibrated = headingCurrentUncal - int((headingCurrentUncal*(B-A)/90) - A;
  // +20, +15, +25, +11, (+20)   190505 BNO055
  // +10, +23, +23, 0, +10  (??)

//RH redo this code:
  if (headingCurrentUncal <= 90) {
    headingCurrentCalibrated = headingCurrentUncal - int((headingCurrentUncal*13)/90) - 10;
  }
  else if (headingCurrentUncal <= 180) {
    headingCurrentCalibrated = headingCurrentUncal - 23;
  }
  else if (headingCurrentUncal <= 270) {
    headingCurrentCalibrated = headingCurrentUncal - int((270 - headingCurrentUncal)*23/90);
  }
  else if (headingCurrentUncal <= 360) {
    headingCurrentCalibrated = headingCurrentUncal - int((headingCurrentUncal - 270)*10/90);
  }
  else headingCurrentCalibrated = headingCurrentUncal;
  
  if (headingCurrentCalibrated >= 360) {
    headingCurrentCalibrated = headingCurrentCalibrated - 360;
  }
  else if (headingCurrentCalibrated < 0) {
    headingCurrentCalibrated = headingCurrentCalibrated + 360;
  }
  headingAvg = headingCurrentCalibrated;

  if (debug >= 1) {
    Serial.print("headingCurrentCalibrated: ");
    Serial.println(headingCurrentCalibrated);
  }
}
*/

/*
// -------------------------------------
void setThrottle() {
  // ramp throttle up or down gradually

  if (debug == 1) {
    Serial.print("newThrottle: ");
    Serial.println(newThrottle);
    Serial.print("valueThrottle: ");
    Serial.println(valueThrottle);
  }
  for (int i = 1; i < 501; i++) {  // to cover max range for fwd or rev (1000, 1500, 2000)
    delay(ThrottleStepRate);  // value? 5 ms
    if (newThrottle > valueThrottle) {
      valueThrottle = valueThrottle + 1;
    }
    else if (newThrottle < valueThrottle) {
      valueThrottle = valueThrottle - 1;
    }
    else if (newThrottle == valueThrottle) {
      i = 501;
    }
    servoThrottle.writeMicroseconds(valueThrottle);
  }
  if (debug == 1) {
    Serial.print("newThrottle: ");
    Serial.println(newThrottle);
    Serial.print("valueThrottle: ");
    Serial.println(valueThrottle);
    Serial.println();
  }
}
*/
