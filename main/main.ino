/**
  FEEG2001 Odometry Task
  Name: main

  @author Aliaksei Pilko
  @version 1.0
*/
#include <Wire.h>
#include <Servo.h>

/* Define I2C bus address, registers and relevant consts
    @see https://www.robot-electronics.co.uk/htm/md25i2c.htm
*/
const byte addr = 0x58;

const byte cmdReg = 0x10; //Command Register
const byte modeReg = 0x0F; //Mode Register
const byte speed1Reg = 0x00; //Speed Register
const byte speed2Reg = 0x01; //Turn Register
const byte enc1Reg = 0x02; //Encoder 1
const byte enc2Reg = 0x06; //Encoder 2
//const byte cur1 = 0x0B; //Current to motor 1
//const byte cur2 = 0x0C; //Current to motor 2

//Common Speed Values
const byte MAX_SPEED = 0xFF; //255
const byte STOP_SPEED = 0x80; //128
const byte MAX_REVERSE_SPEED = 0x00; //00

//Useful Commands
const byte rstEnc = 0x20; //Reset Encoder Counts
const byte ds2secTo = 0x32; //Disable 2 second motor failsafe
const byte en2secTo = 0x33; //Enable 2 second motor failsafe

/* Define Physical Consts
    Units: Base SI
*/
const float wheelCirc = 0.1;
const float axisLength = 0.26;
const float maxSpeedMps = 0.56666667;

/* Define path to follow

*/
//Enumerates the possible states of the robot
enum MotionType {
  Straight,
  PointTurn,
  Arc,
  Stop
};

struct Path {
  MotionType motion;
  float val1;
  float val2; //Used only for arc angle
};

Path course[28] = {
  //Start Point 13
  {Straight, 0.340, NULL},
  {PointTurn, 90.0, NULL}, //Point 1
  {Straight, 0.260, NULL},
  //Drop point
  {PointTurn, -45, NULL}, //Point 2/12 //Angle not defined well
  {Straight, .300, NULL}, //Distance not defined well
  {PointTurn, 135, NULL}, //Point 11 //Angle not defined well
  {Arc, 0.180, -270},
  //Drop Point
  {PointTurn, -90, NULL}, //Point 10
  {Straight, 0.180, NULL},
  {PointTurn, 140, NULL}, //Point 9
  {Straight, 0.500, NULL}, //Distance not defined well
  {PointTurn, 130, NULL}, //Point 8
  {Straight, 0.400, NULL},
  {PointTurn, -90, NULL}, //Point 5
  {Straight, 0.400, NULL},
  //Drop Point
  {PointTurn, -90, NULL}, //Point 6
  {Straight, 0.400, NULL},
  {PointTurn, -90, NULL}, //Point 7
  {Straight, 0.400, NULL},
  //Drop Point
  {PointTurn, 90, NULL}, //Point 8
  {Straight, 0.260, NULL},
  {PointTurn, -90, NULL}, //Point 4
  {Arc, 0.260, -90},
  {PointTurn, 90, NULL}, //Point 3
  {Straight, 0.500, NULL},
  {PointTurn, 45, NULL}, //Point 2/12 //Angle not defined well
  {Straight, 0.428, NULL}
  //  {PointTurn, 720, NULL} //Point 13 //Celebratory spin
};

short courseIndex; //An index into the current position through the course array. Can be short as expected range very small
boolean executing; //Blocking boolean to prevent more than one path segment being active/called. Stops loop() "outpacing" other executing methods
long requiredEnc1Count; //Desired encoder 1 count for each path segment. Needs to be long as enc count can be up to 32bit
MotionType currentMotion; //Current motion type the robot is undergoing
bool timeBased = false; //Change turn modes to measure based on time rather than encoder count if needed

/* Dropper definitions */
Servo dropperServo;
short servoPositions[6] = {750, 550, 950, 1375, 1800, 2500};
short dropperIndex;

/* Define Pins */
const short buzzerPin = 3;
const short LEDPin = 13;
const short servoPin = 9;

void setup() {
  Wire.begin(); //Start I2C Bus
  Serial.begin(9600); //Start Serial output at 9600 baud
  setMode(0); //Set mode 0
  //  setCmd(ds2secTo); //Disable motor timeout

  //Initialise robot state
  setCmd(rstEnc); //Reset encoder counts at start
  courseIndex = 0; //Set first course segment
  dropperIndex = 0; //Set first dropper index
  executing = false;
  requiredEnc1Count = 0;
  currentMotion = Stop;

  //Set pin modes
  pinMode(buzzerPin, OUTPUT);
  pinMode(LEDPin, OUTPUT);
  dropperServo.attach(servoPin); //Attach servo to correct pin
  
}

void loop() {
  //Minimise blocking on loop()
  if (!executing) {
    //Check if robot is currently stopped
    if (currentMotion == Stop) {
      Serial.println((String)"Executing course idx " + courseIndex);
      setCmd(rstEnc); //Reset encoders
      executing = true;
      executePath(course[courseIndex]);
    } else {
      if (abs(readEnc1()) >= abs(requiredEnc1Count)) {
        Serial.println((String)"Encoder 1 count " + readEnc1() + " reached");
        Serial.println((String)"Encoder 2 count " + readEnc2() + " reached");
        setStraightSpeed(STOP_SPEED);
        currentMotion = Stop;
        delay(2000);
      }
    }
  }
}

/**
   @brief Executes a given path segment

   @param path Path segment to be executed
*/
void executePath(Path path) {
  currentMotion = path.motion;
  switch (path.motion) {
    case Straight:
      requiredEnc1Count = (path.val1 / wheelCirc) * 360;
      setStraightSpeed(MAX_SPEED);
      Serial.println((String)"Straight RE1C " + requiredEnc1Count);
      break;
    case PointTurn:
      setPointTurn(
        abs(path.val1),
        abs(path.val1) == path.val1); //Check if value is negative indicating CCW
      break;
    case Arc:
      setArcTurn(
        path.val1,
        abs(path.val2) == path.val2, //Check if value is negative indicating CCW
        abs(path.val2));
      break;
  }
  courseIndex++; //Increment course index for next segment execution
  if(courseIndex >= 29){ //check if finished course
    Serial.println("Finished");
    executing = true; //Block any execution
  }
}

/**
   @brief Sets Command Register to desired command
   @see https://www.robot-electronics.co.uk/htm/md25i2c.htm#command%20register

   @param cmd desired command as byte
*/
void setCmd(byte cmd) {
  Wire.beginTransmission(addr);
  Wire.write(cmdReg);
  Wire.write(cmd);
  Wire.endTransmission();
}

/**
   @brief Sets Mode register to desired mode
   @see https://www.robot-electronics.co.uk/htm/md25i2c.htm#mode

   @param mode desired mode as byte
*/
void setMode(byte mode) {
  Wire.beginTransmission(addr);
  Wire.write(modeReg);
  Wire.write(mode);
  Wire.endTransmission();
}

/**
   @brief Reads value of Encoder 1
   @see https://www.robot-electronics.co.uk/htm/md25i2c.htm#encoders

   @return long value for encoder count
*/
long readEnc1() {
  Wire.beginTransmission(addr);
  Wire.write(enc1Reg);
  Wire.endTransmission();

  // Read Encoder bytes into 32 bit unsigned long
  Wire.requestFrom(addr, 4);
  while (Wire.available() < 4);
  long poss = Wire.read();
  poss <<= 8;
  poss += Wire.read();
  poss <<= 8;
  poss += Wire.read();
  poss <<= 8;
  poss += Wire.read();

  return poss;
}

/**
   @brief Reads value of Encoder 2
   @see https://www.robot-electronics.co.uk/htm/md25i2c.htm#encoders

   @return long value for encoder count
*/
long readEnc2() {
  Wire.beginTransmission(addr);
  Wire.write(enc2Reg);
  Wire.endTransmission();

  // Read Encoder bytes into 32 bit unsigned long
  Wire.requestFrom(addr, 4);
  while (Wire.available() < 4);
  long poss = Wire.read();
  poss <<= 8;
  poss += Wire.read();
  poss <<= 8;
  poss += Wire.read();
  poss <<= 8;
  poss += Wire.read();

  return poss;
}

/**
   @brief Sets same speed on both motors
   @see https://www.robot-electronics.co.uk/htm/md25i2c.htm#speed1
   Speed ranges from 0x00 (Full Reverse) to 0xFF (Full Forward)
   Stop speed is 0x80

   @param speedval desired speed as byte
*/
void setStraightSpeed(byte speedVal) {
  Serial.println((String) "Straight " + speedVal);
  Wire.beginTransmission(addr);
  Wire.write(speed1Reg);
  Wire.write(speedVal);
  Wire.endTransmission();
  Wire.beginTransmission(addr);
  Wire.write(speed2Reg);
  Wire.write(speedVal);
  Wire.endTransmission();
  executing = false;
}

/**
   @brief Turn on the spot

   @param degToTurn integer number of degrees to turn
   @param clockwise true if arc should be followed in a CW direction, false otherwise
*/
void setPointTurn(int degToTurn, bool clockwise) {
  signalWaypoint();
  Serial.println((String) "Point Turn " + degToTurn + "deg direction clockwise " + clockwise);
  float dist = degToTurn * (PI * axisLength / 360); //Circular path distance each wheel must travel
  if (clockwise) {
    requiredEnc1Count = dist * 360 / wheelCirc; //Required Encoder count for 1 wheel
    //Start Motors
    Wire.beginTransmission(addr);
    Wire.write(speed1Reg);
    Wire.write(MAX_SPEED);
    Wire.endTransmission();
    Wire.beginTransmission(addr);
    Wire.write(speed2Reg);
    Wire.write(MAX_REVERSE_SPEED);
    Wire.endTransmission();
  } else {
    requiredEnc1Count = -(dist * 360 / wheelCirc); //Required Encoder count for 1 wheel
    //Start Motors
    Wire.beginTransmission(addr);
    Wire.write(speed1Reg);
    Wire.write(MAX_REVERSE_SPEED);
    Wire.endTransmission();    
    Wire.beginTransmission(addr);
    Wire.write(speed2Reg);
    Wire.write(MAX_SPEED);
    Wire.endTransmission();
  }
  Serial.println((String)"Point Turn RE1C " + requiredEnc1Count);

    if(timeBased){
      int timeOn = 1000 * dist / maxSpeedMps;
      delay(timeOn);// Delay turning motors off to achieve desired angle
      //Stop Motors
      Wire.beginTransmission(addr);
      Wire.write(speed1Reg);
      Wire.write(STOP_SPEED);
      Wire.endTransmission();
      Wire.beginTransmission(addr);
      Wire.write(speed2Reg);
      Wire.write(STOP_SPEED);
      Wire.endTransmission();
    }
  executing = false;
}

/**
   @brief Turn in an arc of defined radius

   @param arcRadius radius of arc followed by midpoint in metres
   @param clockwise true if arc should be followed in a CW direction, false otherwise
   @param angle angle to sweep in arc
*/
void setArcTurn(float arcRadius, bool clockwise, int angle) {
  Serial.println((String) "Arc Turn " + arcRadius + "m radius, direction clockwise " + clockwise + " ,angle " + angle);
  float innerSpeedMps = maxSpeedMps * (arcRadius - (axisLength / 2)) / (arcRadius + (axisLength / 2)); //Inner motor speed in m/s
  byte innerSpeed = (byte) 255 / maxSpeedMps * innerSpeedMps; //Inner speed as percentage of max speed scaled to 255 and cast to byte

  if (clockwise) {
    requiredEnc1Count = (360 * 2 * PI * (arcRadius + (axisLength / 2))) / wheelCirc;
    Wire.beginTransmission(addr);
    Wire.write(speed1Reg);
    Wire.write(MAX_SPEED);
    Wire.endTransmission();
    Wire.beginTransmission(addr);
    Wire.write(speed2Reg);
    Wire.write(innerSpeed);
    Wire.endTransmission();
  } else {
    requiredEnc1Count = (360 * 2 * PI * (arcRadius - (axisLength / 2))) / wheelCirc;
    Wire.beginTransmission(addr);
    Wire.write(speed1Reg);
    Wire.write(innerSpeed);
    Wire.endTransmission();
    Wire.beginTransmission(addr);
    Wire.write(speed2Reg);
    Wire.write(MAX_SPEED);
    Wire.endTransmission();
  }
  Serial.println((String)"Arc Turn RE1C " + requiredEnc1Count);
  
    if(timeBased){
      float midSpeed = (innerSpeedMps + maxSpeedMps) / 2;
      float distance = 2 * PI * arcRadius * angle / 360;
      long timeOn = 1000 * distance / midSpeed;
      delay(timeOn);// Delay turning motors off to achieve desired angle
      //Stop Motors
      Wire.beginTransmission(addr);
      Wire.write(speed1Reg);
      Wire.write(STOP_SPEED);
      Wire.endTransmission();
      Wire.beginTransmission(addr);
      Wire.write(speed2Reg);
      Wire.write(STOP_SPEED);
      Wire.endTransmission();
    }
  executing = false;
}

/**
   @brief Signal waypoint using buzzer and LED
   Flash and buzz for 1 second
*/
void signalWaypoint() {
  Serial.println("Waypoint reached");
  digitalWrite(buzzerPin, HIGH);
  digitalWrite(LEDPin, HIGH);
  delay(500);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(LEDPin, LOW);
}

/**
   @brief Drop a marker using the servo
 */
void dropMarker() {
  Serial.println((String) "Dropping marker from position " + dropperIndex);
  dropperServo.writeMicroseconds(servoPositions[dropperIndex]); //Set servo to start position
  dropperIndex++; //Increment dropper index
}

