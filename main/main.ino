/**
  FEEG2001 Odometry Task
  University of Southampton
  Name: main

  @author Aliaksei Pilko, Tom Perez-Diaz, Thomas Preskett
  @version 1.4
*/
#include <Wire.h> //For I2C communication
#include <Servo.h> //For simplification of servo interface

/* Define I2C bus address, registers and relevant consts
    @see https://www.robot-electronics.co.uk/htm/md25i2c.htm
*/
const byte addr = 0x58; //I2C Bus address of MD25

const byte cmdReg = 0x10; //Command Register
const byte modeReg = 0x0F; //Mode Register
const byte speed1Reg = 0x00; //Speed Register
const byte speed2Reg = 0x01; //Turn Register
const byte enc1Reg = 0x02; //Encoder 1
const byte enc2Reg = 0x06; //Encoder 2
const byte battReg = 0x0A; //Battery Volts Register
//const byte cur1 = 0x0B; //Current to motor 1
//const byte cur2 = 0x0C; //Current to motor 2

//Common Speed Values
//Limit speeds so momentum of robot can be minimised when stopping
const byte MAX_SPEED = 0xB4; //180
const byte STOP_SPEED = 0x80; //128, difference either side is DEC52/0x34 
const byte MAX_REVERSE_SPEED = 0x4C; //76

//Useful Command register values
const byte rstEnc = 0x20; //Reset Encoder Counts
const byte ds2secTo = 0x32; //Disable 2 second motor failsafe
const byte en2secTo = 0x33; //Enable 2 second motor failsafe

/* Define Physical Constants
    Units: Base SI
*/
const float wheelCirc = 0.1 * PI;
const float axisLength = 0.255;
const float maxSpeedMps = 0.5;
const short encCountPerTurn = 360;

//Calibration factors for each segment
const float straightCalib = 0.82;
const float pointTurnCalib = 0.75;
const float arcTurnCalib = 1.03;

/* Define path to follow
*/
//Enumerates the possible motion states of the robot
enum MotionType {
  Straight,
  PointTurn,
  Arc,
  Stop
};

//Struct representing each segment of the course
struct Path {
  MotionType motion; //Type of motion in segment
  float val1; //Distance for Straight segments, Angle for point turns or radius for arcs
  float val2; //Used only for arc angle
};

//Array of Path structs representing all the segments in the course
//Easily extensible course
Path course[28] = {
  //Start Point 13
  {Straight, 0.340, NULL},
  {PointTurn, 90.0, NULL}, //Point 1
  {Straight, 0.250, NULL},
  //Drop point
  {PointTurn, -46, NULL}, //Point 2/12 //Angle not defined well
  {Straight, .34, NULL}, //Distance not defined well
  {PointTurn, 156, NULL}, //Point 11 //Angle not defined well
  {Arc, 0.180, -270},
  //Drop Point
  {PointTurn, -87, NULL}, //Point 10
  {Straight, 0.160, NULL},
  {PointTurn, 164, NULL}, //Point 9
  {Straight, 0.683, NULL}, //Distance not defined well
  {PointTurn, 146, NULL}, //Point 8
  {Straight, 0.395, NULL},
  {PointTurn, -96, NULL}, //Point 5
  {Straight, 0.400, NULL},
  //Drop Point
  {PointTurn, -96, NULL}, //Point 6
  {Straight, 0.410, NULL},
  {PointTurn, -96, NULL}, //Point 7
  {Straight, 0.400, NULL},
  //Drop Point
  {PointTurn, 94, NULL}, //Point 8
  {Straight, 0.235, NULL},
  {PointTurn, -90, NULL}, //Point 4
  {Arc, 0.260, -90},
  {PointTurn, 99, NULL}, //Point 3
  {Straight, 0.500, NULL},
  {PointTurn, 23, NULL}, //Point 2/12 //Angle not defined well
  {Straight, 0.428, NULL}
};

short courseIndex; //An index into the current position through the course array. Can be short as expected range very small
boolean executing; //Blocking boolean to prevent more than one path segment being active/called. Stops loop() "outpacing" other executing methods
long requiredEnc1Count; //Desired encoder 1 count for each path segment. Needs to be long as enc count can be up to 32bit
MotionType currentMotion; //Current motion type the robot is undergoing
bool timeBased = false; //Change turn modes to measure based on time rather than encoder count if needed

/* Dropper definitions */
Servo dropperServo; //Servo object
short servoPositions[5] = {550, 950, 1375, 1800, 2500}; //Servo turn positions
short dropperIndex; //Index into dropPoints with current drop point
short dropPoints[5] = {3, 7, 11, 15, 22}; //Indexes into course at which a marker must be dropped

/* Define Pins */
const short buzzerPin = 3;
const short LEDPin = 12;
const short servoPin = 9;

void setup() {
  Wire.begin(); //Start I2C Bus
//  Serial.begin(9600); //Start Serial output at 9600 baud
  setMode(0); //Set mode 0
  Serial.println((String) "Battery Voltage " + readBatteryVolts());

  //Initialise robot state
  setCmd(rstEnc); //Reset encoder counts at start
  courseIndex = 0; //Set first course segment
  dropperIndex = 0; //Set first dropper index
  executing = false; //Set not executing
  requiredEnc1Count = 0; //Set initial required encoder 1 count
  currentMotion = Stop; //Set current motion

  //Set pin modes
  pinMode(buzzerPin, OUTPUT); //Set buzzer as output
  pinMode(LEDPin, OUTPUT); //Set LED as output

  //Servo setup
  dropperServo.attach(servoPin); //Attach servo to correct pin
  dropperServo.writeMicroseconds(750); //Set servo to start position
  delay(2000); //Start delay to move finger away from switch
}

 /*
   Follow finite state machine type architecture
   @see https://en.wikipedia.org/wiki/Finite-state_machine
  */
void loop() {
  //Minimise blocking on loop() where possible
  if (!executing) {
    //Check if robot is currently stopped
    if (currentMotion == Stop) {
      Serial.println((String)"Executing course idx " + courseIndex);
      setCmd(rstEnc); //Reset encoders ready for next segment
      executing = true; //Block any other execution on loop()
      //Check if drop point reached
      if (courseIndex == dropPoints[dropperIndex]) {
        dropMarker();
        delay(100); //Delay to allow marker to drop properly before moving
      }
      executePath(course[courseIndex]); //Execute next segment in course
    } else {
      //Check if looking for positive or negative encoder 1 count
      if(requiredEnc1Count < 0){
        Serial.println("Neg RE1C");
        if (readEnc1() <= requiredEnc1Count) {
          executing = true; //Block any other execution on loop()
          Serial.println(requiredEnc1Count);
          Serial.println((String)"Encoder 1 count " + readEnc1() + " reached");
          Serial.println((String)"Encoder 2 count " + readEnc2() + " reached");
          setStop(); //Stop robot
          requiredEnc1Count = 0; //Reset Encoder count to prevent race conditions with loop() running again
          delay(200); //Small delay to stabilise motion
          executing = false; //Unblock loop()
        }
      }else{
        Serial.println("Pos RE1C");
        if (readEnc1() >= requiredEnc1Count) {
          executing = true; //Block any other execution on loop()
          Serial.println(requiredEnc1Count);
          Serial.println((String)"Encoder 1 count " + readEnc1() + " reached");
          Serial.println((String)"Encoder 2 count " + readEnc2() + " reached");
          setStop(); //Stop robot
          requiredEnc1Count = 0; //Reset Encoder count to prevent race conditions with loop() running again
          delay(200); //Small delay to stabilise motion
          executing = false; //Unblock loop()
        }
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
  //Establish type of motion in segment
  switch (path.motion) {
    case Straight:
      requiredEnc1Count = (path.val1 / wheelCirc) * encCountPerTurn * straightCalib; //Calculate required encoder 1 count
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
  if (courseIndex >= 29) { //check if finished course
    Serial.println("Finished Course");
    executing = true; //Block any other execution in loop()
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

  // Read Encoder bytes into 32 bit long 8 bits at a time and bitshift up
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

  // Read Encoder bytes into 32 bit long 8 bits at a time and bitshift up
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
   @brief Reads battry voltage value
   @see https://www.robot-electronics.co.uk/htm/md25i2c.htm#battery%20volts

   @return float value for voltage 
 */
 float readBatteryVolts(){
  Wire.beginTransmission(addr);
  Wire.write(battReg);
  Wire.endTransmission();

  Wire.requestFrom(addr, 4);
  int batt = Wire.read();
  return batt / 10; //Divide by 10 as valeu given in tenths of volts
 }

/**
   @brief Sets same speed on both motors
   @see https://www.robot-electronics.co.uk/htm/md25i2c.htm#speed1
   Speed ranges from 0x00 (Full Reverse) to 0xFF (Full Forward)

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
  executing = false; //Unblock loop()
}

/*
  @brief Sets both motor to stop
  @see https://www.robot-electronics.co.uk/htm/md25i2c.htm#speed1
  Stop speed is 0x80
 */
void setStop(){
  Serial.println("Stopped");
  Wire.beginTransmission(addr);
  Wire.write(speed2Reg);
  Wire.write(STOP_SPEED);
  Wire.endTransmission();
  Wire.beginTransmission(addr);
  Wire.write(speed1Reg);
  Wire.write(STOP_SPEED);
  Wire.endTransmission();
  requiredEnc1Count = 0; //Reset Encoder 1 count to prevent race conditions
  currentMotion = Stop; //Set current motion after stopping
}

/**
   @brief Turn on the spot

   @param degToTurn integer number of degrees to turn
   @param clockwise true if arc should be followed in a CW direction, false otherwise
*/
void setPointTurn(int degToTurn, bool clockwise) {
  signalWaypoint(); //Indicate waypoint reached
  Serial.println((String) "Point Turn " + degToTurn + "deg direction clockwise " + clockwise);
  float dist = pointTurnCalib * degToTurn * (PI * axisLength / encCountPerTurn); //Circular path distance each wheel must travel
  if (clockwise) {
    //Calibration factor added as motor 1 systematically stops after motor 2, see setStop()
    requiredEnc1Count =  0.981 * dist * encCountPerTurn / wheelCirc; //Required Encoder 1 count
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
    //Calibration factor added as motor 1 systematically stops after motor 2, see setStop()
    //Motor 1 in reverse for CCW turns so Encoder 1 count is negative
    requiredEnc1Count = -(1.04 * dist * encCountPerTurn / wheelCirc); //Required Encoder 1 count
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

  //Option to make time based instead of encoder count based
  if (timeBased) {
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
  executing = false; //Unblock loop()
}

/**
   @brief Turn in an arc of defined radius

   @param arcRadius radius of arc followed by midpoint in metres
   @param clockwise true if arc should be followed in a CW direction, false otherwise
   @param angle angle to sweep in arc
*/
void setArcTurn(float arcRadius, bool clockwise, int angle) {
  Serial.println((String) "Arc Turn " + arcRadius + "m radius, direction clockwise " + clockwise + " ,angle " + angle);
  //Set outer wheel to max speed and calulate inner wheel speed from that
  float innerSpeedMps = maxSpeedMps * (arcRadius - (axisLength / 2)) / (arcRadius + (axisLength / 2)); //Inner motor speed in m/s
  int innerSpeedOffset = (53 / maxSpeedMps * innerSpeedMps); //Inner speed as percentage of max speed scaled to 53
  byte innerSpeed = 128 + innerSpeedOffset; //Add offset to stop speed so motion is forward as using mode 0

  if (clockwise) {
    requiredEnc1Count = (2 * arcTurnCalib * PI) * (arcRadius + (axisLength / 2)) * (angle / 360) * (encCountPerTurn / wheelCirc); //Calculate encoder counts for enc 1
    Wire.beginTransmission(addr);
    Wire.write(speed1Reg);
    Wire.write(MAX_SPEED);
    Wire.endTransmission();
    Wire.beginTransmission(addr);
    Wire.write(speed2Reg);
    Wire.write(innerSpeed);
    Wire.endTransmission();
  } else {
    requiredEnc1Count = (2 * arcTurnCalib * PI * (arcRadius - (axisLength / 2)) * angle * encCountPerTurn) / (wheelCirc * 360); //Calculate encoder counts for enc 1
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

  //Option to make time based instead of encoder count based
  if (timeBased) {
    float midSpeed = (innerSpeedMps + maxSpeedMps) / 2; //Calculate speed of centre point
    float distance = 2 * PI * arcRadius * angle / encCountPerTurn; //Calculate distance to travel of centre point
    long timeOn = 1000 * distance / midSpeed; //Calculate time required to be on in ms
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
  executing = false; //Unblock loop()
}

/**
   @brief Signal waypoint using buzzer and LED
   Flash and buzz for 1 second
*/
void signalWaypoint() {
  Serial.println("Waypoint reached");
  digitalWrite(buzzerPin, HIGH);
  digitalWrite(LEDPin, HIGH);
  delay(500); //Keep buzzer and LED on for half second
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

