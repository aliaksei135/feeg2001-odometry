/**
  FEEG2001 Odometry Task
  Name: main

  @author Aliaksei Pilko
  @version 1.0
*/
#include <Wire.h>

/* Define I2C bus address, registers and relevant consts
 *  @see https://www.robot-electronics.co.uk/htm/md25i2c.htm
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
const byte MAX_SPEED = 0xFF;
const byte STOP_SPEED = 0x80;
const byte MAX_REVERSE_SPEED = 0x00;

//Useful Commands
const byte rstEnc = 0x20; //Reset Encoder Counts
const byte ds2secTo = 0x32; //Disable 2 second motor failsafe
const byte en2secTo = 0x33; //Enable 2 second motor failsafe

/* Define Physical Consts
 *  Units: Base SI
 */
const float wheelCirc = 0.1;
const float axisLength = 0.2;
const float maxSpeedMps = 17.0;

/** Define Pins */
const int buzzerPin = 3;
const int LEDPin = 13;

void setup() {
  Wire.begin(); //Start I2C Bus
  Serial.begin(9600); //Start Serial output at 9600 baud
  setMode(0x00); //Set mode 0
  setCmd(ds2secTo); //Disable motor timeout

  pinMode(buzzerPin, OUTPUT);
  pinMode(LEDPin, OUTPUT);
}

void loop() {

}

/**
 * @brief Sets Command Register to desired command 
 * @see https://www.robot-electronics.co.uk/htm/md25i2c.htm#command%20register
 * 
 * @param cmd desired command as byte
 */
void setCmd(byte cmd){
  Wire.beginTransmission(addr);
  Wire.write(cmdReg);
  Wire.write(cmd);
  Wire.endTransmission();
}

/**
 * @brief Sets Mode register to desired mode
 * @see https://www.robot-electronics.co.uk/htm/md25i2c.htm#mode
 * 
 * @param mode desired mode as byte
 */
void setMode(byte mode){
  Wire.beginTransmission(addr);
  Wire.write(modeReg);
  Wire.write(mode);
  Wire.endTransmission();
}

/**
 * @brief Reads value of Encoder 1
 * @see https://www.robot-electronics.co.uk/htm/md25i2c.htm#encoders
 * 
 * @return long value for encoder count
 */
long readEnc1(){
  Wire.beginTransmission(addr);
  Wire.write(enc1Reg);
  Wire.endTransmission();

  // Read Encoder bytes into 32 bit unsigned long
  Wire.requestFrom(addr, 4);
  while(Wire.available() < 4); 
  long poss = Wire.read(); 
  poss <<= 8;
  poss += Wire.read(); 
  poss <<= 8;
  poss += Wire.read(); 
  poss <<= 8;
  poss +=Wire.read();

  return poss;
}

/**
 * @brief Reads value of Encoder 2
 * @see https://www.robot-electronics.co.uk/htm/md25i2c.htm#encoders
 * 
 * @return long value for encoder count
 */
long readEnc2(){
  Wire.beginTransmission(addr);
  Wire.write(enc2Reg);
  Wire.endTransmission();

  // Read Encoder bytes into 32 bit unsigned long
  Wire.requestFrom(addr, 4);
  while(Wire.available() < 4); 
  long poss = Wire.read(); 
  poss <<= 8;
  poss += Wire.read(); 
  poss <<= 8;
  poss += Wire.read(); 
  poss <<= 8;
  poss +=Wire.read();

  return poss;
}

/**
 * @brief Sets same speed on both motors
 * @see https://www.robot-electronics.co.uk/htm/md25i2c.htm#speed1
 * Speed ranges from 0x00 (Full Reverse) to 0xFF (Full Forward)
 * Stop speed is 0x80
 * 
 * @param speedval desired speed as byte
 */
void setStraightSpeed(byte speedVal){
  Wire.beginTransmission(addr);
  Wire.write(speed1Reg);
  Wire.write(speedVal);
  Wire.write(speed2Reg);
  Wire.write(speedVal);
  Wire.endTransmission();
}

/**
 * @brief Turns on spot
 * 
 * @param degToTurn integer number of degrees to turn
 * @param clockwise true if arc should be followed in a CW direction, false otherwise
 */
void setPointTurn(int degToTurn, bool clockwise){
  float dist = degToTurn*(PI*axisLength/360); //Circular path distance each wheel must travel
  long timeOn = dist/maxSpeedMps;
  if(clockwise){
    //Start Motors
    Wire.beginTransmission(addr);
    Wire.write(speed1Reg);
    Wire.write(MAX_SPEED);
    Wire.write(speed2Reg);
    Wire.write(MAX_REVERSE_SPEED);
    Wire.endTransmission();
  }else{
    //Start Motors
    Wire.beginTransmission(addr);
    Wire.write(speed1Reg);
    Wire.write(MAX_REVERSE_SPEED);
    Wire.write(speed2Reg);
    Wire.write(MAX_SPEED);
    Wire.endTransmission();
  }
  
  //Stop Motors
  Wire.beginTransmission(addr);
  Wire.write(speed1Reg);
  Wire.write(STOP_SPEED);
  Wire.write(speed2Reg);
  Wire.write(STOP_SPEED);
  delay(timeOn);// Delay turning motors off to achieve desired angle
  Wire.endTransmission();
}

/**
 * @brief Turn in an arc of defined radius
 * 
 * @param arcRadius radius of arc followed by midpoint in metres
 * @param clockwise true if arc should be followed in a CW direction, false otherwise
 * @param distance distance to travel around arc in metres
 */
void setArcTurn(float arcRadius, bool clockwise, float distance){
  float innerSpeedMps = maxSpeedMps*(arcRadius-(axisLength/2))/(arcRadius+(axisLength/2)); //Inner motor speed in m/s
  byte innerSpeed = (byte) 255/maxSpeedMps * innerSpeedMps; //Inner speed as percentage of max speed scaled to 255 and cast to byte

  float midSpeed = (innerSpeedMps + maxSpeedMps)/2;
  long timeOn = distance/midSpeed;
  
  if(clockwise){
    Wire.beginTransmission(addr);
    Wire.write(speed1Reg);
    Wire.write(MAX_SPEED);
    Wire.write(speed2Reg);
    Wire.write(innerSpeed);
    Wire.endTransmission();
  }else{
    Wire.beginTransmission(addr);
    Wire.write(speed1Reg);
    Wire.write(innerSpeed);
    Wire.write(speed2Reg);
    Wire.write(MAX_SPEED);
    Wire.endTransmission();
  }

  //Stop Motors
  Wire.beginTransmission(addr);
  Wire.write(speed1Reg);
  Wire.write(STOP_SPEED);
  Wire.write(speed2Reg);
  Wire.write(STOP_SPEED);
  delay(timeOn);// Delay turning motors off to achieve desired angle
  Wire.endTransmission();
}

/**
 * @brief Signal waypoint using buzzer and LED
 * Flash and buzz for 1 second
 */
void signalWaypoint(){
  digitalWrite(buzzerPin, HIGH);
  digitalWrite(LEDPin, HIGH);
  delay(1000);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(LEDPin, LOW);
}

