#include <Servo.h>
#include <TimedAction.h>
#include <SimpleTimer.h>
#include <Wire.h>

const int SERVOMAX = 2000; // servo performance parameters, can identify the maximum and minimum pulse width
const int SERVOMIN = 500;  // Define the steering position name and number.
const int base = 0;
const int shoulder = 1;
const int elbow = 2;
const int wristflex = 3;
const int wristrot = 4;
const int gripper = 5;

const int stdDelay = 30; // Servo movement delay (in ms)
const int shortDelay = 20;
const int longDelay = 50;

const int maxServos = 6;  // Number of servos
const int centerPos = 90; // Servo position

unsigned long key_millis = 0;
unsigned long button_millis = 0;
int keyDelay = 100;   // Define the delay time
int buttonDelay = 50; // Define key delay
int thisServo = base; // define the starting motor

typedef struct
{                  // array frame structure
  byte startpos;   // initial angle
  byte minPos;     // minimum angle
  byte maxPos;     // maximum angle
  byte delaySpeed; // delay time
  byte curPos;     // steering gear current angle
} ServoPos;        // structure name

ServoPos servosPos[] = {
    // on the steering gear limit
    {90, 180, 0, stdDelay, 0},  // base initial 90, range 0 to 180 degrees.
    {95, 180, 10, stdDelay, 0}, // shoulder initial 95, range 10 to 180 degrees.
    {90, 180, 30, stdDelay, 0}, // elbow initial 90, range 30 to 180 degrees.
    {90, 90, 0, stdDelay, 0},   // wristflex initial 90, range 0 to 90 degrees.
    {90, 90, 0, stdDelay, 0},   // wristrot initial 90, range 0 to 90 degrees.
    {90, 90, 30, stdDelay, 0}   // gripper initial 90, range 30 to 90 degrees.
};

byte flag = 0;

// store the command received by the serial port
char buffer[8];
byte pointer = 0;
byte inByte = 0;

byte serv = 90;
int counter = 0;
int curServo = 0;
int sMove[] = {0, 90, 0};
int sAttach[] = {0, 0};

Servo servos[maxServos];

int destServoPos[maxServos];
int currentServoPos[maxServos];
void doServoFunc();
int hex2dec(byte c);
void Move(int servoNum, int servoPosition, int delayTime);
void resetServo();
void selfTest();

TimedAction servoMove[maxServos] = TimedAction(100, doServoFunc); // delay, delay time is statement time.
//SimpleTimer timer;

void setup()
{ // set

  Serial.begin(9600); // Serial port for debugging

  delay(200);
  Wire.begin();
  delay(500);

  for (int i = 0; i < maxServos; i++)
  {
    servosPos[i].curPos = servosPos[i].startpos;
    servos[i].initPulse(map(servosPos[i].curPos, 0, 180, SERVOMIN, SERVOMAX));
    servos[i].attach(i + 4, SERVOMIN, SERVOMAX);

    destServoPos[i] = servosPos[i].curPos;
    currentServoPos[i] = servosPos[i].curPos;
    servoMove[i].disable();
  }
  //timer.setInterval(5000, servoTestFunc);
}

void loop()
{
  for (int x = 0; x < maxServos; x++)
  {
    curServo = x;
    servoMove[x].check();
  }
  int dstangle;
  int timeDelay;

  if (Serial.available() > 0)
  {
    // read serial data
    inByte = Serial.read();
    delay(2);
    // If you read the # symbol, take the two bytes after the control command

    if (inByte == '#')
    { // # command is used to control the steering wheel rotation, format: # 015A Actuator 1 to 90 degrees.
      while (pointer < 6)
      { // read four numbers consecutively
        delay(2);
        buffer[pointer] = Serial.read(); // stored in the cache
        pointer++;                       // move the pointer down one
      }
      thisServo = hex2dec(buffer[1]);
      dstangle = hex2dec(buffer[3]) + hex2dec(buffer[2]) * 16;
      timeDelay = hex2dec(buffer[5]) + hex2dec(buffer[4]) * 16;
      pointer = 0; // reset the pointer
      flag = 1;
    }
    else
      else(inByte == '*')
      { // Format: ** Returns all servo data to '' interval, * 01 Returns the angle of the No. 1 servos
        delay(2);
        inByte = Serial.read();
        if ('*' == inByte)
        {
          // return the current angle of the steering gear, six bytes
          for (int i = 0; i < 6; i++)
          {
            Serial.print(servos[i].read());
            //Serial.print(servos[i].readMicroseconds());
            Serial.print(' ');
          }
          Serial.println();
        }
        else
        {
          buffer[0] = inByte;
          delay(2);
          buffer[1] = Serial.read();
          Serial.print(servos[hex2dec(buffer[1])].read());
          Serial.print(' ');
        }
      }
    else if (inByte == '!')
    {
      resetServo();
    }
    else if (inByte == '@')
    {
      delay(2);
      inByte = Serial.read();
      self test(trade - in);
    }

    if (1 == flag)
    {
      //Move(thisServo, dstangle, servosPos[thisServo].delaySpeed);
      Move(thisServo, dstangle, timeDelay);
      //      Serial.print(thisServo);
      //      Serial.print(dstangle);
      // Serial.print (timeDelay);
      flag = 0;
    }
  }

  //timer.run();
}

void writeServo()
{
  int servoNum = sMove[0];
  if (servoNum > = 0 && servoNum <= maxServos)
  {
    destServoPos[servoNum] = sMove[1];
    servoMove[servoNum].enable();
    servoMove[servoNum].setInterval(smove[2]);
  }
}

void setServoAttach()
{
  int servo = 1; // sAttach [0]
  int mode = 2;  // sAttach [1]
  if (servo >= 0 && servo <= maxServos)
  {
    if (mode == 1)
      servos[servo].attach(servo + 4);
    else
      servos[servo].detach();
  }
}

void doServoFunc()
{
  int x = curServo;
  if (destServoPos[x] == currentServoPos[x])
  {
    servos[x].write(currentServoPos[x]);
    servoMove[x].disable();
    return;
  }

  if (destServoPos[x] > currentServoPos[x])
    currentServoPos[x]++;
  else
    currentServoPos[x]--;

  servosPos[x].curPos = constrain(currentServoPos[x], servosPos[x].maxPos, servosPos[x].minPos);
  currentServoPos[x] = servosPos[x].curPos;
  servos[x].write(currentServoPos[x]);
  jointPos(x, currentServoPos[x]);
}

void Move(int servoNum, int servoPosition, int delayTime)
{                           // Servo drive instructions
  sMove[0] = servoNum;      // drive the steering gear number
  sMove[1] = servoPosition; // The target position of the steering gear
  sMove[2] = delayTime;     // The delay time of each steering gear movement
  writeServo();
}

void Attach(int servoNum, int servoMode)
{
  sAttach[0] = servoNum;
  sAttach[1] = servoMode;
}
void jointPos(byte t, byte pos)
{ // Defines two byte types of variables, t, pose.
  servos[t].write(pos);
}

void resetServo()
{
  //Serial.println("reset the servo");
  for (int j = 0; j < 6; j++)
  {
    //Serial.println(servosPos[j].startpos);
    Move(j, servosPos[j].startpos, servosPos[j].delaySpeed);
    //Serial.print(servosPos[j].startpos);
    // Serial.print(' ');
  }
}

int hex2dec(byte c)
{
  if (c >= '0' && c <= '9')
  {
    return c - '0';
  }
  else if (c >= 'A' && c <= 'F')
  {
    return c - 'A' + 10;
  }
}

void selfTest(byte testAngle)
{
  Serial.println("self testing");
  if (testAngle == '1')
  {
    for (int i = 0; i < 6; i++)
    {
      Move(i, servosPos[i].maxPos, servosPos[i].delaySpeed);
    }
  }
  else if (testAngle == '2')
  {
    for (int i = 0; i < 6; i++)
    {
      Move(i, servosPos[i].minPos, servosPos[i].delaySpeed);
    }
  }
}
