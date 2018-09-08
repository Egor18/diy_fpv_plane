#include <Servo.h>
#include <Wire.h>

#define BAUD_RATE 57600

enum Channel
{
  ELEVATOR = 0,
  RUDDER   = 1,
  AILERON  = 2,
  THROTTLE = 3,
  CAMERA   = 4,
  OSD      = 5
};

#define OSD_WIRE_ADDR 0x08

enum OSDCommand
{
  OSD_OFF   = 51,
  OSD_ON    = 52,
  DIM_OFF   = 53,
  DIM_ON    = 54,
  ERR_INC   = 55,
  ERR_DEC   = 56,
  SET_HOME  = 57,
  LOST_CONN = 58
};

#define GetNum(byte) (((byte) & 0b11000000) >> 6)
#define GetData(byte) (((byte) & 0b00111111))

#define MIN_THROTTLE 0
#define MAX_THROTTLE 50
#define MIN_POS 0
#define MAX_POS 62

#define ELEVATOR_START 850
#define ELEVATOR_END 1550
#define RUDDER_START 550
#define RUDDER_END 1250
#define AILERON_LEFT_START 1275
#define AILERON_LEFT_END 1975
#define AILERON_RIGHT_START 1200
#define AILERON_RIGHT_END 1900
#define CAMERA_START 450
#define CAMERA_END 2050
#define ENGINE_START 900
#define ENGINE_END 2000

int byteCounter = 0;
unsigned char currentBytes[4];

Servo elevatorServo;
Servo rudderServo;
Servo aileronLeftServo;
Servo aileronRightServo;
Servo cameraServo;
Servo engineESC;

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  elevatorServo.attach(9);
  rudderServo.attach(8);
  aileronLeftServo.attach(7);
  aileronRightServo.attach(6);
  cameraServo.attach(5);
  engineESC.attach(2);

  // Set servos to the middle position
  elevatorServo.writeMicroseconds((ELEVATOR_START + ELEVATOR_END) / 2);
  rudderServo.writeMicroseconds((RUDDER_START + RUDDER_END) / 2);
  aileronLeftServo.writeMicroseconds((AILERON_LEFT_START + AILERON_LEFT_END) / 2);
  aileronRightServo.writeMicroseconds((AILERON_RIGHT_START + AILERON_RIGHT_END) / 2);
  cameraServo.writeMicroseconds((CAMERA_START + CAMERA_END) / 2);

  // Arm engine ESC
  engineESC.writeMicroseconds(ENGINE_START);

  Serial.begin(BAUD_RATE);
  Wire.begin();

  delay(500);
}

void SendByteToOSD(unsigned char data)
{
  Wire.beginTransmission(OSD_WIRE_ADDR);
  Wire.write(data);
  Wire.endTransmission();
}

void ProcessPackageData(unsigned char bytes[4])
{
  unsigned char channel             = GetData(bytes[0]);
  unsigned char channelConfirmation = GetData(bytes[1]);
  unsigned char data                = GetData(bytes[2]);
  unsigned char dataConfirmation    = GetData(bytes[3]);

  if ((channel != channelConfirmation || data != dataConfirmation) ||
      (channel > OSD) ||
      (channel == THROTTLE && data > MAX_THROTTLE) ||
      (channel == OSD && data > LOST_CONN) ||
      (channel != OSD && data > MAX_POS))
  {
    byteCounter = 0;
    SendByteToOSD(ERR_INC);
    return;
  }

  switch (channel)
  {
    case ELEVATOR:
      elevatorServo.writeMicroseconds(map(data, MIN_POS, MAX_POS, ELEVATOR_START, ELEVATOR_END));
      break;

    case RUDDER:
      rudderServo.writeMicroseconds(map(data, MIN_POS, MAX_POS, RUDDER_START, RUDDER_END));
      break;

    case AILERON:
      aileronLeftServo.writeMicroseconds(map(data, MIN_POS, MAX_POS, AILERON_LEFT_START, AILERON_LEFT_END));
      aileronRightServo.writeMicroseconds(map(data, MIN_POS, MAX_POS, AILERON_RIGHT_START, AILERON_RIGHT_END));
      break;

    case CAMERA:
      cameraServo.writeMicroseconds(map(data, MIN_POS, MAX_POS, CAMERA_START, CAMERA_END));
      break;

    case THROTTLE:
      engineESC.writeMicroseconds(map(data, MIN_THROTTLE, MAX_THROTTLE, ENGINE_START, ENGINE_END));
      SendByteToOSD(data);
      break;

    case OSD:
      SendByteToOSD(data);
      break;
  }
}

unsigned long lastDataReceivedTime = 0;

void loop()
{
  if (Serial.available() > 0)
  {
    lastDataReceivedTime = millis();
    int data = Serial.read();
    if (data == -1 || GetNum(data) != byteCounter)
    {
      byteCounter = 0;
      SendByteToOSD(ERR_INC);
    }
    else
    {
      currentBytes[byteCounter++] = (unsigned char) data;
      if (byteCounter == 4)
      {
        ProcessPackageData(currentBytes);
        byteCounter = 0;
      }
    }
  }

  if (millis() - lastDataReceivedTime > 5000 && lastDataReceivedTime != 0)
  {
    SendByteToOSD(LOST_CONN);
    engineESC.writeMicroseconds(map(0, MIN_THROTTLE, MAX_THROTTLE, ENGINE_START, ENGINE_END));
    SendByteToOSD(0);
  }
}
