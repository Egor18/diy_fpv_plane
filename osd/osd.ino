
// Based on DiyOSD written by Dennis Frie

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <Wire.h>

#include "symbols.h"
#include "utils.h"
#include "tiny_gps.h"

TinyGPS gps;
unsigned long prevTime = 0;
unsigned long flightTime = 0;
bool homeSet = false;
float homeLat;
float homeLon;
long homeAltitude;
const int delayTicks = 100; // GPS startup delay (~20 sec)
int currentDelayTicks = 0;
int errors = 0;

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

#define MIN_THROTTLE 0
#define MAX_THROTTLE 50

volatile int line = 0;

#define TOP_LINE_1 29
#define TOP_LINE_2 39
#define TOP_LINE_3 49
#define TOP_LINE_4 59
#define BOTTOM_LINE_1 284
#define BOTTOM_LINE_2 294
#define LINE_HEIGHT 8

volatile int ThrottleText[9] =   { _T, _H, _R, _COLON, _SPACE, _DASH, _DASH, _DASH, _PERCENT };
volatile int SpeedText[11] =     { _S, _P, _D, _COLON, _SPACE, _DASH, _DASH, _DASH, _K, _P, _H };
volatile int AltitudeText[9] =   { _A, _L, _T, _COLON, _SPACE, _DASH, _DASH, _DASH, _M };
volatile int HomeText[9] =       { _H, _O, _M, _E, _COLON, _DASH, _DASH, _DASH, _M };
volatile int Battery1Text[10] =  { _B, _A, _T, _1, _COLON, _0, _0, _DOT, _0, _V };
volatile int Battery2Text[10] =  { _B, _A, _T, _2, _COLON, _0, _0, _DOT, _0, _V };
volatile int TimeText[10] =      { _T, _I, _M, _E, _COLON, _DASH, _DASH, _COLON, _DASH, _DASH };
volatile int HeadingText[7] =    { _H, _D, _G, _COLON, _DASH, _DASH, _DASH };
volatile int SatellitesText[7] = { _S, _A, _T, _COLON, _DASH, _DASH, _DASH };
volatile int ErrorsText[7] =     { _E, _R, _R, _COLON, _0, _0, _0 };

volatile unsigned char dimOnMask  = 0b00000001;
volatile unsigned char dimOffMask = 0b11111110;
#define DimOn  DDRB |= dimOnMask;
#define DimOff DDRB &= dimOffMask;

void EnableOSD()
{
  ACSR |= (1 << ACIE); // Enable interrupt
}

void DisableOSD()
{
  ACSR &= ~(1 << ACIE);  // Disable interrupt
}

void EnableDim()
{
  dimOnMask = 0b00000001;
  dimOffMask = 0b11111110;
}

void DisableDim()
{
  dimOnMask = 0b00000000;
  dimOffMask = 0b11111111;
}

void setup()
{
  pinMode(6, INPUT);  // Analog comparator AIN0
  pinMode(7, INPUT);  // Analog comparator AIN1

  // Init SPI output (will just set all as output)
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);

  // ADC to measure a battery level
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  // Set SPI:
  SPCR = (1 << SPE) |  //Enable SPI
         (1 << MSTR) | // Set as master
         (0 << SPR1) | // Max speed
         (0 << SPR0) | // Max speed
         (1 << CPOL) | // We dont want it to idle high
         (1 << CPHA);

  // SPI double speed - we want 8 mhz output.
  SPSR = (1 << SPI2X);

  // Disable interrupts - necessary when using Arduino ide to avoid default
  // interrupts used for the delay(), millis() etc.
  TIMSK0 = 0;
  TIMSK1 = 0;
  TIMSK2 = 0;

  // Init analog comparator to register new line and frame
  ADCSRB = 0b00000001; // Set analog comparator mode

  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (0 << ADPS2) | (1 << ADPS2);

  // Set Analog comparator interrupt interrupt enable (interrupt on rising output edge)
  ACSR = (1 << ACIE) | (1 << ACIS1) | (1 << ACIS0);

  // Setup timer 1 with max speed. Used to detect if it's a new line or frame.
  TCCR1B = (0 << CS12) | //Prescale 1
           (0 << CS11) | //Prescale 1
           (1 << CS10) | //Prescale 1
           (0 << WGM12); // CTC mode

  TCNT1 = 0;

  Serial.begin(9600); // GPS

  // Connection to Flight Controller
  Wire.begin(OSD_WIRE_ADDR);
  Wire.onReceive(WireEventHandler);
}

// The analog comparator interrupt - contains all line writing etc.
// The code will be executed around 15000 times a second, and the timing is pretty important.
ISR(ANALOG_COMP_vect)
{
  // Reset counter
  TCNT1 = 0;

  // Wait for the sync to go high again
  while ((ACSR & 0b00100000) == 0b00100000) {}

  // If the counter has exceeded the "new line sync time", it's a new frame.
  // In that case, we just need to reset the line-counter.
  if (TCNT1 > 75)
  {
    line = 0;
  }

  // Current symbol line count
  int symbolLine = 0;

  if (line > TOP_LINE_1 && line <= TOP_LINE_1 + LINE_HEIGHT)
  {
    symbolLine = line - TOP_LINE_1 - 1;
    _delay_loop_1(32);

     // Loop unrolling
     DimOn;
     SPDR = SYMBOLS[ThrottleText[0] + symbolLine];
     delay8;
     SPDR = SYMBOLS[ThrottleText[1] + symbolLine];
     delay8;
     SPDR = SYMBOLS[ThrottleText[2] + symbolLine];
     delay8;
     SPDR = SYMBOLS[ThrottleText[3] + symbolLine];
     delay8;
     SPDR = SYMBOLS[ThrottleText[4] + symbolLine];
     delay8;
     SPDR = SYMBOLS[ThrottleText[5] + symbolLine];
     delay8;
     SPDR = SYMBOLS[ThrottleText[6] + symbolLine];
     delay8;
     SPDR = SYMBOLS[ThrottleText[7] + symbolLine];
     delay13;
     SPDR = SYMBOLS[ThrottleText[8] + symbolLine];
     delay15;
     DimOff;
  }
  else if (line > TOP_LINE_2 && line <= TOP_LINE_2 + LINE_HEIGHT)
  {
    symbolLine = line - TOP_LINE_2 - 1;
    _delay_loop_1(25);
    delay2;

    DimOn;
    SPDR = SYMBOLS[SpeedText[0] + symbolLine];
    delay8;
    SPDR = SYMBOLS[SpeedText[1] + symbolLine];
    delay8;
    SPDR = SYMBOLS[SpeedText[2] + symbolLine];
    delay8;
    SPDR = SYMBOLS[SpeedText[3] + symbolLine];
    delay8;
    SPDR = SYMBOLS[SpeedText[4] + symbolLine];
    delay8;
    SPDR = SYMBOLS[SpeedText[5] + symbolLine];
    delay8;
    SPDR = SYMBOLS[SpeedText[6] + symbolLine];
    delay8;
    SPDR = SYMBOLS[SpeedText[7] + symbolLine];
    delay13;
    SPDR = SYMBOLS[SpeedText[8] + symbolLine];
    delay8;
    SPDR = SYMBOLS[SpeedText[9] + symbolLine];
    delay8;
    SPDR = SYMBOLS[SpeedText[10] + symbolLine];
    delay15;
    DimOff;
  }
  else if (line > TOP_LINE_3 && line <= TOP_LINE_3 + LINE_HEIGHT)
  {
    symbolLine = line - TOP_LINE_3 - 1;
    _delay_loop_1(19);

    DimOn;
    SPDR = SYMBOLS[AltitudeText[0] + symbolLine];
    delay8;
    SPDR = SYMBOLS[AltitudeText[1] + symbolLine];
    delay8;
    SPDR = SYMBOLS[AltitudeText[2] + symbolLine];
    delay8;
    SPDR = SYMBOLS[AltitudeText[3] + symbolLine];
    delay8;
    SPDR = SYMBOLS[AltitudeText[4] + symbolLine];
    delay8;
    SPDR = SYMBOLS[AltitudeText[5] + symbolLine];
    delay8;
    SPDR = SYMBOLS[AltitudeText[6] + symbolLine];
    delay8;
    SPDR = SYMBOLS[AltitudeText[7] + symbolLine];
    delay13;
    SPDR = SYMBOLS[AltitudeText[8] + symbolLine];
    delay15;
    DimOff;
  }
  else if (line > TOP_LINE_4 && line <= TOP_LINE_4 + LINE_HEIGHT)
  {
    symbolLine = line - TOP_LINE_4 - 1;
    _delay_loop_1(12);
    delay2;

    DimOn;
    SPDR = SYMBOLS[HomeText[0] + symbolLine];
    delay8;
    SPDR = SYMBOLS[HomeText[1] + symbolLine];
    delay8;
    SPDR = SYMBOLS[HomeText[2] + symbolLine];
    delay8;
    SPDR = SYMBOLS[HomeText[3] + symbolLine];
    delay8;
    SPDR = SYMBOLS[HomeText[4] + symbolLine];
    delay8;
    SPDR = SYMBOLS[HomeText[5] + symbolLine];
    delay8;
    SPDR = SYMBOLS[HomeText[6] + symbolLine];
    delay8;
    SPDR = SYMBOLS[HomeText[7] + symbolLine];
    delay13;
    SPDR = SYMBOLS[HomeText[8] + symbolLine];
    delay15;
    DimOff;
  }
  else if (line > BOTTOM_LINE_1 && line <= BOTTOM_LINE_1 + LINE_HEIGHT)
  {
    symbolLine = line - BOTTOM_LINE_1 - 1;
    _delay_loop_1(8);

    DimOn;
    SPDR = SYMBOLS[Battery1Text[0] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery1Text[1] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery1Text[2] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery1Text[3] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery1Text[4] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery1Text[5] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery1Text[6] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery1Text[7] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery1Text[8] + symbolLine];
    delay13;
    SPDR = SYMBOLS[Battery1Text[9] + symbolLine];
    delay15;
    DimOff;

    _delay_loop_1(40);

    DimOn;
    SPDR = SYMBOLS[TimeText[0] + symbolLine];
    delay8;
    SPDR = SYMBOLS[TimeText[1] + symbolLine];
    delay8;
    SPDR = SYMBOLS[TimeText[2] + symbolLine];
    delay8;
    SPDR = SYMBOLS[TimeText[3] + symbolLine];
    delay8;
    SPDR = SYMBOLS[TimeText[4] + symbolLine];
    delay8;
    SPDR = SYMBOLS[TimeText[5] + symbolLine];
    delay8;
    SPDR = SYMBOLS[TimeText[6] + symbolLine];
    delay8;
    SPDR = SYMBOLS[TimeText[7] + symbolLine];
    delay8;
    SPDR = SYMBOLS[TimeText[8] + symbolLine];
    delay8;
    SPDR = SYMBOLS[TimeText[9] + symbolLine];
    delay15;
    DimOff;

    _delay_loop_1(20);

    DimOn;
    SPDR = SYMBOLS[SatellitesText[0] + symbolLine];
    delay8;
    SPDR = SYMBOLS[SatellitesText[1] + symbolLine];
    delay8;
    SPDR = SYMBOLS[SatellitesText[2] + symbolLine];
    delay8;
    SPDR = SYMBOLS[SatellitesText[3] + symbolLine];
    delay8;
    SPDR = SYMBOLS[SatellitesText[4] + symbolLine];
    delay8;
    SPDR = SYMBOLS[SatellitesText[5] + symbolLine];
    delay8;
    SPDR = SYMBOLS[SatellitesText[6] + symbolLine];
    delay15;
    DimOff;
  }
  else if (line > BOTTOM_LINE_2 && line <= BOTTOM_LINE_2 + LINE_HEIGHT)
  {
    symbolLine = line - BOTTOM_LINE_2 - 1;
    _delay_loop_1(2);

    DimOn;
    SPDR = SYMBOLS[Battery2Text[0] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery2Text[1] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery2Text[2] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery2Text[3] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery2Text[4] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery2Text[5] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery2Text[6] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery2Text[7] + symbolLine];
    delay8;
    SPDR = SYMBOLS[Battery2Text[8] + symbolLine];
    delay13;
    SPDR = SYMBOLS[Battery2Text[9] + symbolLine];
    delay15;
    DimOff;

    _delay_loop_1(47);

    DimOn;
    SPDR = SYMBOLS[HeadingText[0] + symbolLine];
    delay8;
    SPDR = SYMBOLS[HeadingText[1] + symbolLine];
    delay8;
    SPDR = SYMBOLS[HeadingText[2] + symbolLine];
    delay8;
    SPDR = SYMBOLS[HeadingText[3] + symbolLine];
    delay8;
    SPDR = SYMBOLS[HeadingText[4] + symbolLine];
    delay8;
    SPDR = SYMBOLS[HeadingText[5] + symbolLine];
    delay8;
    SPDR = SYMBOLS[HeadingText[6] + symbolLine];
    delay15;
    delay9;
    DimOff;

    _delay_loop_1(28);
    delay2;

    DimOn;
    SPDR = SYMBOLS[ErrorsText[0] + symbolLine];
    delay8;
    SPDR = SYMBOLS[ErrorsText[1] + symbolLine];
    delay8;
    SPDR = SYMBOLS[ErrorsText[2] + symbolLine];
    delay8;
    SPDR = SYMBOLS[ErrorsText[3] + symbolLine];
    delay8;
    SPDR = SYMBOLS[ErrorsText[4] + symbolLine];
    delay8;
    SPDR = SYMBOLS[ErrorsText[5] + symbolLine];
    delay8;
    SPDR = SYMBOLS[ErrorsText[6] + symbolLine];
    delay15;
    delay2;
    DimOff;
  }

  line++;
}

// throttle: [0..100]
void SetThrottleValue(int throttle)
{
  ThrottleText[5] = NumToSymIndex(Digit2(throttle));
  ThrottleText[6] = NumToSymIndex(Digit1(throttle));
  ThrottleText[7] = NumToSymIndex(Digit0(throttle));
}

void ResetThrottleValue()
{
  ThrottleText[5] = _DASH;
  ThrottleText[6] = _DASH;
  ThrottleText[7] = _DASH;
}

// speed: [0..999]
void SetSpeedValue(int speed)
{
  SpeedText[5] = NumToSymIndex(Digit2(speed));
  SpeedText[6] = NumToSymIndex(Digit1(speed));
  SpeedText[7] = NumToSymIndex(Digit0(speed));
}

void ResetSpeedValue()
{
  SpeedText[5] = _DASH;
  SpeedText[6] = _DASH;
  SpeedText[7] = _DASH;
}

// altitude: [-999..999]
void SetAltitudeValue(int altitude)
{
  AltitudeText[4] = (altitude < 0) ? _DASH : _SPACE;
  altitude = abs(altitude);
  AltitudeText[5] = NumToSymIndex(Digit2(altitude));
  AltitudeText[6] = NumToSymIndex(Digit1(altitude));
  AltitudeText[7] = NumToSymIndex(Digit0(altitude));
}

void ResetAltitudeValue()
{
  AltitudeText[5] = _DASH;
  AltitudeText[6] = _DASH;
  AltitudeText[7] = _DASH;
}

// home: [0..999]
void SetHomeValue(int home)
{
  HomeText[5] = NumToSymIndex(Digit2(home));
  HomeText[6] = NumToSymIndex(Digit1(home));
  HomeText[7] = NumToSymIndex(Digit0(home));
}

void ResetHomeValue()
{
  HomeText[5] = _DASH;
  HomeText[6] = _DASH;
  HomeText[7] = _DASH;
}

// integer: [0..99], fraction: [0..9]
void SetBattery1Value(int integer, int fraction)
{
  Battery1Text[5] = NumToSymIndex(Digit1(integer));
  Battery1Text[6] = NumToSymIndex(Digit0(integer));
  Battery1Text[8] = NumToSymIndex(Digit0(fraction));
}

// integer: [0..99], fraction: [0..9]
void SetBattery2Value(int integer, int fraction)
{
  Battery2Text[5] = NumToSymIndex(Digit1(integer));
  Battery2Text[6] = NumToSymIndex(Digit0(integer));
  Battery2Text[8] = NumToSymIndex(Digit0(fraction));
}

// minutes: [0..99], seconds: [0..59]
void SetTimeValue(int minutes, int seconds)
{
  TimeText[5] = NumToSymIndex(Digit1(minutes));
  TimeText[6] = NumToSymIndex(Digit0(minutes));
  TimeText[8] = NumToSymIndex(Digit1(seconds));
  TimeText[9] = NumToSymIndex(Digit0(seconds));
}

void ResetTimeValue()
{
  TimeText[5] = _DASH;
  TimeText[6] = _DASH;
  TimeText[8] = _DASH;
  TimeText[9] = _DASH;
}

// heading: [0..359]
void SetHeadingValue(int heading)
{
  HeadingText[4] = NumToSymIndex(Digit2(heading));
  HeadingText[5] = NumToSymIndex(Digit1(heading));
  HeadingText[6] = NumToSymIndex(Digit0(heading));
}

void ResetHeadingValue()
{
  HeadingText[4] = _DASH;
  HeadingText[5] = _DASH;
  HeadingText[6] = _DASH;
}

// satellites: [0..999]
void SetSatellitesValue(int satellites)
{
  SatellitesText[4] = NumToSymIndex(Digit2(satellites));
  SatellitesText[5] = NumToSymIndex(Digit1(satellites));
  SatellitesText[6] = NumToSymIndex(Digit0(satellites));
}

void ResetSatellitesValue()
{
  SatellitesText[4] = _DASH;
  SatellitesText[5] = _DASH;
  SatellitesText[6] = _DASH;
}

// errors: [0..999]
void SetErrorsValue(int errors)
{
  ErrorsText[4] = NumToSymIndex(Digit2(errors));
  ErrorsText[5] = NumToSymIndex(Digit1(errors));
  ErrorsText[6] = NumToSymIndex(Digit0(errors));
}

void SetErrorsLostConnection()
{
  ErrorsText[4] = _L;
  ErrorsText[5] = _S;
  ErrorsText[6] = _T;
}

void WireEventHandler(int count)
{
  if (Wire.available() > 0)
  {
    int data = Wire.read();
    if (data >= MIN_THROTTLE && data <= MAX_THROTTLE)
    {
      SetThrottleValue(data * 2);
    }
    else if (data == OSD_OFF)
    {
      DisableOSD();
    }
    else if (data == OSD_ON)
    {
      EnableOSD();
    }
    else if (data == DIM_OFF)
    {
      DisableDim();
    }
    else if (data == DIM_ON)
    {
      EnableDim();
    }
    else if (data == ERR_INC)
    {
      errors++;
      if (errors > 999)
      {
        errors = 0;
      }
      SetErrorsValue(errors);
    }
    else if (data == ERR_DEC)
    {
      errors--;
      if (errors < 0)
      {
        errors = 0;
      }
      SetErrorsValue(errors);
    }
    else if (data == SET_HOME)
    {
      homeSet = false;
      ResetHomeValue();
    }
    else if (data == LOST_CONN)
    {
      SetErrorsLostConnection();
    }
  }
}

void ProcessADC()
{
  int voltage1 = analogRead(A1);
  const int K1 = 55; // 0.055
  int integer1 = voltage1 * K1 / 1000;
  int tmp1 = voltage1 * K1 % 1000;
  int fraction1 = tmp1 / 100 + ((tmp1 / 10 % 10 >= 5) ? 1 : 0);
  SetBattery1Value(integer1, fraction1);

  int voltage2 = analogRead(A0);
  const int K2 = 58; // 0.058
  int integer2 = voltage2 * K2 / 1000;
  int tmp2 = voltage2 * K2 % 1000;
  int fraction2 = tmp2 / 100 + ((tmp2 / 10 % 10 >= 5) ? 1 : 0);
  SetBattery2Value(integer2, fraction2);
}

void ProcessGPS()
{
  // Smart delay
  for (volatile int i = 0; i < 20000; i++)
  {
    while (Serial.available())
    {
      gps.encode(Serial.read());
    }
  }

  bool delayDone = (currentDelayTicks >= delayTicks);

  long altitude = gps.altitude();
  if (altitude != TinyGPS::GPS_INVALID_ALTITUDE)
  {
    if (!homeSet && delayDone)
    {
      homeAltitude = altitude / 100;
    }

    if (homeSet)
    {
      SetAltitudeValue(altitude / 100 - homeAltitude);
    }
  }
  else
  {
    ResetAltitudeValue();
  }

  float speed = gps.f_speed_kmph();
  if (speed != TinyGPS::GPS_INVALID_SPEED)
  {
    SetSpeedValue((int)(speed + 0.5f));
  }
  else
  {
    ResetSpeedValue();
  }

  float lat, lon;
  gps.f_get_position(&lat, &lon, NULL);
  if (lat != TinyGPS::GPS_INVALID_F_ANGLE && lon != TinyGPS::GPS_INVALID_F_ANGLE)
  {
    if (!homeSet && delayDone)
    {
      homeLat = lat;
      homeLon = lon;
      homeSet = true;
    }

    if (homeSet)
    {
      int dist = (int)(TinyGPS::distance_between(lat, lon, homeLat, homeLon) + 0.5f);
      SetHomeValue(dist);
    }
  }
  else
  {
    ResetHomeValue();
  }

  unsigned long time;
  gps.get_datetime(NULL, &time, NULL);
  if (time != TinyGPS::GPS_INVALID_TIME)
  {
    if (time != prevTime)
    {
      flightTime++;
      prevTime = time;
    }
    SetTimeValue(flightTime / 60, flightTime % 60);
  }
  else
  {
    ResetTimeValue();
  }

  unsigned long course = gps.course();
  if (course != TinyGPS::GPS_INVALID_ANGLE)
  {
    SetHeadingValue(course / 100);
  }
  else
  {
    ResetHeadingValue();
  }

  unsigned short satellites = gps.satellites();
  if (satellites != TinyGPS::GPS_INVALID_SATELLITES)
  {
    SetSatellitesValue(satellites);
    if (!delayDone)
    {
      currentDelayTicks++;
    }
  }
  else
  {
    ResetSatellitesValue();
  }
}

void loop()
{
  ProcessADC();
  ProcessGPS();
}
