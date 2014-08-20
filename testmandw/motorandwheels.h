#ifndef _MOTORANDWHEELS_H_
#define _MOTORANDWHEELS_H_

#include <Arduino.h>
class EncoderChange
{
 public:
  unsigned long timeoschange;
  int EncoderPin;
  int EncoderValue;
};

class Encoder
{
 public:
  Encoder();
  Encoder( int apin, int bpin );
protected:
  int QuadAPin;
  int QuadBPin;
};


class MotorDriver
{
 public:
  MotorDriver();
  MotorDriver( int enablepin );
protected:
  int EnabledPin;
};

class Motor
{
 public:
  void SetSpeed(int val, boolean forward);
protected:
  int PWMPin;
  int ForwardPin;
  int BackwardPin;
  boolean SinglePinDirection;
  boolean PWMSpeed;
  
  int StallSpeed;
  int MaxSpeed;
};

class Wheel
{
 public:
 Wheel();
 Wheel( int ratio, Encoder* encoder, Motor* motor);
 
  int getRPM();
 
 // methode utilitaire
protected:
  int GearRatio;
  int minRPM;
  int MaxRPM;
  // Encoder
  // Motor
  Encoder* _encoder;
  Motor* _motor;
};

class MotorsAndWheels
{
// liste de motor, motordriver et encoder
// creation et gestion des wheels
public:
  MotorsAndWheels();

  void ProcessWheels();
  
  void CreateDriver( int enablePin);
  void CreateEncoder( int pinA, int pinB );
  void CreateMotor(int pwmpin,int forwardpin,int barckwardpin);
  void CreateWheel( int ratio, Encoder* encoder, Motor* motor );
  
  Encoder* getEncoder( int idx );
  MotorDriver* getDriver( int idx );
  Motor* getMotor( int idx );
  Wheel* getWheel( int idx );
protected:
  int nbrMotors;
  int nbrDrivers;
  int nbrWheels;
  int nbrEncoder;
  MotorDriver** _drivers;
  Motor** _motors;
  Wheel** _wheels;
  Encoder** _encoders;
};

#endif //_MOTORANDWHEELS_H_
