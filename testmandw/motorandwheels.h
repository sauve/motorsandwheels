#ifndef _MOTORANDWHEELS_H_
#define _MOTORANDWHEELS_H_

class EncoderChange
{
  unsigned long timeoschange;
  int EncoderPin;
  int EncoderValue;
}

class Encoder
{
 public:
  
protected:
  int QuadAPin;
  int QuadBPin;
}


class MotorDriver
{
protected:
  int EnabledPin;
}

class Motor
{
protected:
  int PWMPin;
  int ForwardPin;
  int BackwardPin;
  boolean SinglePinDirection;
  boolean PWMSpeed;
  
  int StallSpeed;
  int MaxSpeed;
}

class Wheel
{
  int getRPM();
  int 
protected:
  int GearRatio;
  int minRPM;
  int MaxRPM;
  // Encoder
  // Motor
  Encode *encoder;
  Motor *motor;
}

class MotorsAndWheels
{
// liste de motor, motordriver et encoder
// creation et gestion des wheels

  void ProcessWheels();
}

#endif //_MOTORANDWHEELS_H_
