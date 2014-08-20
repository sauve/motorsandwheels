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
  void SetSpeed(int val, boolean forward);
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
 
 // methode utilitaire
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
public:
  MotorsAndWheels();

  void ProcessWheels();
  
  void CreateMotor();
  void CreateEncoder();
  void CreateMotor();
  
  Encoder* Encoder( int idx );
  MotorDriver* Driver( int idx );
  Motor* Motor( int idx );
  Wheel* Wheel( int idx );
}

#endif //_MOTORANDWHEELS_H_
