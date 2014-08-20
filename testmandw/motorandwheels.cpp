// motorsandwheels
#include "motorandwheels.h"
// interrup functions

void ChangeEncoder1()
{}

void ChangeEncoder2()
{}

void ChangeEncoder3()
{}

void ChangeEncoder4()
{}

// -------- MotorDriver ---------
MotorDriver::MotorDriver()
{
  EnabledPin = 0;
}

MotorDriver::MotorDriver( int enablepin )
{
  EnabledPin = enablepin;
  // check if pin is available, if not set to 0
  
  if ( EnabledPin != 0 )
  {
    pinMode( EnabledPin, OUTPUT );
  }
}

// --------- Motor -----------
void Motor::SetSpeed(int val, boolean forward)
{}



// --------- Encoder -----------
Encoder::Encoder()
{
QuadAPin = 0;
QuadBPin = 0;
}

Encoder::Encoder( int apin, int bpin )
{
QuadAPin = apin;
QuadBPin = bpin;
}


// --------- Wheel -----------
Wheel::Wheel()
{
  
}

Wheel::Wheel( int ratio, Encoder* encoder, Motor* motor)
{
  GearRatio = ratio;
  _encoder = encoder;
  _motor = motor;
}

int Wheel::getRPM()
{
  int ret = 0;
  
  return ret;
}


// --------- MotorsAndWheels -----------
MotorsAndWheels::MotorsAndWheels()
{
   // devrait en fait avoir le nombre d'encoder et autre en parametre a la creation
  _drivers = new MotorDriver*[2];
  _motors = new Motor*[2];
  _wheels = new Wheel*[2];
  _encoders = new Encoder*[2];
  
  nbrMotors = 0;
  nbrDrivers = 0;
  nbrWheels = 0;
  nbrEncoder = 0;
}


void MotorsAndWheels::ProcessWheels()
{
  
}


// Create a new motor driver with an enable pin and add it to the list
void MotorsAndWheels::CreateDriver( int enablePin)
{
  MotorDriver* driver = new MotorDriver(enablePin);
  _drivers[nbrDrivers] = driver;
  nbrDrivers++;
}

void MotorsAndWheels::CreateEncoder( int pinA, int pinB )
{
  Encoder* encoder = new Encoder(pinA, pinB);
  _encoders[nbrEncoder] = encoder;
  nbrEncoder++;
}

void MotorsAndWheels::CreateMotor(int pwmpin,int forwardpin,int barckwardpin)
{

}

void MotorsAndWheels::CreateWheel(int ratio, Encoder* encoder, Motor* motor )
{
  Wheel* wheel = new Wheel(ratio, encoder, motor);
  _wheels[nbrWheels] = wheel;
  nbrWheels++;
}
  
Encoder* MotorsAndWheels::getEncoder( int idx )
{
  // cree le nouvel encoder
  
  // set les pinModes
}

MotorDriver* MotorsAndWheels::getDriver( int idx )
{
  // cree le nouveau motor driver
  
  // set les pinModes
}

Motor* MotorsAndWheels::getMotor( int idx )
{
 // cree le nouveau motor
  
  // set les pinModes
}

Wheel* MotorsAndWheels::getWheel( int idx )
{
 // cree la nouvelle wheel
  
  // set les pinModes
}

