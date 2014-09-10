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

void MotorDriver::Enable( boolean enabled )
{
  if ( EnabledPin == 0 )
    return;
    
  digitalWrite( EnabledPin, enabled == true ? HIGH : LOW );
}

// --------- Motor -----------

Motor::Motor()
{}



Motor::Motor( int pwm, int forward)
{
  PWMPin = pwm;
  ForwardPin = forward;
  BackwardPin = 0;
  motorDriver = 0;
pinMode( pwm, OUTPUT );
pinMode( forward, OUTPUT );
}

Motor::Motor( int pwm, int forward, int reverse )
{
  PWMPin = pwm;
  ForwardPin = forward;
  BackwardPin = reverse;
  motorDriver = 0;
  
pinMode( pwm, OUTPUT );
pinMode( forward, OUTPUT );
pinMode( reverse, OUTPUT );
}

void Motor::Stop()
{
  // devrait verifier si possible de mettre forward et reverse a LOW
  analogWrite( 0, PWMPin );
}


void Motor::SetSpeed(int val, boolean forward)
{
  // if has motor driver, than call enable
  if ( motorDriver != 0 )
  {
    motorDriver->Enable(true);
  }
int fval = HIGH;
int rval = LOW;
if ( !forward )
{
  // forward pin HIGH
  fval = LOW;
  rval = HIGH;
}

digitalWrite( ForwardPin, fval );
if ( BackwardPin != 0 )
  {
  digitalWrite( ForwardPin, fval );
  }
// analogwrite pwm
analogWrite( val, PWMPin );
}

void Motor::SetSpeed(int val)
{
  if ( motorDriver != 0 )
  {
    motorDriver->Enable(true);
  }
  
int fval = HIGH;
int rval = LOW;
int pwmspeed = val;
if ( val < 0 )
  {
  fval = LOW;
  rval = HIGH;
  pwmspeed = -val;
  }

digitalWrite( ForwardPin, fval );
if ( BackwardPin != 0 )
  {
  digitalWrite( ForwardPin, fval );
  }
analogWrite( pwmspeed, PWMPin );
}

void Motor::SetDriver( MotorDriver* driver )
{
  motorDriver = driver;
}

// --------- Encoder -----------
Encoder::Encoder()
{
APin = 0;
QuadBPin = 0;
}

Encoder::Encoder( int apin )
{
APin = apin;
QuadBPin = 0;
}


Encoder::Encoder( int apin, int bpin )
{
APin = apin;
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
   // devrait en fait avoir le nombre max d'encoder et autre en parametre a la creation
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
  // pour chaque wheel, verifi si doit ajuster la vitesse
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
  Motor* motor = new Motor(pwmpin, forwardpin, barckwardpin);
  _motors[nbrMotors] = motor;
  nbrMotors++;
}

void MotorsAndWheels::CreateMotor(int pwmpin,int forwardpin)
{
  Motor* motor = new Motor(pwmpin, forwardpin);
  _motors[nbrMotors] = motor;
  nbrMotors++;
}

void MotorsAndWheels::CreateWheel(int ratio, Encoder* encoder, Motor* motor )
{
  Wheel* wheel = new Wheel(ratio, encoder, motor);
  _wheels[nbrWheels] = wheel;
  nbrWheels++;
}
  
Encoder* MotorsAndWheels::getEncoder( int idx )
{
  // devrati verifier si l'index deborde, si oui return null
  return _encoders[idx];
}

MotorDriver* MotorsAndWheels::getDriver( int idx )
{
  // devrati verifier si l'index deborde, si oui return null
  return _drivers[idx];
}

Motor* MotorsAndWheels::getMotor( int idx )
{
  // devrati verifier si l'index deborde, si oui return null
  return _motors[idx];
}

Wheel* MotorsAndWheels::getWheel( int idx )
{
  // devrati verifier si l'index deborde, si oui return null
  return _wheels[idx];
}

