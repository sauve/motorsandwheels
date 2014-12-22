
// Test motorandwheels library
// cree le system de déplacement de minisamsung 
// cree le system de déplacement de tirobot

#include "motorandwheels.h"

MotorsAndWheels MandWs; 



void SetupPololuTB6612FNG()
{
  // create the motor driver with an enable pin on pin 4
MandWs.CreateDriver( 4 );
// create DC motor A control on PWM pin 5, forward on pin 7 et reverse on pin 8
MandWs.CreateMotor( 5, 7, 8 );
// create DC motor B control on PWM pin 6, forward on pin 9 et reverse on pin 10
MandWs.CreateMotor( 6, 9, 10 );
}



void SetupDFRobotMotorShield()
{
// create DC motor A control on PWM pin 5, forward on pin 4 HIGH
MandWs.CreateMotor( 5, 4 );
// create DC motor B control on PWM pin 6, forward on pin 7 HIGH
MandWs.CreateMotor( 6, 7 );
}


void SetupMiniSamsung()
{
// cree le motordriver toshiba
// cree les deux moteurs lier a l'encoder
// cree les deux roue 
MandWs.CreateDriver( 3 );
MandWs.CreateMotor( 5, 7, 8 );
MandWs.getMotor(0)->SetDriver(MandWs.getDriver(0) );
MandWs.CreateMotor( 6, 9, 10 );
MandWs.getMotor(1)->SetDriver(MandWs.getDriver(0) );
MandWs.CreateWheel( 1, 0, MandWs.getMotor(0) );
MandWs.CreateWheel( 1, 0, MandWs.getMotor(1) );
}

void SetupBetty()
{
MandWs.CreateDriver( 5 );
MandWs.CreateMotor( 6, 7, 8 );
MandWs.getMotor(0)->SetDriver(MandWs.getDriver(0));
MandWs.CreateEncoder(2, 3);
MandWs.CreateWheel( 1, MandWs.getEncoder(0), MandWs.getMotor(0) );
}


// Basée sur un contrôleur Pololu toshiba et des encodeur non quadature
void setupDualEncoderDualMotor()
{
  MandWs.CreateDriver( 3 );
  MandWs.CreateMotor( 5, 7, 8 );
  MandWs.CreateMotor( 6, 9, 10 );
  //MandWs.CreateEncoder( 12 );
  //MandWs.CreateEncoder( 13 );
  //MandWs.CreateWheel( 1, Wheels.Encoder(0), Wheels.Motor(0) );
  //MandWs.CreateWheel( 1, Wheels.Encoder(1), Wheels.Motor(1) );
}

void SetupTiRobot()
{
  /*
Wheels.CreateDriver( 3 );
Wheels.CreateDriver( 4 );

Wheels.CreateMotor( 5, 7, 8 );
Wheels.CreateMotor( 5, 7, 8 );
Wheels.CreateMotor( 5, 7, 8 );
Wheels.CreateMotor( 5, 7, 8 );

Wheels.CreateEncoder( 12 );
Wheels.CreateEncoder( 13 );
Wheels.CreateEncoder( 14 );
Wheels.CreateEncoder( 15 );

Wheels.CreateWheel( 1, Wheels.Encoder(0), Wheels.Motor(0) );
Wheels.CreateWheel( 1, Wheels.Encoder(1), Wheels.Motor(1) );
Wheels.CreateWheel( 1, Wheels.Encoder(2), Wheels.Motor(2) );
Wheels.CreateWheel( 1, Wheels.Encoder(3), Wheels.Motor(3) );
*/
}

void setup()
{
  //
  Serial.begin( 38400 );
  //SetupMiniSamsung();
  SetupDFRobotMotorShield();
}


int mASpeed = 0;
int mBSpeed = 0;
int mAIncrement = 1;
int mBIncrement = 1;

void loopMotorTest()
{
mASpeed += mAIncrement;
mBSpeed += mBIncrement;

if ( mASpeed == -255 || mASpeed == 255 )
  mAIncrement = -mAIncrement;
  
if ( mBSpeed == -255 || mBSpeed == 255 )
  mBIncrement = -mBIncrement;

MandWs.getMotor(0)->SetSpeed(mASpeed);
//MandWs.getMotor(1)->SetSpeed(mBSpeed);
}

void loop()
{
  loopMotorTest();
  // devrait etre appeler au minimum au 1/10 de secondes
  MandWs.ProcessWheels();
  delay(32);
}
