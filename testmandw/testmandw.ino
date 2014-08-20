
// Test motorandwheels library
// cree le system de déplacement de minisamsung 
// cree le system de déplacement de tirobot

#include "motorandwheels.h"

MotorsAndWheels MandWs; 



void SetupMiniSamsung()
{
// cree le motordriver toshiba
// cree les deux moteurs lier a l'encoder
// cree les deux roue 
MandWs.CreateDriver( 3 );
MandWs.CreateMotor( 5, 7, 8 );
MandWs.CreateMotor( 6, 9, 10 );
MandWs.CreateWheel( 1, 0, MandWs.getMotor(0) );
MandWs.CreateWheel( 1, 0, MandWs.getMotor(1) );
}

void SetupBetty()
{
MandWs.CreateDriver( A4 );
MandWs.CreateMotor( 5, 7, 8 );
MandWs.CreateEncoder( 2, 3);
MandWs.CreateWheel( 1, MandWs.getEncoder(0), MandWs.getMotor(0) );

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

Wheels.CreateEncoder( 12, 13 );
Wheels.CreateEncoder( 14, 15 );
Wheels.CreateEncoder( 16, 17 );
Wheels.CreateEncoder( 18, 19 );

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
  SetupMiniSamsung();
}


void loop()
{
  // devrait etre appeler au minimum au 1/10 de secondes
  MandWs.ProcessWheels();
}
