/*****************




******************/

#include <motorandwheels.h>


MotorsAndWheels MaW; 
MotorDriver basedriver;
Motor leftMotor;
Motor rightMotor;
Wheel leftWheel;
Wheel rightWheel;

void setup()
{
  // set les pin et autre info
  basedriver.init();
  // set les pin et autre info du moteur de la roue gauche
  leftMotor.init();
  leftMotor.SetDriver(&basedriver);
  // set les pin et autre info du moteur de la roue droite
  rightMotor.init();
  rightMotor.SetDriver(&basedriver);

  // set info et composant de la roue gauche
  leftWheel.init(&leftMotor, NULL);
  // set info et composant de la roue droite
  rightWheel.init(&rightMotor, NULL);

  MaW.registerWheel(&leftWheel);
  MaW.registerWheel(&rightWheel);

//   // create the motor driver with an enable pin on pin 4
// MaW.CreateDriver( 4 );
// // create DC motor A control on PWM pin 5, forward on pin 7 et reverse on pin 8
// MaW.CreateMotor( 5, 7, 8 );
// // create DC motor B control on PWM pin 6, forward on pin 9 et reverse on pin 10
// MaW.CreateMotor( 6, 9, 10 );
}

int mASpeed = 0;
int mBSpeed = 0;
int mAIncrement = 1;
int mBIncrement = 1;
void loop()
{
  MaW.Process();
  // test si un delay est passe, alors 

  mASpeed += mAIncrement;
  mBSpeed += mBIncrement;
  if ( mASpeed == -255 || mASpeed == 255 )
    mAIncrement = -mAIncrement;
    
  if ( mBSpeed == -255 || mBSpeed == 255 )
    mBIncrement = -mBIncrement;
  
  MaW.getMotor(0)->SetSpeed(mASpeed);
  //MandWs.getMotor(1)->SetSpeed(mBSpeed);

  // devrait etre appeler au minimum au 1/10 de secondes
  MaW.ProcessWheels();
}
