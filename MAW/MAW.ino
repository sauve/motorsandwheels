/*
Motors and wheels

Permet de tester la compilation sans être une librairie

*/

#include "motorandwheels.h"

MotorsAndWheels maw;

// petite test avec un moteur, un encodeur quad sur 2 et 3
void setup()
{
  MotorDriver*  driver = maw.CreateDriver(8);
  Motor* motor = maw.CreateMotor( 9, 4, 5 );
  motor->SetDriver(driver);
  Encoder* encoder = maw.CreateEncoder(2, 3);
  maw.CreateWheel( 100, encoder, motor );

  Serial.begin( 57600 );
}


int speeda = 0;
int speedmodif = 1;
unsigned long timestamp = 0;
int comptcheck = 0;
void loop()
{
  if ( timestamp + 20 < millis() )
  {
    timestamp = millis();
    // doit pouvoir
   
   // modifie la vitesse du moteur
    speeda += speedmodif;
    if ( speeda > 254 )
    {
      speedmodif = -1;
    }
    else if ( speeda < -254 )
    {
      speedmodif = 1;
    }
    
    maw.getMotor(0)->SetSpeed(speeda);
    maw.ProcessWheels();
    Serial.println("process");  
    comptcheck++;
  }
  
  if ( comptcheck % 5 == 0 )
  {
    // liste etat de la roue
  }
}
