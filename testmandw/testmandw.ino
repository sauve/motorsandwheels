
// Test motorandwheels library
// cree le system de déplacement de minisamsung 
// cree le system de déplacement de tirobot

#include "motorandwheels.h"

MotorsAndWheels MandWs; 
Encoder Enc1;
Encoder Enc2;
Encoder Enc3;
Encoder Enc4;
MotorDriver motDriver1;
MotorDriver motDriver2;
MotorControl mot1Ctrl1;
MotorControl mot1Ctrl2;
MotorControl mot1Ctrl3;
MotorControl mot1Ctrl4;
Motor Mot1;
Motor Mot2;
Motor Mot3;
Motor Mot4;
Wheel Wheel1;
Wheel Wheel2;
Wheel Wheel3;
Wheel Wheel4;

// test avec https://www.pololu.com/product/1213
PowerSensor mot1Power;
PowerSensor mot2Power;

Potentiometer inputpot1;
Limit lim1;

void SetupPololuTB6612FNG()
{
//   // create the motor driver with an enable pin on pin 4
// MandWs.CreateDriver( 4 );
// // create DC motor A control on PWM pin 5, forward on pin 7 et reverse on pin 8
// MandWs.CreateMotor( 5, 7, 8 );
// // create DC motor B control on PWM pin 6, forward on pin 9 et reverse on pin 10
// MandWs.CreateMotor( 6, 9, 10 );
}



void SetupDFRobotMotorShield()
{
// // create DC motor A control on PWM pin 5, forward on pin 4 HIGH
// MandWs.CreateMotor( 5, 4 );
// // create DC motor B control on PWM pin 6, forward on pin 7 HIGH
// MandWs.CreateMotor( 6, 7 );
}


void SetupMiniSamsung()
{
// cree le motordriver toshiba
// cree les deux moteurs lier a l'encoder
// cree les deux roue 
// MandWs.CreateDriver( 3 );
// MandWs.CreateMotor( 5, 7, 8 );
// MandWs.getMotor(0)->SetDriver(MandWs.getDriver(0) );
// MandWs.CreateMotor( 6, 9, 10 );
// MandWs.getMotor(1)->SetDriver(MandWs.getDriver(0) );
// MandWs.CreateWheel( 1, 0, MandWs.getMotor(0) );
// MandWs.CreateWheel( 1, 0, MandWs.getMotor(1) );
}

void SetupBetty()
{
// MandWs.CreateDriver( 5 );
// MandWs.CreateMotor( 6, 7, 8 );
// MandWs.getMotor(0)->SetDriver(MandWs.getDriver(0));
// MandWs.CreateEncoder(2, 3);
// MandWs.CreateWheel( 1, MandWs.getEncoder(0), MandWs.getMotor(0) );
}


// Basée sur un contrôleur Pololu toshiba et des encodeur non quadature
void setupDualEncoderDualMotor()
{
  // MandWs.CreateDriver( 3 );
  // MandWs.CreateMotor( 5, 7, 8 );
  // MandWs.CreateMotor( 6, 9, 10 );
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

void steupnewencoder( Encoder* enc, int apin)
{
   enc->Setup( apin );
   enc->Reset();
  int encidx = MandWs.registerEncoder( enc);
  Serial.print( "Enc ID=");
  Serial.println(encidx);
}

void setupSimpleEncoderTest()
{
  // Initialise un seul encoder 
  steupnewencoder( &Enc1, A0);
  steupnewencoder( &Enc2, A1);
  steupnewencoder( &Enc3, A2);
  // register le tout a MAW
}


void setupUNOStructureTest()
{
  // Utilise PS2 2 axe + sel pour input et limit test 
  // A0 = X axis
  inputpot1.setup( A0 );
   // limit A1
   lim1.setup( A1 );

  //Motor driver = Pololu Dual MC33926 Motor Driver Carrier
  // 11 = Enable driver
  motDriver1.setEnablePin( 11 );

  // 7 = IN1
  // 8 = IN2
  mot1Ctrl1.setupDirectionPin( 7, 8 );
  // 6 = Connections pwm mot 1
  mot1Ctrl1.setupPWMPin( 6 );
  // 5 = SF mot 1
  mot1Ctrl1.ConfigErrorPin( 5 );
  // A2 = FB mot 1 Power
  mot1Power.setup( A2 );
  
  // Moteur Cytron SPG30-20K avec Quad encodeur
  // 2 = ENC A 
  // 4 = ENC B
  Enc1.Setup(2, 4);


  // register and link component to the driver
  motDriver1.addPowerSensor(&mot1Power);
  motDriver1.addMotorControler(&mot1Ctrl1);
  
  // motor interface
  Mot1.SetDriver(&motDriver1);
  Mot1.SetControler(&mot1Ctrl1);
  Mot1.SetEncoder(&Enc1);
  Mot1.SetPowerSensor(&mot1Power);

  // register components to MansW interface
  MandWs.registerMotorDriver(&motDriver1);
  MandWs.registerMotor(&Mot1);
  
  MandWs.registerInputPotentiometer(&inputpot1);
    // A1 = Sel
  MandWs.registerLimit(&lim1);
}

void setup()
{
  //
  Serial.begin( 38400 );
  //SetupMiniSamsung();
  //SetupDFRobotMotorShield();
  setupUNOStructureTest();

  MandWs.Start();
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


int curspeed = 0;
int dirmodifspeed = 1;
void loopbasicmotortest()
{
  // utilise motorctrl et driver seulement
  // test set speed et update sensor

  // update la vitesse
  curspeed += dirmodifspeed;
  if ( curspeed < 0 || curspeed > 128)
  {
    dirmodifspeed = -dirmodifspeed;
  }
  mot1Ctrl1.setPWM(curspeed);
  
  // update les info de senseurs
  motDriver1.UpdateSensors();

  // affiche les info des senseurs
  Serial.print("power:");
  Serial.println(mot1Power.getRaw());
  Serial.print("mot err:");
  Serial.println(mot1Ctrl1.inError());
  Serial.print("speed:");
  Serial.println(mot1Ctrl1.getRawSpeed());
  Serial.print("inpot:");

  inputpot1.update();
  Serial.println(inputpot1.getRaw());
  Serial.print("lim:");
  Serial.println(lim1.update());
  // attends 1 seconde
  delay(1000);


}


int compt = 0;
void loop()
{
  //loopMotorTest();
  // devrait etre appeler au minimum au 1/10 de secondes
  // MandWs.Process();
  // compt += 1;
  // if ( compt == 10000 )
  // {
  //   Serial.print(Enc1.getTicks());
  //   Serial.print(",");
  //   Serial.print(Enc2.getTicks());
  //   Serial.print(",");
  //   Serial.println(Enc3.getTicks());
  //   compt = 0;
  //  }
  loopbasicmotortest();
}
