// motorsandwheels
#include "motorandwheels.h"

// // -------- MotorDriver ---------
// MotorDriver::MotorDriver()
// {
//   EnabledPin = 0;
// }

// MotorDriver::MotorDriver( int enablepin )
// {
//   EnabledPin = enablepin;
//   // check if pin is available, if not set to 0
  
//   if ( EnabledPin != 0 )
//   {
//     pinMode( EnabledPin, OUTPUT );
//   }
// }

// void MotorDriver::Enable( boolean enabled )
// {
//   if ( EnabledPin == 0 )
//     return;
    
//   digitalWrite( EnabledPin, enabled == true ? HIGH : LOW );
// }

// // --------- Motor -----------

// Motor::Motor()
// {}



// Motor::Motor( int pwm, int forward)
// {
//   PWMPin = pwm;
//   ForwardPin = forward;
//   BackwardPin = 0;
//   motorDriver = 0;
// pinMode( pwm, OUTPUT );
// pinMode( forward, OUTPUT );
// }

// Motor::Motor( int pwm, int forward, int reverse )
// {
//   PWMPin = pwm;
//   ForwardPin = forward;
//   BackwardPin = reverse;
//   motorDriver = 0;
  
// pinMode( pwm, OUTPUT );
// pinMode( forward, OUTPUT );
// pinMode( reverse, OUTPUT );
// }

// void Motor::Stop()
// {
//   // devrait verifier si possible de mettre forward et reverse a LOW
//   analogWrite( 0, PWMPin );
// }


// void Motor::SetSpeed(int val, boolean forward)
// {
//   // if has motor driver, than call enable
//   if ( motorDriver != 0 )
//   {
//     motorDriver->Enable(true);
//   }
// int fval = HIGH;
// int rval = LOW;
// if ( !forward )
// {
//   // forward pin HIGH
//   fval = LOW;
//   rval = HIGH;
// }

// digitalWrite( ForwardPin, fval );
// if ( BackwardPin != 0 )
//   {
//   digitalWrite( ForwardPin, fval );
//   }
// // analogwrite pwm
// analogWrite( val, PWMPin );
// }

// void Motor::SetSpeed(int val)
// {
//   if ( motorDriver != 0 )
//   {
//     motorDriver->Enable(true);
//   }
  
// int fval = HIGH;
// int rval = LOW;
// int pwmspeed = val;
// if ( val < 0 )
//   {
//   fval = LOW;
//   rval = HIGH;
//   pwmspeed = -val;
//   }

// digitalWrite( ForwardPin, fval );
// if ( BackwardPin != 0 )
//   {
//   digitalWrite( ForwardPin, fval );
//   }
// analogWrite( pwmspeed, PWMPin );
// }

// void Motor::SetDriver( MotorDriver* driver )
// {
//   motorDriver = driver;
// }



//Rebati le code avec une gestion plus simpliste de memoire avec register

bool FastDigitalRead(byte* pinPort, byte bitMask)
{
  return (pinPort & bitMask) != 0;
}

// ---------- ENCODER --------------
Encoder::Encoder()
{
  typeEnc = false;
  APin = 0;
  BPin = 0;
  lastAVal = 0;
  lastBVal = 0;
}

Encoder::Encoder( int apin )
{
  this->Setup( apin );
}


Encoder::Encoder( int apin, int bpin )
{
   this->Setup( apin, bpin );
}

void Encoder::Setup( int apin )
{
  typeEnc = false;
  APin = apin;
  pinMode(apin, INPUT );
  BPin = 0;
  lastAVal = 0;
  lastBVal = 0;
  ticks = 0;
}

void Encoder::Setup( int apin, int bpin)
{
  typeEnc = true;
  APin = apin;
  pinMode(apin, INPUT );
  BPin = bpin;
  pinMode(bpin, INPUT );
  lastAVal = 0;
  lastBVal = 0;
  ticks = 0;
}


bool Encoder::Changed()
{
  bool ret = false;
  // update l'etat de l'encoder
  if ( this->typeEnc )
  {
    //Quad
    int acv = FastDigitalRead(this->portPinInA, this->pinMaskA);
    int bcv = FastDigitalRead(this->portPinInB, this->pinMaskB);
    // Compare si diff et ajuste les tick en function
  }
  else
  {
    int cv = FastDigitalRead(this->portPinInA, this->pinMaskA);
    if ( cv != this->lastAVal)
    {
      this->lastAVal = cv;
      this->ticks += 1;
      ret = true;
    }
  }
  return ret;
}

int Encoder::Diff()
{
  return 0;
}

int Encoder::Position()
{
  return 0;
}

void Encoder::Reset()
{}

void Encoder::setupFastPin()
{
  this->pinMaskA = digitalPinToBitMask(this->APin);
  uint8_t port = digitalPinToPort(this->APin);
  this->portPinInA = portInputRegister(port);

  this->pinMaskB = digitalPinToBitMask(this->BPin);
  port = digitalPinToPort(this->BPin);
  this->portPinInB = portInputRegister(port);
}
 

// SetCurrentValues
// ReadCurrentValues
// UpdateCurrentValue

// ------------ Limit -----------------

Limit::Limit()
{}

Limit::Limit( int pin )
{
  this->setup(pin);
}

Limit::Limit( int pin, bool nopen )
{
  this->setup(pin, nopen);
}

void Limit::setup(int apin )
{
  this->Apin = apin;
  pinMode(apin, INPUT );
  this->normopen = true;
  this->lastVal = 0;
}

void Limit::setup(int apin, bool nopen )
{
  this->Apin = apin;
  this->normopen = nopen;
  this->lastVal = 0;
}

bool Limit::Changed()
{
  bool ret = false;
  bool cv = FastDigitalRead(this->portPinIn, this->pinMask);
  if ( cv != this->lastVal)
  {
    ret = true;
    this->lastVal = cv;
  }
  return ret;
}

int Limit::getRaw()
{
  return this->lastVal;
}

bool Limit::Closed()
{
  // compare lastval avec nopen
  return false;
}

bool Limit::Open()
{
  // compare lastval avec nopen
  return false;
}


void Limit::setupFastPin(int pin)
{
  this->pinMask = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  this->portPinIn = portInputRegister(port);
}

// AnalogInput

bool AnalogInput::Changed()
{
  int cv = analogRead(this->Apin);
  if ( cv != this->lastVal)
  {
     this->lastVal = cv;
    return true;
  }
}

void AnalogInput::setPin( int pin)
{
  this->Apin = pin;
  pinMode(pin, INPUT );
}

void AnalogInput::setMinMax( int min, int max)
{
  this->minval = min;
  this->maxval = max;
}

void AnalogInput::setMapping( int minmap, int maxmap)
{
   this->minMaping = minmap;
   this->maxMaping = maxmap;
}

int AnalogInput::getRaw()
{
  return this->lastVal;
}

int AnalogInput::getMaped()
{
  return map( this->lastVal, this->minval, this->maxval, this->minMaping, this->maxMaping );
}

float AnalogInput::getRange()
{
  return (float)(this->lastVal - this->minval) / (float)(this->maxval);
}

// ------------ Potentiometer ------------------

Potentiometer::Potentiometer()
{
  this->setMinMax(0, 1023);
  this->minAngle = 0;
  this->maxAngle = 0;
}

Potentiometer::Potentiometer(int apin)
{
  this->setPin( apin );
  this->setMinMax(0, 1023);
  this->minAngle = 0;
  this->maxAngle = 0;
}

void Potentiometer::setup(int apin)
{
  this->setPin( apin );
}

int Potentiometer::getAngle()
{
  return 0;
}

float Potentiometer::getRange()
{
  return 0.0f;
}

// ------------ TemperatureSensor --------------

TemperatureSensor::TemperatureSensor()
{
  this->setMinMax(0, 1023);
}

TemperatureSensor::TemperatureSensor(int apin)
{
  this->setPin( apin );
  this->setMinMax(0, 1023);
}


int TemperatureSensor::getCelsius()
{
  return 0;
}

int TemperatureSensor::getFarenheit()
{
  return 0;
}


// ------------ Powersensor --------------------

PowerSensor::PowerSensor()
{}

PowerSensor::PowerSensor( int apin)
{
  this->setup(apin);
}

void PowerSensor::setup(int apin)
{
  this->setPin(apin);
}

int PowerSensor::getMilliAmp()
{
  return 0;
}


// ------------ BatterySensor ------------------

BatterySensor::BatterySensor()
{

}

BatterySensor::BatterySensor( int apin)
{
  this->setup(apin);
}

void BatterySensor::setup(int apin)
{
  this->setPin(apin);
}

int BatterySensor::getVoltage()
{
  return 0;
}




// -------- MotorDriver ---------
MotorDriver::MotorDriver()
{
  EnabledPin = 0;
  powerSens = NULL;
  tempSens = NULL;
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

void MotorDriver::setTemperatureSensor( TemperatureSensor* ts)
{
  this->tempSens = ts;
}

void MotorDriver::setPowerSensor( PowerSensor* ps)
{
  this->powerSens = ps;
}

void MotorDriver::UpdateSensors()
{
  // si power != NULL

  // si temp != NULL
}


// ------------ Motor --------------------------

Motor::Motor()
{

}

Motor::Motor( int pwm, int forward)
{

}

Motor::Motor( int pwm, int forward, int reverse )
{

}
  
void Motor::Stop()
{

}

void Motor::SetSpeed(int val, boolean forward)
{

}

void Motor::SetSpeed(int val)
{

}

void Motor::SetDriver( MotorDriver* driver )
{

}

// ------------ Wheel --------------------------

Wheel::Wheel()
 {

 }

Wheel::Wheel( int ratio, Encoder* encoder, Motor* motor)
 {

 }
 
int Wheel::getRPM()
{
  return 0;
}


// ------------ SerialMAW ----------------------

SerialMAW::SerialMAW()
{

}

void SerialMAW::processMessage()
{

}

bool SerialMAW::hasMessage()
{
  return false;
}

MAWMessage* SerialMAW::getNextMessage()
{
  return NULL;
}




// ------------ MotorAndWheel ------------------

MotorsAndWheels::MotorsAndWheels()
{
   // devrait en fait avoir le nombre max d'encoder et autre en parametre a la creation
  this->encChanges[0].EncoderPin = 0; // Doitfaire un clear du tableau
  
  // Initialise les tableaux de données d'encodeurs, peut etre devrait en faire un seul tableau puisque la taille et toujours de 4 ( moins de jump )?
  nbrMotors = 0;
  nbrDrivers = 0;
  nbrWheels = 0;
  nbrEncoder = 0;

  nextInputCheck = 0;
  nextMessageCheck = 0;
  nextWheelCheck = 0;

  inputCheckUpdate = 50 * 1000;
  messageCheckUpdate = 100 * 1000;
  wheelCheckUpdtae = 200 * 1000;
}

void MotorsAndWheels::Start()
{
  // init
  this->Reset();
}

void MotorsAndWheels::Reset()
{
  unsigned long curmicros = micros();

  this->lastEncoderCheck = curmicros;
  this->lastInputCheck = curmicros;
  this->lastMessageCheck = curmicros;
  this->lastWheelCheck = curmicros;

  this->nextInputCheck = curmicros + this->inputCheckUpdate;
  this->nextMessageCheck = curmicros + this->messageCheckUpdate;
  this->nextWheelCheck = curmicros + this->wheelCheckUpdtae;
}

void MotorsAndWheels::ProcessWheels()
{
  // pour chaque wheel, calcul la vitesse selon les changement d'encodeur et suis les autre senseurs
  // verifi si doit ajuster la vitesse
  unsigned long currmicros = micros();
  if ( currmicros <= this->nextWheelCheck )
  {
    // calcul le prochain check de message
    this->lastWheelCheck = currmicros;
    this->nextWheelCheck = this->wheelCheckUpdtae - (this->nextWheelCheck - currmicros);
    // pour chaque input, fait l'update
  }
}


void MotorsAndWheels::CheckInput()
{
  // Verifie si des potentiomêtre ou autre sont associé pour contrôler la vitesse du mouvement d'un roue
  unsigned long currmicros = micros();
  if ( currmicros <= this->nextInputCheck )
  {
    // calcul le prochain check de message
    this->lastInputCheck = currmicros;
    this->nextInputCheck = this->inputCheckUpdate - (this->nextInputCheck - currmicros);
    // pour chaque input, fait l'update
  }
}

void MotorsAndWheels::CheckEncoders()
{
  Encoder* curEnc;
  this->lastEncoderCheck = micros();
  //Pour chaque encodeur, verifie si l'etat a changer et si oui update le tableau
  for( int encidx = 0; encidx < this->nbrEncoder; ++encidx)
  {
    curEnc = this->_encoders[encidx];
    // check les update et change les tick
    curEnc->Changed();
  }
}

void MotorsAndWheels::CheckMessage()
{
  unsigned long currmicros = micros();
  if ( currmicros <= this->nextMessageCheck )
  {
    // calcul le prochain check de message
    this->lastMessageCheck = currmicros;
    this->nextMessageCheck = this->messageCheckUpdate - (this->nextMessageCheck - currmicros);
    // Verifie si recu un message et changle le comportement selon ou répond a la question
  }
}

void MotorsAndWheels::Process()
{
  // Fait toujours le check d'encodeur
  this->CheckEncoders();

  // Pourrait avoir un minimum pour ne pas appeler le reste, un genre de yield
  this->CheckInput();
  // Verifie si doit faire un check de process Wheel
  this->ProcessWheels();
  // verifie si doit faire un check de message
  this->CheckMessage();
}


int MotorsAndWheels::registerEncoder(Encoder* encoder)
{
  int ret = -1;
  // pourrait valider que l'encoder n'est pas NULL
  // Y a-t-il encore de la place
  if ( this->nbrEncoder < 3 )
  {
    this->_encoders[this->nbrEncoder] = encoder;
    ret = this->nbrEncoder;
    this->nbrEncoder += 1;
  }
  return ret;
}

int MotorsAndWheels::registerLimit(Limit* limit)
{
  int ret = -1;
  // pourrait valider que la limit n'est pas NULL
  // Y a-t-il encore de la place
  if ( this->nbrLimits < 3 )
  {
    this->)_limits[this->nbrLimits] = limit;
    ret = this->nbrLimits;
    this->nbrLimits += 1;
  }
  return ret;
}

int MotorsAndWheels::registerMotorDriver(MotorDriver* mdriver)
{
  return -1;
}

int MotorsAndWheels::registerMotor(Motor* motor)
{
  return -1;
}

int MotorsAndWheels::registerWheel(Wheel* wheel)
{
  return -1;
}

  // input de control
int registerInputPotentiometer(Potentiometer* potenriometer)
{
  return -1;
}

int MotorsAndWheels::releaseEncoder(Encoder* encoder)
{
  return -1;
}

int MotorsAndWheels::releaseLimit(Limit* limit)
{
  return -1;
}-

int MotorsAndWheels::releaseMotorDriver(MotorDriver* mdriver)
{
  return -1;
}

int MotorsAndWheels::releaseMotor(Motor* motor)
{
  return -1;
}

int MotorsAndWheels::releaseWheel(Wheel* wheel)
{
  return -1;
}


int MotorsAndWheels::releaseInputPotentiometer(Potentiometer* potenriometer)
{
  return -1;
}


  
Encoder* MotorsAndWheels::getEncoder( int idx )
{
  return NULL;
}

MotorDriver* MotorsAndWheels::getDriver( int idx )
{
  return NULL;
}

Motor* MotorsAndWheels::getMotor( int idx )
{
  return NULL;
}

Wheel* MotorsAndWheels::getWheel( int idx )
{
  return NULL;
}

Potentiometer* MotorsAndWheels::getInputPotentiometer(int idx )
{
  return NULL;
}
