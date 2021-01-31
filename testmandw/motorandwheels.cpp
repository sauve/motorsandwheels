// motorsandwheels
#include "motorandwheels.h"

//Rebati le code avec une gestion plus simpliste de memoire avec register
bool FastDigitalRead(volatile uint8_t* pinPort, uint8_t bitMask)
{
  return (*pinPort & bitMask) != 0;
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
  this->setupFastPin(apin);
  this->normopen = true;
  this->lastVal = 0;
}

void Limit::setup(int apin, bool nopen )
{
  this->Apin = apin;
  pinMode(apin, INPUT );
  this->setupFastPin(apin);
  this->normopen = nopen;
  this->lastVal = 0;
}

bool Limit::update()
{
  this->lastVal = this->curVal;
  this->curVal = FastDigitalRead(this->portPinIn, this->pinMask);
  return this->curVal;
}

bool Limit::Changed()
{
  bool ret = false;
  if ( this->curVal != this->lastVal)
  {
    ret = true;
  }
  return ret;
}

bool Limit::getRaw()
{
  return this->curVal;
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


// ------- AnalogInput

void AnalogInput::update()
{
  this->lastVal = this->curVal;
  this->curVal = analogRead(this->Apin);
}

// AnalogInput

bool AnalogInput::Changed()
{
  if ( this->curVal != this->lastVal)
  {
    return true;
  }
  return false;
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
  return this->curVal;
}

int AnalogInput::getMaped()
{
  return map( this->curVal, this->minval, this->maxval, this->minMaping, this->maxMaping );
}

float AnalogInput::getRange()
{
  return (float)(this->curVal - this->minval) / (float)(this->maxval);
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

// -------- MotorControl --------

MotorControl::MotorControl()
{
  this->PWMPin1 = 0;
  this->PWMPin2 = 0;
  this->ForwardPin = 0;
  this->BackwardPin = 0;
  this->ErrorPin = 0;
  this->SinglePinDirection = 0;
  this->PWMSpeed = 0;
}

MotorControl::MotorControl(int pwmPin, int directionPin)
{
  this->PWMPin1 = pwmPin;
  this->ForwardPin = directionPin;
  this->BackwardPin = 0;
}

MotorControl::MotorControl(int pwmPin, int forwardPin, int reversePin)
{
  this->PWMPin1 = pwmPin;
  this->ForwardPin = forwardPin;
  this->BackwardPin = reversePin;
}

void MotorControl::setupPWMPin(int pwmPin)
{
  this->PWMPin1 = pwmPin;
  pinMode( pwmPin, OUTPUT );
}

void MotorControl::setupDirectionPin(int directionPin)
{
  this->ForwardPin = directionPin;
  pinMode( directionPin, OUTPUT );
  this->BackwardPin = 0;
}

void MotorControl::setupDirectionPin(int forwardPin, int reversePin)
{
  this->ForwardPin = forwardPin;
  this->BackwardPin = reversePin;
  pinMode( forwardPin, OUTPUT );
  pinMode( reversePin, OUTPUT );
}

void MotorControl::ConfigErrorPin( int pinError )
{
  this->ErrorPin = pinError;
  pinMode( pinError, INPUT );
}


void MotorControl::update()
{
  // devrait lire la pin d'error et autre de status
}


void MotorControl::setPWM( int value)
{
  // hum, est-ce ici que les pin de direction change par defaut
  this->rawSpeed1 = value;

}

void MotorControl::setPWM( int valA, int valB)
{
   // hum, est-ce ici que les pin de direction change par defaut
  this->rawSpeed1 = valA;
  this->rawSpeed2 = valB;
}

bool MotorControl::inError()
{
  // si error pin, read value
  return false;
}


int MotorControl::getRawSpeed()
{
  return this->rawSpeed1;
}

// -------- MotorDriver ---------
MotorDriver::MotorDriver()
{
  this->EnabledPin = 0;
  this->PrecisionPin = 0;
  this->ErrorPin = 0;

  // s'assure que les tableau sont a null?
}

MotorDriver::MotorDriver( int enablepin )
{
  this->EnabledPin = enablepin;
  // check if pin is available, if not set to 0
  
  if ( enablepin != 0 )
  {
    pinMode( enablepin, OUTPUT );
  }
}

void MotorDriver::setEnablePin( int pin )
{
  this->EnabledPin = pin;
  pinMode( pin, OUTPUT );
}

void MotorDriver::setPrecisionPin( int pin )
{
  this->PrecisionPin = pin;
  pinMode( pin, OUTPUT );
}

int MotorDriver::addMotorControler(MotorControl* mctrl)
{
  int i = 0;
  for ( ; i < MOTOR_DRIVER_COMPONENT_SIZE; ++i)
  {
    if ( this->motCtrls[i] != NULL )
    {
       this->motCtrls[i] = mctrl;
       break;
    }
  }
  if ( i == MOTOR_DRIVER_COMPONENT_SIZE )
  {
    return -1;
  }
  return i;
}

int MotorDriver::addTemperatureSensor( TemperatureSensor* ts)
{
  int i = 0;
  for ( ; i < MOTOR_DRIVER_COMPONENT_SIZE; ++i)
  {
    if ( this->tempSens[i] != NULL )
    {
       this->tempSens[i] = ts;
       break;
    }
  }
  if ( i == MOTOR_DRIVER_COMPONENT_SIZE )
  {
    return -1;
  }
  return i;
}

int MotorDriver::addPowerSensor( PowerSensor* ps)
{
  int i = 0;
  for ( ; i < MOTOR_DRIVER_COMPONENT_SIZE; ++i)
  {
    if ( this->powerSens[i] != NULL )
    {
       this->powerSens[i] = ps;
       break;
    }
  }
  if ( i == MOTOR_DRIVER_COMPONENT_SIZE )
  {
    return -1;
  }
  return i;
}

int MotorDriver::addBaterySensor( BatterySensor* bs)
{
  int i = 0;
  for ( ; i < MOTOR_DRIVER_COMPONENT_SIZE; ++i)
  {
    if ( this->baterySens[i] != NULL )
    {
       this->baterySens[i] = bs;
       break;
    }
  }
  if ( i == MOTOR_DRIVER_COMPONENT_SIZE )
  {
    return -1;
  }
  return i;
}


void MotorDriver::UpdateSensors()
{
  // pour chacun des tableau, passer les élément et fait le read
  int i = 0;
  for ( ; i < MOTOR_DRIVER_COMPONENT_SIZE; ++i)
  {
    if ( this->baterySens[i] == NULL )
    {
       break;
    }
    this->baterySens[i]->update();
  } 

  i = 0;
  for ( ; i < MOTOR_DRIVER_COMPONENT_SIZE; ++i)
  {
    if ( this->powerSens[i] == NULL )
    {
       break;
    }
    this->powerSens[i]->update();
  } 

  i = 0;
  for ( ; i < MOTOR_DRIVER_COMPONENT_SIZE; ++i)
  {
    if ( this->tempSens[i] == NULL )
    {
       break;
    }
    this->tempSens[i]->update();
  } 

  i = 0;
  for ( ; i < MOTOR_DRIVER_COMPONENT_SIZE; ++i)
  {
    if ( this->motCtrls[i] == NULL )
    {
       break;
    }
    this->motCtrls[i]->update();
  } 

  // est-ce que les error pins sont ajouter sur intterupt. Serait mieux limit?
}

int MotorDriver::getTotalPower()
{
  // fait l'addition de la consommation de l'ensemble des power sensor
}

int MotorDriver::getAveragePower()
{
  // fait la moyenne de la consommation de l'ensemble des power sensor
  return 0;
}

int MotorDriver::getAverageTemperature()
{
  // fait la moyenne de la temperature
  return 0;
}

void MotorDriver::Enable( boolean enabled )
{
  if ( this->EnabledPin == 0 )
    return;
    
  digitalWrite( EnabledPin, enabled == true ? HIGH : LOW );
}



void MotorDriver::setPrecision( int precision )
{
  if ( this->PrecisionPin == 0 )
    return;

  // problement seulement 0 ou 1 pour le moment
}



// ------------ Motor --------------------------

Motor::Motor()
{
  // tous les composant devrait être null
}

  
void Motor::Stop()
{
  // set speed 0 sur mctrl
}

void Motor::SetSpeed(int val, boolean forward)
{

}

void Motor::SetSpeed(int val)
{
  // map speed -> mctrl value

  // call mctrl
}

void Motor::SetDriver( MotorDriver* driver )
{
  this->motorDriver = driver;
}

void Motor::SetControler( MotorControl* mctrl )
{
  this->motCtrl = mctrl;
}

void Motor::SetPowerSensor( PowerSensor* psens )
{
  this->powerSens = psens;
}

void Motor::SetTemperatureSensor( TemperatureSensor* tsens )
{
  this->tempSens = tsens;
}

void Motor::SetBaterySensor( BatterySensor* bsens )
{
  this->baterySens = bsens;
}


void Motor::SetEncoder( Encoder* encoder )
{
  this->_encoder = encoder;
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
    this->_limits[this->nbrLimits] = limit;
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
int MotorsAndWheels::registerInputPotentiometer(Potentiometer* potenriometer)
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
}

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
