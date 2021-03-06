#ifndef _MOTORANDWHEELS_H_
#define _MOTORANDWHEELS_H_

#include <Arduino.h>

// constante sur la taille des tableau
#define MOTOR_DRIVER_COMPONENT_SIZE 4
#define MAW_COMPONENT_SIZE 4

// Fast digital read, en methode pour le moment, devrait être inline
bool FastDigitalRead(volatile uint8_t* pinPort, uint8_t bitMask);

class EncoderChange
{
 public:
  unsigned long timeoschange;
  int EncoderPin;
  int EncoderValue;
};

class Encoder
{
 public:
  Encoder();
  Encoder( int apin );
  Encoder( int apin, int bpin );

  void Setup( int apin );
  void Setup( int apin, int bpin);
  
  bool Changed();
  int Diff();
  int Position();
  void Reset();
  int getTicks()
    {
      return this->ticks;
    }

protected:
  bool typeEnc;
  int APin;
  int BPin;
  int lastAVal;
  int lastBVal;
  int ticks;

  void setupFastPin();
  volatile uint8_t * portPinInA;
  uint8_t pinMaskA;
  volatile uint8_t * portPinInB;
  uint8_t pinMaskB;

  // doit connaitre sont index pour savoir quel tick volatil utiliser. Devra avoir le pinMask et pointer sur le input register pour faire un test rapide
};


class Limit
{
public:
  Limit();
  Limit( int pin );
  Limit( int pin, bool nopen );

  void setup(int apin );
  void setup(int apin, bool nopen );

  bool update();
  bool Changed();
  bool getRaw();
  bool Closed();
  bool Open();

protected:
  int Apin;
  bool normopen;
  bool lastVal;
  bool curVal;

  // fastpin access
  void setupFastPin(int pin);
  volatile uint8_t * portPinIn;
  uint8_t pinMask;
};



class AnalogInput
{
public:
  bool Changed();
  void setPin( int pin);
  void setMinMax( int min, int max);
  void setMapping( int minmap, int maxmap);

  void update();
  int getRaw();
  int getMaped();
  float getRange();
protected:
  int Apin;
  int lastVal;
  int curVal;
  int minval, maxval;
  int minMaping, maxMaping;
};


// Pourra devenir une interface et implementer une version digitale?
class Potentiometer: public AnalogInput
{
protected:
  int minAngle, maxAngle;
public:

  Potentiometer();
  Potentiometer(int apin);

  void setup(int apin);

  int getAngle();
  float getRange();

};

// Pour le moment se base sur une version analog, pourra augmeter pour gérer une version digital
class TemperatureSensor: public AnalogInput
{
protected:

public:
  TemperatureSensor();
  TemperatureSensor(int apin);
  int getCelsius();
  int getFarenheit();
};


// Gestion de senseur de courant, gestion analogue pour le moment
class PowerSensor: public AnalogInput
{
protected:

public:
  PowerSensor();
  PowerSensor( int apin);

  void setup(int apin);
  int getMilliAmp();

  // operating range
};


// Gestion de la baterie pour compenser la puissance
class BatterySensor: public AnalogInput
{
protected:
public:
  BatterySensor();
  BatterySensor( int apin);

  void setup(int apin);
  int getVoltage();

  // min volt 
  // max volt
};



class MotorControl
{
public:
  MotorControl();
  MotorControl(int pwmPin, int directionPin);
  MotorControl(int pwmPin, int forwardPin, int reversePin);

  void update();

  // config
  void setupPWMPin(int pwmPin);
  void setupDirectionPin(int directionPin);
  void setupDirectionPin(int forwardPin, int reversePin);
  void ConfigErrorPin( int pinError );

  // write
  void setPWM( int value);
  void setPWM( int valA, int valB);

  // query
  bool inError();
  int getRawSpeed();
protected:
  int PWMPin1;
  int PWMPin2;
  int ForwardPin;
  int BackwardPin;
  int ErrorPin;

  bool SinglePinDirection;
  bool PWMSpeed;

  int rawSpeed1;
  int rawSpeed2;
};


class MotorDriver
{
 public:
  MotorDriver();
  MotorDriver( int enablepin );
  void setEnablePin( int pin );
  void setPrecisionPin( int pin );

  int addMotorControler(MotorControl* mctrl);
  int addTemperatureSensor( TemperatureSensor* ts);
  int addPowerSensor( PowerSensor* ps);
  int addBaterySensor( BatterySensor* bs);
  void UpdateSensors();

  // query
  bool hasPowerSensor();
  bool hasTemperatureSensor();
  bool hasBaterySensor();
  int getTotalPower();
  int getAveragePower();
  int getAverageTemperature();

  // action
  void Enable( boolean enabled );
  void setPrecision( int precision );
  
protected:
  int EnabledPin;
  int ErrorPin;
  int PrecisionPin;

  // devrait avoir un array, les moteur pourrons être 
  PowerSensor *powerSens[MOTOR_DRIVER_COMPONENT_SIZE];
  TemperatureSensor *tempSens[MOTOR_DRIVER_COMPONENT_SIZE];
  BatterySensor *baterySens[MOTOR_DRIVER_COMPONENT_SIZE];
  MotorControl *motCtrls[MOTOR_DRIVER_COMPONENT_SIZE];
};

class Motor
{
 public:
   Motor();
   
  void Stop();
  void SetSpeed(int val, boolean forward);
  void SetSpeed(int val);
  void SetDriver( MotorDriver* driver );
  void SetControler( MotorControl* mctrl );
  void SetPowerSensor( PowerSensor* psens );
  void SetTemperatureSensor( TemperatureSensor* tsens );
  void SetBaterySensor( BatterySensor* bsens );

  void SetEncoder( Encoder* encoder );

  bool hasPowerSensor();
  bool hasTemperatureSensor();
  bool hasDriver();

  // ramp, min max accel,
protected:
  bool PWMSpeed;
  MotorDriver* motorDriver;
  PowerSensor* powerSens;
  TemperatureSensor *tempSens;
  MotorControl *motCtrl;
  BatterySensor *baterySens;
  Encoder* _encoder;
  
  int StallSpeed;
  int MaxSpeed;
};

class Wheel
{
 public:
 Wheel();
 Wheel( int ratio, Encoder* encoder, Motor* motor);
 
  int getRPM();
 
 // methode utilitaire
protected:
  int GearRatio;
  int minRPM;
  int MaxRPM;
  // Encoder
  // Motor
  Encoder* _encoder;
  Motor* _motor;
  // power, potentiometer, temp?
};


class MAWMessage
{
public:


protected:
  int type;
  int id;
  int data;

};




class SerialMAW
{
public:
  // devrait passer le serial  et la vitesse
  SerialMAW();

  void processMessage();
  bool hasMessage();
  MAWMessage* getNextMessage();
protected:
  // liste de message a processer

};

//-- I2CMAW
//-- SPIMaw
//-- TCPMaw?

class MotorsAndWheels
{
// liste de motor, motordriver et encoder
// creation et gestion des wheels
public:
  MotorsAndWheels();

  // start, stop, calibration
  void Start();
  void Reset();

  void ProcessWheels();
  void CheckEncoders();
  void CheckInput();
  void CheckMessage();
  void Process();

  int registerEncoder(Encoder* encoder);
  int registerLimit(Limit* limit);
  int registerMotorDriver(MotorDriver* mdriver);
  int registerMotor(Motor* motor);
  int registerWheel(Wheel* wheel);

  // input de control
  int registerInputPotentiometer(Potentiometer* potenriometer);


  int releaseEncoder(Encoder* encoder);
  int releaseLimit(Limit* limit);
  int releaseMotorDriver(MotorDriver* mdriver);
  int releaseMotor(Motor* motor);
  int releaseWheel(Wheel* wheel);

  int releaseInputPotentiometer(Potentiometer* potenriometer);

  
  Encoder* getEncoder( int idx );
  MotorDriver* getDriver( int idx );
  Motor* getMotor( int idx );
  Wheel* getWheel( int idx );
  Potentiometer* getInputPotentiometer(int idx );
protected:
  int nbrMotors;
  int nbrDrivers;
  int nbrWheels;
  int nbrEncoder;
  int nbrLimits;

  EncoderChange encChanges[8];
  // devra identifier les max support selon l'architecture
  MotorDriver* _drivers[MAW_COMPONENT_SIZE];
  Motor* _motors[MAW_COMPONENT_SIZE];
  Wheel* _wheels[MAW_COMPONENT_SIZE];
  Encoder* _encoders[MAW_COMPONENT_SIZE];
  Potentiometer* inputPot[MAW_COMPONENT_SIZE];

  // limit a testes
  Limit* _limits[4];

  // TIMING
  unsigned long lastEncoderCheck;
  unsigned long lastInputCheck;
  unsigned long lastMessageCheck;
  unsigned long lastWheelCheck;

  unsigned long nextInputCheck;
  unsigned long nextMessageCheck;
  unsigned long nextWheelCheck;

  unsigned long inputCheckUpdate;
  unsigned long messageCheckUpdate;
  unsigned long wheelCheckUpdtae;
};

#endif //_MOTORANDWHEELS_H_
