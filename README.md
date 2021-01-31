# motorsandwheels
===============

Permet le controle de moteur, d'encoder et de simuler le control de roue et le lien entre elles.

+ permet de gerer un moteur dc + encodeur et ou potentiometre et de le traiter comme un servo moteur
+ Simplifie le controle stable de vitesse rotation de plusieurs roue 
+ Permet de calibrer la puissance vs force vs vitesse

## Objets pris en compte

+ power sensor
+ temprature sensor
+ potentiometre
+ Limit switch
+ Batery Sensor
+ moteur
+ moteur driver
+ encodeur
+ roue / joint / axe
+ systeme de roues
+ Communication

### Support Power sensor

Permet la lecture de la consommation de courant

### Support tempsenso

Permet la lecture de la temperature du moteur

### Support pour moteur controleur

+ Support direction pin
+ Support forward/backword pin
+ Suport stall / max value

### Support pour moteur driver

+ support enable pin
+ liste de senseur et appel pour somme / moyenne
+ liste de motor ctrl

### Support pour moteur

+ lien vers encodeur
+ lien vers les senseur associer, le driver et le controleur
+ gestion des ramp / accel, decel / torque et rpm

### Support pour encodeur

+ Type d'encodeur
  + Simple
  + Quad

### support pour roue

+ Identifi le moteur associé
+ Identifie l'encodeur associé
+ Encodeur ratio 
+ support table de correspondance moteur power = rpm


### Support System

+ register les roues
+ appel a tous les x temps le chceck d'encodeur
+ Permet de tourner a vistesse constante
+ Permet de gerer les roues avec des accélération et décélération

### Support pour communication

+ Recoit les commande
+ Envoi l'etat du système
+ Serie, I2C et SPI


## Lien entre les elements

+ Input
+ Wheel
  + Encoder
  + Limit
  + Potentiometer
  + Motor
    + Encoder
    + MotorDriver
      + TempSensor
      + PowerSensor