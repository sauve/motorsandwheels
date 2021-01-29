# Roadmap pour le projet Motors And Wheels

## 0.0.2 Version composants independant

Test les éléments indépendamment d'un système inclusif. Gere la configuration des pin associer ( intput, input_pullup ). Mapping lineaire simple entre l'analog et l'unité des composants. Utilise les pinMask et pointeur de registre d'input pour les ATMEL. Permet le register de éléments ainsi que de l'update des éléments:

+ Encoder
  + Tick
  + Direction si simple ( + ou - )?
+ Limit
  + Pull up ou non
  + support normally open ou non
+ Potentiometre
  + range entre min et max
+ Temperature
  + Retour en Celsius ou Farenheit
+ PowerSensor
  + Support pour l'unité?
+ BatterySensor
  + Support pour l'unité?

## 0.0.3 Version Motor

Permet le test de différent driver et moteurs

+ driver avec enable
+ Moteur avec 1 pin pour forward ou reverse
+ Moteur avec 2 pins pour forward et reverse + brake

## 0.0.4 Version calcul et interrupt

Implemente les differente formule pour tester les calculs et de la communication avec le module. Doit supporter different type de calcul d'intterupt. Utilise l'accèes aux régistre et non digitalRead().

+ Puissance
+ Gear Ratio
+ Déplacement 
+ mapping
+ PID
+ Test encoder et lecture PWM avec interrupt
+ Implemente la lecture des encodeur et potentiometre/Input vis intterup
  + Permet de specifier si update sur tous les change, seulement one way pour les encodeurs
  + Voir la difference entre pinInterrupt et les interrupt de base ( pin 2 et 3 ). Detect comment detecter sur le start. 
  + Doit permettre de faire un pause ou éviter de faire un update pendant un moment ( équivalent de stop interrupt )

## 0.0.5 Implemtation des Wheels

Implemtation des objets en tant qu'axe de déplacement

+ Lien entre les composant d'une wheel
+ appel des methode de calcul pour effectuer de commande
+ implementation lineaire de l'accélération et décélération

## 0.0.6 Implementation des commande 

Utilitaire pour communiquer avec le module

+ Implementation de la version serie txt
+ Implementation de la version I2C natif
+ Code de test avec autre arduino

## 0.0.7 Implementation dez outil de communication

Utilitaire pour communiquer avec le module

+ Utilitaire Python avec TkInter
+ Utilitaire avec python flask et HTML 
  + Choix du framwork html
  + Version simple pour monitorer un module


## 0.0.8 Input et mapping sur motor

Permet la gestion d'un ou plusieurs input et leur mapping en commande. Implemente des mapping autre que lineaire

+ Input en tant que potentiometre
+ mapping autre que lineire sur l'input
+ Permet une deadzone en plus du min et max value?
+ sin, cos, coeff
+ table 

## 0.0.9 Version systeme inclusif

Permet l'integration d'un potentiometre associé a une roue et le moteur pour la driver

+ permet de voir la valeur au moteur vs vitesse encoder
+ Amélioration des fonctions de cetains composants
+ Permet de donner des commande selon la vitesse, position et torque


## 0.1.0 Documentation et plublication de la librairie en version naive

Ajout les tag pour de la documentation automatisé et publication de la librairie

+ Choix du moteur de documentation
+ Ajout des fichiers de configuration pour publication de la librairie

## 0.1.1 multi-composant et Calibration

Permet d'effectuer la calibration entre une vitesse et la puissance du moteur et support pour plusieurs composants

+ Ajout de deux roues avec encoder
+ Calibration entre vitesse et puissance de moteur
  + Min et max vitesse
  + vitesse <=> puissance
+ Ajout de la calibration dans les commandes
+ Ajout de la calibration des les outils de communication

## 0.1.2 Compilation multi plateform

  Permet la compilation de la librairie sur differente architexture
  
  + SAMD21
  + Due
  + Mega2560

## 0.1.3 Optimisation simpliste pour UNO

  Optimisation des appel sur ATMEGA 328

  + Test pour utilisation 16 bit vs 32 bit
  + Optimisation des calcul en firgule fixe
  + Test pour appel directement les register et non les methode stype analogRead

## 0.1.3 Optimisation simpliste pour ARM Based ( Due, SAMD )

Optimisation des appel sur processeur ARM

  + Test pour utilisation 16 bit vs 32 bit
  + Optimisation des calcul en firgule fixe
  + Test pour appel directement les register et non les methode stype analogRead

## 0.1.4 prévision pour overshoot

Ajout d'un calcul de prevsion et d'overshoot pour aider 

## 0.2.0 Stepper et calibration

Support pour les moteurs de type stepper

+ Ajout du driver de stepper motor
+ Ajout du type de moteur en tant que stepper
+ Ajout d'outil de calibration dans les outils de comm
  + Facilite le calcul et les test pour les calibrations

## 0.3.0 Digital et Driver

Ajout le support pour l'appel de methode ou de classe externe avec getion d'interface. Permet le support de driver permettant l'interface avec des contoleeur digitaux.

## 0.4.0 Servo motor

Ajout du control de servo moteur

## 0.5.0 Brushless

Support pour les moteur Brushless