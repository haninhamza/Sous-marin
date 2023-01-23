#include <ros.h>
#include <Servo.h>
#include <Wire.h>  // Wire library - used for I2C communication
#include<std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
/*--Define--*/
/*Propulseur*/
#define Propulse_D 9
#define Propulse_G 5
/*Ballaste*/
#define PWM_Ballast_G 3
#define in1_Ballast_G 2
#define in2_Ballast_G 4
#define PWM_Ballast_D 6
#define in1_Ballast_D 7
#define in2_Ballast_D 8
/*UltraSonic Sensor*/
#define FILTER_VALUE 50
#define MEASURE_NUMBER 3
/*--Variable--*/
/*Propulseur*/
Servo Propulseur_D, Propulseur_G;
int tempsPDF = 0;
int tempsPGF = 0;
long tempsPD = 0;
long tempsPG = 0;

/*SécuritBallaste*/
int timerSec;
/*Ballaste*/
int tempsBDF = 0;
int tempsBGF = 0;
long tempsBD = 0;
long tempsBG = 0;
/*Accelerometer*/
int ADXL345 = 0x53; // The ADXL345 sensor I2C address
float Y_out;  // Output. Tête vers le haut, ins vers l'arrière ; - à gauche
/*Pressure senor*/
const int sensorPin = A0;
float actual_depth=0;
/*Battery level*/
const int battery_pin = A2;
/*Water sensor*/
const int waterSensor = A1;
int waterValue = 0;
bool flagWater = 0;
/*UltraSonice sensor*/
int distance;
long timeInit = 0;
const byte TRIGGER = 11; //broche TRIGGER
const byte ECHO = 10; //broche ECHO
/* Constantes pour le timeout */
const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = 8m à 340m/s
const float SOUND_SPEED = 1500 / 1000; // Vitesse du son dans l'eau en mm/us
/*ROS COMMUNICATION*/
bool flagCmd = 0;
String cmd;
ros::NodeHandle nh;
std_msgs::Float32 depth_msg;
std_msgs::Float32 acc_msg;
std_msgs::String str_msg;
std_msgs::Int32 dist_msg;
std_msgs::Int32 waterDetected_msg;
std_msgs::Int32 battery_voltage;
ros::Publisher depth("depth", &depth_msg);
ros::Publisher chatter("chatter", &dist_msg);
ros::Publisher acceleration("acceleration",&acc_msg);
ros::Publisher water("water", &waterDetected_msg);
ros::Publisher battery("battery",&battery_voltage);
void messageCb( const std_msgs::String &toggle_msg) {
  cmd = String(toggle_msg.data);
  flagCmd = 1;
}
ros::Subscriber<std_msgs::String> command("command", &messageCb );

void setup() {
  /*UltraSonic Sensor Setup*/
  pinMode(TRIGGER, OUTPUT);
  digitalWrite(TRIGGER, LOW);
  pinMode(ECHO, INPUT);
  /*ROS COMMUNICATION*/
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(depth);
  nh.advertise(acceleration);
  nh.advertise(battery);
  nh.advertise(water);
  nh.subscribe(command);
  Serial.begin(57600);
  /*Initialisation des propulseurs*/
  Propulseur_D.attach(Propulse_D, 1000, 2000) ;
  Propulseur_G.attach(Propulse_G, 1000, 2000) ;
  Propulseur_D.write(0); // vitesse mise à 0 moteur droit
  Propulseur_G.write(0); // vitesse mise à 0 moteur gauche
  /*Initialisation des ballastes*/
  pinMode(PWM_Ballast_G, OUTPUT);
  pinMode(in1_Ballast_G, OUTPUT);
  pinMode(in2_Ballast_G, OUTPUT);
  pinMode(PWM_Ballast_D, OUTPUT);
  pinMode(in1_Ballast_D, OUTPUT);
  pinMode(in2_Ballast_D, OUTPUT);

  Ballast_Fonction(255, '1', 255, '1');
  delay(2000);
  Ballast_Fonction(0, '0', 0, '0');
  /*Initialisation communication acceleromètre*/
  Wire.begin(); // Initiate the Wire library
  // Set ADXL345 in measuring mode
  Wire.beginTransmission(ADXL345); // Start communicating with the device 
  Wire.write(0x2D); // Access/ talk to POWER_CTL Register - 0x2D
  // Enable measurement
  Wire.write(8); // (8dec -> 0000 1000 binary) Bit D3 High for measuring enable 
  Wire.endTransmission();
  delay(10);
}
void loop() {
  /* Mesure présence eau + remonter à la surface*/
  waterValue = analogRead(waterSensor);
  if(waterValue >= 1000){
      flagWater = 1;
    }
  if (flagWater==1){
      tempsBDF = 999;
      tempsBGF = 999;
      Propulseur_D.write(0);
      Propulseur_G.write(0);
      Ballast_Fonction(255, '1', 255, '1');
      waterDetected_msg.data = 1;
      water.publish(&waterDetected_msg);   
    }
  /*Mesure de la distance*/
  measureDistance();
  /*Mesure de l'accélération en Y*/
  measureAccelerometer();
   /*Toutes les 500ms, envoi d'informations vers la Raspberry*/
  if (millis() - timeInit >= 500)
  {
    /*Envoi distance (détection capteur ultrason)*/
    if(distance!=0){
      Serial.println("Distance mesuré: "+String(distance)+" cm");
      dist_msg.data = distance;
      chatter.publish( &dist_msg);
    }
    /*Mesure et envoi de la profondeur toutes les 500ms*/
    measureDepth();
    depth_msg.data = actual_depth;
    depth.publish(&depth_msg);
    /*Envoi de la profondeur toutes les 500ms*/
    acc_msg.data = Y_out;
    acceleration.publish(&acc_msg);
    /*Envoi de la tension de batterie*/
    battery_voltage.data = analogRead(battery_pin);
    battery.publish(&battery_voltage);
    timeInit = millis();
  }
  /*Decodage si réception d'une commande*/
  if (flagCmd == 1) {
    decodeCommand();
    flagCmd = 0;
    timerSec = millis();
    
  }
  
  /*Check des délais moteurs de la commande*/
  checkMotorDelay();
  /*ROS COMMUNICATION*/
  nh.spinOnce();
}
/*Fonction de contrôle des ballates*/
/*vitesse : 0 à 255 | moteur Droit,Gauche = 0,1 | rotation 0,1,2 = arrêt,horaire,a-hor|*/
void Ballast_Fonction(int vitesseG, char rotG , int vitesseD, char rotD) {
  analogWrite(PWM_Ballast_G, vitesseG);
  tempsBG = millis();
  switch (rotG) {
    case '0':
      digitalWrite(in1_Ballast_G, LOW);
      digitalWrite(in2_Ballast_G, LOW);
      break;

    case '1':
      digitalWrite(in1_Ballast_G, HIGH);
      digitalWrite(in2_Ballast_G, LOW);
      break;

    case '2':
      digitalWrite(in1_Ballast_G, LOW);
      digitalWrite(in2_Ballast_G, HIGH);
      break;

    default:
      break;
  }
  analogWrite(PWM_Ballast_D, vitesseD);
  tempsBD = millis();
  switch (rotD) {
    case '0':
      digitalWrite(in1_Ballast_D, LOW);
      digitalWrite(in2_Ballast_D, LOW);
      break;

    case '1':
      digitalWrite(in1_Ballast_D, HIGH);
      digitalWrite(in2_Ballast_D, LOW);
      break;

    case '2':
      digitalWrite(in1_Ballast_D, LOW);
      digitalWrite(in2_Ballast_D, HIGH);
      break;

    default:
      break;
  }
}
/*Fonction de decodage du protocol de commande*/
void decodeCommand() {
  int vitesseD;
  int vitesseG;
  vitesseD = cmd.substring(1, 4).toInt();
  vitesseG = cmd.substring(7, 10).toInt();
  if (cmd[0] == 'P') {
    tempsPDF = cmd.substring(4, 7).toInt();
    tempsPGF = cmd.substring(10, 13).toInt();
    propulseur_Fonction(vitesseD, vitesseG);
  }
  else if (cmd[0] == 'B') {
    tempsBDF = cmd.substring(4, 7).toInt();
    tempsBGF = cmd.substring(10, 13).toInt();
    char rotationD = cmd[13];
    char rotationG = cmd[14];
    Ballast_Fonction(vitesseG, rotationG, vitesseD, rotationD);
  }
}
/*Fonction de contrôle des ballates*/
/*vitesse moteur : 0 à 180*/
void propulseur_Fonction(int vitesseD, int vitesseG) {
  tempsPD = millis();
  Propulseur_D.write(vitesseD);
  tempsPG = millis();
  Propulseur_G.write(vitesseG);
}
void checkMotorDelay()
{
  /*Potentiel problème d'overfloat de millis et non passage dans la condition*/
  if ((millis() - tempsPD) > tempsPDF * 100)
  {
    Propulseur_D.write(0);
  }
  if ((millis() - tempsPG) > tempsPGF * 100)
  {
    Propulseur_G.write(0);
  }
  if ((millis() - tempsBD) > tempsBDF * 100)
  {
    analogWrite(PWM_Ballast_D, 0);
  }
  if ((millis() - tempsBG) > tempsBGF * 100)
  {
    analogWrite(PWM_Ballast_G, 0);
  }
  if(millis()-timerSec > 50000) {
    digitalWrite(in1_Ballast_G, HIGH);
    digitalWrite(in2_Ballast_G, LOW);

    digitalWrite(in1_Ballast_D, HIGH);
    digitalWrite(in2_Ballast_D, LOW);
    
    analogWrite(PWM_Ballast_G, 255);
    analogWrite(PWM_Ballast_D, 255);
  }
}
/*Fonction de mesure de distance l'ultrason (avec filtre de valeurs)*/
void measureDistance()
{
  float measure = 0;
  float avg_measure = 0;
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  measure = pulseIn(ECHO, HIGH, MEASURE_TIMEOUT);
  int8_t measureNb = 0;
  int8_t i = 1;
  while (i <= MEASURE_NUMBER) {
      float measure_tmp;
      digitalWrite(TRIGGER, HIGH);
      delayMicroseconds(5);
      digitalWrite(TRIGGER, LOW);
      measure_tmp = pulseIn(ECHO,HIGH,MEASURE_TIMEOUT);
      //on ajoute la mesure que si celle-ci est dans une plage autour de la première mesure
      if (measure_tmp <= measure + FILTER_VALUE && measure_tmp >= measure - FILTER_VALUE) {
        avg_measure += measure_tmp;
        measureNb++;
      }
      i++;
   }
  avg_measure /= measureNb;
  int distance_cm = (avg_measure * SOUND_SPEED) / (2*10);
  distance = distance_cm;
}
/*Fonction de mesure de la profondeur sur base de la pression*/
void measureDepth(){
  float sensorValue = 0;  // variable to store the value coming from the sensor
  float pressureWater = 0;
  float pressureTotal = 0;
  sensorValue = analogRead(sensorPin);
  sensorValue -= 128;
  pressureWater = sensorValue * (50000/1023);
  pressureTotal = pressureWater + 101325;
  actual_depth = pressureWater/10000;
  Serial.print("Profondeur :"); 
  Serial.print(actual_depth); 
  Serial.println("m");
}
void measureAccelerometer()
{
  float X_out;
  // === Read acceleromter data === //
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 4, true); // Read 4 registers total, each axis value is stored in 2 registers. Obligé de mettre le X sinon il ne va pas voir
  X_out = ( Wire.read()| Wire.read() << 8); // X-axis value
  Y_out = ( Wire.read()| Wire.read() << 8); // Y-axis value
  Y_out = Y_out/256; //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
  Serial.println(Y_out);
}
