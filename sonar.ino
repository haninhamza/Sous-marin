/*--Define--*/
/*UltraSonic Sensor*/
#define FILTER_VALUE 50
#define MEASURE_NUMBER 3
/*--Variable--*/
/*UltraSonice sensor*/
int distance;
long timeInit = 0;
const byte TRIGGER = 11; //broche TRIGGER
const byte ECHO = 10; //broche ECHO
/* Constantes pour le timeout */
const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = 8m à 340m/s
const float SOUND_SPEED = 340 / 1000; // Vitesse du son dans l'air en mm/us

void setup() {
  /*UltraSonic Sensor Setup*/
  pinMode(TRIGGER, OUTPUT);
  digitalWrite(TRIGGER, LOW);
  pinMode(ECHO, INPUT);
  /*Serial communication setup*/
  Serial.begin(57600);
}
void loop() {

  measureDistance();
   /*Toutes les 500ms, envoi de la distance*/
  if (millis() - timeInit >= 500)
  {
    /*Envoi distance (détection capteur ultrason)*/
    if(distance!=0){
      Serial.println("Distance mesuré: "+String(distance)+" cm");
    }
    timeInit = millis();
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
