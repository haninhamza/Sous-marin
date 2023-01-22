// analog pin = 0
const int waterSensor = 0;

// the read value will be included between 0 (non-conductive) and 1023 (conductive)
int value = 0;

void setup()
{
    Serial.begin(9600); // Initiate serial communication for printing the results on the Serial monitor
}

void loop()
{
    valeurLue = analogRead(monCapteur);
    if (value >= 1000)
    {
      Serial.print("ALERT : Water inside !")
    }
}
