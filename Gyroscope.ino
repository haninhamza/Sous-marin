
    #include <Wire.h>  // Wire library - used for I2C communication
    int ADXL345 = 0x53; // The ADXL345 sensor I2C address
    float X_out, Y_out;  // Output. Tête vers le haut, ins vers l'arrière ; - à gauche
    void setup() {
      Serial.begin(9600); // Initiate serial communication for printing the results on the Serial monitor
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
