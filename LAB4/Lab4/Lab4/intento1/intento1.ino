#include <SPI.h>

#define CS_PIN 5 // Pin de chip select del MPU9250

void setup() {
  Serial.begin(9600);
  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // Desactiva el chip select inicialmente
}

void loop() {
  int16_t ax, ay, az; // Aceleración
  int16_t gx, gy, gz; // Giroscopio
  
  // Lee los valores de aceleración y giroscopio
  readSensorData(ax, ay, az, gx, gy, gz);

  // Muestra los valores leídos
  Serial.print("Aceleración X: ");
  Serial.print(ax);
  Serial.print(" | Aceleración Y: ");
  Serial.print(ay);
  Serial.print(" | Aceleración Z: ");
  Serial.print(az);
  Serial.print(" | Giroscopio X: ");
  Serial.print(gx);
  Serial.print(" | Giroscopio Y: ");
  Serial.print(gy);
  Serial.print(" | Giroscopio Z: ");
  Serial.println(gz);

  delay(1000); // Espera un segundo antes de la próxima lectura
}

void readSensorData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  digitalWrite(CS_PIN, LOW); // Activa el chip select del MPU9250
  
  // Envía la solicitud de lectura de datos de aceleración
  SPI.transfer(0x3B | 0x80); // Dirección del registro de datos de aceleración, con el bit de lectura activado
  ax = (SPI.transfer(0) << 8) | SPI.transfer(0); // Lee los valores de aceleración en el eje X
  ay = (SPI.transfer(0) << 8) | SPI.transfer(0); // Lee los valores de aceleración en el eje Y
  az = (SPI.transfer(0) << 8) | SPI.transfer(0); // Lee los valores de aceleración en el eje Z

  // Envía la solicitud de lectura de datos del giroscopio
  SPI.transfer(0x43 | 0x80); // Dirección del registro de datos del giroscopio, con el bit de lectura activado
  gx = (SPI.transfer(0) << 8) | SPI.transfer(0); // Lee los valores del giroscopio en el eje X
  gy = (SPI.transfer(0) << 8) | SPI.transfer(0); // Lee los valores del giroscopio en el eje Y
  gz = (SPI.transfer(0) << 8) | SPI.transfer(0); // Lee los valores del giroscopio en el eje Z

  digitalWrite(CS_PIN, HIGH); // Desactiva el chip select del MPU9250
}

