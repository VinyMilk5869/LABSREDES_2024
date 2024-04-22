#include <SPI.h>

// Definir pines SPI
#define BME_SCK 18   // Pin SCK del ESP32
#define BME_MISO 19  // Pin MISO del ESP32
#define BME_MOSI 23  // Pin MOSI del ESP32
#define BME_CS 5     // Pin CS del ESP32

void setup() {
  Serial.begin(9600);

  // Inicializar la interfaz SPI
  SPI.begin();

  // Configurar el pin CS como salida
  pinMode(BME_CS, OUTPUT);
  digitalWrite(BME_CS, HIGH); // Desactivar el dispositivo inicialmente
}

void loop() {
  // Leer datos del sensor BME280
  float temperature = readTemperature();
  float humidity = readHumidity();
  float pressure = readPressure();

  // Imprimir los datos leídos
  Serial.print("Temperatura: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Humedad: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Presión: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  delay(2000);
}

float readTemperature() {
  digitalWrite(BME_CS, LOW); // Habilitar el dispositivo

  // Enviar comando de lectura de temperatura y recibir los datos correspondientes
  SPI.transfer(0xFA); // MSB
  uint8_t temp_msb = SPI.transfer(0);
  SPI.transfer(0xFB); // LSB
  uint8_t temp_lsb = SPI.transfer(0);

  digitalWrite(BME_CS, HIGH); // Deshabilitar el dispositivo

  // Combinar los bytes de temperatura
  int temp_raw = (temp_msb << 8) | temp_lsb;
  float temperature = (float)temp_raw / 100.0;

  registers[4] = temperature;
  return temperature;
}

float readHumidity() {
  digitalWrite(BME_CS, LOW); // Habilitar el dispositivo

  // Enviar comando de lectura de humedad y recibir los datos correspondientes
  SPI.transfer(0xFD); // MSB
  uint8_t hum_msb = SPI.transfer(0);
  SPI.transfer(0xFE); // LSB
  uint8_t hum_lsb = SPI.transfer(0);

  digitalWrite(BME_CS, HIGH); // Deshabilitar el dispositivo

  // Combinar los bytes de humedad
  int hum_raw = (hum_msb << 8) | hum_lsb;
  float humidity = (float)hum_raw / 1024.0;
  registers[5] = humidity;
  return humidity;
}

float readPressure() {
  digitalWrite(BME_CS, LOW); // Habilitar el dispositivo

  // Enviar comando de lectura de presión y recibir los datos correspondientes
  SPI.transfer(0xF7); // MSB
  uint8_t pres_msb = SPI.transfer(0);
  SPI.transfer(0xF8); // LSB
  uint8_t pres_lsb = SPI.transfer(0);

  digitalWrite(BME_CS, HIGH); // Deshabilitar el dispositivo

  // Combinar los bytes de presión
  int pres_raw = ((pres_msb << 8) | pres_lsb) >> 4;
  float pressure = (float)pres_raw / 16.0;

  registers[9] = pressure;
  return pressure;
}
