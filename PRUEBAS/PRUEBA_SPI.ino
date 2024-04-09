// #include <SPI.h>
// #define PIN_SCK   18   // Pin SCK para SPI
// #define PIN_MISO  19   // Pin MISO para SPI
// #define PIN_MOSI  23   // Pin MOSI para SPI
// #define PIN_CS    5    // Pin CS para SPI
 
// #define    BME_280_ADDRESS 0x76

// void setup() {
//   Serial.begin(9600);
//   SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS); // Inicializar SPI
//   pinMode(PIN_CS, OUTPUT);
//   digitalWrite(PIN_CS, HIGH); // Desactivar el chip select inicialmente
// }
 
// void loop() {
//   // Leer datos del acelerómetro
//   float accelData[3];
//   readAccelData(accelData);
//   Serial.print("Acelerómetro (g): ");
//   Serial.print(accelData[0]);
//   Serial.print("\t");
//   Serial.print(accelData[1]);
//   Serial.print("\t");
//   Serial.println(accelData[2]);
//   delay(500);
// }
 
// void readAccelData(float* accelData) {
//   // Activar chip select
//   digitalWrite(PIN_CS, LOW);
 
//   // Leer datos del acelerómetro
//   SPI.transfer(BME_280_ADDRESS | 0x3B); // Registro de inicio del acelerómetro
//   for (int i = 0; i < 6; i++) {
//     accelData[i / 2] = SPI.transfer(0) << 8 | SPI.transfer(0);
//   }
 
//   // Desactivar chip select
//   digitalWrite(PIN_CS, HIGH);
// }