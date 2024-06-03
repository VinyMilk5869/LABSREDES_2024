/*                      I2C                                 */
#include <Wire.h> 

const uint8_t Direccion_S1 = 0x40;
const uint8_t Direccion_S2 = 0x68;
const uint8_t Acelerometro = 0x1C;
const uint8_t Acelerometro_Out = 0x3B;
int16_t Temperatura, Humedad;
int16_t accelX, accelY, accelZ;

/*                      WIFI                                */

#include <WiFi.h> 
const char* ssid = "iPhonedeSara";
const char* password = "papijuancho";
WiFiServer server(502);


/*                      SPI                                 */


#include <SPI.h>    

// Registros de configuración del MPU9250
#define ACCEL_XOUT_H 0x3B
// #define GYRO_XOUT_H 0x43
#define LCD_ADDRESS 0x27

#define MPU_CS_PIN 5 // Pin de chip select para el MPU9250
#define SCK_PIN 18   // Pin del reloj de SPI
#define MOSI_PIN 23  // Pin MOSI de SPI
#define MISO_PIN 19  // Pin MISO de SPI



/*                      BLUETOOTH                           */

#include <BLEDevice.h>  
#include <BLEUtils.h>
#include <BLEServer.h>

uint8_t batteryLevel = 100; 
BLEServer *pServer = nullptr;
BLECharacteristic *pBatteryLevelCharacteristic = nullptr; // Declarar globalmente



void setup() {

/*                      I2C                                 */
  setupSI7021(); //SETUP SENSORES SI ES NECESARIO
/*                      WIFI                                */
  WiFi.begin(ssid, password);
/*                      SPI                                 */
  pinMode(MPU_CS_PIN, OUTPUT);
  digitalWrite(MPU_CS_PIN, HIGH); // Deshabilitar el MPU9250 inicialmente
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, MPU_CS_PIN);
/*                      BLUETOOTH                           */
  BLEDevice::init("FARAONLOVESHADYXXX");
  pServer = BLEDevice::createServer();
  // Crear un servicio de batería
  BLEService *pBatteryService = pServer->createService("180F");
  pBatteryLevelCharacteristic = pBatteryService->createCharacteristic(
                                                     "2A19",
                                                     BLECharacteristic::PROPERTY_READ |
                                                     BLECharacteristic::PROPERTY_NOTIFY
                                                   );
  batteryLevel = 100; // Nivel de batería inicial
  pBatteryLevelCharacteristic->setValue(&batteryLevel, 1);
  pBatteryService->start();
  // Iniciar publicidad del servidor BLE
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pBatteryService->getUUID());
  pAdvertising->start();

}

void loop {

/*                      I2C                                 */
  readSI7021();
/*                      WIFI                                */

  server.begin();
/*                      SPI                                 */
  readAccelData(ax, ay, az);
/*                      BLUETOOTH                           */
  batteryLevel--;
  pBatteryLevelCharacteristic->setValue(&batteryLevel, sizeof(batteryLevel));
  pAdvertising->start();
  
}


/*              FUNCIONES                   */

/*********          I2C             **************/

/*              LECTURA DE I2C              */
  Wire.beginTransmission(Direccion_S1);
  Wire.write(0xE3);
  Wire.endTransmission();
  delay(50);
  Wire.requestFrom((uint8_t)Direccion_S1, (uint8_t)2);
  Temperatura = ((Wire.read() << 8) | Wire.read()) * 175.72 / 65536 - 46.85;
/*              ESCRITURA DE I2C            */
  Wire.beginTransmission(Direccion_S1);
  Wire.write(0xE5,##VALOR);
  Wire.endTransmission();

/************       SPI     *******************/

/*              CHIP SELECT                 */
  digitalWrite(MPU_CS_PIN, LOW); // Habilitar el MPU9250

/*              LECTURA POR SPI             */

  selectChip();
  SPI.transfer(ACCEL_XOUT_H | 0x80); // Establecer el bit MSB para indicar una lectura
  ax = (SPI.transfer16(0) << 8) >> 8; // Leer datos de aceleración en el eje X
  ay = (SPI.transfer16(0) << 8) >> 8; // Leer datos de aceleración en el eje Y
  az = (SPI.transfer16(0) << 8) >> 8; // Leer datos de aceleración en el eje Z
  deselectChip();
