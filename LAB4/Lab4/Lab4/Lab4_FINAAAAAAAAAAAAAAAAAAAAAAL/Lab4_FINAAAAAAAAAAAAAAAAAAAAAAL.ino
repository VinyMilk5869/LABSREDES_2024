#include <Wire.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>

float registers[13] = {0}; // Se incrementa el tamaño del array para incluir el registro adicional para contar los pulsos del botón
const char* ssid = "Santiago2021";
const char* password = "santiago2001";
const int ledPin1 = 15;
const int ledPin2 = 4;

const int pulsador = 13;
WiFiServer server(502);
unsigned long previousMillis[2] = {0, 0};

const uint8_t Direccion_S1 = 0x40;
const uint8_t Direccion_S2 = 0x68;
const uint8_t Acelerometro = 0x1C;
const uint8_t Giroscopio = 0x1B;
const uint8_t Acelerometro_Out = 0x3B;
const uint8_t Giroscopio_Out = 0x43;

bool ledState1 = LOW;
bool ledState2 = LOW;
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
int buttonPressCount = 0;

int16_t Temperatura, Humedad;
int16_t accelX, accelY, accelZ;
int16_t giroX, giroY, giroZ;

#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C
#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18
#define ACC_FULL_SCALE_2_G 0x00
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18
// Registros de configuración del MPU9250
#define WHO_AM_I 0x75
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define LCD_ADDRESS 0x27

#define MPU_CS_PIN 5 // Pin de chip select para el MPU9250
#define SCK_PIN 18   // Pin del reloj de SPI
#define MOSI_PIN 23  // Pin MOSI de SPI
#define MISO_PIN 19  // Pin MISO de SPI

LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);
// Variable de estado para controlar qué información se muestra en la pantalla LCD
enum LCDState {
  TEMPERATURE_HUMIDITY,
  ACCELERATION_GYROSCOPE
};
LCDState lcdState = TEMPERATURE_HUMIDITY;

unsigned long lastStateChangeTime = 0;
const unsigned long stateChangeInterval = 5000; // 3 segundos

SPIClass spi(HSPI); // Declaración del bus SPI

void setup() {
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);

  Serial.begin(9600);
  Serial.print("Conectando a ");
  Serial.println(ssid);
  pinMode(MPU_CS_PIN, OUTPUT);

  digitalWrite(MPU_CS_PIN, HIGH); // Deshabilitar el MPU9250 inicialmente

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, MPU_CS_PIN);
  delay(100);
  
  // Verificar la conexión con el MPU9250
  // if (readRegister(WHO_AM_I) != 0x71) { // El valor esperado para el MPU9250 es 0x71
  //   Serial.println("Error al conectar con el MPU9250");
  //   while(1);
  // }
  Serial.println("MPU9250 conectado correctamente");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Conectado a la red WiFi");
  Serial.println("Dirección IP: ");
  Serial.println(WiFi.localIP());
  server.begin();

  setupSI7021();

  // Inicializar la pantalla LCD
  lcd.init();
  lcd.backlight();
}

void loop() {
  int16_t ax, ay, az; // Aceleración en ejes X, Y y Z
  int16_t gx, gy, gz; // Velocidad angular en ejes X, Y y Z

  server.begin();
  blinkLED1();
  blinkLED2();
  readAccelData(ax, ay, az);
  readGyroData(gx, gy, gz);

  readSI7021();
  //readMPU9250();
  delay(500);
  lcd.clear();
  if (lcdState == TEMPERATURE_HUMIDITY) {
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    delay(100);
  }
  else if (lcdState == ACCELERATION_GYROSCOPE) {
    lcd.setCursor(0, 0);
    lcd.print("A:");
    lcd.setCursor(0, 1);
    lcd.print("G:");
    delay(100);
  }

  updateLCD();
  delay(100);
  recibirTrama();
}

void setupSI7021() {
  // Configuración inicial del sensor SI7021, si es necesario
}
void updateLCD() {
  unsigned long currentTime = millis();
  if (currentTime - lastStateChangeTime >= stateChangeInterval) {
    // Cambiar el estado y guardar el tiempo del cambio de estado
    if (lcdState == TEMPERATURE_HUMIDITY) {
      lcdState = ACCELERATION_GYROSCOPE;
    } else {
      lcdState = TEMPERATURE_HUMIDITY;
    }
    lastStateChangeTime = currentTime;
  }
  
  
  if (lcdState == TEMPERATURE_HUMIDITY) {
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(registers[4]);
    lcd.print(" C");
    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(registers[5]);
    lcd.print(" %");
    delay(100);
  } else if (lcdState == ACCELERATION_GYROSCOPE) {
    lcd.setCursor(0, 0);
    lcd.print("A:");
    lcd.print(static_cast<float>(registers[6]) / 10.0, 1);
    lcd.print(",");
    lcd.print(static_cast<float>(registers[7]) / 10.0, 1);
    lcd.print(",");
    lcd.print(static_cast<float>(registers[8]) / 10.0, 1);
    lcd.setCursor(0, 1);
    lcd.print("G:");
    lcd.print(static_cast<float>(registers[9]) / 10.0, 1);
    lcd.print(",");
    lcd.print(static_cast<float>(registers[10]) / 10.0, 1);
    lcd.print(",");
    lcd.print(static_cast<float>(registers[11]) / 10.0, 1);
    delay(100);
  }
}

void readSI7021() {
  Wire.beginTransmission(Direccion_S1);
  Wire.write(0xE3);
  Wire.endTransmission();
  delay(50);
  Wire.requestFrom((uint8_t)Direccion_S1, (uint8_t)2);

  Temperatura = ((Wire.read() << 8) | Wire.read()) * 175.72 / 65536 - 46.85;
  registers[4] = Temperatura;
  Wire.beginTransmission(Direccion_S1);
  Wire.write(0xE5);
  Wire.endTransmission();
  delay(50);
  Wire.requestFrom((uint8_t)Direccion_S1, (uint8_t)2);
  Humedad = ((Wire.read() << 8) | Wire.read()) * 125.0 / 65536 - 6;
  registers[5] = Humedad;
}

//void readMPU9250() {
//  uint8_t Buf[14];
//  
//  digitalWrite(MPU9250_SS, LOW); // Activa el esclavo MPU9250
//  spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // Configura la transferencia SPI
//  
//  spi.transfer(ACCEL_XOUT_H | 0x80); // Indica que queremos leer a partir de la dirección de salida del acelerómetro
//  for (int i = 0; i < 14; i++) {
//    Buf[i] = spi.transfer(0x00); // Realiza la transferencia y guarda los datos en el buffer
//  }
//
//  spi.endTransaction(); // Termina la transferencia SPI
//  digitalWrite(MPU9250_SS, HIGH); // Desactiva el esclavo MPU9250
//
//  int16_t ax = (Buf[0] << 8) | Buf[1];
//  int16_t ay = (Buf[2] << 8) | Buf[3];
//  int16_t az = (Buf[4] << 8) | Buf[5];
//  registers[6] = ax;
//  registers[7] = ay;
//  registers[8] = az;
//  int16_t gx = (Buf[8] << 8) | Buf[9];
//  int16_t gy = (Buf[10] << 8) | Buf[11];
//  int16_t gz = (Buf[12] << 8) | Buf[13];
//  registers[9] = gx;
//  registers[10] = gy;
//  registers[11] = gz;
//}
void selectChip() {
  digitalWrite(MPU_CS_PIN, LOW); // Habilitar el MPU9250
}

void deselectChip() {
  digitalWrite(MPU_CS_PIN, HIGH); // Deshabilitar el MPU9250
}

uint8_t readRegister(uint8_t reg) {
  uint8_t value;
  selectChip();
  SPI.transfer(reg | 0x80); // Establecer el bit MSB para indicar una lectura
  value = SPI.transfer(0x00); // Enviar un byte nulo para leer el valor del registro
  deselectChip();
  return value;
}

void readAccelData(int16_t &ax, int16_t &ay, int16_t &az) {
  selectChip();
  SPI.transfer(ACCEL_XOUT_H | 0x80); // Establecer el bit MSB para indicar una lectura
  ax = (SPI.transfer16(0) << 8) >> 8; // Leer datos de aceleración en el eje X
  ay = (SPI.transfer16(0) << 8) >> 8; // Leer datos de aceleración en el eje Y
  az = (SPI.transfer16(0) << 8) >> 8; // Leer datos de aceleración en el eje Z
  registers[6] = ax;
  registers[7] = ay;
  registers[8] = az;
  deselectChip();
  
}

void readGyroData(int16_t &gx, int16_t &gy, int16_t &gz) {
  selectChip();
  SPI.transfer(GYRO_XOUT_H | 0x80); // Establecer el bit MSB para indicar una lectura
  gx = (SPI.transfer16(0) << 8) >> 8; // Leer datos de giroscopio en el eje X
  gy = (SPI.transfer16(0) << 8) >> 8; // Leer datos de giroscopio en el eje Y
  gz = (SPI.transfer16(0) << 8) >> 8; // Leer datos de giroscopio en el eje Z
  registers[9] = gx;
  registers[10] = gy;
  registers[11] = gz;
  deselectChip();

}

void blinkLED1() {
  if (registers[0] == 0) {
    digitalWrite(ledPin1, LOW);
  } else {
    if (registers[0] == 1 && registers[2] == 0) {
      digitalWrite(ledPin1, HIGH);
    } else {
      if (registers[0] == 1 && registers[2] > 0) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis1 >= registers[2]) {
          previousMillis1 = currentMillis;
          if (ledState1 == LOW) {
            digitalWrite(ledPin1, HIGH);
            ledState1 = HIGH;
          } else {
            digitalWrite(ledPin1, LOW);
            ledState1 = LOW;
          }
        }
      }
    }
  }
}

void blinkLED2() {
  if (registers[1] == 0) {
    digitalWrite(ledPin2, LOW);
  } else {
    if (registers[1] == 1 && registers[3] == 0) {
      digitalWrite(ledPin2, HIGH);
    } else {
      if (registers[1] == 1 && registers[3] > 0) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis2 >= registers[3]) {
          previousMillis2 = currentMillis;
          if (ledState2 == LOW) {
            digitalWrite(ledPin2, HIGH);
            ledState2 = HIGH;
          } else {
            digitalWrite(ledPin2, LOW);
            ledState2 = LOW;
          }
        }
      }
    }
  }
}

void processModbusData(char* buffer, WiFiClient& client) {
  int transactionIdentifier = (buffer[0] << 8) | buffer[1];
  int protocolIdentifier = (buffer[2] << 8) | buffer[3];
  int length = (buffer[4] << 8) | buffer[5];
  int unitIdentifier = buffer[6];
  int functionCode = buffer[7];

  if (functionCode == 0x03) {
    int firstRegister = (buffer[8] << 8) | buffer[9];
    int lastRegister = (buffer[10] << 8) | buffer[11];
    for (int i = 0; i <= lastRegister - firstRegister; ++i) {
      registers[i];
    }
    int responseLength = (2 * (lastRegister - firstRegister + 1) + 2);

    char responseBuffer[255];
    responseBuffer[0] = transactionIdentifier >> 8;
    responseBuffer[1] = transactionIdentifier & 0xFF;
    responseBuffer[2] = protocolIdentifier >> 8;
    responseBuffer[3] = protocolIdentifier & 0xFF;
    responseBuffer[4] = responseLength >> 8;
    responseBuffer[5] = responseLength & 0xFF;
    responseBuffer[6] = unitIdentifier;
    responseBuffer[7] = functionCode;

    for (int i = 0; i < ((responseLength / 2) - 2); i++) {
      int registerValue = registers[firstRegister + i];
      uint16_t registerValueUnsigned = static_cast<uint16_t>(registerValue);
      responseBuffer[8 + 2 * i] = registerValueUnsigned >> 8;
      responseBuffer[9 + 2 * i] = registerValueUnsigned & 0xFF;
    }

    client.write(responseBuffer, responseLength + 6);
  } else if (functionCode == 0x06) {
    int registerAddress = (buffer[8] << 8) | buffer[9];
    int newValue = (buffer[10] << 8) | buffer[11];
    registers[registerAddress] = newValue;
    Serial.print("Se escribió el valor ");
    Serial.print(newValue);
    Serial.print(" en el registro ");
    Serial.println(registerAddress);
    client.write(buffer, length + 6);
  }
}

void recibirTrama() {
  WiFiClient client = server.available();
  while (client.connected()) {
    if (client.available()) {
      char buffer[128];
      int bytesRead = client.readBytes(buffer, sizeof(buffer));
      if (bytesRead > 0) {
        Serial.print("Bytes leídos: ");
        for (int i = 0; i < bytesRead; i++) {
          Serial.print(buffer[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        processModbusData(buffer, client);
        client.stop();
      } else {
        Serial.println("No se leyeron datos.");
      }
    }
  }
}

