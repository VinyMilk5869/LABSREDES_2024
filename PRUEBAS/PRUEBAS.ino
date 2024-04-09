#include <Wire.h>
#include <WiFi.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>

LiquidCrystal_I2C lcd(0x27,16,2);  


float registers[12] = {0};
const char* ssid = "iPhonedeSara";
const char* password = "papijuancho";
const int ledPin1 = 15;
const int ledPin2 = 4;
WiFiServer server(502);
unsigned long previousMillis[2] = {0,0};
const uint8_t Direccion_S2 = 0x68; 
const uint8_t Acelerometro = 0x1C;
const uint8_t Giroscopio = 0x1B;
const uint8_t Acelerometro_Out = 0x3B;
const uint8_t Giroscopio_Out = 0x43;
bool ledState1 = LOW;
bool ledState2 = LOW;
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
int buttonPressCount =0;

#define BME_SCK 18   
#define BME_MISO 19  
#define BME_MOSI 23  
#define BME_CS 5     

#define    MPU9250_ADDRESS            0x68

#define    BME_280_ADDRESS            0x76

#define    GYRO_FULL_SCALE_2000_DPS   0x18


void setup() {
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);

  Serial.begin(9600);
  Serial.print("Conectando a ");
  Serial.println(ssid);

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

  Wire.begin();
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);

  SPI.begin(); // Inicializar SPI
  pinMode(BME_CS, OUTPUT);
  digitalWrite(BME_CS, HIGH);

  lcd.init();
  lcd.backlight();
  lcd.clear(); 
}

void loop() {
  lcd.setCursor(0, 0);               
  lcd.print("L1:");     
  lcd.setCursor(3,0);
  lcd.print(int(registers[0]));
  
  // lcd.setCursor(5, 0);               
  // lcd.print("L2:");     
  // lcd.setCursor(8,0);
  // lcd.print(int(registers[1]));

  lcd.setCursor(10, 0);               
  lcd.print("P:");     
  lcd.setCursor(12,0);
  lcd.print(int(registers[9]));


  lcd.setCursor(12, 1);              
  lcd.print("T:");  
  lcd.setCursor(14,1);
  lcd.print(int(registers[4]));

  lcd.setCursor(5, 0);               
  lcd.print("H:");     
  lcd.setCursor(7,0);
  lcd.print(int(registers[5]));
             
  lcd.setCursor(6, 1);   
  lcd.print("Y:");        
  lcd.setCursor(8,1);
  lcd.print(int(registers[7]));
  
  lcd.setCursor(0, 1);               
  lcd.print("X:");     
  lcd.setCursor(2,1);
  lcd.print(registers[6]/1000);

  delay(1000);          

  server.begin();
  recibirTrama(); 
  
  blinkLED1();
  blinkLED2();

  
  readMPU9250();


  float temperature = readTemperature();
  float humidity = readHumidity();
  float pressure = readPressure();

  
}


void blinkLED1() {
  // Si el valor del registro 0 es 1 y el valor del registro 2 es mayor que cero, realiza el parpadeo
  if (registers[0]==0){
    digitalWrite(ledPin1,LOW);
  }
  else{
    
    if (registers[0]==1 && registers[2]== 0) {
      digitalWrite(ledPin1,HIGH);
    }else{
      
      if (registers[0]==1 && registers[2]> 0){
      unsigned long currentMillis = millis();
      
      if (currentMillis - previousMillis1 >= registers[2]) {
        previousMillis1 = currentMillis;
        
        // Cambia el estado del LED 2
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
  // Si el valor del registro 1 es 1 y el valor del registro 3 es mayor que cero, realiza el parpadeo
  if (registers[1]==0){
    digitalWrite(ledPin2,LOW);
  }
  else{
    
    if (registers[1]==1 && registers[3]== 0) {
      digitalWrite(ledPin2,HIGH);
    }else{
      
      if (registers[1]==1 && registers[3]> 0){
      unsigned long currentMillis = millis();
      
      if (currentMillis - previousMillis2 >= registers[3]) {
        previousMillis2 = currentMillis;
        
        // Cambia el estado del LED 2
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
  float temperature = (float)temp_raw / 248.07;

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



void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}


void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}


void readMPU9250() {
  uint8_t Buf[14];
  Wire.beginTransmission(Direccion_S2);
  Wire.write(Acelerometro_Out);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)Direccion_S2, (uint8_t)14, true); // Request all 14 bytes
  for (int i = 0; i < 14; i++) {
    Buf[i] = Wire.read();
  }
  int16_t ax = (Buf[0] << 8) | Buf[1];  // Combine high and low bytes
  int16_t ay = (Buf[2] << 8) | Buf[3];
  int16_t az = (Buf[4] << 8) | Buf[5];
  registers[6] = ax;
  registers[7] = ay;
  registers[8] = az;
  int16_t gx = (Buf[8] << 8) | Buf[9];  // Combine high and low bytes
  int16_t gy = (Buf[10] << 8) | Buf[11];
  int16_t gz = (Buf[12] << 8) | Buf[13];
  registers[10] = gy;
  registers[11] = gz;
}



void processModbusData(char* buffer,WiFiClient &client){
  int transactionIdentifier = (buffer[0] << 8) | buffer[1];
  int protocolIdentifier = (buffer[2] << 8) | buffer[3];
  int length = (buffer[4] << 8) | buffer[5];
  int unitIdentifier = buffer[6];
  int functionCode = buffer[7];
 
  if (functionCode == 0x03) {
    // Función de lectura de múltiples registros
 
    // Obtener la posición del primer registro a leer
    int firstRegister = (buffer[8] << 8) | buffer[9];
 
    // Obtener la posición del último registro a leer
    int lastRegister = (buffer[10] << 8) | buffer[11];
 
    for (int i = 0; i <= lastRegister - firstRegister; ++i) {
        // Simulación de la lectura del registro desde algún lugar (puedes reemplazar con tu lógica de lectura real)
        registers[i];
    }
    // Calcular la longitud de los datos de la respuesta (en bytes)
    int responseLength = (2 * (lastRegister - firstRegister+1)+2);
 
    // Construir la trama de respuesta
    char responseBuffer[255]; // Tamaño del búfer de respuesta
    // Identificador de transacción
    responseBuffer[0] = transactionIdentifier >> 8; // High byte
    responseBuffer[1] = transactionIdentifier & 0xFF; // Low byte
    // Identificador de protocolo (Modbus TCP)
    responseBuffer[2] = protocolIdentifier >> 8; // High byte
    responseBuffer[3] = protocolIdentifier & 0xFF; // Low byte
    // Longitud
    responseBuffer[4] = responseLength >> 8; // High byte
    responseBuffer[5] = responseLength & 0xFF; // Low byte
    // Identificador de unidad
    responseBuffer[6] = unitIdentifier;
    // Código de función
    responseBuffer[7] = functionCode;
    // Cantidad de bytes de datos
    //responseBuffer[8] = responseLength; // Se asume que la cantidad de bytes de datos es igual a la longitud de la respuesta
    // Llenar el buffer de datos con los valores de los registros
//    for (int i = 0; i < ((responseLength / 2)-2); i++) {
//      // Simulación de la lectura del registro desde algún lugar (puedes reemplazar con tu lógica de lectura real)
//      // Aquí se asume que los valores de los registros están almacenados en un arreglo llamado "registers"
//      // Si tus registros están en otro lugar, deberás ajustar esta parte
//      int registerValue = registers[firstRegister + i];
//      // Escribir el valor del registro en el buffer de datos
//      responseBuffer[8 + 2 * i] = registerValue >> 8; // Alta orden de byte
//      responseBuffer[9 + 2 * i] = registerValue & 0xFF; // Baja orden de byte
//    }
      for (int i = 0; i < ((responseLength / 2)-2); i++) {
          int registerValue = registers[firstRegister + i];
          // Escribir el valor del registro en el buffer de datos
          uint16_t registerValueUnsigned = static_cast<uint16_t>(registerValue);
          responseBuffer[8 + 2 * i] = registerValueUnsigned >> 8; // Alta orden de byte
          responseBuffer[9 + 2 * i] = registerValueUnsigned & 0xFF; // Baja orden de byte
      }

 
    // Enviar la trama de respuesta al cliente
    client.write(responseBuffer, responseLength + 6); // Se añaden 6 bytes del encabezado Modbus TCP/IP
  }
 
  else if (functionCode == 0x06) {
    // Escritura de un solo registro o coil
    int registerAddress = (buffer[8] << 8) | buffer[9];
    int newValue = (buffer[10] << 8) | buffer[11];
    // Realizar operaciones para escribir el registro...
    registers[registerAddress]=newValue;
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
        for(int i=0;i< bytesRead;i++){
            Serial.print(buffer[i],HEX);
            Serial.print(" ");
        }
        Serial.println();
        processModbusData(buffer,client);
        //client.write(buffer, bytesRead);
        client.stop();
      } else {
        Serial.println("No se leyeron datos.");
      }
    }
  }
}