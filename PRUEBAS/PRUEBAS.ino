#include <Wire.h>
#include <WiFi.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);  


const char* LED1 = "OFF";
const char* LED2 = "OFF";
const int P = 22;
const int T = 25;
const int H = 68;
const int X = 42;



float registers[12] = {0};
const char* ssid = "Santiago2021";
const char* password = "santiago2001";
const int ledPin1 = 15;
const int ledPin2 = 4;
const int SSBME = 14;
const int SSMPU = 13;
const int pulsador= 27;
WiFiServer server(502);
unsigned long previousMillis[2] = {0,0};
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
int buttonPressCount =0;




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
  lcd.init();
  lcd.backlight();
  lcd.clear(); 
}

void loop() {
  lcd.setCursor(0, 0);               
  lcd.print("LED1:");     
  lcd.setCursor(5,0);
  lcd.print(LED1);
  lcd.setCursor(8, 0);               
  lcd.print("LED2:");     
  lcd.setCursor(13,0);
  lcd.print(LED2);

  lcd.setCursor(0, 1);               
  lcd.print("P:");     
  lcd.setCursor(2,1);
  lcd.print(P);
  lcd.setCursor(4, 1);               
  lcd.print("T:");     
  lcd.setCursor(6,1);
  lcd.print(T);
  lcd.setCursor(8, 1);               
  lcd.print("H:");     
  lcd.setCursor(10,1);
  lcd.print(H);
  lcd.setCursor(12, 1);               
  lcd.print("X:");     
  lcd.setCursor(14,1);
  lcd.print(X);
  delay(500);          

  server.begin();
  recibirTrama(); 
  blinkLED1();
  blinkLED2();
}














void blinkLED1() {
  // Si el valor del registro 0 es 1 y el valor del registro 2 es mayor que cero, realiza el parpadeo
  if (registers[0]==0){
    digitalWrite(ledPin1,LOW);
    LED1= "OFF";
  }
  else{
    
    if (registers[0]==1 && registers[2]== 0) {
      digitalWrite(ledPin1,HIGH);
      LED1= "ON ";
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
    LED2= "OFF";
  }
  else{
    
    if (registers[1]==1 && registers[3]== 0) {
      digitalWrite(ledPin2,HIGH);
      LED2= "ON ";
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