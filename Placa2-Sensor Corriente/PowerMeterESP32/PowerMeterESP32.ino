#include <WiFi.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MHDS18B20.h"
#include <HardwareSerial.h>
#include <PZEM004T.h>


#define wifi_ssid "12MiCasa12-24G"
#define wifi_password "Almunia-12MZN"
#define mqtt_server "192.168.1.55"
#define Estado_Voltaje_topic "EstadoVoltaje"
#define Estado_Intensidad_topic "EstadoIntensidad"
#define Estado_Potencia_topic "EstadoPotencia"
#define Estado_Energia_topic "EstadoEnergia"
#define Estado_Temperatura_topic "EstadoTemperaturaPowerMeter"

/*asignacion pines  */
const int pinSensorTemp = 21;   // pin D2
// Direcciones de los sensores de temperatura  
DeviceAddress Temperatura1;  
OneWire ds(pinSensorTemp);   
DallasTemperature sensors(&ds);  

// Update these with values suitable for your network.
IPAddress ip(192,168,1,181);  //Node static IP
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

/*instancias*/
WiFiClient espClient;
PubSubClient client(espClient);

HardwareSerial PzemSerial2(2);     // Use hwserial UART2 at pins IO-16 (RX2) and IO-17 (TX2)
PZEM004T pzem(&PzemSerial2);
IPAddress ip2(192,168,1,1);

/*Variables Globales*/
long lastMsg = 0;
long lastMsg2 = 0;
int n=0;
float Irms = 0;
float Potencia = 0;
float Energia = 0;
float Temperatura = 0;
float Voltaje=0;


/*FUNCIONES*/
void callback(char* topic, byte* payload, unsigned int length) { /*Que hacer cuadno llega un mensaje*/
 //gestionar los mensajes recibidos en los topics subscritos    
   Serial.print("Mensaje recibido [");  Serial.print(topic);  Serial.print("] ");
    String dato="";
    for (int i = 0; i < length ; i++) { 
        Serial.print((char)payload[i]);
        dato=dato+(char)payload[i];
    }
  Serial.println("");
    Serial.println(dato);
   // Analizar datos
    if ( dato.equals("Publicar")) {
    digitalWrite(2, LOW);   // Turn the LED on (Note that LOW is the voltage level
    Publicar();
   }
 }

void Publicar(){
  client.publish(Estado_Voltaje_topic, String(Voltaje).c_str(), true);
  client.publish(Estado_Intensidad_topic, String(Irms).c_str(), true);
  client.publish(Estado_Potencia_topic, String(Potencia).c_str(), true);
  client.publish(Estado_Energia_topic, String(Energia).c_str(), true);
  client.publish(Estado_Temperatura_topic, String(Temperatura).c_str(), true);  
}


void setup() {
  pinMode(2, OUTPUT);        /* builtin led*/
  digitalWrite(2, LOW);      /*encender led  */   
  pinMode(pinSensorTemp, INPUT); 
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  ArduinoOTA.setHostname("PowerMeter");
  if (!MDNS.begin("ESP_PowerMeter")) {
    Serial.println("Error setting up MDNS responder!");
  }
  Serial.println("mDNS responder started");
  MDNS.addService("http", "tcp", 80); // Announce esp tcp service on port 8080
  MDNS.addService("mqtt", "tcp", 1883); // Announce esp tcp service on port 8080

  ArduinoOTA.onStart([]() {
   Serial.println("Start");
  });
 ArduinoOTA.onEnd([]() {
  Serial.println("\nEnd");
 });
 ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
 });
 ArduinoOTA.onError([](ota_error_t error) {
  Serial.printf("Error[%u]: ", error);
   if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
 });
 ArduinoOTA.begin();
  digitalWrite(2, HIGH);//apagar led
 
}

void setup_wifi() {
  digitalWrite(2, HIGH);      /*encender led  */   
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet,gateway);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(2, LOW);//apagar led
}

void reconnect() {
  // Loop until we're reconnected
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "PwrMeter";
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe("AccionPowerMeter"); /*SUSCRIBIRSE A LOS IN TOPICS*/
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      //delay(5000);
    }
  }
}

void LecturaPower(){
 Voltaje = pzem.voltage(ip2);
 if (Voltaje < 0.0) Voltaje = 0.0;
   //Serial.print(Voltaje);Serial.print("V; ");
   Irms = pzem.current(ip2);
   //if(Irms >= 0.0){ Serial.print(Irms);Serial.print("A; "); }
   Potencia = pzem.power(ip2);
   //if(Potencia >= 0.0){ Serial.print(Potencia);Serial.print("W; "); }
   Energia = pzem.energy(ip2);
   //if(Energia >= 0.0){ Serial.print(Energia);Serial.print("Wh; "); }
  //Serial.println();  
}

void LeerTemperatura(){
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
 // float celsius, fahrenheit;
  
  if ( !ds.search(addr)) {
   // Serial.println("No more addresses.");
   // Serial.println();
    ds.reset_search();
    delay(150);
    return;
  }
  
  //Serial.print("ROM =");
  //for( i = 0; i < 8; i++) {
  //  Serial.write(' ');
  //  Serial.print(addr[i], HEX);
  //}

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  //Serial.println();
 
  // the first ROM byte indicates which chip
  //switch (addr[0]) {
  //  case 0x10:
  //    Serial.println("  Chip = DS18S20");  // or old DS1820
  //    type_s = 1;
  //    break;
  //  case 0x28:
  //    Serial.println("  Chip = DS18B20");
  //    type_s = 0;
  //    break;
  //  case 0x22:
  //    Serial.println("  Chip = DS1822");
  //    type_s = 0;
  //    break;
  //  default:
  //    Serial.println("Device is not a DS18x20 family device.");
  //    return;
  //} 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(800);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  //Serial.print("  Data = ");
  //Serial.print(present, HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  //  Serial.print(data[i], HEX);
  //  Serial.print(" ");
  }
  //Serial.print(" CRC=");
  //Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  Temperatura = (float)raw / 16.0;
  //fahrenheit = celsius * 1.8 + 32.0;
  //Serial.print("  Temperature = ");
  //Serial.print(celsius);
  //Serial.print(" Celsius, ");
  //Serial.print(fahrenheit);
  //Serial.println(" Fahrenheit"); 
 
}



void loop() {
  
  client.loop();
  ArduinoOTA.handle();
 //hacer cada 10 segundo
 long now2 = millis();  //temporizar conectar wifi
  if (now2 - lastMsg2 > 10000) {
    lastMsg2 = now2;
    if (!client.connected()) {
    reconnect(); //check si esta desconectado de mosquitto y reconectar
  }
 }
 //hacer cada segundo
 long now = millis();  //temporizar publicaciones
  if (now - lastMsg > 1000) {
    LecturaPower();            //leer datos tension, intensidad, potencia y energia
    lastMsg = now;
    LeerTemperatura();
    Publicar();
    reconnect(); //check si esta desconectado de mosquitto y reconectar
  }
  
}
