#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFiUdp.h>

#define wifi_ssid "12MiCasa12-24G"
#define wifi_password "Almunia-12MZN"
#define mqtt_server "192.168.1.55"

/*asignacion pines  ESP8266 */
/*CAMBIO Diciembre2019- se quita reles, solo se usa temperatura*/
/*int rele1 = 14;            //CALDERA D5 
int rele2 = 12;            //SIRENA  D6
int rele3 = 13;             // D7
int rele4 = 15;             // D8 
*/
int Builtin_Led = 2;
#define ONE_WIRE_BUS 2  // DS18B20 pin  D4

// Update these with values suitable for your network.
IPAddress ip(192,168,1,200);  //Node static IP
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

/*instancias*/
WiFiClient espClient;
PubSubClient client(espClient);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
DeviceAddress tempDeviceAddress;

/*Variables Globales*/
// Topic con el que trabajamos
/*const char* topic_OnOffCaldera = "OnOffCaldera";
const char* topic_EstadoCaldera = "EstadoCaldera";
*/
const char* topic_Temp_Lavadora = "Temperatura_Lavadora";

//variables globales
/*String Estado_caldera = "Desconocido";*/
String inData; 
long previousMillis = 0;
long lastTemp = 0;
float temp=0;
long lastMsg = 0;
float offset= -2; 

void byteToChar(byte* bytes, char* chars, unsigned int count){
    for(unsigned int i = 0; i < count; i++)
       chars[i] = (char)bytes[i];
}

/*FUNCIONES
void callback(char* topic, byte* payload, unsigned int length) { /*Que hacer cuadno llega un mensaje
 //gestionar los mensajes recibidos en los topics subscritos    
   Serial.print("Mensaje recibido [");  Serial.print(topic);  Serial.print("] ");
    String dato="";
    for (int i = 0; i < length ; i++) { 
        Serial.print((char)payload[i]);
        dato=dato+(char)payload[i];
    }
  Serial.println("");
    Serial.println(dato);
   // Comprobar si hay que subir o bajar
    if ( dato.equals("Encender")) {
    digitalWrite(2, LOW);   // Turn the LED on (Note that LOW is the voltage level
    EncenderCaldera();
  } else if (dato.equals("Apagar")){
    digitalWrite(2, HIGH);  // Turn the LED off by making the voltage HIGH
    ApagarCaldera();
  }  else if (dato.equals("Toggle")){
    toggle();

  }
 }*/
 

void setup() {
  pinMode(2, OUTPUT);        /* builtin led*/
  digitalWrite(2, LOW);      /*encender led  */   
 /* pinMode(rele1, OUTPUT);  
  pinMode(rele2, OUTPUT);  
  pinMode(rele3, OUTPUT);  
  pinMode(rele4, OUTPUT); 
  digitalWrite(rele1, LOW);     //poner a 0 los reles
  digitalWrite(rele2, LOW);
  */ 
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
/*  client.setCallback(callback); */
   ArduinoOTA.setHostname("HostPlacaReles");
  if (!MDNS.begin("ESP_Placa_Reles_Caldera")) {
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
  DS18B20.begin();
 DS18B20.getAddress(tempDeviceAddress, 0);
 DS18B20.setResolution(tempDeviceAddress,12);
  digitalWrite(2, HIGH);//apagar led
}

void setup_wifi() {
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
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "PlacaReles";
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe("OnOffCaldera"); /*SUSCRIBIRSE A LOS IN TOPICS*/
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  ArduinoOTA.handle();
 
  //enviar topics ciclicamente para refrescar datos
 long now = millis();  //temporizar publicaciones
  if (now - lastMsg > 10000) {
    lastMsg = now;
    GetTemperature();
    Publicar();
  }
  
}

void GetTemperature(){
  DS18B20.requestTemperatures(); 
  temp = DS18B20.getTempCByIndex(0) + offset;
  Serial.println("Temperatura:");
  Serial.println(temp);
}

/*
void ApagarCaldera(){
 digitalWrite(rele1,LOW);
 Estado_caldera = "Apagada";
 digitalWrite(2, HIGH);
 Publicar();
}

void EncenderCaldera(){
 digitalWrite(rele1,HIGH);
  Estado_caldera = "Encendida";
 digitalWrite(2, LOW);
 Publicar();
}

void toggle(){
 if (Estado_caldera.equals("Apagado")){
  EncenderCaldera();
 } else {
  ApagarCaldera();
 }
}
*/
void Publicar(){
/*  client.publish("EstadoCaldera", String(Estado_caldera).c_str(), true);*/
  client.publish("Temperatura_Lavadora", String(temp).c_str(), true);
}
