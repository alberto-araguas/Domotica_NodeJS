#include <ESP8266WiFi.h>
#include <PubSubClient.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#define wifi_ssid "MiCasa"
#define wifi_password "1212cherokee"
#define mqtt_server "192.168.1.55"

#define Estado_Persiana_topic "EstadoPersianaOffice"
//#define Temperatura_topic ""
//#define ONE_WIRE_BUS 0  // DS18B20 pin  D3

/*asignacion pines  NodeMCU */
const int pinSubir = 5; // D1
const int pinBajar = 4; // D2
const int pulsadorSubir = 14; //D5
const int pulsadorBajar = 12; //D6

long tiempo_temp=45000;  //tiempo subir o bajar persianas

// Update these with values suitable for your network.
IPAddress ip(192,168,1,202);  //Node static IP
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
/*instancias*/
WiFiClient espClient;
PubSubClient client(espClient);
//OneWire oneWire(ONE_WIRE_BUS);
//DallasTemperature DS18B20(&oneWire);
//DeviceAddress tempDeviceAddress;
/*Variables Globales*/
String EstadoPersiana = "Desconocido";
long lastMsg = 0;
long lastTemp = 0;
long lastMsgMqtt;
float temp=0;
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
   // Comprobar si hay que subir o bajar
    if ( dato.equals("Subir")) {
    digitalWrite(2, LOW);   // Turn the LED on (Note that LOW is the voltage level
    subir();
  } else if (dato.equals("Bajar")){
    digitalWrite(2, HIGH);  // Turn the LED off by making the voltage HIGH
    bajar();
  }  else if (dato.equals("Parar")){
    parar();
    EstadoPersiana="Parada";
  }
 }

void setup() {
  pinMode(2, OUTPUT);        /* builtin led*/
  digitalWrite(2, LOW);      /*encender led  */   
  pinMode(pinSubir, OUTPUT); /*Pin Subir*/   
  pinMode(pinBajar, OUTPUT);  /*Pin Bajar*/
  pinMode(pulsadorSubir, INPUT_PULLUP);   /*Pin pulsador subir*/
  pinMode(pulsadorBajar, INPUT_PULLUP);   /*Pin pulsador bajar*/
  digitalWrite(pinSubir, HIGH);     //poner a 0 los reles
  digitalWrite(pinBajar, HIGH);
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
   ArduinoOTA.setHostname("HostOffice");
   //ArduinoOTA.setPassword((const char *)"123456");
   
  if (!MDNS.begin("ESP_Persiana_Office")) {
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
 //DS18B20.begin();
 //DS18B20.getAddress(tempDeviceAddress, 0);
 //DS18B20.setResolution(tempDeviceAddress,12);
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
    Serial.print("Attempting MQTT connection...");
    String clientId = "PersianaOffice";
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe("AccionPersianaOffice"); /*SUSCRIBIRSE A LOS IN TOPICS*/
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }
 
}
/*
void GetTemperature(){
  DS18B20.requestTemperatures(); 
  temp = DS18B20.getTempCByIndex(0);
  Serial.println("Temperatura:");
  Serial.println(temp);
}
*/
void leer_entradas_pulsadores(){
 int pSubir=digitalRead(pulsadorSubir);
 int pBajar=digitalRead(pulsadorBajar);
 if(pSubir==LOW && pBajar==HIGH){             //activos con low
   digitalWrite(2, LOW);   // Turn the LED on (Note that LOW is the voltage level
   subir();
   initPulsadores();
 }else if(pSubir==HIGH && pBajar==LOW){
   digitalWrite(2, HIGH);  // Turn the LED off by making the voltage HIGH
   bajar();
   initPulsadores();
 }else if(pSubir==LOW && pBajar==LOW){
   parar();
   EstadoPersiana="Parada";
   initPulsadores();
 }
 delay(300);
}

void loop() {
  //Temporizacion para chekear si el broquer Mqtt esta activo, si no es asi reintenta conectarse a los 5 seg
  long nowMqtt=millis(); 
  if (!client.connected()) {
    if (nowMqtt - lastMsgMqtt > 5000) {
       lastMsgMqtt = millis();
       reconnect();
    }
  } else {
   client.loop();
   //enviar topics ciclicamente para refrescar datos
   long now = millis();  //temporizar publicaciones
   if (now - lastMsg > 30000) {
    lastMsg = now;
    //  GetTemperature();
    Publicar();
   }
  }
  
  ArduinoOTA.handle();
  leer_entradas_pulsadores();
 
  long nowT = millis(); //temporizar accion persianas
  if (nowT - lastTemp > tiempo_temp) {
    if (EstadoPersiana.equals("Subiendo")||EstadoPersiana.equals("Bajando") ){
     parar();
    }
    lastTemp = nowT;
  }

}

void subir(){
 digitalWrite(pinBajar,HIGH);
 delay(150); /*retraso para que no se enganchen los reles chinos*/
 digitalWrite(pinSubir,LOW);
 digitalWrite(2, LOW);//encender led    
 EstadoPersiana = "Subiendo";
 lastTemp = millis();
 Publicar();
 }

void bajar(){
 digitalWrite(pinSubir,HIGH);
 delay(150); /*retraso para que no se enganchen los reles chinos*/
 digitalWrite(pinBajar,LOW);
 digitalWrite(2, LOW);//encender led    
 EstadoPersiana = "Bajando";
 lastTemp = millis();
 Publicar();
}

void parar(){
 digitalWrite(pinBajar,HIGH);
 digitalWrite(pinSubir,HIGH);
 digitalWrite(2, HIGH);//apagar led    
 if (EstadoPersiana.equals("Subiendo")){
  EstadoPersiana="Arriba";
 } else if (EstadoPersiana.equals("Bajando"))  {
  EstadoPersiana="Abajo";
 } else {
  EstadoPersiana="Parada";
 }
 Publicar(); 
}

void Publicar(){
  client.publish(Estado_Persiana_topic, String(EstadoPersiana).c_str(), true);
//  client.publish(Temperatura_topic, String(temp).c_str(), true);
}

void initPulsadores(){
  pinMode(pulsadorSubir, OUTPUT);   /*Pin pulsador subir*/
  pinMode(pulsadorBajar, OUTPUT);   /*Pin pulsador bajar*/
  delay(10);
  pinMode(pulsadorSubir, INPUT_PULLUP);   /*Pin pulsador subir*/
  pinMode(pulsadorBajar, INPUT_PULLUP);   /*Pin pulsador bajar*/
}
