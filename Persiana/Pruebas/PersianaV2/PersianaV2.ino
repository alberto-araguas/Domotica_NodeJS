#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>


#define wifi_ssid "MiCasa"
#define wifi_password "1212cherokee"
#define mqtt_server "192.168.1.55"

#define Estado_Persiana_topic "EstadoPersiana"
#define Temperatura_topic "Temperatura"
#define ONE_WIRE_BUS 14  // DS18B20 pin  D5

/*asignacion pines  WEMOS D1 MINI */
const int pinSubir = 15; // D8
const int pinBajar = 13; // D7
const int pulsadorSubir = 5; //D1
const int pulsadorBajar = 4; //D2

int tiempo_temp=35000;  //tiempo subir o bajar persianas

/*instancias*/
WiFiClient espClient;
PubSubClient client(espClient);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

/*Variables Globales*/
String EstadoPersiana = "Desconocido";
String temperatura= "20";
long lastMsg = 0;
long lastTemp = 0;
long temp=0;
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
  pinMode(pinSubir, OUTPUT); /*Pin Subir*/    
  pinMode(2, OUTPUT);        /* builtin led*/
  digitalWrite(2, LOW);      /*encender led  */   
  pinMode(pinBajar, OUTPUT);  /*Pin Bajar*/
  pinMode(pulsadorSubir, INPUT);   /*Pin pulsador subir*/
  pinMode(pulsadorBajar, INPUT);   /*Pin pulsador bajar*/
  digitalWrite(pulsadorSubir, HIGH); /*Activar pullups*/
  digitalWrite(pulsadorBajar, HIGH); 
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  digitalWrite(2, HIGH);//apagar led
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);
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
    String clientId = "Persiana";
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe("AccionPersiana"); /*SUSCRIBIRSE A LOS IN TOPICS*/
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void GetTemperature(){
  DS18B20.requestTemperatures(); 
  temp = DS18B20.getTempCByIndex(0);
  Serial.println("TEmperatura:");
  Serial.println(temp);
}

void leer_entradas_pulsadores(){
 int pSubir=digitalRead(pulsadorSubir);
 int pBajar=digitalRead(pulsadorBajar);
 if(pSubir==HIGH && pBajar==LOW){
   digitalWrite(2, LOW);   // Turn the LED on (Note that LOW is the voltage level
   subir();
 }else if(pSubir==LOW && pBajar==HIGH){
   digitalWrite(2, HIGH);  // Turn the LED off by making the voltage HIGH
   bajar();
 }else if(pSubir==HIGH && pBajar==HIGH){
   parar();
   EstadoPersiana="Parada";
 }
 delay(500); 
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  leer_entradas_pulsadores();
  //enviar topics ciclicamente para refrescar datos
 long now = millis();  //temporizar publicaciones
  if (now - lastMsg > 5000) {
    lastMsg = now;
    GetTemperature();
    Publicar();
  }
  long nowT = millis(); //temporizar accion persianas
  if (nowT - lastTemp > tiempo_temp) {
    lastTemp = nowT;
    parar();
  }
  
}

void subir(){
 digitalWrite(pinBajar,LOW);
 delay(150); /*retraso para que no se enganchen los reles chinos*/
 digitalWrite(pinSubir,HIGH);
 digitalWrite(2, LOW);//encender led    
 EstadoPersiana = "Subiendo";
}

void bajar(){
 digitalWrite(pinSubir,LOW);
 delay(150); /*retraso para que no se enganchen los reles chinos*/
 digitalWrite(pinBajar,HIGH);
 digitalWrite(2, LOW);//encender led    
 EstadoPersiana = "Bajando";
}

void parar(){
 digitalWrite(pinBajar,LOW);
 digitalWrite(pinSubir,LOW);
 digitalWrite(2, HIGH);//apagar led    
 if (EstadoPersiana.equals("Subiendo")){
  EstadoPersiana="Arriba";
 } else if (EstadoPersiana.equals("Bajando"))  {
  EstadoPersiana="Abajo";
 } else {
  EstadoPersiana="Parada";
 }
}
void Publicar(){
  client.publish(Estado_Persiana_topic, String(EstadoPersiana).c_str(), true);
  client.publish(Temperatura_topic, String(temp).c_str(), true);
 
}
