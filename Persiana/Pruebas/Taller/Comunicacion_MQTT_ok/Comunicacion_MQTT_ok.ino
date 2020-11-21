#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Bounce2.h>


//declaraciones
const char* ssid = "MiCasa";
const char* password = "1212cherokee";
const char* mqtt_server = "192.168.1.55";/*"IP Mosquitto server";*/
/*Lista de topics*/
const char* TopicIn1 = "AccionPersianaTaller"; /*Subir, bajar o parar*/
const char* TopicOutIP = "IPTaller";
const char* TopicOut1 = "EstadoPersianaTaller"; /*Arriba, abajo, parada, averia*/
const char* TopicOut2 = "TemperaturaTaller";
const char* TopicOut3 = "EstadoEquipoTaller";
/*Lista de pines*/
int pinSubir = 1;
int pinBajar = 3;
int pulsadorSubir = 4;
int pulsadorBajar = 5;
//tiempo subir o bajar persianas
int tiempo_temp=35000;
/*instancias*/
WiFiClient espClient;
PubSubClient client(espClient);
// Instantiate a Bounce object
Bounce debouncer1 = Bounce(); 
Bounce debouncer2 = Bounce(); 

String EstadoPersiana = "Desconocido";
String temperatura= "20";
long lastMsg = 0;
char msg[50];
long lastTemp=0;
IPAddress ipLocal;

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



void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "nodeMCUTaller";
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
        
      // ... and resubscribe
      client.subscribe(TopicIn1);
      client.subscribe(TopicOutIP);
      client.subscribe(TopicOut1);
      client.subscribe(TopicOut2);
      client.subscribe(TopicOut3);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  pinMode(pinSubir, OUTPUT); /*Pin Subir*/    
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);//encender led     
  pinMode(pinBajar, OUTPUT);  /*Pin Bajar*/
  pinMode(pulsadorSubir, INPUT);   /*Pin pulsador subir*/
  pinMode(pulsadorBajar, INPUT);   /*Pin pulsador bajar*/ 
  // Activate internal pull-up (optional) 
  digitalWrite( pulsadorSubir ,HIGH);
  digitalWrite( pulsadorBajar ,HIGH);
  debouncer1.attach(pulsadorSubir);
  debouncer1.interval(150); // interval in ms
  debouncer2.attach(pulsadorBajar);
  debouncer2.interval(150); // interval in ms
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  digitalWrite(2, HIGH);//apagar led
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  //enviar topics ciclicamente para refrescar datos
 long now = millis();  //temporizar publicaciones
  if (now - lastMsg > 4000) {
    lastMsg = now;
    Publicar();
  }
  long nowT = millis(); //temporizar accion persianas
  if (nowT - lastTemp > tiempo_temp) {
    lastTemp = now;
    parar();
  }
  debouncer1.update();
  debouncer2.update();
  /*Leer pulsadores*/
  int value1 = debouncer1.read();
  int value2 = debouncer2.read();
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);
   WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  ipLocal=WiFi.localIP();
}

void subir(){
 digitalWrite(pinBajar,LOW);
 delay (10);
 digitalWrite(pinSubir,HIGH);
 digitalWrite(2, LOW);//encender led    
 EstadoPersiana = "Subiendo";
}
void bajar(){
 digitalWrite(pinSubir,LOW);
 delay (10);
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
 } 
}

void Publicar(){
    EstadoPersiana.toCharArray(msg,15);
    client.publish(TopicOut1, msg);
    temperatura.toCharArray(msg,10);
    client.publish(TopicOut2, msg);
    Serial.println(ipLocal);
    //ipLocal.toCharArray(msg,15);
    client.publish(TopicOutIP, msg);
}

