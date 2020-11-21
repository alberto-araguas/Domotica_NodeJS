#include <ESP8266WiFi.h>
#include <PubSubClient.h>



#define wifi_ssid "12MiCasa12-24G"
#define wifi_password "Almunia-12MZN"
#define mqtt_server "192.168.1.55"


/*asignacion pines  NodeMCU */
const int pinSubir = 5; // D1
const int pinBajar = 4; // D2
const int pulsadorSubir = 14; //D5
const int pulsadorBajar = 12; //D6

int tiempo_temp=35000;  //tiempo subir o bajar persianas

// Update these with values suitable for your network.
IPAddress ip(192,168,1,101);  //Node static IP
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
/*instancias*/
WiFiClient espClient;
PubSubClient client(espClient);


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

  } else if (dato.equals("Bajar")){
    digitalWrite(2, HIGH);  // Turn the LED off by making the voltage HIGH

  }  else if (dato.equals("Parar")){

    EstadoPersiana="Parada";
  }
 }

void setup() {
  pinMode(2, OUTPUT);        /* builtin led*/
  digitalWrite(2, LOW);      /*encender led  */   
  pinMode(pinSubir, OUTPUT); /*Pin Subir*/   
  pinMode(pinBajar, OUTPUT);  /*Pin Bajar*/
  pinMode(pulsadorSubir, INPUT);   /*Pin pulsador subir*/
  pinMode(pulsadorBajar, INPUT);   /*Pin pulsador bajar*/
  digitalWrite(pinSubir, HIGH);     //poner a 0 los reles
  digitalWrite(pinBajar, HIGH);
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
    String clientId = "Persiana2Comedor";
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe("AccionPersiana2Comedor"); /*SUSCRIBIRSE A LOS IN TOPICS*/
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



void leer_entradas_pulsadores(){
 int pSubir=digitalRead(pulsadorSubir);
 int pBajar=digitalRead(pulsadorBajar);

 Serial.print(pSubir);
 Serial.println(pBajar);
 if(pSubir==LOW && pBajar==HIGH){             //activos con low
   digitalWrite(2, LOW);   // Turn the LED on (Note that LOW is the voltage level
  
 }else if(pSubir==HIGH && pBajar==LOW){
   digitalWrite(2, HIGH);  // Turn the LED off by making the voltage HIGH
 

 }else if(pSubir==LOW && pBajar==LOW){
 
   EstadoPersiana="Parada";
 }
 delay(300); 
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  leer_entradas_pulsadores();

  
  }
 
  


void subir(){
 digitalWrite(pinBajar,HIGH);
 delay(150); /*retraso para que no se enganchen los reles chinos*/
 digitalWrite(pinSubir,LOW);
 digitalWrite(2, LOW);//encender led    
 EstadoPersiana = "Subiendo";

}

void bajar(){
 digitalWrite(pinSubir,HIGH);
 delay(150); /*retraso para que no se enganchen los reles chinos*/
 digitalWrite(pinBajar,LOW);
 digitalWrite(2, LOW);//encender led    
 EstadoPersiana = "Bajando";

}

void parar(){
 digitalWrite(pinBajar,HIGH);
 digitalWrite(pinSubir,HIGH);
 
 }



void Publicar(){

 
}
