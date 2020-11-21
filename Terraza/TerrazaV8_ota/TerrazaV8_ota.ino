#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>


#define wifi_ssid "12MiCasa12-24G"
#define wifi_password "Almunia-12MZN"
#define mqtt_server "192.168.1.55"
#define humidity_topic "ExtHumedad"
#define clima_topic "ExtClima"
#define lum_topic "ExtLux"
#define temperature_topic "ExtTemperatura"
#define DHTTYPE DHT22

//asignacion pines
const int DHTPIN=14;
const int RED = 15;
const int GREEN = 12;
const int BLUE = 13;
const int PINLLUVIA= 5;
// Update these with values suitable for your network.
IPAddress ip(192,168,1,128);  //Node static IP
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

//instancias
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

void setup() {
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(PINLLUVIA, INPUT);
  digitalWrite(BLUE,HIGH);
  Serial.begin(115200);
  Serial.println("Starting...");
  setup_wifi();
  client.setServer(mqtt_server, 1883);
 // Start sensor
  dht.begin();
   if (!MDNS.begin("Terraza")) {
    Serial.println("Error setting up MDNS responder!");
  }
  Serial.println("mDNS responder started");
  MDNS.addService("http", "tcp", 80); // Announce esp tcp service on port 8080
  MDNS.addService("mqtt", "tcp", 1883); // Announce esp tcp service on port 8080
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  Serial.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", "Terraza");
  digitalWrite(BLUE,LOW);
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
    digitalWrite(GREEN,LOW);
    digitalWrite(RED,HIGH); 
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("Terraza")) {
      digitalWrite(GREEN,HIGH);
      digitalWrite(RED,LOW);
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


long lastMsg = 0;
float temp = 0.0;
float hum = 0.0;


void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  httpServer.handleClient();
  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    // Check if any reads failed and exit early (to try again).
    if (isnan(t) || isnan(h) ) {
     Serial.println("Failed to read from DHT sensor!");
     return;
    }else{   //si la medida es correcta se almacena
     temp=t;
     hum=h;
    }
    Serial.println(String(temp).c_str());
    client.publish(temperature_topic, String(temp).c_str(), true);
    Serial.println(String(hum).c_str());
    client.publish(humidity_topic, String(hum).c_str(), true);
    //leer estado clima
    boolean lluvia = digitalRead(PINLLUVIA);
    String Clima="";
    if (lluvia){
     Clima="Mojado"; 
    } else {
     Clima="Seco";
    }
    client.publish(clima_topic, String(Clima).c_str(), true);
    int ldrValue= analogRead(A0);
    client.publish(lum_topic, String(ldrValue).c_str(), true);
  }
}
