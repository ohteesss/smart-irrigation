#include <WiFi.h>
#include <DHT.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// WiFi credentials
const char* ssid = "oluwatobi";
const char* password = "123456789";

// MQTT Broker settings
const char* mqtt_server = ***"mqtt.mydevices.com";
const int mqtt_port = 8883;
const char* mqtt_username = "ohteesss";
const char* mqtt_password = "Project2025";
const char* client_id = "ESP32_Master";

// Sensor Configuration
#define DHTPIN 4         // Digital pin for DHT
#define DHTTYPE DHT11    // DHT 11 (AM2302)
#define MOISTURE1_PIN 34 // Analog pin for moisture sensor 1
#define MOISTURE2_PIN 35 // Analog pin for moisture sensor 2

const int RELAY_PIN = 12; // Pin for irrigation relay



// Moisture thresholds for hysteresis control
int MOISTURE_LOW_THRESHOLD = 30;  // Start irrigation below this (%)
int MOISTURE_HIGH_THRESHOLD = 70; // Stop irrigation above this (%)

bool irrigationOn = false;

unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 100000; // 100 seconds


DHT dht(DHTPIN, DHTTYPE);

// WiFi client
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

// Connect to MQTT broker
void connectMQTT() {
 unsigned long startTime = millis();
    const unsigned long timeout = 30000; // 30 seconds timeout
    


    while (!mqttClient.connected() ) {
        Serial.print("Attempting MQTT connection...");
        
        if (mqttClient.connect(client_id, mqtt_username, mqtt_password)) {
            Serial.println("connected");
            return;
        }
        
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" try again in 5 seconds");
        delay(5000);
    }
    
    if (!mqttClient.connected()) {
        Serial.println("MQTT connection timed out");
    }
}

void checkWiFi() {
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 10000) { // Check every 10 seconds
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected, reconnecting...");
            WiFi.disconnect();
            WiFi.begin(ssid, password);
        }
        lastCheck = millis();
    }
}

// Connect to WiFi
void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
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
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == "farm/control/moisture_low") {
    MOISTURE_LOW_THRESHOLD = message.toInt();
    Serial.printf("Updated LOW threshold: %d\n", MOISTURE_LOW_THRESHOLD);
  }
  else if (String(topic) == "farm/control/moisture_high") {
    MOISTURE_HIGH_THRESHOLD = message.toInt();
    Serial.printf("Updated HIGH threshold: %d\n", MOISTURE_HIGH_THRESHOLD);
  }
}

void setup() {
  Serial.begin(115200);


  espClient.setInsecure();

  setupWiFi();
  
 
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  
  connectMQTT();
  
  // Initialize sensors
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("DHT sensor initialized");

  dht.begin();
  
  // Set up WiFi in STA mode
  WiFi.mode(WIFI_STA);
  
   mqttClient.publish("microcontroller1/system", "ESP32 started");

   delay(5000);

}






void publishSensorData() {
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  int moisturePercentOne = map(analogRead(MOISTURE1_PIN), 0, 4095, 100, 0);
  int moisturePercentTwo = map(analogRead(MOISTURE2_PIN), 0, 4095, 100, 0);

 if (isnan(temp) || isnan(humidity)) {
      Serial.println("DHT read failed!");
      return;
    }

  Serial.println("Publishing sensor data...");
  Serial.printf("Temp: %.1f°C, Humidity: %.1f%%, Moisture1: %d%%, Moisture2: %d%%",
    temp, humidity, moisturePercentOne, moisturePercentTwo);

  String payload = "{";
  payload += "\"temperature\":" + String(temp) + ",";
  payload += "\"humidity\":" + String(humidity) + ",";
  payload += "\"moistureOne\":" + String(moisturePercentOne) + ",";
  payload += "\"moistureTwo\":" + String(moisturePercentTwo) + ",";
  payload += "}";

  mqttClient.publish("microcontroller1/sensors", payload.c_str());
}

void handleIrrigation(){
  int moisturePercentOne = map(analogRead(MOISTURE1_PIN), 0, 4095, 100, 0);
  int moisturePercentTwo = map(analogRead(MOISTURE2_PIN), 0, 4095, 100, 0);

  int moistureAverage = (moisturePercentOne + moisturePercentTwo) / 2;


  if (!irrigationOn && moistureAverage < MOISTURE_LOW_THRESHOLD ) {
    digitalWrite(RELAY_PIN, HIGH); // Turn on water pump
    irrigationOn = true;
    Serial.println("Irrigation ON");

     // Publish start message with timestamp
    String payload = "{";
    payload += "\"action\":\"started\",";
    payload += "\"timestamp\":" + String(millis());
    payload += "}";
    mqttClient.publish("microcontroller1/irrigation", payload.c_str());
  } else if (irrigationOn && moistureAverage > MOISTURE_HIGH_THRESHOLD)  {
    digitalWrite(RELAY_PIN, LOW); // Turn off water pump
    irrigationOn = false;
    Serial.println("Irrigation OFF");

     // Publish stop message with timestamp
    String payload = "{";
    payload += "\"action\":\"stopped\",";
    payload += "\"timestamp\":" + String(millis());
    payload += "}";
    mqttClient.publish("microcontroller1/irrigation", payload.c_str());
  }
}




void loop() {

  unsigned long lastLoopTime = millis();
  // Check for WiFi connection
  checkWiFi();
  // Maintain MQTT connection
  if (!mqttClient.connected()) {
    connectMQTT();
  }
while (millis() - lastLoopTime < 10000) {
    mqttClient.loop();
    delay(100);
}

  if (millis() - lastPublishTime >= publishInterval) {
    publishSensorData();
    lastPublishTime = millis();
  }


  handleIrrigation();

  delay(1000); 
}



