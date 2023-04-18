#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <WiFi.h>

#define DHTPIN 26      // DHT22 sensor pin
#define DHTTYPE DHT22  // DHT22 sensor type
DHT dht(DHTPIN, DHTTYPE);

const int PIR_PIN = 33;    // PIR sensor pin
const int LIGHT_PIN = 21;  // Light control pin
const int FAN_PIN = 22;    // Fan control pin
const int TEMT_PIN = 34;   // TEMT6000 sensor pin
int pirValue;
int lightValue;
float h;
float t;
float volts;      // Convert reading to voltage
float amps;       // Convert to amps across 10K resistor
float microamps;  // Convert amps to microamps

int pirState = LOW;
int lightState = LOW;
int fanState = LOW;
int temt6000Value = 0;

const char *ssid = "GalaxyA52s";
const char *password = "mlea4130";
bool lightAuto = true;
bool fanAuto = true;

WiFiServer server(80);
IPAddress local_IP(192, 168, 43, 164);
//it wil set the gateway static IP address to 192, 168, 43, 121
IPAddress gateway(192, 168, 43, 121);

// Following three settings are optional
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

unsigned long motionStartTime = 0;
unsigned long currentTime = 0;
unsigned long duration = 8000;  // 1 minute in milliseconds

unsigned long previousMillis = 0;
const long interval = 5000;  // Update interval in milliseconds

void setup() {
  // Initialize serial and wait for port to open:
  pinMode(PIR_PIN, INPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  // Serial.begin(9600);

  dht.begin();
  Serial.begin(115200);

  // This part of code will try create static IP address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  delay(1000);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  server.begin();
}

void loop() {
  // Check if a client has connected
  WiFiClient client = server.available();

  //  Your code here
  pirValue = digitalRead(PIR_PIN);
  lightValue = analogRead(TEMT_PIN);
  h = dht.readHumidity();
  t = dht.readTemperature();
  // temperature = t;
  // humidity = h;
  volts = lightValue * 5.0 / 1024.0;  // Convert reading to voltage
  amps = volts / 10000.0;             // Convert to amps across 10K resistor
  microamps = amps * 1000000.0;       // Convert amps to microamps
  // luminosity = microamps * 2.0;      // 2 microamps = 1 lux

  if (pirValue == HIGH) {
    if (pirState == LOW) {
      Serial.println("Motion detected!");
      pirState = HIGH;
      // motion = pirState;
    }
  } else {
    if (pirState == HIGH) {
      Serial.println("Motion ended.");
      pirState = LOW;
      // motion = pirState;
    }
  }

  if (millis() - previousMillis >= interval) {
    previousMillis = millis();
    temt6000Value = map(lightValue, 0, 4095, 0, 100);
    Serial.print("Light level: ");
    Serial.print(temt6000Value);
    Serial.println("%");
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.println(" °C");
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.println(" %");
  }

  if (pirState == HIGH) {
    if (lightAuto) {
      currentTime = millis();  // get the current time
      if (lightState == LOW && temt6000Value < 30) {
        digitalWrite(LIGHT_PIN, HIGH);
        Serial.println("Light on");
        lightState = HIGH;
        motionStartTime = currentTime;  // record the motion start time
      } else if (lightState == HIGH && temt6000Value < 50) {
        digitalWrite(LIGHT_PIN, HIGH);
        Serial.println("Light on");
        lightState = HIGH;
        motionStartTime = currentTime;  // record the motion start time
      } else {
        digitalWrite(LIGHT_PIN, LOW);
        lightState = LOW;
        motionStartTime = 0;  // reset the motion start time
      }
    }
    if (fanAuto) {
      currentTime = millis();  // get the current time
      if (fanState == LOW && t > 25) {
        digitalWrite(FAN_PIN, HIGH);
        Serial.println("Fan on");
        fanState = HIGH;
        motionStartTime = currentTime;  // record the motion start time
      } else if (fanState == HIGH && t > 25) {
        digitalWrite(FAN_PIN, HIGH);
        Serial.println("Fan on");
        fanState = HIGH;
        motionStartTime = currentTime;  // record the motion start time
      } else {
        digitalWrite(FAN_PIN, LOW);
        Serial.println("Fan on");
        fanState = LOW;
        motionStartTime = 0;  // reset the motion start time
      }
    }
  }

  //When motion is undetected
  else {
    currentTime = millis();  // get the current time
    if (lightAuto && lightState == HIGH && currentTime - motionStartTime >= duration) {
      digitalWrite(LIGHT_PIN, LOW);
      Serial.println("Light off");
      lightState = LOW;
      motionStartTime = 0;  // reset the motion start time
    }
    if (fanAuto && fanState == HIGH && currentTime - motionStartTime >= duration) {
      digitalWrite(FAN_PIN, LOW);
      Serial.println("Fan off");
      fanState = LOW;
      motionStartTime = 0;  // reset the motion start time
    }
    if (client)
      while (client.connected())
        if (client.available()) {
          String message = client.readStringUntil('\r');
          Serial.println(message);
          if (message.indexOf("light-auto") != -1) {
            lightAuto = true;
          }

          if (message.indexOf("fan-auto") != -1) {
            fanAuto = true;
          }
          if (lightState == LOW) {
            // onLightChange();
            if (message.indexOf("light-on") != -1) {
              lightAuto = false;
              digitalWrite(LIGHT_PIN, HIGH);
              Serial.println("Light On");
              lightState = HIGH;
            }
          }
          if (lightState == HIGH) {
            // onLightChange();
            if (message.indexOf("light-off") != -1) {
              lightAuto = false;
              digitalWrite(LIGHT_PIN, LOW);
              Serial.println("Light Off");
              lightState = LOW;
            }
          }
          if (fanState == LOW) {
            // onFanChange();
            if (message.indexOf("fan-on") != -1) {
              fanAuto = false;
              digitalWrite(FAN_PIN, HIGH);
              Serial.println("Fan On");
              fanState = HIGH;
            }
          }
          if (fanState == HIGH) {
            // onFanChange();
            if (message.indexOf("fan-off") != -1) {
              fanAuto = false;
              digitalWrite(FAN_PIN, LOW);
              Serial.println("Fan Off");
              fanState = LOW;
            }
          }
        }
  }

  // Send a response to the client


  delay(50);

  // Close the connection
  client.stop();
}