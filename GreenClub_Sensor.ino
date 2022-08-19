#define VERSION "v0.1"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP32Servo.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#include <ArduinoJson.h>

WiFiManager wm;

#define LED_BUILTIN 2

String id;
int idOnServer = 0;
#define POST_FREQUENCY 3600000
unsigned int lastPost;

// BME280
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

// Servo
Servo myservo;
int servoPin = 18;

// Sensor data
byte packet[10];
int packetIndex = 0;
float temperature;
float pressure;
float humidity;
uint16_t pm25, pm10;

// Servo settings
#define SERVO_MIN 0
#define SERVO_MAX 50
#define SERVO_SENSOR temperature

// HTTPS Client
const String idRequest = "https://greenclub.world:1337/api/sensors?filters[chipId][$eq]=";
const String postRequest = "https://greenclub.world:1337/api/sensor-data";

const char* rootCACertificate = \
                                "-----BEGIN CERTIFICATE-----\n" \
                                "MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n" \
                                "TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n" \
                                "cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n" \
                                "WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n" \
                                "ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n" \
                                "MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n" \
                                "h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n" \
                                "0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n" \
                                "A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n" \
                                "T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n" \
                                "B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n" \
                                "B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n" \
                                "KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n" \
                                "OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n" \
                                "jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n" \
                                "qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n" \
                                "rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n" \
                                "HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n" \
                                "hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n" \
                                "ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n" \
                                "3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n" \
                                "NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n" \
                                "ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n" \
                                "TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n" \
                                "jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n" \
                                "oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n" \
                                "4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n" \
                                "mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n" \
                                "emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n" \
                                "-----END CERTIFICATE-----\n";


void setup() {
  id = generateId();

  Serial2.begin(9600);
  Serial.begin(115200);

  Serial.print("Green Club Sensor ");
  Serial.print(VERSION);
  Serial.print(" ID: ");
  Serial.println(id);

  pinMode(LED_BUILTIN, OUTPUT);

  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  wm.setConfigPortalBlocking(false);
  wm.setConfigPortalTimeout(240);
  if (wm.autoConnect(String(String("GreenClub-") + id).c_str())) {
    Serial.println("Connected to Wi-Fi.");
  }
  else {
    Serial.println("Configuration portal running...");
  }

  // Initialise BME280
  unsigned status;
  status = bme.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  // Setup servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000);
}

void loop() {
  wm.process();
  sensorHandler();

  if (WiFi.status() == WL_CONNECTED) {
    if (lastPost == 0 || millis() - lastPost >= POST_FREQUENCY) {
      // Send results
      postSensorData();
    }
  }
}

void postSensorData() {
  if (idOnServer == 0) {
    setClock();
    getSensorId();
  }

  WiFiClientSecure *client = new WiFiClientSecure;
  if (client) {
    client -> setCACert(rootCACertificate);
    {
      // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is
      HTTPClient https;

      Serial.print("[HTTPS] begin...\n");
      if (https.begin(*client, postRequest)) {  // HTTPS
        https.addHeader("Content-Type", "application/json");
        Serial.print("[HTTPS] POST...\n");
        // start connection and send HTTP header
        String output;
        StaticJsonDocument<128> doc;
        JsonObject data = doc.createNestedObject("data");
        data["sensor"] = idOnServer;
        data["pm25"] = pm25;
        data["pm10"] = pm10;
        data["temperature"] = temperature;
        data["humidity"] = humidity;
        data["pressure"] = int(pressure);
        serializeJson(doc, output);
        Serial.println(output);
        int httpCode = https.POST(output);

        if (httpCode > 0) {
          Serial.printf("[HTTPS] POST... code: %d\n", httpCode);
          if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            Serial.println("Success");
            lastPost = millis();
          }
        } else {
          Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
        }
        https.end();
      } else {
        Serial.printf("[HTTPS] Unable to connect\n");
      }

    }

    delete client;
  } else {
    Serial.println("Unable to create client");
  }
}

void getSensorId() {

  WiFiClientSecure *client = new WiFiClientSecure;
  if (client) {
    client -> setCACert(rootCACertificate);
    {
      // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is
      HTTPClient https;

      Serial.print("[HTTPS] begin...\n");
      if (https.begin(*client, String(idRequest + id).c_str())) {  // HTTPS
        Serial.print("[HTTPS] GET...\n");
        // start connection and send HTTP header
        int httpCode = https.GET();

        // httpCode will be negative on error
        if (httpCode > 0) {
          // HTTP header has been send and Server response header has been handled
          Serial.printf("[HTTPS] GET... code: %d\n", httpCode);

          // file found at server
          if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            String payload = https.getString();
            Serial.println(payload);
            StaticJsonDocument<1024> doc;
            DeserializationError error = deserializeJson(doc, payload);
            if (error) {
              Serial.print("deserializeJson() failed: ");
              Serial.println(error.c_str());
              return;
            }
            idOnServer = doc["data"][0]["id"]; // 1
            Serial.print("ID on server: ");
            Serial.println(idOnServer);
          }
        } else {
          Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
        }

        https.end();
      } else {
        Serial.printf("[HTTPS] Unable to connect\n");
      }

      // End extra scoping block
    }

    delete client;
  } else {
    Serial.println("Unable to create client");
  }
}

void sensorHandler() {
  while (Serial2.available()) {
    byte inByte = Serial2.read();
    packet[packetIndex] = inByte;
    packetIndex++;
    if (packetIndex == 10) {
      // Found tail.
      processPmData(packet, packetIndex);
      fetchBmeValues();
      printValues();
      myservo.write(map(SERVO_SENSOR, SERVO_MIN, SERVO_MAX, 0, 180));
      packetIndex = 0;
      memset(packet, 0x00, sizeof(packet));
    }
    if (packetIndex > 1 && inByte == 0xAA) {
      Serial.println("Warning: Incomplete PM packet.");
      memset(packet, 0x00, sizeof(packet));
      packet[0] = inByte;
      packetIndex = 1;
    }
  }
}

void processPmData(byte *p, int i) {
  if (i == 10) {
    pm25 = p[2];
    pm25 += (p[3] << 8);
    pm10 = p[4];
    pm10 += (p[5] << 8);
  }
}

void fetchBmeValues() {
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
}

void printValues() {
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");

  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("PM 2.5 = ");
  Serial.print(pm25);
  Serial.println();

  Serial.print("PM 10 = ");
  Serial.print(pm10);
  Serial.println();

  Serial.println();
}
