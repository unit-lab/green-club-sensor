#define VERSION "v0.1"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP32Servo.h>
#include <WiFiManager.h>
WiFiManager wm;

String id;

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
int temperature;
int pressure;
int humidity;
uint16_t pm25, pm10;

// Servo settings
#define SERVO_MIN 0
#define SERVO_MAX 50
#define SERVO_SENSOR temperature

void setup() {
  id = generateId();
  
  Serial2.begin(9600);
  Serial.begin(115200);

  Serial.print("Green Club Sensor ");
  Serial.print(VERSION);
  Serial.print(" ID: ");
  Serial.println(id);

  WiFi.mode(WIFI_STA);
  wm.setConfigPortalBlocking(false);
  wm.setConfigPortalTimeout(240);

  if (wm.autoConnect(String(String("GreenClub-") + id).c_str())) {
    Serial.println("Connected to Wi-Fi.");
  }
  else {
    Serial.println("Configuration portal running...");
  }

  unsigned status;

  // default settings
  status = bme.begin(0x76);
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000);
}

void loop() {
  pmSensorHandler();
}

void pmSensorHandler() {
  wm.process();
  while (Serial2.available()) {
    byte inByte = Serial2.read();
    packet[packetIndex] = inByte;
    packetIndex++;
    if (packetIndex == 10) {
      // Found tail.
      processPmData(packet, packetIndex);
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
    printValues();
  }
}

void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.print("PM 2.5 = ");
  Serial.print(pm25);
  Serial.println();

  Serial.print("PM 10 = ");
  Serial.print(pm10);
  Serial.println();

  Serial.println();

  myservo.write(map(SERVO_SENSOR, SERVO_MIN, SERVO_MAX, 0, 180));
}
