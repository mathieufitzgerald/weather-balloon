#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "CHT8305C.h"
#include "DHT.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

const int redLedPin = 8; 
const int yellowLedPin = 9;
const int greenLedPin = 10;

//defining cs pin for
const int chipSelect = 52; // Change to your CS pin if different

// defining gps stuff
static const int RXPin = 4, TXPin = 3; // Change these pins according to your setup
static const uint32_t GPSBaud = 9600;

//defining interior temp sensor pins
#define DHTPIN 2     // digital pin for dht 22
#define DHTTYPE DHT22   // DHT 22 (AM2302)
DHT dht(DHTPIN, DHTTYPE);

// init rtc, accelgyro, cht, bmp
MPU6050 accelgyro;
CHT8305C cht8305c;
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

//init gyro
int16_t ax, ay, az;
int16_t gx, gy, gz;

bool testGPS() {
  // Test GPS by checking if it can get a valid reading
  for (int i = 0; i < 100; i++) {
    while (ss.available() > 0) {
      if (gps.encode(ss.read()) && gps.location.isValid()) {
        return true;
      }
    }
    delay(10);
  }
  return false;
}

bool testSensors() {
  // Test other sensors (BMP390, MPU6050, DHT22, etc.)
  if (!bmp.performReading()) return false;
  if (isnan(dht.readTemperature()) || isnan(dht.readHumidity())) return false;
  // Test MPU6050
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  if (ax == 0 && ay == 0 && az == 0) return false;
  // Add tests for other sensors if necessary
  return true;
}

bool testSDCard() {
  // Test SD card by attempting to open a file
  File testFile = SD.open("test.txt", FILE_WRITE);
  if (testFile) {
    testFile.close();
    SD.remove("test.txt");
    return true;
  }
  return false;
}

void setup() {

  // Initialize LED pins
  pinMode(redLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);

  // Open serial communications
  Serial.begin(9600);
  Wire.begin();
  digitalWrite(yellowLedPin, HIGH);

  cht8305c.begin();

  accelgyro.initialize();
  if (accelgyro.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
    while (1);
  }

  // Initialize DHT22
  dht.begin();

  // gps init
  ss.begin(GPSBaud);
  
  // Check if RTC is connected properly
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
    while (1);
  }

  // Check if the RTC lost power and if so, set the time
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // The following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Initialize SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
    return;
  }
  Serial.println("initialization done.");

  // Initialize BMP390
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP390 sensor, check wiring!");
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
    while (1);
  }

  delay(10000);

  // Run tests
  bool gpsOk = testGPS();
  bool sensorsOk = testSensors();
  bool sdCardOk = testSDCard();

  // Indicate test results
  if (gpsOk && sensorsOk && sdCardOk) {
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(yellowLedPin, LOW);
  } else {
    digitalWrite(redLedPin, HIGH);
    digitalWrite(yellowLedPin, LOW);
  }

  delay(20000);

  // Turn off LEDs
  digitalWrite(greenLedPin, LOW);
  digitalWrite(redLedPin, LOW);

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop() {
  // Create a new file in append mode
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // If the file is available, write to it
  if (dataFile) {
    DateTime now = rtc.now();
    dataFile.print(now.unixtime()); // Print Unix time
    dataFile.print(", ");

    while (ss.available() > 0) {
        if (gps.encode(ss.read())) {
            if (gps.location.isValid()) {
                // GPS data is valid, log it
                float latitude = gps.location.lat();
                float longitude = gps.location.lng();

                dataFile.print("Latitude: "); dataFile.print(latitude, 6); dataFile.print(", ");
                dataFile.print("Longitude: "); dataFile.print(longitude, 6);
            }
        }
    }

    // start of data log
    if (bmp.performReading()) {
      dataFile.print(bmp.temperature);
      dataFile.print(", ");
      dataFile.print(bmp.pressure / 100.0);
      dataFile.print(", ");
      dataFile.print(bmp.readAltitude(1013.25)); // Adjust to your local sea level pressure
    } else {
      dataFile.print("Failed to read BMP390");
    }
    
    //gyro logging
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    dataFile.print("AX: "); dataFile.print(ax); dataFile.print(", ");
    dataFile.print("AY: "); dataFile.print(ay); dataFile.print(", ");
    dataFile.print("AZ: "); dataFile.print(az);

    // outside temp logging
    float temperature = cht8305c.readTemperature();
    float humidity = cht8305c.readHumidity();
    dataFile.print("Exterior Temp: "); dataFile.print(temperature); dataFile.print(", ");
    dataFile.print("Exterior Humidity: "); dataFile.print(humidity);

    // Read from DHT22
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }

    dataFile.print("Interior Humidity: "); dataFile.print(h); dataFile.print(", ");
    dataFile.print("Interior Temp: "); dataFile.print(t);

    dataFile.println();
    dataFile.close();
    Serial.println("Data written");
  }
  else {
    Serial.println("Error opening datalog.txt");
  }
  // Wait for a bit before the next log
  delay(1000);
}

