/***********************************************************************
 * Flight Computer Software for Teensy 4.1
 * 
 * Components:
 * - BMP390: Atmospheric Pressure Sensor
 * - BNO085: 9-DOF Absolute Orientation Sensor
 * - PCF8523: Real-Time Clock (RTC)
 * - TMP117: High-Precision Temperature Sensor
 * - GPS NEO-6M: GPS Module
 * - XBee: Wireless Communication Module
 * - SD Card: Data Logging
 * 
 * Features:
 * - Sensor Data Acquisition
 * - Data Filtering (if necessary)
 * - Data Transmission via XBee
 * - Data Logging to SD Card in CSV Format
 * - Robust Error Handling
 * 
 * Author: [Your Name]
 * Date: [Date]
 ***********************************************************************/

// ============================
// Library Inclusions
// ============================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_PCF8523.h>
#include <Adafruit_TMP117.h>
#include <XBee.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>

// ============================
// Pin Definitions
// ============================

// I2C Pins for Teensy 4.1 (Hardware I2C0)
#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19

// XBee Module connected to Serial1 (TX1: Pin 0, RX1: Pin 1)
#define XBEE_SERIAL_PORT Serial1

// GPS Module connected to Serial2 (TX2: Pin 7, RX2: Pin 8)
#define GPS_SERIAL_PORT Serial2

// SD Card Chip Select Pin
#define SD_CS_PIN 10  // You can change this if needed

// Digital Output Pins
const uint8_t BUZZER_PIN = 22;
const uint8_t FIRST_LED_PIN = 23;
const uint8_t SECOND_LED_PIN = 24;
const uint8_t THIRD_LED_PIN = 25;
const uint8_t FIRST_LED_OUTER_PIN = 26;
const uint8_t SECOND_LED_OUTER_PIN = 27;
const uint8_t THIRD_LED_OUTER_PIN = 28;
const uint8_t FIRST_MOSFET_PIN = 29;
const uint8_t SECOND_MOSFET_PIN = 30;
const uint8_t THIRD_MOSFET_PIN = 31;

// ============================
// Constants
// ============================

#define ALTITUDE_REFERENCE 0           // Initial altitude at ground level
#define SEA_LEVEL_PRESSURE 1013.25      // Sea level pressure in hPa

// XBee 64-bit Address (Replace with your XBee's actual address)
const uint64_t XBEE_ADDRESS = 0x0013A20040B41234;  // Example Address

// Logging Interval
const unsigned long LOG_INTERVAL_MS = 100;  // 100 milliseconds

// ============================
// Object Instantiations
// ============================

// Sensors
Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno;
Adafruit_PCF8523 rtc;
Adafruit_TMP117 tmp117;

// Communication
XBee xbee = XBee();
TinyGPSPlus gps;

// ============================
// Data Structures
// ============================

struct Data_Package {
    // Orientation
    float Roll;
    float Pitch;
    float Yaw;

    // Environmental Data
    float BMP_Temperature;    // From BMP390
    float BMP_Pressure;       // From BMP390
    float BMP_Altitude;       // Calculated

    // TMP117 Temperature
    float TMP117_Temperature;

    // RTC Time (formatted as YYYY-MM-DD HH:MM:SS)
    char RTC_Time[20];        // Fixed-size char array for consistency

    // GPS Data
    double GPS_Latitude;
    double GPS_Longitude;
    double GPS_Altitude;
    unsigned long GPS_Time;    // Unix timestamp
} data;

// ============================
// Function Prototypes
// ============================

bool initializeSensors();
bool initializeSDCard();
void initializeLogging();
void readSensors();
void readBMP390();
void readBNO085();
void readRTC();
void readTMP117();
void readGPS();
void sendDataXBee();
void logDataToSD();
void outputSerial();
void handleXBee();

// ============================
// Setup Function
// ============================

void setup() {
    // Initialize Serial for Debugging
    Serial.begin(9600);
    while (!Serial) { delay(10); } // Wait for Serial to be ready

    Serial.println("Flight Computer Initialization Started...");

    // Initialize I2C
    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin();

    // Initialize Sensors
    if (!initializeSensors()) {
        Serial.println("Sensor Initialization Failed! Entering infinite loop.");
        while (1); // Halt if sensors fail to initialize
    }

    // Initialize Communication Modules
    XBEE_SERIAL_PORT.begin(9600); // XBee default baud rate
    xbee.setSerial(XBEE_SERIAL_PORT);

    GPS_SERIAL_PORT.begin(9600);  // GPS NEO-6M default baud rate

    // Initialize Digital Pins
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(FIRST_LED_PIN, OUTPUT);
    pinMode(SECOND_LED_PIN, OUTPUT);
    pinMode(THIRD_LED_PIN, OUTPUT);
    pinMode(FIRST_LED_OUTER_PIN, OUTPUT);
    pinMode(SECOND_LED_OUTER_PIN, OUTPUT);
    pinMode(THIRD_LED_OUTER_PIN, OUTPUT);
    pinMode(FIRST_MOSFET_PIN, OUTPUT);
    pinMode(SECOND_MOSFET_PIN, OUTPUT);
    pinMode(THIRD_MOSFET_PIN, OUTPUT);

    // Initialize SD Card and Logging
    if (!initializeSDCard()) {
        Serial.println("SD Card Initialization Failed! Data logging will be disabled.");
        // Optionally, set a flag to disable logging
    } else {
        initializeLogging();
    }

    Serial.println("Initialization Complete.");
}

// ============================
// Loop Function
// ============================

void loop() {
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();

    // Execute at defined intervals
    if (currentMillis - previousMillis >= LOG_INTERVAL_MS) {
        previousMillis = currentMillis;

        // Read Sensor Data
        readSensors();

        // Send Data via XBee
        sendDataXBee();

        // Log Data to SD Card
        logDataToSD();

        // Output Data to Serial for Debugging
        outputSerial();
    }

    // Handle Incoming XBee Data (if needed)
    handleXBee();
}

// ============================
// Function Implementations
// ============================

/**
 * @brief Initializes all sensors. Returns true if all sensors are initialized successfully.
 */
bool initializeSensors() {
    bool success = true;

    // Initialize BMP390
    if (!bmp.begin_I2C()) {  // Use I2C interface
        Serial.println("Could not find a valid BMP390 sensor, check wiring!");
        success = false;
    } else {
        Serial.println("BMP390 initialized successfully.");
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    }

    // Initialize BNO085
    if (!bno.begin()) {
        Serial.println("Could not find a valid BNO085 sensor, check wiring!");
        success = false;
    } else {
        Serial.println("BNO085 initialized successfully.");
    }

    // Initialize PCF8523 RTC
    if (!rtc.begin()) {
        Serial.println("Couldn't find PCF8523 RTC!");
        success = false;
    } else {
        Serial.println("PCF8523 RTC initialized successfully.");
        if (!rtc.initialized()) {
            Serial.println("RTC is NOT initialized. Setting RTC to compile time.");
            rtc.adjust(DateTime(F(_DATE), F(TIME_)));
        }
    }

    // Initialize TMP117
    if (!tmp117.begin_I2C()) {
        Serial.println("Could not find a valid TMP117 sensor, check wiring!");
        success = false;
    } else {
        Serial.println("TMP117 initialized successfully.");
    }

    return success;
}

/**
 * @brief Initializes the SD card. Returns true if successful.
 */
bool initializeSDCard() {
    // Initialize SPI for SD Card
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
        return false;
    }
    Serial.println("SD card initialized successfully.");
    return true;
}

/**
 * @brief Initializes the log file by creating headers if the file is new.
 */
void initializeLogging() {
    // Create a unique filename based on RTC time
    char filename[20];
    DateTime now = rtc.now();
    sprintf(filename, "flight_%04d%02d%02d.csv",
            now.year(), now.month(), now.day());

    // Open the log file and write headers if the file is new
    File logFile = SD.open(filename, FILE_WRITE);
    if (logFile) {
        if (logFile.size() == 0) {
            logFile.println("Timestamp,Roll,Pitch,Yaw,BMP_Temp,BMP_Pressure,BMP_Altitude,TMP117_Temp,RTC_Time,GPS_Lat,GPS_Lon,GPS_Alt,GPS_Time");
            Serial.print("Log file ");
            Serial.print(filename);
            Serial.println(" created with headers.");
        } else {
            Serial.print("Log file ");
            Serial.print(filename);
            Serial.println(" opened.");
        }
        logFile.close();
    } else {
        Serial.println("Error opening log file for writing.");
    }
}

/**
 * @brief Reads all sensors and updates the global data structure.
 */
void readSensors() {
    readBMP390();
    readBNO085();
    readRTC();
    readTMP117();
    readGPS();
}

/**
 * @brief Reads data from BMP390 sensor.
 */
void readBMP390() {
    if (!bmp.performReading()) {
        Serial.println("BMP390 reading failed.");
        return;
    }
    data.BMP_Temperature = bmp.temperature;                      // Celsius
    data.BMP_Pressure = bmp.pressure / 100.0;                    // Convert Pa to hPa
    data.BMP_Altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE) - ALTITUDE_REFERENCE; // Meters
}

/**
 * @brief Reads data from BNO085 sensor.
 */
void readBNO085() {
    sensors_event_t orientation_event;
    sensors_event_t accel_event;
    sensors_event_t gyro_event;

    // Get Euler Angles
    bno.getEvent(&orientation_event, Adafruit_BNO08x::VECTOR_EULER);
    data.Roll = orientation_event.orientation.x;
    data.Pitch = orientation_event.orientation.y;
    data.Yaw = orientation_event.orientation.z;

    // Get Accelerometer Data
    bno.getEvent(&accel_event, Adafruit_BNO08x::VECTOR_ACCELEROMETER);
    // If needed, store accelerometer data separately or within Data_Package
    // data.AccX = accel_event.acceleration.x;
    // data.AccY = accel_event.acceleration.y;
    // data.AccZ = accel_event.acceleration.z;

    // Get Gyroscope Data
    bno.getEvent(&gyro_event, Adafruit_BNO08x::VECTOR_GYROSCOPE);
    // If needed, store gyroscope data separately or within Data_Package
    // data.GyroX = gyro_event.gyro.x;
    // data.GyroY = gyro_event.gyro.y;
    // data.GyroZ = gyro_event.gyro.z;
}

/**
 * @brief Reads current time from PCF8523 RTC.
 */
void readRTC() {
    DateTime now = rtc.now();
    // Format: YYYY-MM-DD HH:MM:SS
    sprintf(data.RTC_Time, "%04d-%02d-%02d %02d:%02d:%02d",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
}

/**
 * @brief Reads temperature from TMP117 sensor.
 */
void readTMP117() {
    data.TMP117_Temperature = tmp117.readTempC();
}

/**
 * @brief Reads data from GPS NEO-6M module.
 */
void readGPS() {
    while (GPS_SERIAL_PORT.available() > 0) {
        char c = GPS_SERIAL_PORT.read();
        gps.encode(c);
    }

    if (gps.location.isUpdated()) {
        data.GPS_Latitude = gps.location.lat();
        data.GPS_Longitude = gps.location.lng();
        data.GPS_Altitude = gps.altitude.meters();
    }

    if (gps.time.isUpdated()) {
        // Convert GPS time to Unix timestamp (requires date information)
        if (gps.date.isValid() && gps.time.isValid()) {
            data.GPS_Time = gps.date.value() + gps.time.value(); // Simplified; consider proper conversion
        }
    }
}

/**
 * @brief Sends the data package via XBee.
 */
void sendDataXBee() {
    // Serialize the Data_Package structure into a byte array
    uint8_t payload[sizeof(Data_Package)];
    memcpy(payload, &data, sizeof(Data_Package));

    // Define XBee Address (Replace with actual 64-bit address)
    XBeeAddress64 addr = XBeeAddress64((uint32_t)(XBEE_ADDRESS >> 32), (uint32_t)(XBEE_ADDRESS & 0xFFFFFFFF));

    // Create ZBTxRequest
    ZBTxRequest zbTx = ZBTxRequest(addr, payload, sizeof(payload));

    // Send the packet
    xbee.send(zbTx);

    // Wait for the transmission status
    if (xbee.readPacket(500)) {
        if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
            ZBTxStatusResponse txStatus = ZBTxStatusResponse();
            xbee.getResponse().getZBTxStatusResponse(txStatus);

            if (txStatus.getStatus() == SUCCESS) {
                Serial.println("XBee transmission successful.");
            } else {
                Serial.print("XBee transmission failed with status: 0x");
                Serial.println(txStatus.getStatus(), HEX);
            }
        }
    } else if (xbee.getResponse().isError()) {
        Serial.print("Error reading XBee response: ");
        Serial.println(xbee.getResponse().getErrorCode());
    }
}

/**
 * @brief Logs the data package to the SD card in CSV format.
 */
void logDataToSD() {
    // Define Filename (Replace with desired naming convention)
    char filename[20];
    DateTime now = rtc.now();
    sprintf(filename, "flight_%04d%02d%02d.csv",
            now.year(), now.month(), now.day());

    // Open the log file in append mode
    File logFile = SD.open(filename, FILE_WRITE);

    if (logFile) {
        // Construct the CSV line
        char csvLine[200];
        sprintf(csvLine, "%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,\"%s\",%.6f,%.6f,%.2f,%lu\n",
                data.RTC_Time,
                data.Roll,
                data.Pitch,
                data.Yaw,
                data.BMP_Temperature,
                data.BMP_Pressure,
                data.BMP_Altitude,
                data.TMP117_Temperature,
                data.RTC_Time,
                data.GPS_Latitude,
                data.GPS_Longitude,
                data.GPS_Altitude,
                data.GPS_Time);

        // Write to the file
        logFile.print(csvLine);

        // Close the file
        logFile.close();
    } else {
        Serial.println("Error opening log file for writing.");
    }
}

/**
 * @brief Outputs the data package to Serial for debugging.
 */
void outputSerial() {
    Serial.print("Roll: ");
    Serial.print(data.Roll, 2);
    Serial.print(", Pitch: ");
    Serial.print(data.Pitch, 2);
    Serial.print(", Yaw: ");
    Serial.print(data.Yaw, 2);
    Serial.print(", BMP Temp: ");
    Serial.print(data.BMP_Temperature, 2);
    Serial.print(" C, BMP Pressure: ");
    Serial.print(data.BMP_Pressure, 2);
    Serial.print(" hPa, BMP Altitude: ");
    Serial.print(data.BMP_Altitude, 2);
    Serial.print(" m, TMP117 Temp: ");
    Serial.print(data.TMP117_Temperature, 2);
    Serial.print(" C, RTC Time: ");
    Serial.print(data.RTC_Time);
    Serial.print(", GPS Lat: ");
    Serial.print(data.GPS_Latitude, 6);
    Serial.print(", GPS Lon: ");
    Serial.print(data.GPS_Longitude, 6);
    Serial.print(", GPS Alt: ");
    Serial.print(data.GPS_Altitude, 2);
    Serial.print(" m, GPS Time: ");
    Serial.println(data.GPS_Time);
}

/**
 * @brief Handles incoming XBee data (if required).
 *        This function can be expanded based on application needs.
 */
void handleXBee() {
    xbee.readPacket();

    if (xbee.getResponse().isAvailable()) {
        // Handle incoming XBee packets here
        // Example: Parse incoming data or execute commands
        Serial.println("Incoming XBee data received.");
    }
}
