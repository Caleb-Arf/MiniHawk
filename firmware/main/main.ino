#include <MAVLink.h>
#include <SD.h>
#include <SensirionI2cSen66.h>
#include <Wire.h>
#include <math.h>

#define SD_CS_PIN   2
#define I2C_SDA     22
#define I2C_SCL     21

// SEN66 sensor instance
SensirionI2cSen66 sensor;

// Error handling
static char errorMessage[64];
static int16_t error;

// macro definitions
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

bool sdReady = false;
File logFile;
const char* LOG_FILE = "/datalog.csv";

// Timing variables for non-blocking sensor reads
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 2000; // Read every 2 seconds

// Sine wave test signal
float sinePhase = 0.0;
const float SINE_PERIOD_MS = 10000.0; // 10 second period for sine wave

void initSD()
{
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("[SD] Mount failed — logging disabled");
    sdReady = false;
    return;
  }

  // Write CSV header if file doesn't exist yet
  if (!SD.exists(LOG_FILE)) {
    logFile = SD.open(LOG_FILE, FILE_WRITE);
    if (logFile) {
      logFile.println("timestamp_ms,temperature,humidity,co2_ppm");
      logFile.close();
      Serial.println("[SD] Created datalog.csv with header");
    }
  } else {
    Serial.println("[SD] Appending to existing datalog.csv");
  }

  sdReady = true;
}

void logToSD(float temperature, float humidity, uint16_t co2)
{
  if (!sdReady) return;

  logFile = SD.open(LOG_FILE, FILE_APPEND);
  if (!logFile) {
    Serial.println("[SD] Failed to open file for writing");
    return;
  }

  logFile.print(millis());
  logFile.print(",");
  logFile.print(temperature, 2);
  logFile.print(",");
  logFile.print(humidity, 2);
  logFile.print(",");
  logFile.print(co2);
  logFile.println();
  logFile.close();
}

void sendFloat(const char* name, float value)
{
  mavlink_msg_named_value_float_pack(
      1,
      MAV_COMP_ID_ONBOARD_COMPUTER,
      &msg,
      millis(),
      name,
      value
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
  Serial1.flush(); // Ensure data is sent before continuing
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  
  Serial1.begin(57600, SERIAL_8N1, 16, 17);   // RX, TX
  delay(100);
  
  if (!Serial1) {
    Serial.println("[ERROR] Serial1 initialization failed!");
  }

  // Initialize I2C for SEN66 sensor
  Wire.begin(I2C_SDA, I2C_SCL);
  sensor.begin(Wire, SEN66_I2C_ADDR_6B);

  // Reset the sensor
  error = sensor.deviceReset();
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute deviceReset(): ");
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.println(errorMessage);
  }
  
  delay(1200);
  
  // Get and print serial number
  int8_t serialNumber[32] = {0};
  error = sensor.getSerialNumber(serialNumber, 32);
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.println(errorMessage);
  } else {
    Serial.print("SEN66 Serial Number: ");
    Serial.println((const char*)serialNumber);
  }
  
  // Start continuous measurement
  error = sensor.startContinuousMeasurement();
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute startContinuousMeasurement(): ");
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.println(errorMessage);
  } else {
    Serial.println("SEN66 continuous measurement started");
  }

  initSD();
  
  // Initialize timing
  lastSensorRead = millis();
}

void loop()
{
  unsigned long ms = millis();
  
  // Only read sensor at defined intervals to prevent blocking
  if (ms - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = ms;
    
    // SEN66 measurement variables (only using temp, humidity, co2)
    float temperature = 0.0;
    float humidity = 0.0;
    uint16_t co2 = 0;
    
    // Dummy variables required by the API
    float massConcentrationPm1p0 = 0.0;
    float massConcentrationPm2p5 = 0.0;
    float massConcentrationPm4p0 = 0.0;
    float massConcentrationPm10p0 = 0.0;
    float vocIndex = 0.0;
    float noxIndex = 0.0;
    
    // Read sensor data
    error = sensor.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, humidity, temperature, vocIndex, noxIndex,
        co2);
        
    if (error != NO_ERROR) {
      Serial.print("[ERROR] readMeasuredValues(): ");
      errorToString(error, errorMessage, sizeof(errorMessage));
      Serial.println(errorMessage);
      
      // Log error to SD
      if (sdReady) {
        logFile = SD.open(LOG_FILE, FILE_APPEND);
        if (logFile) {
          logFile.print(ms);
          logFile.print(",NaN,NaN,NaN");
          logFile.println();
          logFile.close();
        }
      }
    } else {
      // Success - send and log data
      Serial.print("[DATA] ms=");
      Serial.print(ms);
      Serial.print(" | Temp: ");
      Serial.print(temperature, 2);
      Serial.print("C | Humidity: ");
      Serial.print(humidity, 2);
      Serial.print("% | CO2: ");
      Serial.print(co2);
      Serial.println(" ppm");
      
      // Send values via MAVLink
      sendFloat("TEMP", temperature);
      sendFloat("HUMIDITY", humidity);
      sendFloat("CO2", (float)co2);

      // Log to SD
      logToSD(temperature, humidity, co2);
    }
  }

  // Send test sine wave at same rate as sensor
  if (ms - lastSensorRead >= SENSOR_INTERVAL) {
    // Calculate sine value with 10 second period
    float sineValue = sin(2.0 * PI * (ms % (unsigned long)SINE_PERIOD_MS) / SINE_PERIOD_MS);
    sendFloat("TEST_SINE", sineValue);
  }

  // Small delay to prevent busy-looping
  delay(50);
}
