#include <MAVLink.h>
#include <SD.h>
#include <SensirionI2cSen66.h>
#include <TaskScheduler.h>
#include <Wire.h>
#include <math.h>

#define SD_CS_PIN   2
#define I2C_SDA     22
#define I2C_SCL     21
#define H2S_PIN      26
#define SO2_PIN      27

// Acquisition and publication rates.  Keep the gas acquisition jobs separate
// so an SEN66 transaction cannot determine the H2S/SO2 sampling cadence.
#define CO2_READ_INTERVAL_MS      1000
#define H2S_READ_INTERVAL_MS       250
#define SO2_READ_INTERVAL_MS       250
#define PUBLISH_INTERVAL_MS       1000
#define TEST_SINE_INTERVAL_MS     2000
#define SEN66_BOOT_WAIT_MS        1200

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

// Sine wave test signal
const float SINE_PERIOD_MS = 10000.0; // 10 second period for sine wave

// Latest successfully acquired values.  Acquisition tasks only update these
// values; the publisher task is the sole place that sends/logs a sample set.
float latestTemperature = NAN;
float latestHumidity = NAN;
float latestCo2 = NAN;
float latestH2s = NAN;
float latestSo2 = NAN;
bool sen66ReadingValid = false;
bool h2sReadingValid = false;
bool so2ReadingValid = false;

Scheduler scheduler;

void finishSen66Startup();
void readCo2Sensor();
void readH2sSensor();
void readSo2Sensor();
void publishSensorData();
void sendTestSine();

// Tasks are enabled once the SEN66 continuous measurement has started.
Task sen66StartupTask(SEN66_BOOT_WAIT_MS, TASK_ONCE, &finishSen66Startup);
Task co2ReadTask(CO2_READ_INTERVAL_MS, TASK_FOREVER, &readCo2Sensor);
Task h2sReadTask(H2S_READ_INTERVAL_MS, TASK_FOREVER, &readH2sSensor);
Task so2ReadTask(SO2_READ_INTERVAL_MS, TASK_FOREVER, &readSo2Sensor);
Task publishTask(PUBLISH_INTERVAL_MS, TASK_FOREVER, &publishSensorData);
Task testSineTask(TEST_SINE_INTERVAL_MS, TASK_FOREVER, &sendTestSine);

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
      logFile.println("timestamp_ms,temperature,humidity,co2_ppm,h2s_ppm,so2_ppm");
      logFile.close();
      Serial.println("[SD] Created datalog.csv with header");
    }
  } else {
    Serial.println("[SD] Appending to existing datalog.csv");
  }

  sdReady = true;
}

void logToSD(float temperature, float humidity, float co2, float h2s, float so2)
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
  logFile.print(co2, 2);
  logFile.print(",");
  logFile.print(h2s, 2);
  logFile.print(",");
  logFile.print(so2, 2);
  logFile.println();
  logFile.close();
}

float read4to20mA(int pin)
{
  long adcSum = 0;
  const int samples = 10;
  for (int i = 0; i < samples; ++i) {
    adcSum += analogRead(pin);
    delay(1);
  }

  const float voltage = (float)adcSum / samples * 3.3f / 4095.0f;
  return voltage / 100.0f * 1000.0f;  // 100 ohm shunt, result in mA
}

float mAtoPPM(float milliamps, float fullScale)
{
  const float ppm = ((milliamps - 4.0f) / 16.0f) * fullScale;
  return ppm < 0.0f ? 0.0f : ppm;
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

void finishSen66Startup()
{
  // Get and print serial number after the required post-reset wait.
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

  // Start continuous measurement.
  error = sensor.startContinuousMeasurement();
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute startContinuousMeasurement(): ");
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.println(errorMessage);
  } else {
    Serial.println("SEN66 continuous measurement started");
  }

  initSD();
  co2ReadTask.enable();
  h2sReadTask.enable();
  so2ReadTask.enable();
  publishTask.enable();
  testSineTask.enable();
}

void readCo2Sensor()
{
  // The current project uses the SEN66 over I2C for CO2, temperature, and
  // humidity.  This task can be replaced with a UART CO2 parser if a separate
  // serial CO2 module is connected.
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

    sen66ReadingValid = false;
    return;
  }

  latestTemperature = temperature;
  latestHumidity = humidity;
  latestCo2 = (float)co2;
  sen66ReadingValid = true;
}

void readH2sSensor()
{
  latestH2s = mAtoPPM(read4to20mA(H2S_PIN), 100.0f);
  h2sReadingValid = true;
}

void readSo2Sensor()
{
  latestSo2 = mAtoPPM(read4to20mA(SO2_PIN), 50.0f);
  so2ReadingValid = true;
}

void publishSensorData()
{
  const unsigned long ms = millis();

  if (!sen66ReadingValid || !h2sReadingValid || !so2ReadingValid) {
    Serial.println("[DATA] Waiting for initial readings before publishing");
    return;
  }

  Serial.print("[DATA] ms=");
  Serial.print(ms);
  Serial.print(" | Temp: ");
  Serial.print(latestTemperature, 2);
  Serial.print("C | Humidity: ");
  Serial.print(latestHumidity, 2);
  Serial.print("% | CO2: ");
  Serial.print(latestCo2, 2);
  Serial.print(" ppm | H2S: ");
  Serial.print(latestH2s, 2);
  Serial.print(" ppm | SO2: ");
  Serial.print(latestSo2, 2);
  Serial.println(" ppm");

  // Keep the existing MAVLink NAMED_VALUE_FLOAT identifiers.
  sendFloat("TEMP", latestTemperature);
  sendFloat("HUMIDITY", latestHumidity);
  sendFloat("CO2", latestCo2);
  sendFloat("H2S", latestH2s);
  sendFloat("SO2", latestSo2);

  logToSD(latestTemperature, latestHumidity, latestCo2, latestH2s, latestSo2);
}

void sendTestSine()
{
  const unsigned long ms = millis();
  const float sineValue = sin(2.0 * PI *
      (ms % (unsigned long)SINE_PERIOD_MS) / SINE_PERIOD_MS);
  sendFloat("TEST_SINE", sineValue);
}

void setup()
{
  Serial.begin(115200);

  Serial1.begin(57600, SERIAL_8N1, 16, 17);   // RX, TX

  if (!Serial1) {
    Serial.println("[ERROR] Serial1 initialization failed!");
  }

  // Initialize I2C for SEN66 sensor
  analogReadResolution(12);
  Wire.begin(I2C_SDA, I2C_SCL);
  sensor.begin(Wire, SEN66_I2C_ADDR_6B);

  // Reset the sensor
  error = sensor.deviceReset();
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute deviceReset(): ");
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.println(errorMessage);
  }

  scheduler.addTask(sen66StartupTask);
  scheduler.addTask(co2ReadTask);
  scheduler.addTask(h2sReadTask);
  scheduler.addTask(so2ReadTask);
  scheduler.addTask(publishTask);
  scheduler.addTask(testSineTask);

  // Preserve the required 1.2 second post-reset wait without blocking loop().
  sen66StartupTask.enableDelayed();
}

void loop()
{
  scheduler.execute();
}
