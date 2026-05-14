/*
Name: Ahmed Ahmed, Ahmed Ahmed Ahmed
Company: ASME UM
Project: Autonomous Rover + NeoPixel Status
Purpose: Integrated 16-channel MUX, Sabertooth Motor Control via CAN,
         Encoder telemetry, NeoPixel status strip, and LSM9DS1 IMU YPR.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <FlexCAN_T4.h>
#include <Encoder.h>
#include <Adafruit_NeoPixel.h>

#include <SparkFunLSM9DS1.h>
#include <MadgwickAHRS.h>

/* --- Hardware Definitions --- */
#define SabertoothSerial Serial2
const int LED_PIN = 13;      // Onboard Teensy LED

// NeoPixel Definitions
#define PIXEL_PIN    A7      // Data pin for LEDs
#define NUMPIXELS    100     // Number of pixels
#define PIXEL_DELAY  100     // Animation speed

// Sabertooth Addresses
const byte ADDR_129 = 129;
const byte ADDR_130 = 130;

// MUX Pin Definitions
const int S0 = 38;
const int S1 = 39;
const int S2 = 40;
const int S3 = 41;
const int SIG_PIN = A0;
const int ACTUATOR_MUX_CH = 9;

/* --- IMU Settings --- */
const float SAMPLE_RATE_HZ = 100.0;
unsigned long microsPerReading = 0;
unsigned long microsPrevious = 0;

const int GYRO_CAL_SAMPLES = 30000;
const int GYRO_CAL_DELAY_MS = 1;

float gxBias = -0.724494;
float gyBias = -2.545511;
float gzBias = 1.893234;

float yawDeg = 0.0;
float pitchDeg = 0.0;
float rollDeg = 0.0;
float yawOffsetDeg = 0.0;

/* --- Object Initialization --- */
Encoder myEnc(6, 5);
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
Adafruit_NeoPixel pixels(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

LSM9DS1 imu;
Madgwick filter;

/* --- Timing & State Variables --- */
const float STEPS_PER_CM = 844.27;
unsigned long lastCommandTime = 0;
const int commandTimeout = 1000;

unsigned long lastCanSendTime = 0;
const int canSendInterval = 100;

unsigned long lastPixelUpdate = 0;
bool pixelToggleState = false;

/* --- Function Prototypes --- */
void drive(byte address, byte command, byte data, String label = "");
void stopAllMotors();
void onReceiveCan(const CAN_message_t &msg);
void selectChannel(const uint8_t channel);
void sendTelemetry();
void sendImuTelemetry();
void updateStatusLEDs();

void updateIMU();
float wrap180(float angleDeg);
int16_t degToCentideg(float angleDeg);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // MUX Setup
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SIG_PIN, INPUT);

  Serial.begin(115200);
  Wire.begin();

  // IMU Setup
  if(imu.begin()) {
    Serial.println("LSM9DS1 connected.");
  }


  filter.begin(SAMPLE_RATE_HZ);
  microsPerReading = 1000000UL / SAMPLE_RATE_HZ;
  microsPrevious = micros();

  // NeoPixel Setup
  pixels.begin();
  pixels.setBrightness(50);
  pixels.show();

  SabertoothSerial.begin(9600);

  // CAN Bus Setup
  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
  Can1.onReceive(onReceiveCan);

  delay(2000);
  SabertoothSerial.write(170); // Sabertooth handshake
  delay(100);

  stopAllMotors();
  Serial.println(">>> BOOT SUCCESSFUL: ROVER, LEDS, IMU ACTIVE <<<");
}

void loop() {
  Can1.events();

  // Update IMU at fixed rate
  updateIMU();

  // Blink Onboard LED, Teensy heartbeat
  digitalWrite(LED_PIN, (millis() / 500) % 2);

  // Safety Timeout: Stop motors if Jetson goes offline
  if (millis() - lastCommandTime > commandTimeout) {
    stopAllMotors();
  }

  // Non-blocking CAN Transmission
  if (millis() - lastCanSendTime >= canSendInterval) {
    lastCanSendTime = millis();

    sendTelemetry();     // Existing encoder + actuator telemetry
    sendImuTelemetry();  // New YPR telemetry
  }

  // Non-blocking LED Update
  if (millis() - lastPixelUpdate >= PIXEL_DELAY) {
    lastPixelUpdate = millis();
    updateStatusLEDs();
  }
}

/* --- IMU Logic --- */

void updateIMU() {
  unsigned long microsNow = micros();

  if (microsNow - microsPrevious >= microsPerReading) {
    if (imu.accelAvailable()) {
      imu.readAccel();
    }

    if (imu.gyroAvailable()) {
      imu.readGyro();
    }

    // Madgwick expects accel in g and gyro in deg/s
    float ax = imu.calcAccel(imu.ax);
    float ay = imu.calcAccel(imu.ay);
    float az = imu.calcAccel(imu.az);

    float gx = imu.calcGyro(imu.gx) - gxBias;
    float gy = imu.calcGyro(imu.gy) - gyBias;
    float gz = imu.calcGyro(imu.gz) - gzBias;

    // IMU-only fusion: accel + gyro, no magnetometer
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    float rawYaw = filter.getYaw();

    yawDeg = wrap180(rawYaw - yawOffsetDeg);
    pitchDeg = filter.getPitch();
    rollDeg = filter.getRoll();

    microsPrevious += microsPerReading;
  }
}

float wrap180(float angleDeg) {
  while (angleDeg > 180.0) angleDeg -= 360.0;
  while (angleDeg < -180.0) angleDeg += 360.0;
  return angleDeg;
}

int16_t degToCentideg(float angleDeg) {
  angleDeg = wrap180(angleDeg);

  float scaled = angleDeg * 100.0;

  if (scaled > 32767.0) scaled = 32767.0;
  if (scaled < -32768.0) scaled = -32768.0;

  return (int16_t)scaled;
}

/* --- NeoPixel Logic, Non-Blocking --- */

void updateStatusLEDs() {
  pixelToggleState = !pixelToggleState;

  for (int i = 7; i < NUMPIXELS; i++) {
    if ((i % 2 == 0) == pixelToggleState) {
      pixels.setPixelColor(i, pixels.Color(0, 255, 0));    // Green
    } else {
      pixels.setPixelColor(i, pixels.Color(255, 140, 0));  // Orange
    }
  }

  pixels.show();
}

/* --- Telemetry Functions --- */

void sendTelemetry() {
  selectChannel(ACTUATOR_MUX_CH);
  delayMicroseconds(50);

  uint16_t current_adc = analogRead(SIG_PIN);
  long current_steps = myEnc.read();
  float distance_cm = current_steps / STEPS_PER_CM;

  CAN_message_t txmsg;
  txmsg.id = 0x200001;
  txmsg.flags.extended = 1;
  txmsg.len = 6;

  txmsg.buf[0] = current_adc & 0xFF;
  txmsg.buf[1] = (current_adc >> 8) & 0xFF;
  memcpy(&txmsg.buf[2], &distance_cm, sizeof(distance_cm));

  Can1.write(txmsg);
}

void sendImuTelemetry() {
  int16_t yaw_cd   = degToCentideg(yawDeg);
  int16_t pitch_cd = degToCentideg(pitchDeg);
  int16_t roll_cd  = degToCentideg(rollDeg);

  CAN_message_t txmsg;
  txmsg.id = 0x200002;
  txmsg.flags.extended = 1;
  txmsg.len = 6;

  memcpy(&txmsg.buf[0], &yaw_cd, sizeof(int16_t));
  memcpy(&txmsg.buf[2], &pitch_cd, sizeof(int16_t));
  memcpy(&txmsg.buf[4], &roll_cd, sizeof(int16_t));

  Can1.write(txmsg);

  Serial.print("YPR: ");
  Serial.print(yawDeg, 2);
  Serial.print(", ");
  Serial.print(pitchDeg, 2);
  Serial.print(", ");
  Serial.println(rollDeg, 2);
}

/* --- Receive & Control Functions --- */

void onReceiveCan(const CAN_message_t &msg) {
  if (msg.id == 0x100001 && msg.len >= 4) {
    lastCommandTime = millis();

    int16_t act_raw;
    int16_t depo_raw;

    memcpy(&act_raw, &msg.buf[0], sizeof(int16_t));
    memcpy(&depo_raw, &msg.buf[2], sizeof(int16_t));

    // Drive the linked actuators, Address 130, M1 and M2
    uint8_t act_val = constrain(abs(act_raw), 0, 127);

    if (act_raw >= 0) {
      drive(ADDR_130, 0, act_val, "ACT1 FWD"); // M1
      drive(ADDR_130, 4, act_val, "ACT2 FWD"); // M2 synced
    } else {
      drive(ADDR_130, 1, act_val, "ACT1 REV"); // M1
      drive(ADDR_130, 5, act_val, "ACT2 REV"); // M2 synced
    }

    // Drive the deposition motor, Address 129, M1
    uint8_t depo_val = constrain(abs(depo_raw), 0, 127);

    if (depo_raw >= 0) {
      drive(ADDR_129, 0, depo_val, "DEPO FWD");
    } else {
      drive(ADDR_129, 1, depo_val, "DEPO REV");
    }
  }
}

void stopAllMotors() {
  drive(ADDR_129, 0, 0);
  drive(ADDR_130, 0, 0);
  drive(ADDR_130, 4, 0);
}

void drive(byte address, byte command, byte data, String label) {
  uint8_t checksum = (uint8_t(address) + uint8_t(command) + uint8_t(data)) & 0x7F;

  SabertoothSerial.write(address);
  SabertoothSerial.write(command);
  SabertoothSerial.write(data);
  SabertoothSerial.write(checksum);

  if (label != "" && data != 0) {
    Serial.print(label);
    Serial.print(": ");
    Serial.println(data);
  }
}

void selectChannel(const uint8_t channel) {
  digitalWrite(S0, channel & 0x01);
  digitalWrite(S1, channel & 0x02);
  digitalWrite(S2, channel & 0x04);
  digitalWrite(S3, channel & 0x08);
}