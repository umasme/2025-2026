/*
Name: Ahmed Ahmed, Ahmed Ahmed Ahmed
Company: ASME UM
Project: Autonomous Rover + NeoPixel Status
Purpose: Integrated 16-channel MUX, Sabertooth Motor Control via CAN,
         Encoder telemetry, and NeoPixel Green/Orange status strip.
*/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Encoder.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>

/* --- Hardware Definitions --- */
#define SabertoothSerial Serial2

const int LED_PIN = 13;

// NeoPixel Definitions
#define PIXEL_PIN    A7
#define NUMPIXELS    100
#define PIXEL_DELAY  100

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

/* --- Object Initialization --- */
Encoder myEnc(6, 5);
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
Adafruit_NeoPixel pixels(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// IMU Constants
sh2_SensorId_t reportType = SH2_GAME_ROTATION_VECTOR;
long reportIntervalUs = 5000;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

euler_t ypr = {0.0f, 0.0f, 0.0f};

/* --- Timing & State Variables --- */
const float STEPS_PER_CM = 844.27f;

unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 1000;

unsigned long lastCanSendTime = 0;
const unsigned long canSendInterval = 100;

unsigned long lastPixelUpdate = 0;
bool pixelToggleState = false;

/* --- Function Prototypes --- */
void drive(byte address, byte command, byte data, String label = "");
void stopAllMotors();
void onReceiveCan(const CAN_message_t &msg);
void selectChannel(uint8_t channel);
void sendTelemetry();
void updateStatusLEDs();

void quaternionToEuler(
  float qr,
  float qi,
  float qj,
  float qk,
  euler_t *ypr,
  bool degrees = false
);

void quaternionToEulerRV(
  sh2_RotationVectorWAcc_t *rotational_vector,
  euler_t *ypr,
  bool degrees = false
);

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
  if (!bno08x.begin_I2C(0x4B)) {
    Serial.println("Failed to find BNO08x chip.");
  } else {
    Serial.println("BNO08x Found.");

    if (!bno08x.enableReport(reportType, reportIntervalUs)) {
      Serial.println("Could not enable BNO08x report.");
    }
  }

  // NeoPixel Setup
  pixels.begin();
  pixels.setBrightness(50);
  pixels.show();

  // Sabertooth Serial
  SabertoothSerial.begin(9600);

  // CAN Bus Setup
  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
  Can1.onReceive(onReceiveCan);

  delay(2000);
  SabertoothSerial.write(170);   // Sabertooth autobaud handshake
  delay(100);

  stopAllMotors();

  Serial.println(">>> BOOT SUCCESSFUL: ROVER & LEDS ACTIVE <<<");
}

void loop() {
  Can1.events();

  // Blink onboard LED heartbeat
  digitalWrite(LED_PIN, ((millis() / 500) % 2) ? HIGH : LOW);

  // Safety timeout
  if (millis() - lastCommandTime > commandTimeout) {
    stopAllMotors();
  }

  // Get IMU event and update YPR
 if (bno08x.getSensorEvent(&sensorValue)) {
  if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    quaternionToEuler(
      sensorValue.un.gameRotationVector.real,
      sensorValue.un.gameRotationVector.i,
      sensorValue.un.gameRotationVector.j,
      sensorValue.un.gameRotationVector.k,
      &ypr,
      true
    );
  }
}

  Serial.print("yaw: ");
  Serial.print(ypr.yaw, 6);

  Serial.print(" pitch: ");
  Serial.print(ypr.pitch, 6);

  Serial.print(" roll: ");
  Serial.println(ypr.roll, 6);
  
  // Send telemetry
  if (millis() - lastCanSendTime >= canSendInterval) {
    lastCanSendTime = millis();
    sendTelemetry();
  }

  // Update LEDs
  if (millis() - lastPixelUpdate >= PIXEL_DELAY) {
    lastPixelUpdate = millis();
    updateStatusLEDs();
  }
}

/* --- IMU Helper Functions --- */
void quaternionToEuler(
  float qr,
  float qi,
  float qj,
  float qk,
  euler_t *ypr,
  bool degrees
) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(
    2.0f * (qi * qj + qk * qr),
    (sqi - sqj - sqk + sqr)
  );

  ypr->pitch = asin(
    -2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)
  );

  ypr->roll = atan2(
    2.0f * (qj * qk + qi * qr),
    (-sqi - sqj + sqk + sqr)
  );

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(
  sh2_RotationVectorWAcc_t *rotational_vector,
  euler_t *ypr,
  bool degrees
) {
  quaternionToEuler(
    rotational_vector->real,
    rotational_vector->i,
    rotational_vector->j,
    rotational_vector->k,
    ypr,
    degrees
  );
}

/* --- NeoPixel Logic --- */
void updateStatusLEDs() {
  pixelToggleState = !pixelToggleState;

  for (int i = 7; i < NUMPIXELS; i++) {
    if (((i % 2) == 0) == pixelToggleState) {
      pixels.setPixelColor(i, pixels.Color(0, 255, 0));      // Green
    } else {
      pixels.setPixelColor(i, pixels.Color(255, 140, 0));    // Orange
    }
  }

  pixels.show();
}

/* --- Telemetry Function --- */
void sendTelemetry() {
  selectChannel(ACTUATOR_MUX_CH);
  delayMicroseconds(50);

  uint16_t current_adc = analogRead(SIG_PIN);
  long current_steps = myEnc.read();
  float distance_cm = current_steps / STEPS_PER_CM;

  // Message 1: actuator ADC + encoder distance
  CAN_message_t txmsg1;
  txmsg1.id = 0x200001;
  txmsg1.flags.extended = 1;
  txmsg1.len = 6;

  txmsg1.buf[0] = current_adc & 0xFF;
  txmsg1.buf[1] = (current_adc >> 8) & 0xFF;
  memcpy(&txmsg1.buf[2], &distance_cm, sizeof(distance_cm));

  Can1.write(txmsg1);

  // Message 2: yaw, pitch, roll x100
  CAN_message_t txmsg2;
  txmsg2.id = 0x200002;
  txmsg2.flags.extended = 1;
  txmsg2.len = 6;

  int16_t yaw_x100 = (int16_t)(ypr.yaw * 100.0f);
  int16_t pitch_x100 = (int16_t)(ypr.pitch * 100.0f);
  int16_t roll_x100 = (int16_t)(ypr.roll * 100.0f);

  memcpy(&txmsg2.buf[0], &yaw_x100, sizeof(yaw_x100));
  memcpy(&txmsg2.buf[2], &pitch_x100, sizeof(pitch_x100));
  memcpy(&txmsg2.buf[4], &roll_x100, sizeof(roll_x100));

  Can1.write(txmsg2);
}

/* --- Receive & Control Functions --- */
void onReceiveCan(const CAN_message_t &msg) {
  if (msg.id == 0x100001 && msg.len >= 4) {
    lastCommandTime = millis();

    int16_t act_raw = 0;
    int16_t depo_raw = 0;

    memcpy(&act_raw, &msg.buf[0], sizeof(act_raw));
    memcpy(&depo_raw, &msg.buf[2], sizeof(depo_raw));

    uint8_t act_val = (uint8_t)constrain(abs((int)act_raw), 0, 127);
    uint8_t depo_val = (uint8_t)constrain(abs((int)depo_raw), 0, 127);

    // Linked actuators on address 130
    if (act_raw >= 0) {
      drive(ADDR_130, 0, act_val, "ACT1 FWD");
      drive(ADDR_130, 4, act_val, "ACT2 FWD");
    } else {
      drive(ADDR_130, 1, act_val, "ACT1 REV");
      drive(ADDR_130, 5, act_val, "ACT2 REV");
    }

    // Deposition motor on address 129
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

void selectChannel(uint8_t channel) {
  digitalWrite(S0, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(S1, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(S2, (channel & 0x04) ? HIGH : LOW);
  digitalWrite(S3, (channel & 0x08) ? HIGH : LOW);
}