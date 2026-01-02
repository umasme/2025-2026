// Name: Ahmed Ahmed
// ASME Lubabotics University of Miami
// Function: Send a speed control signal to Spark Max


#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS_PIN 19 // Change it based on your use
MCP_CAN CAN0(CAN_CS_PIN);

// ================= REV CONSTANTS =================
#define CAN_EFF 0x80000000 // just place holder for the extended frame

#define HEARTBEAT_ID 0x2052C80 // heartbeat is importnat to keep the Controller responding
#define SPEED_SET_ID 0x2050480 // Set speed CAN ID

const uint8_t HEARTBEAT_DATA[8] = {
  255, 255, 255, 255, 255, 255, 255, 255
};

// ================= TIMING =================
const unsigned long HEARTBEAT_INTERVAL = 50;   // ms  You can play with that based on your usage
const unsigned long CONTROL_INTERVAL   = 20;   // ms

unsigned long lastHeartbeat = 0;
unsigned long lastControl   = 0;


// ================= SPEED RAMP =================
float currentRPM = 0.0f;
float targetRPM  = 1000.0f;   // change this any time

const float MAX_ACCEL_RPM_PER_SEC = 500.0f;  // smoothness

// ================= FUNCTIONS =================

void sendHeartbeat()
{
  CAN0.sendMsgBuf(
    HEARTBEAT_ID | CAN_EFF,
    1,
    8,
    (uint8_t*)HEARTBEAT_DATA
  );
}

void sendSpeed(uint8_t device_id, float rpm)
{
  uint8_t data[8];

  // Bytes 0–3: RPM float (little endian)
  memcpy(data, &rpm, 4);

  // Here it is very important to configure your PID in the REV Hardware Client.
  /* 
  PID Configuration: 
  kP = 0.00025 --> Proportional --> Too high motor overshoots
  kI = 0 // Integaral
  kD = 0 // Derivative
  kF = 0.0002 //  --> applies motor power based on the commanded speed. This is Forward Feedback. Also too high --> Overshoot
  */
  data[4] = 0x04; // Byte 4: ArbFF=1, PID slot=0

  // Bytes 5–7: must be zero
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;

  CAN0.sendMsgBuf(
    (SPEED_SET_ID + device_id) | CAN_EFF,
    1,
    8,
    data
  );
}

float rampRPM(float current, float target)
{
  float maxStep = MAX_ACCEL_RPM_PER_SEC *
                  (CONTROL_INTERVAL / 1000.0f);

  if (target > current + maxStep)
    return current + maxStep;
  else if (target < current - maxStep)
    return current - maxStep;
  else
    return target;
}

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Initializing CAN...");

  if (CAN0.begin(MCP_STDEXT, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) { // important to use 1000 kbit and check the clock speed of your module
    Serial.println("CAN Init OK");
  } else {
    Serial.println("CAN Init Failed");
    while (1);
  }

  CAN0.setMode(MCP_NORMAL);
  Serial.println("CAN Normal Mode");
}

// ================= LOOP =================
void loop()
{
  unsigned long now = millis();

  // --- Heartbeat ---
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    lastHeartbeat = now;
    sendHeartbeat();
  }

  // --- Speed Control ---
  if (now - lastControl >= CONTROL_INTERVAL) {
    lastControl = now;

    currentRPM = rampRPM(currentRPM, targetRPM);
    sendSpeed(1, currentRPM);
  }

  // Example: change direction every 5 seconds
  static unsigned long lastFlip = 0;
  if (now - lastFlip > 5000) {
    lastFlip = now;
    targetRPM = -targetRPM;  // reverse smoothly
  }
}
