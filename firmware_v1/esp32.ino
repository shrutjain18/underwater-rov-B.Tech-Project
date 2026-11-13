#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ============================================================================
// 1. PIN DEFINITIONS
// ============================================================================
// White pins are unused (unidirectional thrusters). Throttle PWM is on the YELLOW pins.
// #define LEFT_ESC_WHITE_PIN    1
#define LEFT_ESC_YELLOW_PIN   2
// #define RIGHT_ESC_WHITE_PIN   3
#define RIGHT_ESC_YELLOW_PIN  4
#define DIVE_PLANE_SERVO_PIN  5
#define RUDDER_SERVO_PIN      6
#define BALLAST_DIR_POS_PIN   7
#define BALLAST_DIR_NEG_PIN   8
#define ENCODER_PIN_A         9
#define ENCODER_PIN_B         10
#define BALLAST_ENABLE_PIN    11
// Analog Depth Sensor (12-bit ADC, max 3.3V input)
#define DEPTH_SENSOR_PIN      15 

// MPU6050 I2C
#define I2C_SCL_PIN           13
#define I2C_SDA_PIN           14

// 433MHz Telemetry Radio
#define ESP_UART_RX_PIN       18 // Connects to Radio TX
#define ESP_UART_TX_PIN       17 // Connects to Radio RX
#define RADIO_BAUD            57600

// Serial monitor debug settings
#define SERIAL_TELEM_PRINT_MS 500   // how often telemetry is printed on Serial Monitor

// Failsafe: if no CMD packet arrives for this long, thrusters are stopped
#define FAILSAFE_TIMEOUT_MS   1000
#define FAILSAFE_THRUSTER_US  1000  // "thruster off" pulse - keep equal to Min PWM set in the dashboard

// ============================================================================
// 2. HARDWARE OBJECTS & CONFIG
// ============================================================================
HardwareSerial RadioSerial(1);
Adafruit_MPU6050 mpu;
bool mpuOk = false;

// IMU & Depth Config (Updated dynamically from Dashboard via "CFG,..." packet)
float cfg_depthVDiv = 1.5;      // If 4.5V is stepped down to 3.0V via 10k/20k divider
float cfg_depthOffset = 0.483;  // Zero-pressure voltage
float cfg_depthMult = 250.0;    // kPa per Volt
float cfg_waterDensity = 1000.0;// kg/m3 (Freshwater)
float cfg_axTrim = 0.0, cfg_ayTrim = 0.0, cfg_azTrim = 0.0;
float cfg_gxTrim = 0.0, cfg_gyTrim = 0.0, cfg_gzTrim = 0.0;
float cfg_imuAlpha = 0.96;

// IMU Filter State
float currentPitch = 0.0;
float currentRoll = 0.0;
unsigned long lastImuTime = 0;

// Received Ground Commands
int cmdSurge = 0, cmdYaw = 0, cmdHeave = 0;
float cmdPitch = 0.0;
// Final actuator values computed by the dashboard (sent inside the CMD packet)
int cmdLeftUs = FAILSAFE_THRUSTER_US, cmdRightUs = FAILSAFE_THRUSTER_US;
float cmdPlaneDeg = 90.0, cmdRudderDeg = 90.0;

unsigned long packetCounter = 0;
unsigned long lastTelemTime = 0;
unsigned long lastSerialPrintTime = 0;
unsigned long lastLocalStatusTime = 0;
unsigned long totalRadioBytesRx = 0;
unsigned long lastCmdTime = 0;
bool failsafeActive = false;

// Last interpreted hardware values (for printing)
int hwLeftYellowUs = FAILSAFE_THRUSTER_US;
int hwRightYellowUs = FAILSAFE_THRUSTER_US;
float hwPlaneDeg = 90.0, hwRudderDeg = 90.0;
int hwBallastPWM = 0;
const char* hwBallastState = "HOLD";

// Encoder
volatile long encoderCount = 0;
#define BALLAST_MAX_COUNTS 1000
void IRAM_ATTR readEncoderISR() {
  if (digitalRead(ENCODER_PIN_A) == digitalRead(ENCODER_PIN_B)) encoderCount++; else encoderCount--;
}

// ============================================================================
// 3. HARDWARE PWM HELPERS
// ============================================================================
uint32_t usToDuty14Bit(int us) { return (uint32_t)((constrain(us, 500, 2500) * 16384UL) / 20000UL); }

void setupPwm(uint8_t pin, uint8_t ch, uint32_t freq, uint8_t res) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(pin, freq, res);
#else
  ledcSetup(ch, freq, res); ledcAttachPin(pin, ch);
#endif
}

void writePwm(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

// ============================================================================
// 4. ACTUATOR OUTPUT (Applied instantly on received command)
// ============================================================================
void updateHardwareOutputs() {
  // Thrusters: unidirectional. Dashboard sends the final pulse width (us) for each.
  hwLeftYellowUs  = cmdLeftUs;
  hwRightYellowUs = cmdRightUs;

  writePwm(LEFT_ESC_YELLOW_PIN, 1, usToDuty14Bit(hwLeftYellowUs));
  writePwm(RIGHT_ESC_YELLOW_PIN, 3, usToDuty14Bit(hwRightYellowUs));

  // Servos: dashboard sends the final angles
  hwPlaneDeg  = constrain(cmdPlaneDeg, 0.0f, 180.0f);
  hwRudderDeg = constrain(cmdRudderDeg, 0.0f, 180.0f);
  writePwm(DIVE_PLANE_SERVO_PIN, 4, usToDuty14Bit(500 + (hwPlaneDeg / 180.0f) * 2000));
  writePwm(RUDDER_SERVO_PIN, 5, usToDuty14Bit(500 + (hwRudderDeg / 180.0f) * 2000));

  // Ballast
  long err = (long)((cmdHeave / 100.0f) * BALLAST_MAX_COUNTS) - encoderCount;
  if (abs(err) <= 10) {
    digitalWrite(BALLAST_DIR_POS_PIN, LOW); digitalWrite(BALLAST_DIR_NEG_PIN, LOW);
    hwBallastPWM = 0; hwBallastState = "HOLD";
  } else {
    digitalWrite(BALLAST_DIR_POS_PIN, err > 0); digitalWrite(BALLAST_DIR_NEG_PIN, err <= 0);
    hwBallastPWM = 200; hwBallastState = (err > 0) ? "FILL(+)" : "EMPTY(-)";
  }
  writePwm(BALLAST_ENABLE_PIN, 6, hwBallastPWM);
}

// ============================================================================
// 5. PACKET PARSING
// ============================================================================
void parseGroundPacket(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("CMD,")) {
    // CMD,Surge,Yaw,Heave,Pitch,LeftUs,RightUs,PlaneDeg,RudderDeg
    int idx[8];
    idx[0] = line.indexOf(',');
    for (int i = 1; i < 8; i++) idx[i] = (idx[i-1] > 0) ? line.indexOf(',', idx[i-1]+1) : -1;
    if (idx[7] > 0) {
      packetCounter++;
      cmdSurge  = line.substring(idx[0]+1, idx[1]).toInt();
      cmdYaw    = line.substring(idx[1]+1, idx[2]).toInt();
      cmdHeave  = line.substring(idx[2]+1, idx[3]).toInt();
      cmdPitch  = line.substring(idx[3]+1, idx[4]).toFloat();
      cmdLeftUs   = line.substring(idx[4]+1, idx[5]).toInt();
      cmdRightUs  = line.substring(idx[5]+1, idx[6]).toInt();
      cmdPlaneDeg  = line.substring(idx[6]+1, idx[7]).toFloat();
      cmdRudderDeg = line.substring(idx[7]+1).toFloat();

      lastCmdTime = millis();
      if (failsafeActive) {
        failsafeActive = false;
        Serial.println("[FAILSAFE CLEARED] CMD packets resumed");
      }
      updateHardwareOutputs();

      Serial.println("--------------------------------------------------------------------------------");
      Serial.printf("[RAW RX #%lu] %s\n", packetCounter, line.c_str());
      Serial.printf("  -> COMMANDS       | Surge: %d | Yaw: %d | Heave: %d | Pitch: %.1f\n",
                    cmdSurge, cmdYaw, cmdHeave, cmdPitch);
      Serial.printf("  -> LEFT THRUSTER  | GPIO2(Yellow): %4d us\n", hwLeftYellowUs);
      Serial.printf("  -> RIGHT THRUSTER | GPIO4(Yellow): %4d us\n", hwRightYellowUs);
      Serial.printf("  -> SERVOS         | GPIO5(Planes): %5.1f deg | GPIO6(Rudder): %5.1f deg\n",
                    hwPlaneDeg, hwRudderDeg);
      Serial.printf("  -> BALLAST MOTOR  | Target: %3d%% | State: %s | GPIO11(PWM): %3d | Enc(GPIO9/10): %ld\n",
                    cmdHeave, hwBallastState, hwBallastPWM, encoderCount);
    } else {
      Serial.printf("[BAD CMD PACKET] -> %s\n", line.c_str());
    }
  } 
  else if (line.startsWith("CFG,")) {
    // Dynamic Config Packet from Dashboard Settings Tab
    // CFG,VDiv,Offset,Mult,Density,AxTrim,AyTrim,AzTrim,GxTrim,GyTrim,GzTrim,ImuAlpha
    int idx[11];
    idx[0] = line.indexOf(',');
    for (int i=1; i<11; i++) idx[i] = (idx[i-1] > 0) ? line.indexOf(',', idx[i-1]+1) : -1;
    if(idx[10] > 0) {
      cfg_depthVDiv = line.substring(idx[0]+1, idx[1]).toFloat();
      cfg_depthOffset = line.substring(idx[1]+1, idx[2]).toFloat();
      cfg_depthMult = line.substring(idx[2]+1, idx[3]).toFloat();
      cfg_waterDensity = line.substring(idx[3]+1, idx[4]).toFloat();
      cfg_axTrim = line.substring(idx[4]+1, idx[5]).toFloat();
      cfg_ayTrim = line.substring(idx[5]+1, idx[6]).toFloat();
      cfg_azTrim = line.substring(idx[6]+1, idx[7]).toFloat();
      cfg_gxTrim = line.substring(idx[7]+1, idx[8]).toFloat();
      cfg_gyTrim = line.substring(idx[8]+1, idx[9]).toFloat();
      cfg_gzTrim = line.substring(idx[9]+1, idx[10]).toFloat();
      cfg_imuAlpha = line.substring(idx[10]+1).toFloat();
      Serial.printf("[SYS] Received new Sensor CFG: %s\n", line.c_str());
    } else {
      Serial.printf("[BAD CFG PACKET] -> %s\n", line.c_str());
    }
  }
  else {
    // Print ANY other non-empty string that arrives over RadioSerial
    Serial.printf("[NON-CMD RADIO DATA] -> %s\n", line.c_str());
  }
}

// ============================================================================
// 6. SETUP & LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000); // give ESP32-S3 USB serial time to connect
  Serial.println("\n=======================================================");
  Serial.println("  ESP32-S3 ROV AIR MODULE (ADVANCED: IMU + DEPTH) INIT ");
  Serial.printf("  Radio UART1 -> ESP_RX: GPIO%d | ESP_TX: GPIO%d @ %d\n",
                ESP_UART_RX_PIN, ESP_UART_TX_PIN, RADIO_BAUD);
  Serial.println("=======================================================");

  RadioSerial.begin(RADIO_BAUD, SERIAL_8N1, ESP_UART_RX_PIN, ESP_UART_TX_PIN);
  
  // Actuator Pins
  setupPwm(LEFT_ESC_YELLOW_PIN, 1, 50, 14);
  setupPwm(RIGHT_ESC_YELLOW_PIN, 3, 50, 14);
  setupPwm(DIVE_PLANE_SERVO_PIN, 4, 50, 14); setupPwm(RUDDER_SERVO_PIN, 5, 50, 14);
  pinMode(BALLAST_DIR_POS_PIN, OUTPUT); pinMode(BALLAST_DIR_NEG_PIN, OUTPUT);
  setupPwm(BALLAST_ENABLE_PIN, 6, 1000, 8);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP); pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), readEncoderISR, CHANGE);

  // Depth ADC
  analogReadResolution(12);

  // I2C MPU6050
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  mpuOk = mpu.begin();
  if (!mpuOk) Serial.println("WARNING: Failed to find MPU6050 chip on I2C!");
  else {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    Serial.println("[SYS] MPU6050 found and configured.");
  }
  
  updateHardwareOutputs();
  lastImuTime = millis();
  Serial.println("[SYS] Setup complete. Running...");
}

void loop() {
  // 1. READ COMMANDS
  static String rxBuf = "";
  while (RadioSerial.available()) {
    char c = RadioSerial.read();
    totalRadioBytesRx++;
    if (c == '\n') { parseGroundPacket(rxBuf); rxBuf = ""; }
    else if (c != '\r') rxBuf += c;
  }

  // Failsafe: link lost -> stop thrusters (servos and ballast keep last command)
  if (packetCounter > 0 && !failsafeActive && (millis() - lastCmdTime) > FAILSAFE_TIMEOUT_MS) {
    cmdLeftUs = FAILSAFE_THRUSTER_US;
    cmdRightUs = FAILSAFE_THRUSTER_US;
    failsafeActive = true;
    Serial.println("[FAILSAFE] No CMD received for >1s -> thrusters stopped");
  }
  
  // Continuously update ballast closed-loop position
  updateHardwareOutputs();

  // 2. READ SENSORS & TRANSMIT TELEMETRY (10 Hz = every 100ms)
  unsigned long now = millis();
  if (now - lastTelemTime >= 100) {
    float dt = (now - lastImuTime) / 1000.0f;
    lastTelemTime = now; lastImuTime = now;
    
    // --- IMU Processing ---
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      // Raw values + User Trim Offsets
      ax = a.acceleration.x + cfg_axTrim; ay = a.acceleration.y + cfg_ayTrim; az = a.acceleration.z + cfg_azTrim;
      gx = g.gyro.x * 57.2958 + cfg_gxTrim; gy = g.gyro.y * 57.2958 + cfg_gyTrim; gz = g.gyro.z * 57.2958 + cfg_gzTrim; // rad/s to deg/s
      
      // Accelerometer Absolute Angles
      float accelPitch = atan2(-ax, sqrt(ay*ay + az*az)) * 57.2958;
      float accelRoll  = atan2(ay, az) * 57.2958;
      
      // Complementary Filter Fusion (Gyro handles fast motion, Accel pulls it back to absolute gravity)
      currentPitch = cfg_imuAlpha * (currentPitch + gy * dt) + (1.0f - cfg_imuAlpha) * accelPitch;
      currentRoll  = cfg_imuAlpha * (currentRoll  + gx * dt) + (1.0f - cfg_imuAlpha) * accelRoll;
    }

    // --- Depth Sensor Processing ---
    // 12-bit ADC = 0 to 4095. Max Voltage = 3.3V.
    float espVoltage = (analogRead(DEPTH_SENSOR_PIN) * 3.3f) / 4095.0f;
    float sensorTrueVolts = espVoltage * cfg_depthVDiv;
    float pressureKpa = (sensorTrueVolts - cfg_depthOffset) * cfg_depthMult;
    if (pressureKpa < 0) pressureKpa = 0; // Prevent negative depth near surface
    float depthMeters = (pressureKpa * 1000.0f) / (cfg_waterDensity * 9.81f); // P = rho * g * h

    // --- Format & Transmit (accel sent in g for the dashboard; Serial Monitor below stays in m/s2) ---
    String tel = "TELEM," + String(ax/9.81f,2) + "," + String(ay/9.81f,2) + "," + String(az/9.81f,2) + "," + 
                 String(gx,1) + "," + String(gy,1) + "," + String(gz,1) + "," + 
                 String(currentPitch,1) + "," + String(currentRoll,1) + "," + 
                 String(sensorTrueVolts,3) + "," + String(pressureKpa,1) + "," + String(depthMeters,2) + ",0\n";
                 
    RadioSerial.print(tel);

    // --- Serial Monitor print (throttled so it stays readable) ---
    if (now - lastSerialPrintTime >= SERIAL_TELEM_PRINT_MS) {
      lastSerialPrintTime = now;
      Serial.printf("[TELEM] IMU:%s | Acc(m/s2) X:%6.2f Y:%6.2f Z:%6.2f | Gyro(d/s) X:%6.1f Y:%6.1f Z:%6.1f\n",
                    mpuOk ? "OK" : "MISSING", ax, ay, az, gx, gy, gz);
      Serial.printf("        Pitch: %6.1f deg | Roll: %6.1f deg | Depth: %.3f V | %.1f kPa | %.2f m\n",
                    currentPitch, currentRoll, sensorTrueVolts, pressureKpa, depthMeters);
    }
  }

  // 3. Local heartbeat if no valid command packet has arrived yet
  if (now - lastLocalStatusTime >= 1000) {
    lastLocalStatusTime = now;
    if (packetCounter == 0) {
      Serial.printf("[WAITING FOR GROUND RADIO] Uptime: %lus | Radio Bytes RX so far: %lu (ESP_RX=GPIO%d, ESP_TX=GPIO%d)\n",
                    now / 1000, totalRadioBytesRx, ESP_UART_RX_PIN, ESP_UART_TX_PIN);
    }
  }
}
