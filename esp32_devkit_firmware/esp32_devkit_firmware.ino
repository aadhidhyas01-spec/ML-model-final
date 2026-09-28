/*
 * ==============================================================================
 * ESP32 DevKit - 100% Wireless Smart Waste Robot Receiver & Arm Controller
 * ==============================================================================
 * - Wireless Protocol: Wi-Fi UDP (Port 8888)
 * - Base Movement: 4 DC Motors (2x L298N / TB6612 Drivers)
 * - Robotic Arm: 5 DOF Servos (Base, Shoulder, Elbow, Wrist, Gripper)
 * ==============================================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>

// ------------------------------------------------------------------------------
// SERVO DRIVER: Auto-detects ESP32Servo library or uses built-in LEDC PWM
// ------------------------------------------------------------------------------
#if __has_include(<ESP32Servo.h>)
  #include <ESP32Servo.h>
#else
  // Built-in zero-dependency ESP32 Servo driver (no library installation needed!)
  class BuiltinServo {
  private:
    int _pin;
    int _channel;
    static int _channelCounter;
  public:
    BuiltinServo() : _pin(-1), _channel(-1) {}
    void attach(int pin) {
      _pin = pin;
      _channel = _channelCounter++;
      #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcAttach(_pin, 50, 14); // 50 Hz, 14-bit resolution
      #else
        ledcSetup(_channel, 50, 14);
        ledcAttachPin(_pin, _channel);
      #endif
    }
    void write(int angle) {
      angle = constrain(angle, 0, 180);
      // 50Hz = 20ms period, 14-bit resolution = 16384 total steps
      // 0 deg (~0.5ms) = 410 | 180 deg (~2.5ms) = 2048
      int duty = map(angle, 0, 180, 410, 2048);
      #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcWrite(_pin, duty);
      #else
        ledcWrite(_channel, duty);
      #endif
    }
  };
  int BuiltinServo::_channelCounter = 0;
  #define Servo BuiltinServo
#endif

// ------------------------------------------------------------------------------
// 1. WI-FI CONFIGURATION (Connect to Laptop Hotspot or Local Wi-Fi)
// ------------------------------------------------------------------------------
const char* ssid     = "HP";
const char* password = "12345678";
const unsigned int localUdpPort = 8888;

WiFiUDP udp;
char packetBuffer[255];

// ------------------------------------------------------------------------------
// 2. MOTOR DRIVER PIN ASSIGNMENTS (4 DC Motors)
// ------------------------------------------------------------------------------
// Left Motors (Driver 1)
#define MOTOR_L_IN1  16
#define MOTOR_L_IN2  17
#define MOTOR_L_PWM  4

// Right Motors (Driver 2)
#define MOTOR_R_IN3  18
#define MOTOR_R_IN4  19
#define MOTOR_R_PWM  5

// Motor Speed (0 - 255)
int motorSpeed = 180;

// ------------------------------------------------------------------------------
// 3. 5 DOF ROBOTIC ARM SERVO CONFIGURATION
// ------------------------------------------------------------------------------
Servo servoBase;      // Joint 1: Left / Right rotation
Servo servoShoulder;  // Joint 2: Arm up / down
Servo servoElbow;     // Joint 3: Reach forward / backward
Servo servoWrist;     // Joint 4: Tilt angle
Servo servoGripper;   // Joint 5: Open / Close gripper

#define PIN_SERVO_BASE      13
#define PIN_SERVO_SHOULDER  12
#define PIN_SERVO_ELBOW     14
#define PIN_SERVO_WRIST     27
#define PIN_SERVO_GRIPPER   26

// Default Rest / Home Angles
int angleBase     = 90;
int angleShoulder = 90;
int angleElbow    = 90;
int angleWrist    = 90;
int angleGripper  = 30; // 30 = Open, 90 = Closed

// Safety Watchdog
unsigned long lastPacketTime = 0;
const unsigned long SAFETY_TIMEOUT_MS = 800; // Stop motors if signal lost for 800ms

// ==============================================================================
// MOTOR CONTROL FUNCTIONS
// ==============================================================================
void moveForward() {
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_PWM, motorSpeed);

  digitalWrite(MOTOR_R_IN3, HIGH);
  digitalWrite(MOTOR_R_IN4, LOW);
  analogWrite(MOTOR_R_PWM, motorSpeed);
}

void moveBackward() {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  analogWrite(MOTOR_L_PWM, motorSpeed);

  digitalWrite(MOTOR_R_IN3, LOW);
  digitalWrite(MOTOR_R_IN4, HIGH);
  analogWrite(MOTOR_R_PWM, motorSpeed);
}

void steerLeft() {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  analogWrite(MOTOR_L_PWM, motorSpeed * 0.7);

  digitalWrite(MOTOR_R_IN3, HIGH);
  digitalWrite(MOTOR_R_IN4, LOW);
  analogWrite(MOTOR_R_PWM, motorSpeed * 0.7);
}

void steerRight() {
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_PWM, motorSpeed * 0.7);

  digitalWrite(MOTOR_R_IN3, LOW);
  digitalWrite(MOTOR_R_IN4, HIGH);
  analogWrite(MOTOR_R_PWM, motorSpeed * 0.7);
}

void stopMotors() {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_PWM, 0);

  digitalWrite(MOTOR_R_IN3, LOW);
  digitalWrite(MOTOR_R_IN4, LOW);
  analogWrite(MOTOR_R_PWM, 0);
}

// ==============================================================================
// ROBOTIC ARM PICK AND PLACE SEQUENCES
// ==============================================================================
void armHome() {
  servoBase.write(90);
  servoShoulder.write(90);
  servoElbow.write(90);
  servoWrist.write(90);
  servoGripper.write(30); // Open
  delay(500);
}

void executePickSequence(String wasteClass) {
  stopMotors();
  Serial.print("Executing Pick for: ");
  Serial.println(wasteClass);

  // 1. Open Gripper & Reach forward down
  servoGripper.write(30); // Open
  delay(300);
  servoShoulder.write(50);
  servoElbow.write(60);
  servoWrist.write(70);
  delay(800);

  // 2. Close Gripper to pick up item
  servoGripper.write(95); // Grip item
  delay(500);

  // 3. Lift Arm Up
  servoShoulder.write(110);
  servoElbow.write(100);
  delay(800);

  // 4. Sort based on waste class (Rotate base to bin)
  if (wasteClass.indexOf("GLASS") >= 0) {
    servoBase.write(0); // Bin 3: Glass / General Recyclable
  } else if (wasteClass.indexOf("PLASTIC") >= 0 || wasteClass.indexOf("BOTTLE") >= 0) {
    servoBase.write(45); // Bin 1: Plastics (Left)
  } else if (wasteClass.indexOf("CAN") >= 0 || wasteClass.indexOf("BATTERY") >= 0) {
    servoBase.write(135); // Bin 2: Metals (Right)
  } else {
    servoBase.write(0); // Bin 3: General Recyclable
  }
  delay(800);

  // 5. Open Gripper to drop in bin
  servoGripper.write(30); // Drop
  delay(500);

  // 6. Return to Home position
  armHome();
}

// ==============================================================================
// SETUP
// ==============================================================================
void setup() {
  Serial.begin(115200);

  // Setup Motor Pins
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_R_IN3, OUTPUT);
  pinMode(MOTOR_R_IN4, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  stopMotors();

  // Setup Servos
  servoBase.attach(PIN_SERVO_BASE);
  servoShoulder.attach(PIN_SERVO_SHOULDER);
  servoElbow.attach(PIN_SERVO_ELBOW);
  servoWrist.attach(PIN_SERVO_WRIST);
  servoGripper.attach(PIN_SERVO_GRIPPER);
  armHome();

  // Connect to Wi-Fi
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWi-Fi Connected!");
  Serial.print("ESP32 DevKit IP Address: ");
  Serial.println(WiFi.localIP());

  // Start UDP Listener
  udp.begin(localUdpPort);
  Serial.print("Listening on UDP port: ");
  Serial.println(localUdpPort);
}

// ==============================================================================
// MAIN LOOP
// ==============================================================================
void loop() {
  int packetSize = udp.parsePacket();
  
  if (packetSize > 0) {
    int len = udp.read(packetBuffer, 254);
    if (len > 0) {
      packetBuffer[len] = 0; // Null-terminate
    }

    String cmd = String(packetBuffer);
    cmd.trim();
    // Serial.println("Received: " + cmd);
    lastPacketTime = millis();

    // Process Movement Commands
    if (cmd == "CMD:FORWARD") {
      moveForward();
    } else if (cmd == "CMD:BACKWARD") {
      moveBackward();
    } else if (cmd == "CMD:LEFT") {
      steerLeft();
    } else if (cmd == "CMD:RIGHT") {
      steerRight();
    } else if (cmd == "CMD:STOP") {
      stopMotors();
    }
    // Process Robotic Arm Commands
    else if (cmd.startsWith("ARM:PICK:")) {
      String itemType = cmd.substring(9);
      executePickSequence(itemType);
    }
  }

  // Safety Watchdog: If no wireless command received recently, stop motors
  if (millis() - lastPacketTime > SAFETY_TIMEOUT_MS) {
    stopMotors();
  }
}
