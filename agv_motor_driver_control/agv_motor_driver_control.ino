#include <ModbusMaster.h>
#include <SoftwareSerial.h>

ModbusMaster node;

// RS485 Module communication pins
#define MAX485_DE 2
#define MAX485_RE 3

#define RS485_RX 10 
#define RS485_TX 11
#define DE_PIN 2
#define RE_PIN 3

// Constants for motor control
#define node_BAUDRATE 9600
#define MOVE_DISTANCE 600 // Distance in mm
#define MOTOR_SPEED 1000 // Speed in RPM

// node register addresses (from datasheet)
#define REG_SERVO_CONTROL 0x4657  // Servo Enable/Disable
#define POSITION_REGISTER 0x43C6 // Position Command Register
#define SPEED_REGISTER 0x42E8    // Speed
#define CTRL_MODE_REGISTER 0x465A // Control mode selection
#define MOTOR_TYPE_REGISTER 0x42EB // 2 for PMSM
#define DIRECTION_REGISTER 0x466D // 0 CCW and 1 CW
#define REG_BAUD_RATE 0x4670     // Baud Rate Configuration Register
#define REG_node_ADDRESS 0x466C // node Address Register
#define REG_CANOpen_ENABLE 0x4674 // Need to disable it to use MODBUS

SoftwareSerial RS485Serial(RS485_RX,RS485_TX);

// Motor driver addresses
const uint8_t TRACTION_DRIVERS[4] = {1, 2, 3, 4}; // IDs for traction motors
const uint8_t STEERING_DRIVERS[4] = {5, 6, 7, 8}; // IDs for steering motors
const int16_t ENCODER_CPR = 10000;   // Encoder counts per revolution (4 * lines)
// Steering Motor Encoder Counts Parameters
const float COUNTS_PER_DEGREE = ENCODER_CPR / 360.0; // Counts per degree
const int16_t MAX_COUNTS = 3333; // ±120° mapped to counts
const int16_t MIN_COUNTS = -3333; 
// Traction Motor Encoder Counts Parameters
const float WHEEL_RADIUS_MM = 80.0;   // Radius of the traction motor wheel in mm
const float WHEEL_CIRCUMFERENCE = 2 * 3.14 * WHEEL_RADIUS_MM; // Circumference in mm
const float DISTANCE_PER_COUNT = WHEEL_CIRCUMFERENCE / ENCODER_CPR; // mm/count
const uint16_t SPEED = 1000;  // speed in 0.1 r/min
const uint8_t FORWARD = 0;
const uint8_t BACKWARD = 1;

bool checkServoEnabled = false;

void setup() {
  Serial.begin(9600);
  // Setup RS485
  RS485Serial.begin(9600);
  pinMode(DE_PIN, OUTPUT);
  pinMode(RE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);
  digitalWrite(RE_PIN, LOW);
  // Initialize node communication
  node.begin(1, RS485Serial);
  // Set communication parameters dynamically
  // node.setBaudRate(node_BAUDRATE);
  // node.setnodeAddress(node_ADDRESS);
  node.preTransmission(transmitEnable);
  node.postTransmission(receiveEnable);

  Serial.println("AGV Motor Control Initialized. Enter a command: Servo ON, Servo OFF, Move Forward, Move Backward, Move Left, Move Right");

}

void loop() {
  if (Serial.available()){
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("Servo ON")){
      Serial.println("Enabling Servo Motors!!");
      for (int i = 0; i < 4; i++) {
        // Enable Traction Drivers
        node.begin(TRACTION_DRIVERS[i], RS485Serial);
        sendModbusCommand(i, REG_SERVO_CONTROL, 1); 
        // Enable Steering Drivers
        node.begin(STEERING_DRIVERS[i], RS485Serial);
        sendModbusCommand(i, REG_SERVO_CONTROL, 1); 
        checkServoEnabled = true;
      }
      Serial.println("Servos Enabled");
      
    }
    else if (cmd.equalsIgnoreCase("Servo OFF")){
      Serial.println("Disabling Servo Motors!!");
      for (int i = 0; i < 4; i++) {
        // Disable Traction Drivers
        node.begin(TRACTION_DRIVERS[i], RS485Serial);
        sendModbusCommand(i, REG_SERVO_CONTROL, 0); 
        // Disable Steering Drivers
        node.begin(STEERING_DRIVERS[i], RS485Serial);
        sendModbusCommand(i, REG_SERVO_CONTROL, 0); 
        checkServoEnabled = false;
      }
      Serial.println("Servos Disabled");
      
    }
    else if (cmd.equalsIgnoreCase("Move Forward")){
      if (checkServoEnabled){
        Serial.println("Moving AGV Forward!!");
        moveSteeringMotors(0); // Align wheels to the Forward Direction
        moveTractionMotors(600.0, SPEED, FORWARD);
      }else {Serial.println("Servos Are Disabled!!");}
    }
    else if (cmd.equalsIgnoreCase("Move Backward")){
      if (checkServoEnabled){
        Serial.println("Moving AGV Backward!!");
        moveSteeringMotors(0); //  Align wheels to the Backward Direction
        moveTractionMotors(-600.0, SPEED, BACKWARD);
      }else {Serial.println("Servos Are Disabled!!");}
    }
    else if (cmd.equalsIgnoreCase("Move Left")){
      if (checkServoEnabled){
        Serial.println("Moving AGV Left Lateraly");
        moveSteeringMotors(90); // 90 = Align wheels to the Left Direction
        moveTractionMotors(600.0, SPEED, FORWARD);
      }else {Serial.println("Servos Are Disabled!!");}
    }
    else if ( cmd.equalsIgnoreCase("Move Right")){
      if (checkServoEnabled){
        Serial.println("Moving AGV Right Lateraly");
        moveSteeringMotors(-90); // -90 = Align wheels to the Right Direction
        moveTractionMotors(600.0, SPEED, FORWARD);
      }else {Serial.println("Servos Are Disabled!!");}

    } else {
      Serial.println("Invalid command. Valid Commands: Servo ON, Servo OFF, Move Forward, Move Backward, Move Left, Move Right");
    }
  }

}



// Transmit-Only (Permanent Transmit)
void transmitEnable() {
  digitalWrite(DE_PIN, HIGH);
  digitalWrite(RE_PIN, HIGH);
}
//Recieve-Only (Permanent Receive)
void receiveEnable() {
  digitalWrite(DE_PIN, LOW);
  digitalWrite(RE_PIN, LOW);
}

// Display MODBUS Data Frame
void displayModbusDataFrame(uint8_t deviceID, uint8_t functionCode, uint16_t reg, uint16_t value) {
  uint8_t frame[6]; // Frame without CRC

  // Construct MODBUS Frame
  frame[0] = deviceID;                  // Slave Address
  frame[1] = functionCode;              // Function Code
  frame[2] = (reg >> 8) & 0xFF;         // High byte of Register Address
  frame[3] = reg & 0xFF;                // Low byte of Register Address
  frame[4] = (value >> 8) & 0xFF;       // High byte of Data
  frame[5] = value & 0xFF;              // Low byte of Data

  // Print MODBUS Frame
  Serial.print("MODBUS Frame Sent (No CRC): ");
  for (int i = 0; i < 6; i++) {
    Serial.print(frame[i], HEX);
    if (i < 5) Serial.print(" ");
  }
  Serial.println();
}

// Convert distance to encoder counts
int32_t distanceToCounts(float distance_mm) {
    return distance_mm / DISTANCE_PER_COUNT;
}
// Move traction motors for a specific distance
void moveTractionMotors(float distance_mm, uint16_t speed, uint8_t direction) {
  int32_t counts = distanceToCounts(distance_mm);

  for (int i = 0; i < 4; i++) {
    node.begin(TRACTION_DRIVERS[i], RS485Serial);
    sendModbusCommand(i, CTRL_MODE_REGISTER, 3); // Set position control mode
    sendModbusCommand(i, POSITION_REGISTER, counts); // Set position in counts
    sendModbusCommand(i, SPEED_REGISTER, speed);    // Set speed
    sendModbusCommand(i, DIRECTION_REGISTER, direction); // Set direction
  }
  Serial.print("Traction motors moved for ");
  Serial.print(distance_mm);
  Serial.println(" mm.");
}

// Map angles to encoder counts
int16_t angleToCounts(int16_t angle) {
  if (angle > 120 || angle < -120) {
    Serial.println("Angle out of range. Must be between -120 and 120 degrees.");
    return 0;
  }
  return angle * COUNTS_PER_DEGREE;
}

// Rotate Steering Motor
void moveSteeringMotors(int16_t angle) {
  int16_t counts = angleToCounts(angle);

  if (counts != 0) {
    for (int i = 0; i < 4; i++) {
      node.begin(STEERING_DRIVERS[i], RS485Serial);
      sendModbusCommand(i, CTRL_MODE_REGISTER, 3); // Set position control mode
      sendModbusCommand(i, POSITION_REGISTER, counts); // Set position in counts
    }
    Serial.print("Wheels steered to ");
    Serial.print(angle);
    Serial.println(" degrees.");
  }
}

// Send MODBUS Command
void sendModbusCommand(uint8_t deviceID, uint16_t reg, uint16_t value) {
  // node.begin(deviceID, rs485Serial);
  displayModbusDataFrame(deviceID, 0x06, reg, value); // Display frame without CRC
  node.writeSingleRegister(reg, value);
  
  }
