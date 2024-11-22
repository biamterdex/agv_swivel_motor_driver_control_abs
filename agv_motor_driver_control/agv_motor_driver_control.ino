#include <ModbusMaster.h>
#include <SoftwareSerial.h>

ModbusMaster node;

#define RS485_RX 10 
#define RS485_TX 11
#define RS485_ENABLE_PIN 2

SoftwareSerial RS485Serial(RS485_RX,RS485_TX);

const uint16_t MOVE_DISTANCE = 600; // Unit Movement Distance in mm
const uint16_t SPEED = 1000; // Command Rotation Speed in rpm

void setup() {
  Serial.begin(9600);
  RS485Serial.begin(9600);
  Serial.println("AGV Motor Control Initialized");
  node.begin(1, RS485Serial);

}

void loop() {
  // put your main code here, to run repeatedly:

}
