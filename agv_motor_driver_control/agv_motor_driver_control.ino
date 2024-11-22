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
  pinMode(RS485_ENABLE_PIN, OUTPUT);
  digitalWrite(RS485_ENABLE_PIN, LOW);

}

void loop() {
  if (Serial.available()){
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "Servo ON"){
      Serial.println("Enabling Servo Motors!!");
    }
    else if (cmd == "Servo OFF"){
      Serial.println("Disabling Servo Motors!!");
    }
    else if (cmd == "Move Forward"){
      Serial.println("Moving AGV Forward!!");
    }
    else if (cmd == "Move Backward"){
      Serial.println("Moving AGV Backward!!");
    }
    else if (cmd == "Move Left"){
      Serial.println("Moving AGV Left Lateraly");
    }
    else if ( cmd == "Move Right"){
      Serial.println("Moving AGV Right Lateraly");
    } else {
      Serial.println("Invalid command. Valid Commands: Servo ON, Servo OFF, Move Forward, Move Backward, Move Left, Move Right");
    }
  }

}
