#include <Arduino.h>
#include <SoftwareSerial.h>

#define VEL_MAX 255
#define VEL_CURVE 155

// Pin definitions
const int motor1Pin1 = 11;  // IN1
const int motor1Pin2 = 12;  // IN2
const int motor2Pin1 = 8;   // IN3
const int motor2Pin2 = 7;   // IN4
const int enA = 9;          // ENA
const int enB = 10;         // ENB

SoftwareSerial bluetooth(2, 3); // RX, TX pins for HC-05 module

class Car {
  private:
    int motor1Pin1;
    int motor1Pin2;
    int motor2Pin1;
    int motor2Pin2;
    int enA;
    int enB;
  
  public:
    Car(int pin1, int pin2, int pin3, int pin4, int enA_pin, int enB_pin)
        : motor1Pin1(pin1), motor1Pin2(pin2), motor2Pin1(pin3), motor2Pin2(pin4),
          enA(enA_pin), enB(enB_pin) {}
  
    void initialize() {
      pinMode(motor1Pin1, OUTPUT);
      pinMode(motor1Pin2, OUTPUT);
      pinMode(motor2Pin1, OUTPUT);
      pinMode(motor2Pin2, OUTPUT);
      pinMode(enA, OUTPUT);
      pinMode(enB, OUTPUT);
      Serial.begin(9600);
      bluetooth.begin(9600);
    }
  
    void processCommand() {
      if (bluetooth.available()) {
        char command = bluetooth.read();
        
        switch (command) {
          case '2': // Move forward
            moveForward();
            break;
          case '8': // Move backward
            moveBackward();
            break;
          case '4': // Turn left
            turnLeft();
            break;
          case '6': // Turn right
            turnRight();
            break;
          case 'x': // Stop
            stopMoving();
            break;
        }
      }
    }
  
    void moveForward() {
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
      
      analogWrite(enA, VEL_MAX);
      analogWrite(enB, VEL_MAX);
    }
  
    void moveBackward() {
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
      
      analogWrite(enA, VEL_MAX);
      analogWrite(enB, VEL_MAX);
    }
  
    void turnLeft() {
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
      
      analogWrite(enA, VEL_CURVE);
      analogWrite(enB, VEL_CURVE);
    }
  
    void turnRight() {
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
      
      analogWrite(enA, VEL_CURVE);
      analogWrite(enB, VEL_CURVE);
    }
  
