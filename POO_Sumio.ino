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

class Motor {
  private:
    int pin1;
    int pin2;
  
  public:
    Motor(int pin1, int pin2) : pin1(pin1), pin2(pin2) {
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
    }
  
    void forward() {
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
    }
  
    void backward() {
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
    }
  
    void stop() {
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
    }
};

class Car {
  private:
    Motor motor1;
    Motor motor2;
    int enA_pin;
    int enB_pin;
  
  public:
    Car(int pin1, int pin2, int pin3, int pin4, int enA_pin, int enB_pin)
        : motor1(pin1, pin2), motor2(pin3, pin4), enA_pin(enA_pin), enB_pin(enB_pin) {
      pinMode(enA_pin, OUTPUT);
      pinMode(enB_pin, OUTPUT);
    }
  
    void initialize() {
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
      motor1.forward();
      motor2.forward();
      
      analogWrite(enA_pin, VEL_MAX);
      analogWrite(enB_pin, VEL_MAX);
    }
  
    void moveBackward() {
      motor1.backward();
      motor2.backward();
      
      analogWrite(enA_pin, VEL_MAX);
      analogWrite(enB_pin, VEL_MAX);
    }
  
    void turnLeft() {
      motor1.backward();
      motor2.forward();
      
      analogWrite(enA_pin, VEL_CURVE);
      analogWrite(enB_pin, VEL_CURVE);
    }
  
    void turnRight() {
      motor1.forward();
      motor2.backward();
      
      analogWrite(enA_pin, VEL_CURVE);
      analogWrite(enB_pin, VEL_CURVE);
    }
  
    void stopMoving() {
      motor1.stop();
      motor2.stop();
      
      analogWrite(enA_pin, 0);
      analogWrite(enB_pin, 0);
    }
};

Car car(motor1Pin1, motor1Pin2, motor2Pin1, motor2Pin2, enA, enB);

void setup() {
  car.initialize();
}

void loop() {
  car.processCommand();
}
