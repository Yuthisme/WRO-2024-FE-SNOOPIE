#include "robot.h"
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>

Adafruit_BNO055 bno = Adafruit_BNO055();
Servo myservo;

// Sample rate delay for BNO055 (IMU)
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// TOF sensor distances
uint32_t TOF_distance = 0;
uint32_t TOF_distance1 = 0;

struct Data {
  uint32_t tof_2;
  uint32_t tof_1;
};

struct Packet {
  uint16_t start_seq;
  struct Data tx_data;
  uint16_t end_seq;
};

// Packet to store the received data
struct Packet rx_packet;

int turn_count = 0;          // Counter for 90-degree turns
int error = 0;               // Error for PID control
double prevValue = 0;        // Previous IMU roll value
double rollValue = 0;        // Current IMU roll value
double delta_omega = 0;      // Change in roll
bool car_started = false;    // Flag to check if car should start
const int buttonPin = 6;     // Button pin

void setup() {
  Wire.begin();
  Wire.setClock(10000);
  
  // Initialize servo and set initial position
  myservo.attach(7);
  myservo.write(65);
  
  // Initialize serial communication
  Serial.begin(115200);
  Serial2.begin(115200);
  
  // Set up ultrasonic sensor pins
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  pinMode(buttonPin, INPUT);  // Button input

  // Initialize BNO055 (IMU sensor)
  if (!bno.begin()) {
    Serial.println("BNO055 initialization failed. Please check your connections.");
    while (1);  // Loop indefinitely if initialization fails
  }

  Serial.println("BNO055 connection successful");

  // Set IMU to NDOF mode
  bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_NDOF);
}

void loop() {
  // Check button state to start the car
  int buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    car_started = true;
  }

  // If the car hasn't started, do nothing
  if (!car_started) {
    return;
  }

  // Check for incoming TOF data
  if (Serial2.available() >= sizeof(rx_packet)) {
    byte buffer[sizeof(rx_packet)];
    Serial2.readBytes(buffer, sizeof(buffer));
    
    // Copy received data into the structure
    memcpy(&rx_packet, buffer, sizeof(rx_packet));

    // Validate and process TOF data
    if (rx_packet.start_seq == 0x0210) {
      TOF_distance = rx_packet.tx_data.tof_1;
      TOF_distance1 = rx_packet.tx_data.tof_2;
    }
  }

  // Turn the car if a gap is detected (TOF_distance > 200 cm)
  if (TOF_distance > 2000 && turn_count < 12) {
    Serial.println("Gap detected! Turning...");

    // Perform a 90-degree turn using IMU data
    double target_heading = rollValue + 90;
    target_heading = normDiff(degreesToRadians(target_heading));

    while (abs(error_omega(target_heading)) > 1) {
      // Continue turning until IMU difference reaches 90 degrees
      analogWrite(2, 250);  // Left wheel motor speed
      analogWrite(3, 0);    // Right wheel motor speed
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }

    // Stop the car after the turn
    analogWrite(2, 0);
    analogWrite(3, 0);
    delay(3000);  // 3-second stop

    // Increase the turn count
    turn_count++;
    Serial.print("Turn count: ");
    Serial.println(turn_count);
  }

  // Stop the car after 12 turns
  if (turn_count >= 12) {
    Serial.println("12 turns completed. Stopping the car.");
    analogWrite(2, 0);
    analogWrite(3, 0);
    while (true);  // Stop execution indefinitely
  }

  // Delay for sensor sampling
  delay(BNO055_SAMPLERATE_DELAY_MS);
}


