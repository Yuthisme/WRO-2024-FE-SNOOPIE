#include "robot.h"
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>

// BNO055 sensor object
Adafruit_BNO055 bno = Adafruit_BNO055();
Servo myservo;

// Sample rate delay for BNO055 (IMU)
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Define pins for ultrasonic sensors
const int trigPin1 = 9;
const int echoPin1 = 10;
const int trigPin3 = 11;
const int echoPin3 = 12;

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

// Create a variable to store the received packet
struct Packet rx_packet;

int turn_count = 0;  // Counter for 90-degree turns
int error = 0;       // Error variable for PID control
float imu_initial = 0; // Initial IMU orientation
float imu_current = 0; // Current IMU orientation

bool car_started = false;  // Flag to check if car should start
const int buttonPin = 6;   // Button pin

void setup() {
  Wire.begin();
  Wire.setClock(10000);
  // Initialize servo and write initial position
  myservo.attach(7);
  myservo.write(65);
  
  // Initialize serial ports for communication
  Serial.begin(115200);
  Serial2.begin(115200);
  
  // Initialize ultrasonic sensor pins
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  pinMode(buttonPin, INPUT);  // Button input pin

  // Initialize BNO055 (IMU sensor)
  if (!bno.begin()) {
    Serial.println("BNO055 initialization failed. Please check your connections.");
    while (1);  // Loop indefinitely if initialization fails
  }

  Serial.println("BNO055 connection successful");

  // Set IMU mode to NDOF (Nine Degrees of Freedom)
  bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_NDOF);
}

void loop() {
  // Read the button state
  int buttonState = digitalRead(buttonPin);
  
  // Start the car when the button is pressed (HIGH state)
  if (buttonState == HIGH) {
    car_started = true;
  }

  // If the car has not started, do nothing
  if (!car_started) {
    return;  // Wait for button press
  }

  // Read IMU orientation
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu_current = euler.x();  // Roll orientation

  // Check if data is available to read from Serial2 (TOF data)
  if (Serial2.available() >= sizeof(rx_packet)) {
    byte buffer[sizeof(rx_packet)];
    Serial2.readBytes(buffer, sizeof(buffer));
    
    // Debugging raw packet data
    Serial.print("Raw packet data: ");
    for (int i = 0; i < sizeof(buffer); i++) {
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Copy received data into the structure
    memcpy(&rx_packet, buffer, sizeof(rx_packet));

    // Validate received packet and process TOF data
    if (rx_packet.start_seq == 0x0210) {
      Serial.print("TOF 1: ");
      Serial.println(rx_packet.tx_data.tof_1);
      Serial.print("TOF 2: ");
      Serial.println(rx_packet.tx_data.tof_2);
      TOF_distance = rx_packet.tx_data.tof_1;  // Example: assign TOF_distance
      TOF_distance1 = rx_packet.tx_data.tof_2; // Example: assign TOF_distance1
    } else {
      Serial.println("Invalid packet received.");
    }
  }

  // Check if the TOF sensor detects a gap larger than 200 cm
  if (TOF_distance > 2000 && turn_count < 12) {
    Serial.println("Gap detected! Turning...");
    
    // Record initial IMU orientation
    imu_initial = imu_current;
    
    // Turn until the IMU detects a 90-degree turn
    while (abs(imu_current - imu_initial) < 90) {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      imu_current = euler.x();  // Update IMU orientation

      // Example motor control to turn the car
      analogWrite(2, 150);  // Left wheel motor speed
      analogWrite(3, 0);    // Right wheel motor speed
      
      // Optionally, add PID control here for smoother turning
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }

    // Stop the car after the turn is complete
    analogWrite(2, 0);
    analogWrite(3, 0);
    delay(3000);  // 3-second delay to stop the car

    // Increase turn count after a successful turn
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
