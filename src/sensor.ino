void ultr_sonic_1(){
   // Clears the trigPin
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration_1 = pulseIn(echoPin1, HIGH);
  // Calculating the distance
  distance_1 = duration_1 * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  // Serial.print("Distance_front: ");
  // Serial.println(distance_1);
}

void ultr_sonic_3(){
   // Clears the trigPin
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration_3 = pulseIn(echoPin3, HIGH);
  // Calculating the distance
  distance_3 = duration_3 * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  // Serial.print("Distance_back: ");
  // Serial.println(distance_3);
}
