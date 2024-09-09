// Function to calculate omega (change in orientation)
double calc_omega() {
  sensors_event_t event;
  bno.getEvent(&event);
  rollValue = event.orientation.x;

  // Calculate change in roll angle
  delta_omega = rollValue - prevValue;
  if (delta_omega < 180) {
    prevValue = rollValue;
  } else if (delta_omega > 180) {
    delta_omega -= 360;
    prevValue = rollValue;
  }
  return delta_omega;
}

// Function to convert degrees to radians
float degreesToRadians(float degrees) {
  return degrees * PI / 180.0;
}

// Function to calculate error in orientation from the target heading
double error_omega(double target_heading) {
  sensors_event_t event;
  bno.getEvent(&event);
  rollValue = event.orientation.x;

  // Calculate error in roll
  delta_omega = rollValue - target_heading;
  if (delta_omega > 180){
    delta_omega -= 360;
  }
  return delta_omega;
}

// Normalize angle between -PI and PI
double normDiff(double angle) {
  angle = fmod(angle, 2.0 * PI);
  if (angle >= PI) {
    return angle - 2.0 * PI;
  }
  if (angle < -PI) {
    return angle + 2.0 * PI;
  }
  return angle;
}
