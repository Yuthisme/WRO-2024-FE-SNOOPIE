float map1(float Input, float Min_Input , float Max_Input , float Min_Output, float Max_Output) {

  return (float) ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}

long PID(long error, long Kp, long Ki, long Kd){
   // Calculate integral
  integral += error;
  integral = constrain(integral, -50, 50); // Prevent integral windup by constraining its value
  // Calculate derivative
  long derivative = error - previousError;
  // Calculate control signal
  long control = Kp * error + Kd * derivative + Ki * integral; 

   previousError = error; 

   return control;
}
