class PID:
    def __init__(self, error, kp, ki, settle_error=3, settle_time=0):
        self.error = error
        self.kp = kp
        self.ki = ki
        self.settle_error = settle_error
        self.settle_time = settle_time
        self.time_spent_settled = 0
        self.time_spent_running = 0
        self.settled = False

    def calculate_pid_output(self, error):
        kd = 0.0
        integral = 0.0
        previous_error = 0.0
        proportional = self.kp * error
        integral += self.ki * error
        derivative = kd * (error - previous_error)
        previous_error = error

        if abs(error) < self.settle_error:
            #print(f"fabs(error): {abs(error)}")
            # self.time_spent_settled += 10  # Assuming 10 is the time increment, e.g., milliseconds
            self.settled = True
        else:
            # self.time_spent_settled = 0
            self.settled = False

        return proportional + integral + derivative

    def is_settled(self):
        #print(f"time_spent_settled: {self.time_spent_settled}")
        
        # if self.time_spent_settled > self.settle_time:
        #     return True
        return self.settled
    