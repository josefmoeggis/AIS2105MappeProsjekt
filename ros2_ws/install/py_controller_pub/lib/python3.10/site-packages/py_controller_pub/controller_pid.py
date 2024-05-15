import time

class controller:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.ki_lim = 100
        self.setpoint = setpoint
        self.integral = 0
        self.prev_error = 0
        self.error = 0
        self.last_integral_time = 0
        self.dt = 0
        self.prev_t = 0

    def update(self, current_val):
        current_t = time.time()
        self.dt = current_t - self.prev_t
        self.error = self.setpoint - current_val

        self.integral += self.ki * self.error * self.dt

        





        self.prev_t = current_t
