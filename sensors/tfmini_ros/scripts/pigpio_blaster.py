"""
Motivation:
Pigpio was being used to read sensors onto the pi before being piped into ROS.
The ServoBlaster daemon (servod) was blocking the pigpio (pigpiod) daemon.
This code offers similar functionality as the servoblaster ServoAngle class

Note:
Caution, here convention is anticlockwise
Max angle is to the right, Min angle is to the left
Max duty cycle is 2500 to the left, Min duty cycle is to right 500

AJM 16MAR22
"""

import pigpio
import time

class PigpioBlaster():
    def __init__(self, gpio_port, angle_min, angle_max, min_duty, max_duty):
        self.pi = pigpio.pi()
        self._gpio_port = gpio_port
        self._duty_min = min_duty
        self._duty_max = max_duty

        self._angle_min = angle_min
        self._angle_max = angle_max
        self._angle = 0
        self._angle_to_duty = (self._duty_max - self._duty_min)/(self._angle_min - self._angle_max)
        
    def angle_to_duty(self, angle):
        """
        Convert angle to duty cycle
        """
        duty = (angle - self._angle_max)*self._angle_to_duty + self._duty_min
        return duty
        
    def update(self, angle):
        """
        Sets servo pulse width to be that proportional to desired angle.
        Assigns member variable to this desired angle.
        If the desired angle calls for a pwm width outside the upper bounds,
        this function saturates the pulse width to the max duty cycle.
        """
        if self.angle_to_duty(angle) > self._duty_max:
            self.pi.set_servo_pulsewidth(self._gpio_port, self._duty_max)
            self._angle = self._angle_min
        else:
            self.pi.set_servo_pulsewidth(self._gpio_port, self.angle_to_duty(angle))
            self._angle = angle
        
    def set_to_min_angle(self):
        """
        Commands servo to minimum angle
        """
        self.update(self._angle_min)

    def set_to_max_angle(self):
        """
        Commands servo to maximum angle
        """
        self.update(self._angle_max)

    def set_to_middle_angle(self):
        """
        Commands servo to middle angle
        """
        self.update((self._angle_min + self._angle_max)*0.5)   

    def teardown(self):
        """
        Destructor
        """
        print("Shutting down pigpio-blaster")
        self.pi.stop()


    @property
    def angle(self):
        return(self._angle)
        
    @property
    def angle_min(self):
        return(self._angle_min)
        
    @property
    def angle_max(self):
        return(self._angle_max)
