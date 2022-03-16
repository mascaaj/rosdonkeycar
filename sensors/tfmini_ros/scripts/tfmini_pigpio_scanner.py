import pigpio_blaster as pb
import tfmini
import time
import math

class TfminiServoScanner():
    def __init__(self, 
            servo_gpio=18,              #- [ ]      Gpio port assigned to the servo 
            angle_min=-90,              #- [deg]    Minimum angle of the servo
            angle_max=90,               #- [deg]    Maximum angle of the servo
            duty_min=500,               #- [us]     duty cycle corresponding to minimum angle
            duty_max=2500,              #- [us]     duty cycle corresponding to minimum angle
            n_steps=20,                 #- [ ]      number of measurements in the min-max range
            time_min_max=0.7,           #- [s]      time for the servo to move from min to max
            serial_port="/dev/ttyAMA0"  # serial port for tfmini
            ):

        self.servo = pb.PigpioBlaster(servo_gpio, angle_min, angle_max, duty_min, duty_max)
        self.laser = tfmini.TfMini(serial_port)                     #-- Create an object laser
        self._delta_angle = (angle_max - angle_min) / (n_steps - 1)     #-- Calculate the angle step
        self._time_min_max = time_min_max                           #-- Calculate the servo speed
        self._servo_speed = (angle_max - angle_min) / time_min_max
        self._min_time_pause = self._delta_angle / self._servo_speed                #-- Calculate the minimum pause after each step command
        self._move_dir = 1                                          #-- initialize the rotational direction to 1
        
    def read_laser(self):       
        """
        Read the laser and return the value
        """
        return self.laser.get_data()

    def reset_servo(self):
        """
        Set the servo to min duty cycle position
        """
        self.servo.set_to_max_angle()
        self._move_dir = -1
        time.sleep(self._time_min_max)

    def move_servo(self):
        """
        Move the servo of one step
        """
        angle = self.angle
        #-- When reached the end, change direction
        if angle + self._delta_angle>= self.servo.angle_max:
            self._move_dir = -1
        if angle - self._delta_angle <= self.servo.angle_min :
            self._move_dir = 1    
        angle += self._delta_angle*self._move_dir           #- Get the commanded angle
        self.servo.update(angle)                            #- Command the servo
        time.sleep(self._min_time_pause)                    #- Sleep

    def scan(self, scale_factor=1.0, reset=False):
        """
        Read laser distance, step the servo
        Return results after sweep
        """
        if reset: self.reset_servo()
        ini_angle = self.angle
        self.servo.update(ini_angle)
        ranges    = []
        angle     = ini_angle
        move_dir  = self._move_dir
        time_init = time.time()

        while True:
            dist  = self.read_laser()*scale_factor
            angle = self.angle
            # print("d = %4.2f  a = %4.2f"%(dist, self.angle))
            ranges.append(dist)
            self.move_servo()

            #-- If changed sign: break
            if move_dir*self._move_dir < 0:
                break
        time_increment  = (time.time() - time_init)/(len(ranges) - 1)
        angle_increment = (angle - ini_angle)/(len(ranges) - 1 )

        return(ini_angle, angle, time_increment, angle_increment, ranges)

    def teardown(self):
        """
        Teardown of pigpio blaster ala pigpiod
        """
        print("Shutting down tfmini servo scanner")
        self.servo.teardown()

    @property
    def angle(self):
        return(self.servo.angle)

    @property
    def time_between_measurements(self):
        return(self._min_time_pause)

    @property
    def step(self):
        return(self._delta_angle)


if __name__ == "__main__":
    #-- Convention: clockwise is positive (left negative, right positive)
    tfminiscanner = TfminiServoScanner()
    tfminiscanner.reset_servo()
    time.sleep(1)

    while True:
        tfminiscanner.scan(reset=True)
    tfminiscanner.teardown()

