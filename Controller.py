from gpioConfig import *


class Controller:
    def __init__(self) -> None:
        self.freq_array = []

    def change_direction(self, error):
        # Turn off motor if error small enough
        if (-0.5 < (error) < 0.5):
            PWM.ChangeDutyCycle(0)
            self.freq_array.append(0)

        # Use the sign of the current error to determine the required direction
        # of vertical motio
        elif ((error) < 0):
            GPIO.output(DIR, UP)
        else:
            GPIO.output(DIR, DOWN)

    # Step mode functions
    def fullstep(self):
        GPIO.output(M0, 0)
        GPIO.output(M1, 0)
        GPIO.output(M2, 0)
        fs = 1000
        return fs

    def halfstep(self):
        GPIO.output(M0, 1)
        GPIO.output(M1, 0)
        GPIO.output(M2, 0)
        fs = 3000
        return fs

    def quarterstep(self):
        GPIO.output(M0, 0)
        GPIO.output(M1, 1)
        GPIO.output(M2, 0)
        fs = 11000
        return fs

    def eighthstep(self):
        GPIO.output(M0, 1)
        GPIO.output(M1, 1)
        GPIO.output(M2, 0)
        fs = 260000
        return fs

    def sixteenthstep(self):
        GPIO.output(M0, 0)
        GPIO.output(M1, 0)
        GPIO.output(M2, 1)
        fs = 240000
        return fs

    def microstep(self):
        GPIO.output(M0, 1)
        GPIO.output(M1, 0)
        GPIO.output(M2, 1)
        fs = 250000
        return fs

    def choosestep(self, e):
        if (9 <= e):
            fs = self.quarterstep()
        elif (5 < e < 9):
            fs = self.eighthstep()
        elif (2 < e <= 5):
            fs = self.sixteenthstep()
        else:
            fs = self.microstep()
        self.freq_array.append(fs / 1000)

        if ((len(self.freq_array) < 2) or (
                self.freq_array[-1] != self.freq_array[-2])):
            PWM.ChangeFrequency(fs)
            PWM.ChangeDutyCycle(50)
