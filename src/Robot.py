from time import sleep
import RPi.GPIO as GPIO
from multiprocessing import Value

pins = {"Enable": 17,
        "MotorLeft": 5,
        "MotorRight": 6,
        "DirectionMotorLeft": 12,
        "DirectionMotorRight": 26,
        "MS1": 20,
        "MS2": 21,
        "Encoder": 23}

rotary_now = 0


class Robot:

    def __init__(self):

        GPIO.setwarnings(False)

        with open('facing.txt', 'r') as f:
            self.facing = f.read()

        GPIO.setmode(GPIO.BCM)
        for pin in pins.values():
            GPIO.setup(pin, GPIO.OUT)

        self.frequency = 150
        self.step_mode = self.change_step_mode(1/8)

        # Enable and configure motors
        GPIO.output(pins["Enable"], GPIO.LOW)
        self.motor_left = GPIO.PWM(pins["MotorLeft"], self.frequency)
        self.motor_right = GPIO.PWM(pins["MotorRight"], self.frequency)

        # Direction is forward
        GPIO.output(pins["DirectionMotorRight"], GPIO.LOW)
        GPIO.output(pins["DirectionMotorLeft"], GPIO.HIGH)

        # rotary counter
        GPIO.setup(pins["Encoder"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(pins["Encoder"], GPIO.RISING, self.encoder_counter)

    def shutdown(self):
        """Cleanup of the GPIO pins and writing the direction the robot is facing to a file.
        """

        with open('facing.txt', 'w') as f:
            f.write(self.facing)
        GPIO.cleanup()
        GPIO.output(pins["Enable"], GPIO.HIGH)

    @staticmethod
    def encoder_counter(channel):
        """Add rotary every time there is a high in encoder

        :param channel: NEEDED BECAUSE IT'S A DETECTABLE EVENT. DONT REMOVE!!!
        """

        global rotary_now
        rotary_now = rotary_now + 1

    def go_straight(self, distance, map_direction, gyro_angle, interrupt=Value('i', 0)):
        """ Goes straight a given distance.

        :param distance: Distance to travel
        :param map_direction: Right, Left, Up or Down, relative to the map
        :param gyro_angle: gyro angle for corrections
        :param interrupt: 1: interrupt, 0: no interrupt
        :type distance: 'd' multiprocessing Value
        :type map_direction: List[int]
        :type gyro_angle: 'd' multiprocessing Value
        :type interrupt: 'i' multiprocessing Value
        :return: Whether the route was completed or not
        :rtype: Boolean
        """

        # Tweak these for optimal acceleration!
        start_frequency = 150
        max_frequency = 1400
        frequency_step = 20
        slowdown = 0.002

        angle_multiplier = 10
        stop_rotations = 50
        seconds_to_wait = 10

        rotary_goal = round(distance / 0.98)
        global rotary_now
        rotary_now = 0

        self.frequency = start_frequency

        initial_angle = gyro_angle.value
        add_time = 0

        self.motor_left.start(50.0)
        self.motor_right.start(50.0)

        while rotary_now < rotary_goal:

            # right is minus, left is plus
            current_angle = int(round(gyro_angle.value - initial_angle))

            print("Frequency: %.2f - Angle: %.2d - Distance: %.2d"
                  % (self.frequency, current_angle, rotary_now))

            # if we are going slow enough to stop and there is an interrupt, start waiting
            if self.frequency == start_frequency and interrupt.value == 1:

                self.motor_left.stop()
                self.motor_right.stop()

                # If there is an interrupt, stop and wait 12 seconds
                while interrupt.value == 1:
                    sleep(1)
                    seconds_to_wait -= 1

                    # If we have waited 12 seconds
                    if seconds_to_wait == 0:

                        # Revert the movement
                        interrupt.value = 0

                        if gyro_angle[0] == 0:
                            next_direction = [180, -180]
                        elif gyro_angle[0] == 180:
                            next_direction = [0, 0]
                        elif gyro_angle[0] == 90:
                            next_direction = [-90, 270]
                        else:
                            next_direction = [90, -270]

                        self.turn(direction="Left", map_direction=next_direction, gyro_angle=gyro_angle)
                        self.go_straight(rotary_now, next_direction, interrupt)

                        return False

                seconds_to_wait = 12
                self.motor_left.start(50.0)
                self.motor_right.start(50.0)

            # if going straight, reset frequencies
            if current_angle == 0:
                self.motor_left.ChangeFrequency(self.frequency)

            # If going too far from the current path
            while abs(current_angle) >= 5:
                self.motor_left.stop()
                self.motor_right.stop()

                self.frequency = start_frequency
                add_time = 0
                sleep(0.5)

                # Minus means too far right, plus means too far left
                if current_angle < 0:       # too far right
                    print("Turn Left")
                    self.turn("Left", map_direction, gyro_angle)
                else:                       # too far left
                    print("Turn Right")
                    self.turn("Right", map_direction, gyro_angle)

                current_angle = int(round(gyro_angle.value - initial_angle))
                sleep(0.5)

                self.motor_left.start(50.0)
                self.motor_right.start(50.0)

            # accelerate, compensation from angle
            # deceleration relative to the current speed (frequency)
            if self.frequency < max_frequency and rotary_goal - rotary_now \
                    > ((self.frequency - start_frequency) / (max_frequency - start_frequency)) * stop_rotations\
                    and interrupt.value == 0:

                self.frequency += frequency_step
                self.motor_right.ChangeFrequency(self.frequency)
                self.motor_left.ChangeFrequency(self.frequency + (current_angle * angle_multiplier))
                add_time += slowdown

            # decelerate, compensation from angle
            elif self.frequency > start_frequency:
                self.frequency -= frequency_step
                self.motor_right.ChangeFrequency(self.frequency)
                self.motor_left.ChangeFrequency(self.frequency + (current_angle * angle_multiplier))
                add_time = 0

            sleep(0.1 + add_time)

        self.motor_left.stop()
        self.motor_right.stop()

        return True

    def turn(self, direction, map_direction, gyro_angle):
        """Turn to a certain direction

        :param direction: Direction to turn in: "Left" or "Right"
        :param map_direction: Right, Left, Up or Down, relative to the map
        :param gyro_angle: gyro angle to calculate from
        :type direction: String
        :type map_direction: List[int]
        :type gyro_angle: 'd' multiprocessing Value
        """

        initial_angle = gyro_angle.value
        start_frequency = 150
        max_frequency = 300
        add = 0

        # Change the wheel spinning direction to spin in place
        direction_pin = "DirectionMotor" + str(direction)
        GPIO.output(pins[direction_pin], not GPIO.input(pins[direction_pin]))

        self.motor_right.ChangeFrequency(start_frequency)
        self.motor_left.ChangeFrequency(start_frequency)

        self.motor_left.start(50.0)
        self.motor_right.start(50.0)

        print("Initial angle: " + str(initial_angle))

        while int(round(gyro_angle.value)) not in map_direction:
            # print("Angle: %.2f" % gyro_angle.value)

            if start_frequency + add < max_frequency:
                add += 1
                self.motor_right.ChangeFrequency(start_frequency + add)
                self.motor_left.ChangeFrequency(start_frequency + add)
                sleep(0.005)

        self.motor_left.stop()
        self.motor_right.stop()

        print("End angle: " + str(gyro_angle.value))

        # change the motor back to the original direction
        GPIO.output(pins[direction_pin], not GPIO.input(pins[direction_pin]))

    @staticmethod
    def change_step_mode(step_mode=1/8):
        """Change step mode and calculate new step length based on that

        :param step_mode: step mode of the motors
        :type step_mode: float
        """
        if step_mode == 1:
            GPIO.output((pins["MS1"], pins["MS2"]), GPIO.LOW)
            return 1
        if step_mode == 1/2:
            GPIO.output(pins["MS1"], GPIO.HIGH)
            GPIO.output(pins["MS2"], GPIO.LOW)
            return 1/2
        if step_mode == 1/4:
            GPIO.output(pins["MS1"], GPIO.LOW)
            GPIO.output(pins["MS2"], GPIO.HIGH)
            return 1/4
        else:         # 1/8
            GPIO.output((pins["MS1"], pins["MS2"]), GPIO.HIGH)
            return 1/8

    def get_inverse_direction(self):

        if self.facing == "Right":
            return "Left"
        elif self.facing == "Left":
            return "Right"
        elif self.facing == "Up":
            return "Down"
        else:           # Down
            return "Up"

    def decide_turn_direction(self, next_direction):
        """Decide the direction to turn in relative to  the robot

        :param next_direction: map direction to turn to
        :type next_direction = String
        :return: relative direction
        :rtype: String
        """

        # if facing backwards, try to initially go forwards.
        if GPIO.input(pins["DirectionMotorRight"]):
            GPIO.output(pins["DirectionMotorRight"], GPIO.LOW)
            GPIO.output(pins["DirectionMotorLeft"], GPIO.HIGH)
            self.facing = not self.facing

        if self.facing == "Right":
            if next_direction == "Down":
                return "Right"
            elif next_direction == "Up":
                return "Left"
            else:                 # Left
                return "Left"

        elif self.facing == "Left":
            if next_direction == "Down":
                return "Left"
            elif next_direction == "Up":
                return "Right"
            else:                 # Right
                return "Right"

        elif self.facing == "Up":
            if next_direction == "Right":
                return "Right"
            elif next_direction == "Left":
                return "Left"
            else:                 # Down
                return "Left"

        else:   # down
            if next_direction == "Right":
                return "Left"
            elif next_direction == "Left":
                return "Right"
            else:                  # Up
                return "Right"
