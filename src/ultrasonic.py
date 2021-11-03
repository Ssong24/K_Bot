
import serial
from time import sleep

"""
This program handles the communication
between R.pi 2 and a HC-SR04 Ultrasonic sensor
Made by Songeun Kim
"""


class Ultrasonic:
    def __init__(self, stop_at=12, steer_at=20):
        self.stop_distance = stop_at
        self.steer_distance = steer_at

        self.dont_read = [0,9,2,3,10,12,13,14,15]

        self.ser = serial.Serial("/dev/ttyACM0", 115200)
        self.ser.flushInput()

    def get_distance(self, data_list, robot_direction):
        """Get the measurement from the ultrasonic sensors continuously

        :param data_list: list of the values form the ultrasonic sensors
        :param robot_direction: 1 = forward mode, 0 = backward mode
        :type data_list: list
        :type robot_direction: 'i' multiprocessing Value
        """

        while True:

            # Select the pins that shouldn't be read
            if robot_direction.value == 1:
                self.dont_read = [0,9,2,3,10,12,13,14,15]
            else:
                self.dont_read = [4,5,2,3,10,12,13,14,15]

            new_data_list = [0] * 16

            # Sensor gives the same measurement 4 times
            # Take the average of 8 measurements -> 8 * 4 = 32
            for cal in range(32):

                # Measure all the sensors, except those in "dont_read"
                for sensor in range(len(data_list)):
                    if sensor in self.dont_read:
                        continue

                    # If there is a new measurement in queue, save it
                    if self.ser.inWaiting() > 0:
                        s_data_list = self.ser.readline() #type(data_list)= str
                        split_data_list = s_data_list.split(b' ')
                        if len(split_data_list) < 17:
                            continue
                        if split_data_list[len(split_data_list)-1] == b'\r\n' :
                            split_data_list.remove(b'\r\n')
                        data_list = list(map(int, split_data_list))
                        print(data_list)
                # Sum the sensors throughout the 32 measurements
                new_data_list = [x + y for x, y in zip(data_list, new_data_list)]

            # Take the average of the
            new_data_list = [i / 32 for i in data_list]
            data_list = new_data_list

    def detect_object(self, data_list, robot_direction, interrupt):
        """If there is an object within stop_distance, interrupt

        :param data_list: list of the values form the ultrasonic sensors
        :param robot_direction: 1 = forward mode, 0 = backward mode
        :param interrupt: stop if object detected [gyro, ultrasonic]
        :type data_list: list
        :type robot_direction: 'i' multiprocessing Value
        :type interrupt: 'i' multiprocessing Value [gyro, ultrasonic]
        """

        while True:

            # Robot in forward mode
            if robot_direction.value == 1:
                check1 = data_list[4]
                check2 = data_list[5]

            # Robot in backward mode
            else:
                check1 = data_list[0]
                check2 = data_list[6]

            # If we are too close, interrupt
            if check1 < self.stop_distance or check2 < self.stop_distance:
                interrupt[1] = 1

                # wait 9 seconds to check again
                sleep(9)
            else:
                interrupt[1] = 0

    def align_robot(self, data_list, robot_direction, change_angle):
        """Steer the robot to either direction if it's too close to a "wall"

        :param data_list: list of the values form the ultrasonic sensors
        :param robot_direction:1 = forward mode, 0 = backward mode
        :param change_angle: 0 = no change, 1 = left motor, 2 = right motor
        :type data_list: list
        :type robot_direction: 'i' multiprocessing Value
        :type change_angle: 'i' multiprocessing Array
        """

        while True:

            # If we are too close on the left, turn right
            if (data_list[0] + data_list[9]) / 2 < self.steer_distance:

                # If robot is in forward mode, turn left wheel
                if robot_direction.value == 1:
                    change_angle[1] = 1

                # If robot is in backward mode, turn right wheel
                else:
                    change_angle[1] = 2

            # If we are too close on the right, turn left
            elif (data_list[6] + data_list[7]) / 2 < self.steer_distance:

                # If robot is in forward mode, turn right wheel
                if robot_direction.value == 1:
                    change_angle[1] = 2

                # If robot is in backward mode, turn left wheel
                else:
                    change_angle[1] = 1

            # If we aren't too close, don't turn
            else:
                change_angle[1] = 0
