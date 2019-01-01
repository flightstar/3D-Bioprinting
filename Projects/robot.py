# Modele utilise pour controler le bras robotique SCARA (R T R)

from math import atan2, pi, sqrt, atan, cos, sin
from scipy.interpolate import interp1d
import time
import Adafruit_PCA9685


class Point:
    """ Defines a point in cartesian coordinates (x,y, z). """

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Path:
    """ Defines a path as a list of points. """

    def __init__(self):
        self._points_list = []

    @property
    def points_list(self):
        """ List of points defining the path."""
        return self._points_list

    def add_point(self, point):
        self._points_list.append(point)

    @points_list.getter
    def get_points_list(self):
        return self._points_list


class Robot:
    """ Implements methods to instantiate and control a SCARA robot arm.
        Attributes:
            a: length of the first joint.
            b: length of the second joint.
    """

    # Servo motors controller (done via the 'Adafruit_PCA9685 I2C PWM Controller')
    _servo_controller = Adafruit_PCA9685.PCA9685()

    # Default channels used on the Adafruit shield for each servo.
    _base_servo = 15
    _vert_servo = 14
    _arm_servo = 1
    _gripper_servo = 0

    # Min and max pulses for each servo obtained by manually calibrating robot.
    # Necessary to map a given angle/location with a pulse (given to the Adafruit shield).
    BASE_MIN_PULSE = 650
    BASE_MAX_PULSE = 155
    BASE_MIN_ANGLE = 0
    BASE_MAX_ANGLE = pi
    ARM_MIN_PULSE = 150
    ARM_MAX_PULSE = 630
    ARM_MIN_ANGLE = - pi / 2
    ARM_MAX_ANGLE = pi / 2
    GRIP_CLOSED_PULSE = 600
    GRIP_OPENED_PULSE = 250
    MAX_HEIGHT = 7.5  # inches
    MIN_HEIGHT = 3.5  # inches
    MAX_HEIGHT_PULSE = 250
    MIN_HEIGHT_PULSE = 425

    # x coordinate increment when drawing a line
    LINE_INCREMENT = 0.1

    # State variables
    _last_h = 5  # last height recorded (inches)
    is_at_set_point = False
    debug = False

    def __init__(self, joint_a_length, joint_b_length, debug=False):
        self.a = joint_a_length  # Arm A length
        self.b = joint_b_length  # Arm B length
        self._base_angle = 0
        self._arm_angle = 0
        self._h = 5  # Height (vertical position)

        self._servo_controller.set_pwm_freq(60)  # 60 Hz frequency for the servos
        self.initial_position = Point(joint_a_length + joint_b_length, 0, self.MAX_HEIGHT)
        self.reset_position()
        self.debug = debug

    def inverse_kinematics(self, px, py, pz, nx, ny):
        """Inverse kinematics : Given a position and orientation of end effector,
            updates the robot variables to match that location.
            Args:
                px: X coordinate for the end point.
                py: Y coordinate for the end point.
                pz: Z coordinate for the end point (height).
                nx: Nx orientation coordinate.
                ny: Ny orientation coordinate.

            """

        # These equations have been derived beforehand from forward kinematics equations.
        self._last_h = self._h  # Updates last height of robot

        self._h = pz
        self._base_angle = atan2((py - self.b * nx) / self.a, (px - self.b * nx) / self.a)
        self._arm_angle = atan2(ny, nx) - self._base_angle

    def update_variables(self, px, py, pz):
        """ Given a position for the end effector, updates the robot variables to match that location.
            Note:
                This method should yield same results as inverse_kinematics, if the correct nx and ny are given.
                It is the preferable way to update variables, since only position is given.
                Adapted from source below:
                    https://github.com/edmundofuentes/raspberry-scara-robot-python/blob/master/classes/ScaraRobot.py

            Args:
                px: X coordinate of end effector.
                py: Y coordinate of end effector.
                pz: Z coordinate of end effector (height).
        """
        self._last_h = self._h
        self._h = pz
        self._arm_angle = 2 * atan(
            sqrt(((self.a + self.b) ** 2 - (px ** 2 + py ** 2)) / ((px ** 2 + py ** 2) - (self.a - self.b) ** 2)))

        # Check sign of arm_angle (elbow up or down)
        if abs(self._arm_angle - self._arm_angle) > abs(self._arm_angle + self._arm_angle):
            self._arm_angle = -self._arm_angle

        # Base_angle computation
        phi = atan2(py, px)
        psi = atan2(self.b * sin(self._arm_angle), self.a + self.b * cos(self._arm_angle))
        self._base_angle = phi - psi

    def print_variables(self):
        """ Prints the current value of all the robot variables."""
        print("Base: ", self._base_angle, ", Arm: ", self._arm_angle, ", H: ", self._h)

    def move_base(self, angle):
        """ Move the base servo to the specified angle.
            Args:
                angle (radians): Angle to set the base servo at.
            Returns:
                True if successful, False if not (angle unreachable).
                """
        if not (self.BASE_MIN_ANGLE <= angle <= self.BASE_MAX_ANGLE):
            print("Base angle unreachable: ", angle)
            return False
        else:
            # Valid angle, find the corresponding 'pulse'
            ratio = (angle - self.BASE_MIN_ANGLE) / (self.BASE_MAX_ANGLE - self.BASE_MIN_ANGLE)

            # Max(BASE_MIN_PULSE, BASE_MAX_PULSE) might not work for other robot config, to be tested.
            pulse = int(round(max(self.BASE_MIN_PULSE, self.BASE_MAX_PULSE)
                              - (self.BASE_MIN_PULSE - self.BASE_MAX_PULSE) * ratio))
            # Set servo to the computed pulse
            self._servo_controller.set_pwm(self._base_servo, 0, pulse)
            # time.sleep(0.1)
            return True

    def move_arm(self, angle):
        """ Move the arm servo to the specified angle.
            Args:
                angle (radians): Angle to set the arm servo at.
            Returns:
                True if successful, False if not (angle unreachable).
        """
        if not (self.ARM_MIN_ANGLE <= angle <= self.ARM_MAX_ANGLE):
            print("Arm angle unreachable: ", angle)
            return False
        else:
            # Valid angle, find corresponding 'pulse'
            ratio = (angle - self.ARM_MIN_ANGLE) / (self.ARM_MAX_ANGLE - self.ARM_MIN_ANGLE)
            pulse = int(round(min(self.ARM_MIN_PULSE, self.ARM_MAX_PULSE)
                              + (self.ARM_MAX_PULSE - self.ARM_MIN_PULSE) * ratio))
            self._servo_controller.set_pwm(self._arm_servo, 0, pulse)
            # time.sleep(0.1)
            return True

    def move_vertical(self, height):
        """ Moves the robot vertically to the given height.
            Args:
                height (inches) : height to set the robot to.
            Returns:
                True if successful, False if not (height unreachable).
        """
        if not (self.MIN_HEIGHT <= height <= self.MAX_HEIGHT):
            print("Height unreachable: ", height)
            return False
        else:
            # Inches -> Pulse conversion.
            ratio = (height - self.MIN_HEIGHT) / (self.MAX_HEIGHT - self.MIN_HEIGHT)
            pulse = int(round(max(self.MIN_HEIGHT_PULSE, self.MAX_HEIGHT_PULSE)
                              - abs(self.MAX_HEIGHT_PULSE - self.MIN_HEIGHT_PULSE) * ratio))
            self._servo_controller.set_pwm(self._vert_servo, 0, pulse)
            time.sleep(0.1)
            return True

    def go_to_set_point(self):
        """ Moves the robot to its currently computed variables"""

        self.is_at_set_point = self.move_vertical(self._h)
        if self.is_at_set_point:
            # We are using a single shield with one alimentation source to power different kinds of motors.
            # The vertical servo is the most powerful one and requires a lot more power.
            # With 3.5 V we give enough to the little servos, but this one is moving way slower,
            # therefore this delay is needed to set the height first before moving sideways.
            delay = abs(self._h - self._last_h)  # delay proportionnal to the height change
            time.sleep(delay)

        self.is_at_set_point = self.move_base(self._base_angle)
        # time.sleep(0.1)
        self.is_at_set_point = self.move_arm(self._arm_angle)
        # time.sleep(0.1)

    def follow_path(self, path):
        """ Follows the given path (set of points).
            Returns:
                True if every point in the path was reached.
        """
        result = True
        for point in path.get_points_list():
            if self.debug:
                print("Point P({0},{1},{2})".format(point.x, point.y, point.z))
                self.print_variables()
            self.update_variables(point.x, point.y, point.z)
            self.go_to_set_point()
            if not self.is_at_set_point:
                result = False
        return result

    def release_motors(self):
        """ Releases all the motors.  """
        self._servo_controller.set_all_pwm(0, 0)

    def go_to_coordinates(self, x, y, z):
        """ Moves the robot to the given coordinates. """
        self.update_variables(x, y, z)
        self.go_to_set_point()

    def go_to_point(self, point):
        """ Moves the robot to the given point. """
        self.go_to_coordinates(point.x, point.y, point.z)

    def reset_position(self):
        """ Returns robot to the initial position. """
        self.move_vertical(self.MAX_HEIGHT)
        time.sleep(2)
        self.go_to_point(self.initial_position)
        time.sleep(1)
        self.release_motors()

    def close_gripper(self):
        self._servo_controller.set_pwm(self._gripper_servo, 0, self.GRIP_CLOSED_PULSE)

    def open_gripper(self):
        self._servo_controller.set_pwm(self._gripper_servo, 0, self.GRIP_OPENED_PULSE)

    def draw_line(self, start_point, stop_point):
        """ Draws a line between the two given points.
            Note:
                Linear interpolation is used to generate points between the two given ones.
                When the line is vertical a simple loop is used.
        """

        # Decimal conversion to avoid arithmetic errors
        start_x = start_point.x
        start_y = start_point.y
        stop_x = stop_point.x
        stop_y = stop_point.y

        # Interpolation bounds
        x = [start_x, stop_x]
        y = [start_y, stop_y]

        # Current position
        pos_x = start_x
        pos_y = start_y

        if not isclose(start_x, stop_x):  # Line is not vertical
            # Linear interpolation function -> y = f(x)
            f = interp1d(x, y)

            while not isclose(pos_x, stop_x):
                self.go_to_coordinates(pos_x, pos_y, stop_point.z)
                if pos_x < stop_x:
                    pos_x += self.LINE_INCREMENT
                    if isclose(pos_x, stop_x):
                        self.go_to_coordinates(stop_x, stop_y, stop_point.z)  # end of line
                        if self.debug:
                            print(" End of line. Robot is at (", pos_x, ", ", pos_y, ")")
                        break
                    pos_y = f(pos_x)
                elif pos_x > stop_x:
                    pos_x -= self.LINE_INCREMENT
                    if isclose(pos_x, stop_x):
                        self.go_to_coordinates(stop_x, stop_y, stop_point.z)  # end of line
                        if self.debug:
                            print(" End of line. Robot is at (", pos_x, ", ", pos_y, ")")
                        break
                    pos_y = f(pos_x)
                if self.debug:
                    print("Going to (", pos_x, ", ", pos_y, ")")
        else:
            # Vertical line
            while not isclose(pos_y, stop_y):
                self.go_to_coordinates(pos_x, pos_y, stop_point.z)
                if pos_y < stop_y:
                    pos_y += self.LINE_INCREMENT
                elif pos_y > stop_y:
                    pos_y -= self.LINE_INCREMENT
                if self.debug:
                    print("Going to (", pos_x, ", ", pos_y, ")")


def isclose(num_a, num_b, rel_tol=1e-09, abs_tol=0.0):
    """ Utility function returning whether two floats are equal (close) to each other. """
    return abs(num_a - num_b) <= max(rel_tol * max(abs(num_a), abs(num_b)), abs_tol)
