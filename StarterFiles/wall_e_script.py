import numpy as np
from robots import *
import time
from coppeliasim_zmqremoteapi_client import *
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import keyboard
from enum import Enum

client = RemoteAPIClient()
sim = client.require("sim")

# HANDLES FOR ACTUATORS AND SENSORS
robot = Robot_OS(sim, DeviceNames.ROBOT_OS)

top_camera = ImageSensor(sim, DeviceNames.TOP_IMAGE_SENSOR_OS)
front_camera = ImageSensor(sim, DeviceNames.SMALL_IMAGE_SENSOR_OS)

left_motor = Motor(sim, DeviceNames.MOTOR_LEFT_OS, Direction.CLOCKWISE)
right_motor = Motor(sim, DeviceNames.MOTOR_RIGHT_OS, Direction.CLOCKWISE)


class Behaviours(Enum):
    NEEDCHARGING = 1
    CHARGED = 2
    SEARCH = 3
    HANDLE_CUBE = 4
    COMPRESS = 5
    DELIVERING = 6
    AVOID_WALL = 7


# -----------------------------------------------------------------------------------------------
# configurations variables
MAX_SPEED = 6
TURN_SPEED = 1
FORWARD_SPEED = 5
BACKWARD_SPEED = 3
SLOW_SPEED = 1
MIN_PIXELS = 25
LOW_BATTERY = 0.25
CLEAR_SONAR = 0.21


# -----------------------------------------------------------------------------------------------
# Utility functions

# Split an RGB image into red, green and blue array

def rgb_parts(image: np.ndarray):
    image = image.astype(np.float32)
    red = image[:, :, 0]
    green = image[:, :, 1]
    blue = image[:, :, 2]

    return red, green, blue


# Return True for pixels that match the colour
def colour_mask(image, colour_name):
    red, green, blue = rgb_parts(image)

    # Plant cubes
    if colour_name == "green":
        return (green > 70) & (green > red + 20) & (green > blue + 20)

    # Trash cubes
    if colour_name == "brown":
        return (red > 70) & (red < 210) & (green > 15) & (green < 95) & (blue < 80) & (red > green + 35) & (
                    red > blue + 35)

    # Plant container
    if colour_name == "blue":
        return (red < 60) & (green > 70) & (blue > 100)

    # Trash container
    if colour_name == "red":
        return (red > 95) & (red < 160) & (red > blue + 70) & (red > blue + 70) & (green < 30) & (blue < 30)

    # Charging area
    if colour_name == "yellow":
        return (red > 90) & (green > 60) & (blue < 90) & (red > blue + 30)

    # compressed cube
    if colour_name == "black":
        return (red < 50) & (green < 50) & (blue < 50)

    # wall
    if colour_name == "gray":
        brightness = (red + green + blue) / 3
        return (brightness > 180) & (brightness < 235) & (abs(red - green) < 20) & (abs(red - blue) < 40) & (
                    abs(green - blue) < 35)


# find a colour in the image
def find_colour(image, colour_name):
    mask = colour_mask(image, colour_name)

    # find the pixels with true value (the pixels that matching the required colour)
    y_positions, x_positions = np.where(mask)

    # count the matching pixels
    pixels = len(x_positions)

    # ignore small pixels detections
    if pixels < MIN_PIXELS:
        return None

    # Calculates the average position of all matching pixels
    center_x = float(np.mean(x_positions))
    center_y = float(np.mean(y_positions))

    # convert to Normalized coorinates
    # for a 64 x 64 pixels image
    # x = 0 left side
    # x = 32 center
    # x = 64 right
    # after normalization
    # normal_x = (center_x - width /2 )/(width/2)
    # normal_x = -1 far left       normal_y = -1    top
    # normal_x = 0 	center         normal_y =  0    center
    # normal_x = +1 far right      normal_y = +1    bottom

    normal_x = (center_x - 32) / 32
    normal_y = (center_y - 32) / 32

    return {
        "pixels": pixels,
        "x": normal_x,
        "y": normal_y,
    }


# choose the best visible cube to approach
def find_cube(image):
    green_cube = find_colour(image, "green")
    brown_cube = find_colour(image, "brown")

    if green_cube is None and brown_cube is None:
        return None, None
    if green_cube is None:
        return brown_cube, "trash"
    if brown_cube is None:
        return green_cube, "plant"

    # return the colour with more visible pixels
    if green_cube["pixels"] >= brown_cube["pixels"]:
        return green_cube, "plant"
    else:
        return brown_cube, "trash"


class Wall_e:
    def __init__(self):
        self.last_print_time = 0.0
        self.drive_timer_started = False
        self.timer_time = 0
        self.behaviour = Behaviours.SEARCH  # Status.DONE
        self.previous_state = 0
        self.do_not_interrupt = False
        self.in_position = False

        self.delivery_start_time = 0

        self.cube_type = None
        self.cube = None
        self.search_step = 0
        self.compress_step = 0
        self.delivery_step = 0

    # ----------------------Basic robot functions-------------------------

    def read_sensors(self):
        top_camera._update_image()
        front_camera._update_image()

        return {
            "battery": robot.get_battery(),
            "sonar": robot.get_sonar_sensor(),
            "bumper": robot.get_bumper_sensor(),
            "top_image": top_camera.get_image(),
            "front_image": front_camera.get_image(),
        }

    # limit the motor speed between the max and min
    def limit(self, value):
        return max(-MAX_SPEED, min(MAX_SPEED, value))

    # drive the left and right motors
    def drive(self, left_speed, right_speed):
        left_motor.run(self.limit(left_speed))
        right_motor.run(self.limit(right_speed))

    # stop the robot
    def stop(self):
        self.drive(0, 0)

    # rotate the robot direction 1 right -1 left
    def rotate(self, direction):
        self.drive(direction * TURN_SPEED, - direction * TURN_SPEED)

    # move forward
    def Move_forward(self, slow=False):
        if slow:
            self.drive(SLOW_SPEED, SLOW_SPEED)
        else:
            self.drive(FORWARD_SPEED, FORWARD_SPEED)

    # move backward
    def Move_backword(self, slow=False):
        if slow:
            self.drive(-SLOW_SPEED, -SLOW_SPEED)
        else:
            self.drive(-BACKWARD_SPEED, -BACKWARD_SPEED)

    # move towards a target
    def drive_to(self, target):
        # target [x]: is -1 on the left and 0 in the center and +1 on the right
        # using its value to give steering correction
        steering = 1.5 * target["x"]

        left = FORWARD_SPEED + steering
        right = FORWARD_SPEED - steering
        self.drive(left, right)

    # drive forward for a time
    def drive_time(self, Move_type="F", sec=0):
        if not self.drive_timer_started:
            self.timer_time = time.time() + sec
            self.drive_timer_started = True

        if time.time() < self.timer_time:
            if Move_type == "F":
                self.Move_forward()
            if Move_type == "B":
                self.Move_backword()
            if Move_type == "R":
                self.rotate(1)
        else:
            self.stop()
            self.drive_timer_started = False
            return True

    # ------------------------behaviours------------------------------

    def close_to_wall(self, top_image):
        wall = find_colour(top_image, "gray")

        # wall is 0.6 in the height of the image
        if wall is not None:
            if wall["y"] > 0.6 and not self.do_not_interrupt:
                return True
            return False

    def go_to_charger(self, Top_image):
        charger = find_colour(Top_image, "yellow")

        if charger is None:
            self.rotate(1)
            return
        if charger["pixels"] >= 2000:
            self.do_not_interrupt = True
            if self.drive_time("F", 0.5):
                self.stop()
                return True
        self.drive_to(charger)

    # move and rotate searching for cubes
    def search(self):
        if self.search_step == 0:
            if self.drive_time("R", 0.7):
                self.search_step = 1
        if self.search_step == 1:
            if self.drive_time("F", 1.3):
                self.search_step = 0

    # Reset search parameter after interupting search
    def reset_serch(self):
        self.drive_timer_started = False
        self.timer_time = 0
        self.search_step = 0

    # moving towards cube
    def handle_cube(self, top_image, front_image, cube_type):
        if cube_type == None:
            return
        if cube_type == "trash":
            cube = find_colour(top_image, "brown")
            front_object = find_colour(front_image, "brown")
        elif cube_type == "plant":
            cube = find_colour(top_image, "green")
            front_object = find_colour(front_image, "green")
        else:
            return False

        if front_object is not None:
            if front_object["pixels"] >= 4000:
                self.stop()
                return True
        if cube is not None:
            self.drive_to(cube)
        else:
            self.Move_forward()

    # compress and handel compressed trash cube
    def Compress_trash(self, front_image, sonar):
        compressed_cube = find_colour(front_image, "black")

        # compress the trash cube
        if self.compress_step == 0:
            robot.compress()
            if sonar >= CLEAR_SONAR:
                self.compress_step = 1

        # moving forward creating space for rotation
        if self.compress_step == 1:
            if self.drive_time("F", 0.2):
                self.compress_step = 2

        # rotate to the compressed cube
        if self.compress_step == 2:
            self.rotate(1)
            if compressed_cube is not None:
                if compressed_cube["pixels"] >= 2100:
                    self.stop()
                    self.compress_step = 3

        # move forward to the compressed cube
        if self.compress_step == 3:
            if self.drive_time("F", 0.5):
                self.compress_step = 0
                return True

    # deliver cube to the container
    def cube_delivery(self, top_image, front_image, sonar, cube_type):
        if cube_type == "trash":
            container = find_colour(top_image, "red")
            no_cube = find_colour(front_image, "red")
        elif cube_type == "plant":
            container = find_colour(top_image, "blue")
            no_cube = find_colour(front_image, "blue")
        else:
            return False

        # recorde the delivery starting time
        if self.delivery_step == 0:
            self.delivery_start_time = time.time()
            self.delivery_step = 1

        delivery_time = time.time() - self.delivery_start_time

        # find the container and go there
        if self.delivery_step == 1:
            if container is None:
                self.search()
                return False
            else:
                self.reset_serch()
                self.drive_to(container)
                if container["pixels"] >= 2000:
                    self.stop()
                    self.delivery_step = 2

        # move slowly forward until drop the cube in the container
        if self.delivery_step == 2:
            self.do_not_interrupt = True
            self.Move_forward(True)
            if no_cube is not None:
                if no_cube["pixels"] >= 2000:  # is None:#sonar >= CLEAR_SONAR or
                    print("cube dropped")
                    self.stop()
                    self.delivery_step = 3

        # move backward away from the container
        if self.delivery_step == 3:
            if self.drive_time("B", 0.7):
                self.delivery_step = 4

        # rotate away from the container
        if self.delivery_step == 4:
            if self.drive_time("R", 0.7):
                self.do_not_interrupt = False
                self.delivery_step = 0
                return True

        # end the delivery if it takes too long
        if delivery_time > 20:
            print("delivery ended it takes too long")
            return True

    # -----------------------Controller--------------------------------
    def controller(self):
        sensors = self.read_sensors()
        now = time.time()

        #  useful information every one sec
        if now - self.last_print_time > 1.0:
            print(" current active behaviour is :", self.behaviour)
            print(" Battery : ", sensors["battery"])
            print(" current cube type:", self.cube_type)
            self.last_print_time = now

        # -----------------------------------------------------
        # charging is the highest priority status
        if sensors["battery"] < LOW_BATTERY and not self.behaviour == Behaviours.CHARGED:
            if self.behaviour != Behaviours.NEEDCHARGING:
                if self.drive_time("B", 0.7):
                    self.behaviour = Behaviours.NEEDCHARGING

            # always active check if the robot near a wall if True start avoiding the wall
        if self.close_to_wall(sensors["top_image"]):
            if self.behaviour != Behaviours.AVOID_WALL:
                self.previous_state = self.behaviour
            self.behaviour = Behaviours.AVOID_WALL

        if self.behaviour == Behaviours.AVOID_WALL:
            if self.drive_time("R", 1.3):
                self.behaviour = self.previous_state

        if self.behaviour == Behaviours.NEEDCHARGING:
            if not self.in_position:
                self.in_position = self.go_to_charger(sensors["top_image"])

            if sensors["battery"] >= 0.95:
                self.in_position = False
                self.behaviour = Behaviours.CHARGED

        if self.behaviour == Behaviours.CHARGED:
            if self.drive_time("B", 0.5):
                self.do_not_interrupt = False
                self.behaviour = Behaviours.SEARCH

        if self.behaviour == Behaviours.DELIVERING:
            if self.cube_delivery(sensors["top_image"], sensors["front_image"], sensors["sonar"], self.cube_type):
                self.behaviour = Behaviours.SEARCH

        if self.behaviour == Behaviours.HANDLE_CUBE:
            if self.handle_cube(sensors["top_image"], sensors["front_image"], self.cube_type):
                if self.cube_type == "trash":
                    self.behaviour = Behaviours.COMPRESS

                if self.cube_type == "plant":
                    self.behaviour = Behaviours.DELIVERING

        if self.behaviour == Behaviours.COMPRESS:
            if self.Compress_trash(sensors["front_image"], sensors["sonar"]):
                print("compressed")
                self.behaviour = Behaviours.DELIVERING

        if self.behaviour == Behaviours.SEARCH:
            self.search()
            self.cube, self.cube_type = find_cube(sensors["top_image"])
            if self.cube is not None:
                self.reset_serch()
                self.stop()
                self.behaviour = Behaviours.HANDLE_CUBE


# -------------------------------------------------------------------------------------------------------------
# MAIN CONTROL LOOP


def main():
    # Starts Coppeliasim simulation if not done already
    sim.startSimulation()
    time.sleep(0.5)
    wall_e = Wall_e()
    while True:
        state = sim.getSimulationState()
        if state == sim.simulation_advancing_running:
            wall_e.controller()
            time.sleep(0.05)


if __name__ == "__main__":
    main()
