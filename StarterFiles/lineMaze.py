from robots import *
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require("sim")
# sim.loadScene('./StarterFiles/scenes/lineMaze.ttt') # from: https://www.youtube.com/watch?v=18P7azRsJY0

# HANDLES FOR ACTUATORS AND SENSORS
left_motor_cw = Motor(sim, DeviceNames.MOTOR_LEFT_LINE, Direction.CLOCKWISE)
left_motor_ccw = Motor(sim, DeviceNames.MOTOR_LEFT_LINE, Direction.COUNTERCLOCKWISE)

right_motor_cw = Motor(sim, DeviceNames.MOTOR_RIGHT_LINE, Direction.CLOCKWISE)
right_motor_ccw = Motor(sim, DeviceNames.MOTOR_RIGHT_LINE, Direction.COUNTERCLOCKWISE)

color_sensor = ImageSensor(sim, DeviceNames.IMAGE_SENSOR_LINE)

def is_red_detected(color_sensor):
    """
    Calculates the relative intensity of the red channel compared to
    other channels
    """
    red_ratio_threshold = 1.5
    red, green, blue = color_sensor.rgb()
    print(red, green, blue)
    red_intensity = red / (green + blue)

    return red_intensity > red_ratio_threshold


def is_blue_detected(color_sensor):
    """
       Calculates the relative intensity of the blue channel compared to
       other channels
       """
    blue_ratio_threshold = 1.5
    red, green, blue = color_sensor.rgb()
    blue_intensity = blue / (red + green)

    return blue_intensity > blue_ratio_threshold


def follow_line():
    """
    A very simple line follower that should be improved.
    """
    color_sensor._update_image() # Updates the internal image
    reflection = color_sensor.reflection() # Gets the reflection from the image
    print(reflection)

    left_motor_cw.run(speed=5) # Runs the left motor at speed=5
    right_motor_cw.run(speed=5) # Runs the right motor at speed=5

def img_baseline(middle_skip: int = 0):
    """
    To be called at the beginning of every PID controller trial.
    Make sure the robot is centered approximately on the edge of the line. 
    As it will likely not be centered perfectly, the middle skip gives a percentage of the middle
    image that can be ignored if desired.

    Returns a dictionary of baseline initial conditions.
    """
    img = color_sensor.get_image()
    left_reflec, right_reflec = split_image(img, middle_skip = middle_skip)

    ambient = color_sensor.ambient()
    r, g, b = color_sensor.rgb()

    dict_keys = ['left_reflec', 'right_reflec', 'ambient', 'r', 'g', 'b']
    dict_vals = [left_reflec, right_reflec, ambient, r, g, b]

    return dict(zip(dict_keys, dict_vals))

def split_image(image: np.ndarray, middle_skip: int = 0):
    """
    For an image from the ImageSensor, it splits it into a left and right part and returns
    the ambient reflection for the left and right part.

    Assumes that image is 3D (height, width, rgb channels) and that lower halve of indices across
    the width represent the left and the upper halve of the indices across the width represent the right, 
    with left and right defined with respect to the 'body' of the robot. 

    When putting the robot on the right edge of the line with the surrounding, the camera
    is likely not perfectly centered. To account for this, middle_skip can be used.
    It is a percentage indicating which portion of the width in the middle of the image to skip,
    such that we have a clean left part and a clean white part, of pure white and black, or pure black and white.
    """
    assert middle_skip >= 0, 'middle skip should be larger or equal to 0'
    assert middle_skip <= 10, 'middle_skip is number of pixels, upper bounded by 10 by us (16 is pixel width image)'
    assert middle_skip % 2 == 0, 'middle_skip should be even'
    
     # split image in two halves

    split_idx = image.shape[1] / 2

    left_img = image[:, 0:int(split_idx - middle_skip / 2), :]
    right_img = image[:, int(split_idx + middle_skip / 2):, :]
    
    return (np.mean(left_img) / 255 * 100), (np.mean(right_img) / 255 * 100)


def error_signal(bs_dict: dict,
                 rgb_weights: tuple[float] = (1,1,1),
                 normalize: bool = True):
    """
    """
    
    if sum(list(rgb_weights)) > 3 or sum(list(rgb_weights)) < 0:
        raise ValueError('some or all of your rgb weights are not between 0 and 1') 

    r, g, b = color_sensor.rgb()
    ambient = color_sensor.ambient()

    if normalize:
        r = r / ambient
        g = g / ambient
        b = b / ambient

        r_bs = bs_dict['r'] / bs_dict['ambient']
        g_bs = bs_dict['g'] / bs_dict['ambient']
        b_bs = bs_dict['b'] / bs_dict['ambient']

        left_reflec = bs_dict['left_reflec'] / bs_dict['ambient']
        right_reflec = bs_dict['right_reflec'] / bs_dict['ambient']
    else:
        r_bs = bs_dict['r']
        g_bs = bs_dict['g']
        b_bs = bs_dict['b']

        left_reflec = bs_dict['left_reflec']
        right_reflec = bs_dict['right_reflec']

    abs_error = rgb_weights[0] * abs(r - r_bs) + rgb_weights[1] * abs(g - g_bs) + rgb_weights[2] * abs(b - b_bs)

    # determine sign
    left_dif = abs(left_reflec - ambient)
    right_dif = abs(right_reflec - ambient)

    if left_dif < right_dif:
        sign = 1 # move right
    elif left_dif > right_dif:
        sign = -1 # move left
    else:
        return 0

    error = abs_error * sign

    return error 
    
    


def PID_control(error, kP: float = 1, kI: float = 1, kD: float = 1):
    """
    Implementation of  (P)roportional,
                       (I)ntegral,
                       (D)ifferential,
                       error response. 
    """
    control_signal = kP * error

    return control_signal


# if __name__ == "__main__":

#     print(img_baseline())

# Starts coppeliasim simulation if not done already
sim.startSimulation()

# MAIN CONTROL LOOP
while True:
	follow_line()


# # Starts coppeliasim simulation if not done already
# simulation_duration = 10

# sim.startSimulation()

# # MAIN CONTROL LOOP
# middle_skip = 4
# initialized = False
# while (t := sim.getSimulationTime()) < simulation_duration:

#     if not initialized:
#         bs_dict = img_baseline(middle_skip = middle_skip)
#         initialized = True
    
#     print(error_signal(bs_dict))

#     left_motor_cw.run(speed=5) # Runs the left motor at speed=5
#     right_motor_cw.run(speed=5) # Runs the right motor at speed=5

# sim.stopSimulation()


# example code from: https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm 
# to use copellia with python
# 
# from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# client = RemoteAPIClient()
# sim = client.require('sim')

# sim.setStepping(True)

# sim.startSimulation()
# while (t := sim.getSimulationTime()) < 3:
#     print(f'Simulation time: {t:.2f} [s]')
#     sim.step()
# sim.stopSimulation()