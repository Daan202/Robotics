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


# def follow_line():
#     """
#     A very simple line follower that should be improved.
#     """
#     color_sensor._update_image() # Updates the internal image
#     reflection = color_sensor.reflection() # Gets the reflection from the image
#     print(reflection)

#     left_motor_cw.run(speed=5) # Runs the left motor at speed=5
#     right_motor_cw.run(speed=5) # Runs the right motor at speed=5

def img_baseline(middle_skip: int = 0):
    """
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
    assert middle_skip >= 0, 'middle_skip is percentage, so must be larger than or equal to 0'
    assert middle_skip <= 100, ' middle_skip is percentage, so must be smaller than or equal to 100'
    # split image in two halves
    img_width = image.shape[2]
    half_n_idxs2skip = round((img_width * middle_skip / 100) / 2)

    if img_width % 2 == 0:
        split_idx = img_width / 2
    else:
        split_idx = (img_width - 1) / 2

    left_img, right_img = image[:, 0:split_idx - half_n_idxs2skip, :], image[:, split_idx + half_n_idxs2skip:-1, :]

    return (np.mean(left_img) / 255 * 100), (np.mean(right_img) / 255 * 100)


def error_signal(bs_dict: dict,
                 line_color: str = 'white', 
                 surround_color: str = 'black', 
                 rgb_weights: tuple[float] = (1,1,1),
                 normalize: bool = True):
    """
    """
    if not (line_color == 'white' or line_color == 'black'):
        raise ValueError('line_color should be either white or black')        
    elif line_color == 'white' and not surround_color == 'black':
        raise ValueError('surround_color should be black when line_color is white')
    elif line_color == 'black' and not surround_color == 'white':
        raise ValueError('surround_color should be white when line_color is black')
    
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

    abs_error = rgb_weights[0] * abs(r - r_bs) + rgb_weights[1] * abs(g - g_bs) + rgb_weights[2] * abs(r - r_bs)

    # determine sign
    left_dif = abs(left_reflec - ambient)
    right_dif = abs(right_reflec - ambient)

    if left_dif < right_dif:
        sign = 1 # move right
    elif left_dif > right_dif:
        sign = -1 # move left

    error = abs_error * sign

    return error

    
    



    img = color_sensor.get_image()
    left_reflec, right_reflec = split_image(img, middle_skip = 0)
    



    
    


def PID_control(kP, kI, kD, error):
    """
    Implementation of  (P)roportional,
                       (I)ntegral,
                       (D)ifferential,
                       error response. 
    """
    pass



# Starts coppeliasim simulation if not done already
simulation_duration = 100

sim.setStepping(True)

sim.startSimulation()

# MAIN CONTROL LOOP
while (t := sim.getSimulationTime()) < simulation_duration:
    follow_line()
    # sim.step()
sim.stopSimulation()


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