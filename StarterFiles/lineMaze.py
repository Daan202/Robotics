from robots import *
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require("sim")
# sim.loadScene('./StarterFiles/scenes/lineMaze.ttt') # from: https://www.youtube.com/watch?v=18P7azRsJY0

# HANDLES FOR ACTUATORS AND SENSORS
left_motor = Motor(sim, DeviceNames.MOTOR_LEFT_LINE, Direction.CLOCKWISE)
right_motor = Motor(sim, DeviceNames.MOTOR_RIGHT_LINE, Direction.CLOCKWISE)
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


    left_motor.run(speed=5) # Runs the left motor at speed=5
    right_motor.run(speed=5) # Runs the right motor at speed=5





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