from robots import *
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

client = RemoteAPIClient()
sim = client.require("sim")

# HANDLES FOR ACTUATORS AND SENSORS
left_motor = Motor(sim, DeviceNames.MOTOR_LEFT_LINE, Direction.CLOCKWISE)
right_motor = Motor(sim, DeviceNames.MOTOR_RIGHT_LINE, Direction.CLOCKWISE)

color_sensor = ImageSensor(sim, DeviceNames.IMAGE_SENSOR_LINE)


def get_error(baseline: float):
    """
    """
    return color_sensor.ambient() - baseline

def ls_regression(x: list[int], y: list[float]):
    """
    Calculates the least squares regression on a list of numbers.
    """
    sum_x = sum(x)
    sum_y = sum(y)
    sum_x_2 = sum([i**2 for i in x])
    sum_xy = sum([i*j for i,j in zip(x,y)])
    N = len(x)

    return (N * sum_xy - sum_x * sum_y) / (N * sum_x_2 - sum_x**2)

def PID_control(error: float, 
                
                kP: float, 
                kI: float, 
                kD: float, 

                I: float, 
                D_N: int, 
                D_data: list, 
                D_time: list):
    """
    Implementation of  (P)roportional,
                       (I)ntegral,
                       (D)ifferential,
                       error response. 
    """
    I = I + error # update the integrated error

    if len(D_data) == D_N:
        D = ls_regression(D_time, D_data)
        control_signal = kP * error + kI * I + kD * D

        D_data = D_data[1:].append(error)
        D_time = D_time[1:].append(int(time.time() * 1000))
    else:
        control_signal = kP * error + kI * I

        D_data.append[error]
        D_time.append(int(time.time() * 1000))
    

    return control_signal, I, D_data, D_time

def follow_line(baseline: float, 
                base_speed: float, 
                line_color: str,

                kP: float,
                kI: float,
                kD: float,

                I: float,
                D_N: int,
                D_data: list,
                D_time: list):
    """
    """
    error = get_error(baseline)

    control_signal, I, D_data, D_time = PID_control(error = error, 
                
                                                    kP = kP, 
                                                    kI = kI, 
                                                    kD = kD, 

                                                    I = I, 
                                                    D_N = D_N, 
                                                    D_data = D_data, 
                                                    D_time = D_time)
    



    
 




# Starts coppeliasim simulation if not done already
sim.startSimulation()

# PARAMETERS
kP = 1
kI = 1
kD = 1
D_N = 6
base_speed = 3
line_color = 'white'

# INITIALIZATIONS
I = 0
D_data = [], 
D_time = []
baseline = color_sensor.ambient()
left_motor.run(speed = base_speed)
right_motor.run(speed = 5)

# MAIN CONTROL LOOP
while True:

	I, D_data, D_time = follow_line(baseline = baseline, 
                                    base_speed = base_speed,
                                    line_color = line_color,
                                    
                                    kP = kP,
                                    kI = kI,
                                    kD = kD,
                                    
                                    I = I,
                                    D_N = D_N,
                                    D_data = D_data,
                                    D_time = D_time)
     
      
