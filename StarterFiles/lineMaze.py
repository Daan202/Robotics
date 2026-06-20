from robots import *
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

client = RemoteAPIClient()
sim = client.require("sim")

# HANDLES FOR ACTUATORS AND SENSORS
left_motor = Motor(sim, DeviceNames.MOTOR_LEFT_LINE, Direction.CLOCKWISE)
right_motor = Motor(sim, DeviceNames.MOTOR_RIGHT_LINE, Direction.CLOCKWISE)
color_sensor = ImageSensor(sim, DeviceNames.IMAGE_SENSOR_LINE)

# PID settings
KP =0.03#0.04#0.0.03 #0.025 #0.04
KI =0.015#0.09 #0.012#0.015
KD = 0.001#0.001#0.002
TD = 0.04

sample_time = 0.015#0.001#0.015

# speed settings
base_speed = 2
max_speed = 30.0
min_speed = -30.0
max_correction = abs(max_speed-base_speed)

# Robot and enviroment settings
reflection_setpoint = 60
edge_direction =-1

# PID intial state
integral =0.0
derivative =0.0
previous_error =0.0
previous_output = 0.0
previous_time = time.monotonic()

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

def limit (value,minimum,maximum):
    #limit the value between the max and min
    return max (minimum, min(maximum,value))

def pid_control(error):
    global integral , derivative, previous_error ,previous_output,previous_time #TODO : convert the PID to class 
   #  Calculate the dt
    current_time = time.monotonic()
    dt = current_time-previous_time

   # the PID can not run faster than the sampling time 
    if dt < sample_time:
       return previous_output
   
   # Calculate integral 
    integral += error * dt
    print('integral',integral)
    integral = limit(integral, -100,100)

    # calculate derivative
    derivative_raw = (error-previous_error) /dt
    alpha = TD/(TD+dt)
    derivative = alpha *derivative +(1-alpha)*derivative_raw

    print('derivative',derivative)
    print('P',KP * error)
    print('I',KI * integral)
    print('D',KD * derivative)

   #calculate the output
    output = KP * error + KI * integral + KD * derivative
    print ('output befor limit', output)
    output = limit(output,-max_correction ,max_correction)
    print ('limited output', output)

   # update the pervious values 
    previous_error = error
    previous_output = output
    previous_time = current_time

    return output

def follow_line():
    color_sensor._update_image() # Updates the internal image
    reflection = color_sensor.reflection() # Gets the reflection from the image

    # Starts coppeliasim simulation if not done already
    sim.startSimulation()
 
   # Calculate the error        
    error = edge_direction *( reflection_setpoint - reflection )
    print('error', error)
    print('reflection',reflection)
    # get the PID control signal
    control_signal  = pid_control(error)
    print('control signal',control_signal)

      # control the robot 
    left_speed = limit(base_speed -control_signal, min_speed, max_speed)
    right_speed = limit(base_speed +control_signal, min_speed, max_speed)
    print ('leftmotor speed',left_speed)
    print ('right motor speed',right_speed)

    left_motor.run(speed=left_speed) 
    right_motor.run(speed=right_speed)
          
def main():
	# Starts coppeliasim simulation if not done already
	sim.startSimulation()
	time.sleep(0.5)
	while True:
         state = sim.getSimulationState()
         if state == sim.simulation_advancing_running:
               follow_line()
               time.sleep(0.001)

if  __name__ == "__main__":
	main()