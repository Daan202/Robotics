from numpy import dtype

from robots import *
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import keyboard
import time

client = RemoteAPIClient()
sim = client.require("sim")

# HANDLES FOR ACTUATORS AND SENSORS
left_motor = Motor(sim, DeviceNames.MOTOR_LEFT_LINE, Direction.CLOCKWISE)
right_motor = Motor(sim, DeviceNames.MOTOR_RIGHT_LINE, Direction.CLOCKWISE)
color_sensor = ImageSensor(sim, DeviceNames.IMAGE_SENSOR_LINE)

# PID settings
KP =0.03#0.0.03 #0.025 #0.04
KI = 0.015#0.015
KD = 0.000#0.002
sample_time = 0.001#0.001#0.015

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
    #print ('dt',dt)
    
   # the PID can not run faster than the sampling time 
    if dt < sample_time:
       return previous_output
   
   # Calculate integral and derivative part
    integral += error * dt
    print('integral',integral)
    #integral = limit(integral, -50,50)
    derivative = (error-previous_error) /dt
    print('derivative',derivative)

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
    """
    A very simple line follower that should be improved.
    """
    color_sensor._update_image() # Updates the internal image
    reflection = color_sensor.reflection() # Gets the reflection from the image
    red, green, blue = color_sensor.rgb()
    image = color_sensor.get_image()
    ambient = color_sensor.ambient()


    # Starts coppeliasim simulation if not done already
    sim.startSimulation()
    Manual = False
    if Manual:
         #print ("manual is active")

         if keyboard.is_pressed('w') :   # move forward
            speed_f =20
            left_motor.run(speed=speed_f) # Runs the left motor at speed=5
            right_motor.run(speed=speed_f) # Runs the right motor at speed=5
            print('reflection',reflection)
            print ("red",red)
            print("green",green)
            print("blue",blue)
           # print ('image',image)
            print ('ambient',ambient)
            
         elif keyboard.is_pressed('s'):   # move Backword
            left_motor.run(speed=-5) # Runs the left motor at speed=5
            right_motor.run(speed=-5) # Runs the right motor at speed=5
            print('reflection',reflection)
            print ("red",red)
            print("green",green)
            print("blue",blue)

            print ('ambient',ambient)

         elif keyboard.is_pressed('d'):   # move to the right
            left_motor.run(speed= 5) # Runs the left motor at speed=5
            right_motor.run(speed= 2) # Runs the right motor at speed=5 
            print('reflection',reflection)
            print ("red",red)
            print("green",green)
            print("blue",blue)

            print ('ambient',ambient)
         elif keyboard.is_pressed('a'):   # move to the left
            left_motor.run(speed= 2) # Runs the left motor at speed=5
            right_motor.run(speed= 5) # Runs the right motor at speed=5  
            print('reflection',reflection)
            print ("red",red)
            print("green",green)
            print("blue",blue)
            print ('ambient',ambient)
         else:
            left_motor.run(speed=0) # Runs the left motor at speed=5
            right_motor.run(speed=0) # Runs the right motor at speed=5
            print('ambient',ambient)
            print('reflection',reflection)
    else: 
      # Calculate the error        
      error = edge_direction *( reflection_setpoint - reflection )
      print('error', error)
      print('reflection',reflection)
      # get the PID control signal
      control_signal  = pid_control(error)
      print('control signal',control_signal)

      # control the robot 
      #print('control signal',control_signal)
      #print('reflection',reflection)

      left_speed = limit(base_speed -control_signal, min_speed, max_speed)
      right_speed = limit(base_speed +control_signal, min_speed, max_speed)
      print ('leftmotor speed',left_speed)
      print ('right motor speed',right_speed)

      left_motor.run(speed=left_speed) 
      right_motor.run(speed=right_speed)



# MAIN CONTROL LOOP
while True:
   #if keyboard.is_pressed('n'):
      follow_line()
      #time.sleep(0.001)
   
