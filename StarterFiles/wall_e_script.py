
import numpy as np
from robots import *
import time
from coppeliasim_zmqremoteapi_client import *
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import keyboard


client = RemoteAPIClient()
sim = client.require("sim")


# HANDLES FOR ACTUATORS AND SENSORS
robot = Robot_OS(sim, DeviceNames.ROBOT_OS)

top_camera = ImageSensor(sim, DeviceNames.TOP_IMAGE_SENSOR_OS)
front_camera = ImageSensor(sim, DeviceNames.SMALL_IMAGE_SENSOR_OS)

left_motor = Motor(sim, DeviceNames.MOTOR_LEFT_OS, Direction.CLOCKWISE)
right_motor = Motor(sim, DeviceNames.MOTOR_RIGHT_OS, Direction.CLOCKWISE)

#-----------------------------------------------------------------------------------------------
# configurations variables
MAX_SPEED = 6
TURN_SPEED = 2
FORWARD_SPEED = 5
BACKWARD_SPEED = 3
SLOW_SPEED = 1

MIN_PIXELS = 25

#-----------------------------------------------------------------------------------------------
# State variables
last_print_time = 0.0

#-----------------------------------------------------------------------------------------------
# Basic robot movment Helper functions


#limit the motor speed between the max and min
def limit (value):
    return max (-MAX_SPEED, min(MAX_SPEED,value))

# drive the left and right motors 
def drive (left_speed , right_speed):
	left_motor.run(limit(left_speed))
	right_motor.run(limit(right_speed))

# stop the robot
def stop ():
	drive (0,0)

#rotate the robot direction 1 right -1 left 
def rotate(direction):
	drive(direction *TURN_SPEED , - direction* TURN_SPEED )

# Move forward 
def Move_forward(slow =False):
	if slow:
		drive(SLOW_SPEED,SLOW_SPEED)
	drive(FORWARD_SPEED,FORWARD_SPEED)

# Move backword
def Move_backword(slow = False):
	if slow:
		drive(-SLOW_SPEED,-SLOW_SPEED)
	drive(-BACKWARD_SPEED,-BACKWARD_SPEED)
	
#-----------------------------------------------------------------------------------------------
#sensors Helper functions

def show_image(image):
    plt.imshow(image)
    plt.show()

def read_sensors ():
	top_camera._update_image() 
	front_camera._update_image()

	return{
		"battery"        : robot.get_battery(),
		"sonar"          : robot.get_sonar_sensor(),
		"bumper"         : robot.get_bumper_sensor(),
		"top_image"      : top_camera.get_image(),
		"top_image_rgp"  : top_camera.rgb(),
		"front_image"    : front_camera.get_image(),
		"front_image_rgp": front_camera.rgb(),
	}

#Split an RGB image into red, green and blue array
def rgb_parts(image  : np.ndarray):

    image = image.astype(np.float32)
    red = image[:, :, 0]
    green = image[:, :, 1]
    blue = image[:, :, 2]

    return red, green, blue

#Return True for pixels that match the colour
def colour_mask(image, colour_name):

    red, green, blue = rgb_parts(image)
    
	# Plant cubes
    if colour_name == "green":
        return (green > 70) & (green > red + 20) & (green > blue + 20)
	
    # Trash cubes
    if colour_name == "brown":
        return (red > 45) & (green > 18) & (green < 120) & (blue < 90) & (red > blue + 15)
	
    # Plant container
    if colour_name == "blue":
        return (blue > 75) & (blue > red + 20) & (blue > green + 10)
	
    # Trash container
    if colour_name == "red":
        return (red > 85) & (red > green + 30) & (red > blue + 30) & (green < 120)
	
    # Charging area
    if colour_name == "yellow":
        return (red > 90) & (green > 60) & (blue < 90) & (red > blue + 30)


def find_colour (image , colour_name):
	mask = colour_mask(image, colour_name)

	# find the pixels with true value (the pixels that matching the requierd colour)
	y_positions, x_positions = np.where(mask)

	# count the matching pixels
	pixels = len(x_positions) 

	# ignore small pixels detections 
	if pixels < MIN_PIXELS:
		return None 
	
	#Calculates the average position of all matching pixels 
	center_x = float(np.mean(x_positions))
	center_y = float(np.mean(y_positions))

	#convert to Normalized coorinates 
	# for a 64 x 64 pixels image 
	# x = 0 left side 
	# x = 32 center 
	# x = 64 right 
	# after normalization 
	# normal_x = (center_x - width /2 )/(width/2)
	# normal_x = -1 far left       normal_y = -1    top
	# normal_x = 0 	center         normal_y =  0    center
	# normal_x = +1 far right      normal_y = +1    bottom
	
	normal_x = (center_x -32 ) / 32
	normal_y = (center_y - 32) / 32

	return normal_x, normal_y ,pixels




	


def wall_e():
	sensors = read_sensors()

	Manual = True
	if Manual:
			#print ("manual is active")

		if keyboard.is_pressed('w') :   # move forward
				Move_forward()			
		elif keyboard.is_pressed('s'):   # move Backword
				Move_backword()
		elif keyboard.is_pressed('d'):   # move to the right
				rotate(1)
		elif keyboard.is_pressed('a'):   # move to the left
				rotate(-1)
		elif keyboard.is_pressed('c'):   
				robot.compress()
				print('compress is active ')
		elif keyboard.is_pressed('b'):   
				print('battery level',sensors["battery"])

		else:
			stop()


	now  = time.time()
	global last_print_time
	
	if  now - last_print_time > 1.0:
		#print ('battery level =',sensors["battery"])
		#print ("bumper sensor ",sensors["bumper"])
		#print(" sonar sensor ",sensors["sonar"])
		print ("sensor RGP :", sensors ["top_image_rgp"])
		print ( "function RGP :", rgb_parts (sensors ["top_image"]))
		print (" colouer mask :", colour_mask(sensors ["top_image"],"green"))
		print ("find the colouer :", find_colour(sensors ["top_image"],"green"))
		last_print_time = now
	




# MAIN CONTROL LOOP
def main():
	# Starts coppeliasim simulation if not done already
	sim.startSimulation()
	
	while True:
		state = sim.getSimulationState()	
		#print ('state :',state)

		if state == sim.simulation_advancing_running:
				wall_e()
				time.sleep(0.05)

if  __name__ == "__main__":
	main()