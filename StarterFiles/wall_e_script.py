
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

class Status (Enum):
	CHARGING =1
	CHARGED  =2
	SEARCH  =3 


#-----------------------------------------------------------------------------------------------
# configurations variables
MAX_SPEED = 6
TURN_SPEED = 1
FORWARD_SPEED = 5
BACKWARD_SPEED = 3
SLOW_SPEED = 1

MIN_PIXELS = 25
LOW_BATTERY = 100

#-----------------------------------------------------------------------------------------------
# State variables
last_print_time = 0.0
drive_timer_started = False
timer_time = 0
status = Status.SEARCH
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

# Move towards a target 
def drive_to (target):
	# target [x]: is -1 on the left and 0 in the center and +1 on the right 
	# use its value to give steering correction 
	steering = 1.5 * target ["x"]

	if target ["x"] >= 0 :
		turn_direction =1
	else:
		turn_direction =-1 
	
	left = FORWARD_SPEED + steering
	right = FORWARD_SPEED -steering
	drive(left , right)

# drive forward for a time 
def drive_time (sec):
	global drive_timer_started,timer_time

	if not drive_timer_started :
		timer_time = time.time() +sec
		drive_timer_started = True
		
	if time.time()< timer_time :
		Move_forward()
	else:
		stop()
		drive_timer_started = False
		return True

	


	
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
	
	# compressed 
    if colour_name == "black":
       return (red <50) & (green <50) & (blue <50)
	
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

	return {
        "pixels": pixels,
        "x": normal_x,
        "y": normal_y,
    }

#choose the best visible cube to approach
def choose_cube (image):
	green_cube = find_colour(image,"green")
	brown_cube = find_colour(image,"brown")

	if green_cube is None and brown_cube is None:
		return None, None
	if green_cube is None:
		return brown_cube, "trash"
	if brown_cube is None:
		return green_cube, "plant"
	
	#return the colour with more visible pixels 
	if green_cube["pixels"] >= brown_cube["pixels"] : 
		return green_cube, "plant"
	else:
		return brown_cube, "trash"


#-----------------------------------------------------------------------------------------------
# high level Actions

def start_delivery (container_colour):
	return False

def go_to_charger (Top_image):
	charger = find_colour(Top_image,"yellow")

	if charger is None:
		rotate (1)
		return
	if charger["pixels"] >=3500:
		stop()
		return
	drive_to(charger)
	print ( "pixels ",charger["pixels"])


#-----------------------------------------------------------------------------------------------
# Wall_e controller 
def wall_e():
	global status
	sensors = read_sensors()
	now  = time.time()

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



	global last_print_time
	
	if  now - last_print_time > 1.0:
		print ('battery level =',sensors["battery"])
		#print ("bumper sensor ",sensors["bumper"])
		#print(" sonar sensor ",sensors["sonar"])
		#print ("sensor RGP :", sensors ["top_image_rgp"])
		#print ( "function RGP :", rgb_parts (sensors ["top_image"]))
		#print (" colouer mask :", colour_mask(sensors ["top_image"],"green"))
		#print ("find the colouer :", find_colour(sensors ["top_image"],"green"))
		last_print_time = now
	#-----------------------------------------------------
	#steps

	#charging is the highest poriorty status 
	#if sensors["battery"] <  LOW_BATTERY: 
	#	status = Status.CHARGING 
	
	match status:
		case Status.CHARGING :
			#go_to_charger(sensors["top_image"])
			
			arrive = drive_time(5)
			print (" move for time ")

			if arrive :# or sensors["battery"] >= 0.99 : 
				status = Status.CHARGED

		case Status.CHARGED :
			print ("charged")

		case Status.SEARCH :
						
			arrive = drive_time(5)
			print (" move for time ")

			if arrive :# or sensors["battery"] >= 0.99 : 
				status = Status.CHARGED
		
		





	




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