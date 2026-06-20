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

class Behaviours (Enum):
	NEEDCHARGING =1
	CHARGED  =2
	SEARCH  =3 
	HANDLE_CUBE =4
	COMPRESS =5
	DELIVERING =6
	AVOID_WALL =7
#-----------------------------------------------------------------------------------------------
# configurations variables
MAX_SPEED = 6
TURN_SPEED = 1
FORWARD_SPEED = 5
BACKWARD_SPEED = 3
SLOW_SPEED = 1


MIN_PIXELS = 25
LOW_BATTERY = 0.25
CLEAR_SONAR = 0.21

#-----------------------------------------------------------------------------------------------
# State variables
last_print_time = 0.0
drive_timer_started = False
timer_time = 0
behaviour = Behaviours.SEARCH   #Status.DONE   
previous_state = 0
do_not_interrupt = False
in_position = False

delivery_start_time = 0

cube_type = None
cube = None
end_search = False
search_step =0
compress_step =0 
delivery_step =0
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
	else:
		drive(FORWARD_SPEED,FORWARD_SPEED)

# Move backword
def Move_backword(slow = False):
	if slow:
		drive(-SLOW_SPEED,-SLOW_SPEED)
	else:
		drive(-BACKWARD_SPEED,-BACKWARD_SPEED)

# Move towards a target 
def drive_to (target):
	# target [x]: is -1 on the left and 0 in the center and +1 on the right 
	# using its value to give steering correction 
	steering = 1.5 * target ["x"]

	left = FORWARD_SPEED + steering
	right = FORWARD_SPEED -steering
	drive(left , right)

# drive forward for a time 
def drive_time (Move_type ="F",sec=0 ):
	global drive_timer_started,timer_time

	if not drive_timer_started :
		timer_time = time.time() +sec
		drive_timer_started = True

	
	if time.time()< timer_time :
		if Move_type =="F":
			Move_forward()
		if Move_type =="B":
			Move_backword()
		if Move_type =="R":
			rotate(1)
	else:
		stop()
		drive_timer_started = False
		return True	
#-----------------------------------------------------------------------------------------------
#sensors Helper functions

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
      return (red > 70) &(red < 210) &(green > 15) &(green < 95) &(blue < 80) & (red > green + 35) & (red > blue + 35) 
		     
    # Plant container
    if colour_name == "blue":
       return (red < 60) & (green > 70)  & (blue > 100) 
	
    # Trash container
    if colour_name == "red":
        return (red > 95) & (red < 160) & (red > blue + 70) & (red > blue + 70) & (green < 30)&(blue < 30)

    # Charging area
    if colour_name == "yellow":
        return (red > 90) & (green > 60) & (blue < 90) & (red > blue + 30)
	
	# compressed cube
    if colour_name == "black":
       return (red <50) & (green <50) & (blue <50)
	
	#wall
    if colour_name == "gray":
        brightness = (red + green + blue) / 3
        return (brightness >180) & (brightness <235) & (abs(red - green) < 20) & (abs(red - blue) < 40) & (abs(green - blue) < 35)

       
        
# find a colour in the image 	
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
def find_cube (image):
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
# high level Layers 

def close_to_wall (top_image):
	global do_not_interrupt
	wall = find_colour(top_image,"gray")
	
	# wall is  is 0.6 in the hight of the image 
	if wall is not None:
		if wall["y"] > 0.6 and not do_not_interrupt:
			return True
		return False


def go_to_charger (Top_image):
	global do_not_interrupt
	charger = find_colour(Top_image,"yellow")

	if charger is None:
		rotate (1)
		return
	if charger["pixels"] >=2000:
		do_not_interrupt = True
		if drive_time("F",0.5):
			stop()
			return True
	drive_to(charger)

# move and rotate searching for cubes 
def search ():
	global search_step
	if search_step == 0 :
		if drive_time("R",0.7) :
			search_step =1
	if search_step == 1 :
		if drive_time("F",1.3) :
			search_step =0

# Reset search paramter after interupting search
def reset_serch():
	global drive_timer_started,timer_time,search_step
	drive_timer_started = False
	timer_time = 0
	search_step =0

# moving towarde cube 
def handle_cube(top_image,front_image,cube_type):
	if cube_type ==None:
		return
	if cube_type == "trash":
		cube = find_colour(top_image,"brown")
		front_opject = find_colour(front_image,"brown")
	if cube_type == "plant":
		cube = find_colour(top_image,"green")
		front_opject = find_colour(front_image,"green")
	
	if front_opject is not None:
		if front_opject["pixels"] >= 4000 :
			stop()
			return True	
	if cube is not None:
		drive_to(cube)
	else:
		Move_forward()
			
#compress and handel compressed trash cube 	
def Compress_trash(front_image,sonar) :
	global compress_step
	compressed_cube = find_colour(front_image,"black")
	#print("compress_step",compress_step)

	# compress the trash cube 
	if compress_step == 0:
		robot.compress()
		#print ("sonar", sonar)
		if sonar >= CLEAR_SONAR:
			compress_step = 1

	# moving forwarde creating space for rotation
	if compress_step == 1:
		if drive_time("F",0.2):
			compress_step = 2

	# rotate to the compressed cube 
	if compress_step == 2:
		rotate(1)
		if compressed_cube is not None:
			#print("compressed cube pixels:",compressed_cube["pixels"])
			if compressed_cube["pixels"] >= 2100:
				stop()
				compress_step =3

	# move forwarde to the compressed cube
	if compress_step == 3:
		if drive_time("F",0.5):
			compress_step = 0
			return True	
		
# deliver cube to the container 				
def cube_delivery(top_image,front_image,sonar,cube_type):

	global delivery_step,do_not_interrupt,delivery_start_time

	if cube_type == "trash":
		container = find_colour(top_image,"red")
		no_cube = find_colour(front_image,"red")
	if cube_type == "plant":
		container = find_colour(top_image,"blue")
		no_cube = find_colour(front_image,"blue")

	delivery_time = time.time() - delivery_start_time
	# recorde the delivery starting time 
	if delivery_step ==0 :
		delivery_start_time = time.time()
		delivery_step =1

	#finde the container and go there 
	if delivery_step ==1 :
		if container is None:
			search ()
			return False
		else:
			reset_serch()
			drive_to(container)
			if container["pixels"] >=2000:
				stop()
				delivery_step =2

	# move slowly forwarde until drop the cube inthe container
	if delivery_step==2:
		do_not_interrupt = True
		Move_forward(True)
		if no_cube is not None:
			if  no_cube["pixels"] >=2000: #is None:#sonar >= CLEAR_SONAR or
				print ("cube droped")
				stop()
				delivery_step =3

	# move backword away from the container 	
	if delivery_step==3:
		if drive_time("B",0.7):	
			delivery_step =4

	# rotate away from the container 
	if delivery_step==4:
		if drive_time("R",0.7):
			do_not_interrupt = False
			delivery_step =0
			return True
		
	# end the delivery if it take too longe
	if delivery_time > 20 :
		print ("delivery ended it takes too longe ")
		return True
#-----------------------------------------------------------------------------------------------
# Wall_e controller 
def wall_e_controller():
	global behaviour,cube , cube_type ,in_position,previous_state,do_not_interrupt,last_print_time

	sensors = read_sensors()
	now  = time.time()

	#print useful information every one sec
	if  now - last_print_time > 1.0:
		print (" current active behaviour is :", behaviour)
		print (" Battery : ", sensors["battery"])
		print (" current cube type:", cube_type)
		last_print_time = now
	#-----------------------------------------------------
	#charging is the highest poriorty status 
	if sensors["battery"] <  LOW_BATTERY and not behaviour == Behaviours.CHARGED : 
		if behaviour !=Behaviours.NEEDCHARGING:
			if drive_time("B",0.7):
				behaviour = Behaviours.NEEDCHARGING 
	
	#always active check if the robot near a wall if True start avoiding the wall
	if close_to_wall(sensors["top_image"]):
		if behaviour != Behaviours.AVOID_WALL:
			previous_state = behaviour
		behaviour = Behaviours.AVOID_WALL
	

	if behaviour == Behaviours.AVOID_WALL:
			if drive_time("R",1.3):
				behaviour = previous_state

	if behaviour == Behaviours.NEEDCHARGING :
			if not in_position :
				in_position = go_to_charger(sensors["top_image"])

			if  sensors["battery"] >= 0.95 : 
				in_position = False
				behaviour = Behaviours.CHARGED

	if behaviour == Behaviours.CHARGED :
			if drive_time("B",0.5) :
				do_not_interrupt = False
				behaviour = Behaviours.SEARCH

	if behaviour == Behaviours.DELIVERING:
			if cube_delivery(sensors["top_image"],sensors["front_image"],sensors["sonar"],cube_type):
				behaviour = Behaviours.SEARCH

	if behaviour == Behaviours.HANDLE_CUBE:
			if handle_cube(sensors["top_image"],sensors["front_image"],cube_type):
				if cube_type == "trash":
					behaviour = Behaviours.COMPRESS

				if cube_type == "plant":
					behaviour = Behaviours.DELIVERING
			
	if behaviour == Behaviours.COMPRESS:
			if Compress_trash(sensors["front_image"],sensors["sonar"]):
				print ("commpressed")
				behaviour = Behaviours.DELIVERING

	if behaviour == Behaviours.SEARCH :
			search()
			cube , cube_type = find_cube(sensors["top_image"])
			if cube is not None:
				reset_serch()
				stop()
				behaviour = Behaviours.HANDLE_CUBE
			
#-------------------------------------------------------------------------------------------------------------
# MAIN CONTROL LOOP
def main():
	# Starts coppeliasim simulation if not done already
	sim.startSimulation()
	time.sleep(0.5)
	while True:
		state = sim.getSimulationState()	
		if state == sim.simulation_advancing_running:
				wall_e_controller()
				time.sleep(0.05)

if  __name__ == "__main__":
	main()