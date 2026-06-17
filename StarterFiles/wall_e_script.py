
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
	HANDLE_CUBE =4
	COMPRESS =5
	DELIVERING =6
	DONE =7
	AVOID_WALL =8
#-----------------------------------------------------------------------------------------------
# configurations variables
MAX_SPEED = 6
TURN_SPEED = 1
FORWARD_SPEED = 5
BACKWARD_SPEED = 3
SLOW_SPEED = 1


MIN_PIXELS = 25
LOW_BATTERY = 0.25
CLEAR_SONAR = 0.47

#-----------------------------------------------------------------------------------------------
# State variables
last_print_time = 0.0
drive_timer_started = False
timer_time = 0
status = Status.SEARCH  #Status.DONE #Status.DELIVERING 
previous_state = 0
do_not_interrupt = False
in_position = False

cube_type = None
cube = None

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
    #print ("red", red)
    #print ("green", green)
    #print ("blue", blue)

	# Plant cubes
    if colour_name == "green":
        return (green > 70) & (green > red + 20) & (green > blue + 20)
	
    # Trash cubes
    if colour_name == "brown":
       # return (red > 70) & (green > 18) & (green < 120) & (blue < 90) & (red > blue + 15)
      return (red > 70) &(red < 210) &(green > 15) &(green < 95) &(blue < 80) & (red > green + 35) & (red > blue + 35) 
		     
    # Plant container
    if colour_name == "blue":
        return (blue > 75) & (blue > red + 20) & (blue > green + 10)
	
    # Trash container
    if colour_name == "red":
        return (red > 85) & (red > green + 30) & (red > blue + 30) & (green < 120)

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
	   # return ( (red == 137) & 
			# (green == 137) & 
			# (blue == 137) )
			 #(abs(red - green) < 25) & 
			 #(abs(red - blue) < 45) & 
			 #(abs(green - blue) < 45))
       
        
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
	wall = find_colour(top_image,"gray")
	
	# wall is  is 0.6 in the hight of the image 
	if wall is not None:
		#print("wall y ",wall["y"])
		if wall["y"] > 0.6 and not do_not_interrupt:
			return True


def go_to_charger (Top_image):
	charger = find_colour(Top_image,"yellow")

	if charger is None:
		rotate (1)
		return
	if charger["pixels"] >=2000:
		#print ("charger pixels",charger["pixels"])
		if drive_time("F",0.5):
			#print ("arrive")
			stop()
			return True
	drive_to(charger)

# move and rotate searching for cubes 
def search ():
	"""
		global search_step
	if search_step == 0 :
		rotate = drive_time("R",1)
		if rotate :
			search_step =1
			rotate = False
	if search_step == 1 :
		forward = drive_time("F",0.7)
		if forward :
			search_step =0
			forward = False
	"""
	rotate(1)


# moving towarde cube 
def handle_cube(top_image,front_image,cube_type):
	
	if cube_type ==None:
		return
	if cube_type == "trash":
		cube = find_colour(top_image,"brown")
		front_opject = find_colour(front_image,"brown")
	if cube_type == "plant":
		cube = find_colour(top_image,"green")
		front_opject = find_colour(front_image,"brown")

	if front_opject is not None:
		#print ("front_opject Pixels",front_opject["pixels"])
		if front_opject["pixels"] >= 4000 :
			stop()
			return True	
	if cube is not None:
		drive_to(cube)

#compress and handel compressed trash cube 	
def Compress_trash(front_image,sonar) :
	global compress_step
	compressed_cube = find_colour(front_image,"black")

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
	global delivery_step,do_not_interrupt

	cube = find_colour(front_image,"black")
	if cube_type == "trash":
		container = find_colour(top_image,"red")
	if cube_type == "plant":
		container = find_colour(top_image,"blue")

	#finde the container and go there 
	if delivery_step ==0 :
		if container is None:
			rotate (1)
			return
		
		drive_to(container)
		if container["pixels"] >=2500:
			#print ("charger pixels",container["pixels"])
			stop()
			delivery_step =1

	# move slowly forwarde until drop the cube inthe container
	if delivery_step==1:
		do_not_interrupt = True
		Move_forward(True)
		#print ("sonar", sonar)
		if  cube is None:#sonar >= CLEAR_SONAR or
			print ("cube droped")
			stop()
			delivery_step =2

	# move backword away from the container 	
	if delivery_step==2:
		if drive_time("B",0.7):	
			delivery_step =3

	# rotate away from the container 
	if delivery_step==3:
		if drive_time("R",0.7):
			do_not_interrupt = False
			return True
	

			
#-----------------------------------------------------------------------------------------------
# Wall_e controller 
def wall_e():
	global status,cube , cube_type ,in_position,previous_state,do_not_interrupt
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
		#print ('battery level =',sensors["battery"])
		#print ("bumper sensor ",sensors["bumper"])
		#print(" sonar sensor ",sensors["sonar"])
		#print ("sensor RGP :", sensors ["top_image_rgp"])
		#print ( "function RGP :", rgb_parts (sensors ["top_image"]))
		#print (" colouer mask :", colour_mask(sensors ["top_image"],"green"))
		#print ("find the colouer :", find_colour(sensors ["top_image"],"green"))
		#print ( "cube:",cube)
		#print ("cube type :",cube_type)
		last_print_time = now
	#-----------------------------------------------------
	#steps

	#charging is the highest poriorty status 
	if sensors["battery"] <  LOW_BATTERY and not status == Status.CHARGED : 
		if drive_time("B",0.7):
			status = Status.CHARGING 
	
	#Avoid walls interrupt
	if close_to_wall(sensors["top_image"]):
		print ("wall is detected")
		if status != Status.AVOID_WALL:
			previous_state = status
		status = Status.AVOID_WALL
	
	
	match status:

		case Status.AVOID_WALL:
			print ("avoid wall")
			if drive_time("R",3):
				status = previous_state

		case Status.CHARGING :
			if not in_position :
				in_position = go_to_charger(sensors["top_image"])
				print (" CHARGING ")

			if  sensors["battery"] >= 0.95 : 
				in_position = False
				status = Status.CHARGED

		case Status.CHARGED :
			print ("CHARGED")
			if drive_time("B",0.5) :
				status = Status.SEARCH

		case Status.SEARCH :
			print ("SEARCH")
			search()
			cube , cube_type = find_cube(sensors["top_image"])

			if cube is not None:
				stop()

				status = Status.HANDLE_CUBE

		case Status.HANDLE_CUBE:
			print ("HANDLE_CUBE")
			if handle_cube(sensors["top_image"],sensors["front_image"],cube_type):
				if cube_type == "trash":
					status = Status.COMPRESS

				if cube_type == "plant":
					status = Status.DELIVERING
			
		case Status.COMPRESS:
			print ("COMPRESS")
			if Compress_trash(sensors["front_image"],sensors["sonar"]):
				print ("commpressed")
				status = Status.DELIVERING
		
		case Status.DELIVERING:
			print ("DELIVERING")
			if cube_delivery(sensors["top_image"],sensors["front_image"],sensors["sonar"],cube_type):
				status = Status.SEARCH

		case Status.DONE:
			print ("DONE")
			



						


		
		





	




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