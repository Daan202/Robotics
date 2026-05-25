from robots import *
import time
from coppeliasim_zmqremoteapi_client import *
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

client = RemoteAPIClient()
sim = client.require("sim")

# HANDLES FOR ACTUATORS AND SENSORS
robot = Robot_OS(sim, DeviceNames.ROBOT_OS)

top_image_sensor = ImageSensor(sim, DeviceNames.TOP_IMAGE_SENSOR_OS)
small_image_sensor = ImageSensor(sim, DeviceNames.SMALL_IMAGE_SENSOR_OS)

left_motor = Motor(sim, DeviceNames.MOTOR_LEFT_OS, Direction.CLOCKWISE)
right_motor = Motor(sim, DeviceNames.MOTOR_RIGHT_OS, Direction.CLOCKWISE)
color_sensor = ImageSensor(sim, DeviceNames.IMAGE_SENSOR_LINE)

# HELPER FUNCTION
def show_image(image):
    plt.imshow(image)
    plt.show()

# Starts coppeliasim simulation if not done already
sim.startSimulation()
time.sleep(0.5)
def low_battery():
	return robot.get_battery() < 20

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

def is_green_detected(color_sensor):
    """
    Calculates the relative intensity of the green channel compared to
    other channels
    """

    green_ratio_threshold = 1.5
    red, green, blue = color_sensor.rgb()
    green_intensity = green / (red + blue)

    return green_intensity > green_ratio_threshold

def is_yellow_detected(r,g,b):
	if r > 100 and g > 100 and  b < 224:
		return True
	return False

def is_brown(r,g,b):
    return 80 < r < 200 and g < 150 and b < 120

def avoid_obstacles():
    # lowest layer: Avoid collasions and obstacles
	# use distance sensor not bumper the real robot has only distance sensor for now 

	bumper = robot.get_bumper_sensor()
	if any(bumper): 
		left_motor.run(-3)
		right_motor.run(-3)
		return True
	return False 

# brown red green 

def charge_battery():
	# battery layer
	# If low battery -> search until yellow area found -> recharge
	top_image_sensor._update_image()
	r, g, b = top_image_sensor.rgb()
	if is_yellow_detected(r,g,b):
		left_motor.run(0)
		right_motor.run(0)
		robot.set_integer_signal("charge", 1)
		return True 
	else: 
		left_motor.run(2)
		right_motor.run(2)
		return False 
		
def explore():
    # If you see no boxes move to find them 
	left_motor.run(2)
	right_motor.run(2)
	# random.shuffle(moves) 

def object_detection():
	global target
	color_sensor._update_image()
	r, g, b = color_sensor.rgb()

	if target is None:
		if is_red_detected(color_sensor):
			target = "RED_CUBE"

		elif is_brown(r, g, b):
			# Robot found a rash
			target = "BROWN_CUBE"
		
		elif is_green_detected(color_sensor):
			# Robot found the plant
			target = "GREEN_CUBE"
	else:
		pass
	
def move_boxes():
	""" 
    Highest layer: Move the boxes to the designed place 
	Must decide = Some of these are only for sim not for real life 
	Green -> Plant, Blue -> plant place, Red -> Tharsh place, Brown -> Tharsh 
	"""

	global target

	color_sensor._update_image() # Updates the internal image
	top_image_sensor._update_image()

	distance = robot.get_sonar_sensor()

	# If it touches the it can push it to the specific zone
	bumper = robot.get_bumper_sensor()

	# If in spike(real life) we can just say if Red do that if Brown do that.

	#----------Trash Brown cube-----------#

	if any(bumper):
		if target == "BROWN_CUBE":
			# Compress and leave alone
			left_motor.run(0)
			right_motor.run(0)
			robot.compress()
			target = None
			# This behavior took the control of the robot
			return True 
		
		#----------Red cube-----------#

		if target == "RED_CUBE":

			if is_red_detected(top_image_sensor):

				if distance < 0.5:
					left_motor.run(0)
					right_motor.run(0)
					target = None
					# This behavior took the control of the robot
					return True
				
				# Robot moves to the red zone
				left_motor.run(2)
				right_motor.run(2)
				return True 
			
			# Red zone is not in the view yet turn so you find it
			left_motor.run(1)
			right_motor.run(2)
			return True 

		#----------Plant Green cube-----------#

		elif target == "GREEN_CUBE":

			if is_blue_detected(top_image_sensor):

				if distance < 0.5:
					left_motor.run(0)
					right_motor.run(0)
					target = None
					# This behavior took the control of the robot
					return True
			
				# Robot moves to the Blue zone
				left_motor.run(2)
				right_motor.run(2)
				return True 
		
		
			# Blue zone is not in the view yet
			left_motor.run(1)
			right_motor.run(2)
			return True 
	
	return False

# MAIN CONTROL LOOP
while True:
	print(robot.get_battery())

	if avoid_obstacles():
		continue

	if low_battery():
		charge_battery()
		continue

	object_detection()

	if move_boxes():
		continue

	explore()
    
	"""
       Robot needs charging -> Must go to charging area
       Thrash -> Approach from the front -> Compress them -> sort in red container
       Plant -> Sort in Blue container
    """