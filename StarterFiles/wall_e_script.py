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

def avoid_obstacles():
    # lowest layer: Avoid collasions and obstacles

	bumper = robot.get_bumper_sensor()
	if bumper[0] > 0 or bumper[1] > 0 or bumper[2] > 0: 
		left_motor.run(-3)
		right_motor.run(-3)
		return True
	return False 


def is_yellow(r,g,b):
	if r > 60 and b < 40 and g > 60:
		return True
	return False

def charge_battery():
	# battery layer
	# If low battery -> search until yellow area found -> recharge
	top_image_sensor._update_image()
	r, g, b = top_image_sensor.rgb()
	if is_yellow(r,g,b):
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

def move_boxes():
    # Highest layer: Move the boxes to the designed place 
	if Tharsh:
		move_to_front()
		Compress
		move to red location
	
	elif Plant:
		pick_up()
		move_to_blue()

	return True 


 
# MAIN CONTROL LOOP
while True:
	print(robot.get_battery())
	if collision():
		continue
	if low_battery():
		charge_battery()
		continue
	if handle_object():
		continue

	explore()
    
	"""
       Robot needs charging -> Must go to charging area
       Thrash -> Approach from the front -> Compress them -> sort in red container
       Plant -> Sort in Blue container
    """