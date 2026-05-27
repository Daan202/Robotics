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

top_image_sensor = ImageSensor(sim, DeviceNames.TOP_IMAGE_SENSOR_OS)
small_image_sensor = ImageSensor(sim, DeviceNames.SMALL_IMAGE_SENSOR_OS)

left_motor = Motor(sim, DeviceNames.MOTOR_LEFT_OS, Direction.CLOCKWISE)
right_motor = Motor(sim, DeviceNames.MOTOR_RIGHT_OS, Direction.CLOCKWISE)

# HELPER FUNCTION
def show_image(image):
    plt.imshow(image)
    plt.show()




def wall_e():

	# Starts coppeliasim simulation if not done already
	sim.startSimulation()
	time.sleep(0.5)

	battery_level = robot.get_battery()
	bumper_sensor = robot.get_bumper_sensor()
	sonar_sensor = robot.get_sonar_sensor()

	top_image_sensor._update_image() # Updates the internal image
	timg_reflection = top_image_sensor.reflection() # Gets the reflection from the image
	timg_red, timg_green, timg_blue = top_image_sensor.rgb()
	timg_image = top_image_sensor.get_image()
	timg_ambient = top_image_sensor.ambient()

	small_image_sensor._update_image() # Updates the internal image
	simg_reflection = small_image_sensor.reflection() # Gets the reflection from the image
	simg_red, simg_green, simg_blue = small_image_sensor.rgb()
	simg_image = small_image_sensor.get_image()
	simg_ambient = small_image_sensor.ambient()

	



	Manual = True
	if Manual:
			#print ("manual is active")

		if keyboard.is_pressed('w') :   # move forward
				speed_f =5
				left_motor.run(speed=speed_f) # Runs the left motor at speed=5
				right_motor.run(speed=speed_f) # Runs the right motor at speed=5
			
		elif keyboard.is_pressed('s'):   # move Backword
				left_motor.run(speed=-5) # Runs the left motor at speed=5
				right_motor.run(speed=-5) # Runs the right motor at speed=5

		elif keyboard.is_pressed('d'):   # move to the right
				left_motor.run(speed= 5) # Runs the left motor at speed=5
				right_motor.run(speed= 2) # Runs the right motor at speed=5 

		elif keyboard.is_pressed('a'):   # move to the left
				left_motor.run(speed= 2) # Runs the left motor at speed=5
				right_motor.run(speed= 5) # Runs the right motor at speed=5  
		
		elif keyboard.is_pressed('c'):   
				robot.compress()
				print('compress is active ')
				
		elif keyboard.is_pressed('b'):   
				print('battery level',battery_level)

		else:
			left_motor.run(speed=0) # Runs the left motor at speed=5
			right_motor.run(speed=0) # Runs the right motor at speed=5

	print ('battery level =',battery_level)
	print ("bumper sensor ",bumper_sensor)
	print(" sonar sensor ",sonar_sensor)

	print (" top image sensor reflection :",timg_reflection)
	print (" top image sensor red : ",timg_red)
	print (" top image sensor green : ",timg_green)
	print (" top image sensor blue : ",timg_blue)
	print (" top image sensor ambient :",timg_ambient)

	print (" small image sensor reflection :",simg_reflection)
	print (" small image sensor red : ",simg_red)
	print (" small image sensor green : ",simg_green)
	print (" small image sensor blue : ",simg_blue)
	print (" small image sensor ambient :",simg_ambient)

# MAIN CONTROL LOOP
while True:

    wall_e()
    