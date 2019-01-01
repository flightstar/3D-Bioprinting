# Description
This is an API for controlling a SCARA-type (3 degrees of freedom : RTR) robot arm using a raspberry pi and multiple servos.

# Dependencies
- Adafruit PCA9685 : The shield provides multiple PWM channels and a python library to control them (used for servo control). https://github.com/adafruit/Adafruit_Python_PCA9685

- Scipy: This python library is mostly used for line interpolation and must be installed on the RPi in order to draw lines with the robot. 
https://www.scipy.org/install.html

# How to use
1. Clone the repository.
2. Install all the dependencies.
3. Modify robot.py accordingly to the robot configuraion :
  - Channels used for every servo
  - Min/Max pulse for every servo (see Adafruit_PCA9685 reference)
  - Line increment

4. Write the main application. AppRobot.py gives a good idea on how to use the robot module. Depending on servos' power, delays must be introduced between move commands to get smooth trajectories. 

##Quickstart example:
```
from robot import *

first_joint = 15  # centimeters (all length/coordinates values must be in the same unit)
second_joint = 10  
debug = True  # debug ON

my_robot = Robot(first_joint, second_joint, debug)

# Go to point P using cartesian coordinates (x, y, z)  
my_robot.go_to_coordinates(8.0, 9.0, 4.0)

# Same thing using the Point class
p = Point(8.0, 9.0, 4.0)
my_robot.go_to_point(p)

# Draw line between two points
a = Point (8.0, 9.0, 4.0) 
b = Point (3.0, 9.0, 4.0)
my_robot.draw_line(a, b)

# Follow trajectory/given path
a = Point (8.0, 9.0, 4) 
b = Point (3.0, 9.0, 4.0)
c = Point (0.0, 5.0, 4.0)
d = Point (-2.5, 3.8, 4.0)
points_list = [a, b, c, d]
my_path = Path()
for point in points_list:
  my_path.add_point(point)
my_robot.follow_path(my_path)

```


 
