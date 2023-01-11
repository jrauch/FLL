from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from spike.operator import equal_to
from math import *

##############################################################
# Initialization                                            #
##############################################################

hub = PrimeHub()
color_sensor1 = ColorSensor('E')
color_sensor2 = ColorSensor('F')
counter = 0
motorleft = Motor('D') # left motor
motorright = Motor('C')# right motor
motor_pair = MotorPair('D', 'C') # used for tank moves


##############################################################
# Alignment functions                                        #
##############################################################
def align_to_line(color, sensor1, sensor2):
    motor_pair.set_default_speed(15)

    while sensor1.get_color() == color or sensor2.get_color() == color:
        if sensor1.get_color() != color and sensor2.get_color() != color:
            motor_pair.stop()
        elif sensor1.get_color() != color:
            motor_pair.stop()
            turn_right(1)
        elif sensor2.get_color() != color:
            motor_pair.stop()
            turn_left(1)
        else:
            motor_pair.start(0)


    print("done")
    motor_pair.stop()
    return


##############################################################
# Complex move functions                                    #
##############################################################

# PID tuneable line following function that takes function to evaluate "done" as variable fun
# by default it wants to use sensor 1 and track lines where white falls to the left of black.
# using the invert boolean allows you to track the other side of lines, where white is to the right
# of black
def follow_line_pid_until(fun, value=False, line_sensor=color_sensor1, color_sensor=color_sensor1, invert=False):
    Kp = 0.3
    Ki = 0.001
    Kd = 1.0

    I = 0
    previous_error = 0
    base_power = 40
    #motor_pair.set_default_speed(10)
    mixed_average = int((black_reflected + white_reflected)/2)

    while (fun(value, color_sensor)):
        light_sensor_value = line_sensor.get_reflected_light()

        error = light_sensor_value - mixed_average #50
        P = error
        I = I + error
        D = error - previous_error
        previous_error = error

        correction = int((P * Kp) + (I * Ki) + (D * Kd))
        if invert:
            left_motor = base_power - correction
            right_motor = base_power + correction
        else:
            left_motor = base_power + correction
            right_motor = base_power - correction
        #print(correction)
        motor_pair.start_tank_at_power(left_motor, right_motor)
    motor_pair.stop()

# PID tuneable go in a straight line function that takes function to evaluate "done" as variable fun
# color_sensor gets passed to the evaluation function - probably should modify this function and
# the one above to just use kwargs and have that passed instead
def move_forward_pid_until(fun, value=False, color_sensor=color_sensor1):
    Kp = 0.3
    Ki = 0.001
    Kd = 1.0
    motorleft.set_degrees_counted(0)
    I = 0
    previous_error = 0
    base_power = 40
    #motor_pair.set_default_speed(10)
    #mixed_average = int((black_reflected + white_reflected)/2)
    hub.motion_sensor.reset_yaw_angle()
    while (fun(value, color_sensor)):
        yaw = hub.motion_sensor.get_yaw_angle()
        #print(yaw)
        error = yaw #50
        P = error
        I = I + error
        D = error - previous_error
        previous_error = error

        correction = int((P * Kp) + (I * Ki) + (D * Kd))
        left_motor = base_power - correction
        right_motor = base_power + correction
        motor_pair.start_tank_at_power(left_motor, right_motor)
    motor_pair.stop()


##############################################################
# Turning functions                                        #
##############################################################

def turn_left(angle):
    turn(-angle)

def turn_right(angle):
    turn(angle)

def turn(angle):
    motor_pair.set_stop_action('brake')
    hub.motion_sensor.reset_yaw_angle()
    motor_pair.start(100 if (angle>0) else -100)
    wait_until(hub.motion_sensor.get_yaw_angle, equal_to, angle)
    motor_pair.stop()

def turn_to_square(): # turn the robot to 0 - good for adjusting from drift
    motor_pair.set_stop_action('brake')
    angle = hub.motion_sensor.get_yaw_angle()
    motor_pair.start(100 if (angle<0) else -100)
    wait_until(hub.motion_sensor.get_yaw_angle, equal_to, 0)
    motor_pair.stop()


##############################################################
# Simple motion functions                                    #
##############################################################

def move_forward(dist, speed=15):
    motor_pair.set_default_speed(speed)
    motor_pair.move(dist, "in")

def move_backwards(dist, speed=15):
    motor_pair.set_default_speed(speed)
    motor_pair.move(-dist, "in")

hub = PrimeHub()
color_sensor1 = ColorSensor('E') # right sensor
color_sensor2 = ColorSensor('F') # left sensor


##############################################################
# Utility functions                                        #
##############################################################
def degrees_to_distance(degrees):
    one_degree = 6.9/360
    return one_degree * degrees



##############################################################
# Condition functions - used with "x_until" functions        #
##############################################################

# this function will transition between a bunch of colors on the sensor before returning "False""

index = 0

# tracks transitions, as passed in as an array of color names to "val"
# for simplicity reasons you MUST reset the global "index" variable to 0
# before using this function
def go_on_transitions(val, color_sensor=color_sensor1):
    global index
    color = color_sensor.get_color()
    if color == val[index]:
        print(color)
        index += 1
        if index == len(val):
            return False
    return True

def go_distance(val, val2):
    if degrees_to_distance(abs(motorleft.get_degrees_counted())) >= val:
        return False
    return True

##############################################################
# Challenge specific code goes below!                        #
##############################################################



# set some defaults
motor_pair.set_default_speed(15) # going too fast results in more errors.
black_reflected = 20 # calibrated in jeremy's office 1/4
white_reflected = 99 # calibrated in jeremy's office 1/4

# for this challenge, start in the left start area, line up to the red line by looking for both sensors to
# NOT be white
# start with the left sensor _roughly_ aligned to the left side of the line we're going to follow
#align_to_line("white", color_sensor1, color_sensor2)
move_forward(2) # move forward a little since there's a lot going on at that line that can confuse the sensors

# at this point, start following the line until you reach the end of the straight part on the left.
# we'll see the color sensor transition from seeing green, to white and then black when it hits the
# horizontal black line.Stop then.
index=0
follow_line_pid_until(go_on_transitions, ["green","white", "black"], line_sensor = color_sensor2) # follow the line until you see a transition from green to white to black to white to black again on the sensor

move_forward(2) # scoot the robot forward a little so we get the correct (left) sensor for line following

# turn right to get ready to follow the line.
turn_right(90)

# reset the yaw angle so we can correct at the end of this line
hub.motion_sensor.reset_yaw_angle()

# follow the line until we transition black to white and back to black - the end of the horizontal line across
# the top.
index=0
follow_line_pid_until(go_on_transitions, ["black", "white", "black"], line_sensor = color_sensor2) # follow the line until you see a transition from black to white to black again on the sensor


# it likes to drift at the end of the line before the line following conditional trips.Re-square to zero
turn_to_square()


# move forward using the gyro sensor pid function until we go from yellow to black.
index=0
move_forward_pid_until(go_on_transitions, ["yellow", "black"], color_sensor2)

turn_left(45) # lined up with wind vane pusher

# follow the short line until the "end" - might be better to use distance instead?
index=0
motorleft.set_degrees_counted(0) # lets set up to use the line follow with a distance measure function

# follow the line for a fixed distance (4 inches)
follow_line_pid_until(go_distance, 4, color_sensor1, color_sensor2, True)


move_forward(3.7)
move_backwards(2.5)

move_forward(3.7)
move_backwards(2.5)

move_forward(3.7)
move_backwards(2.5)

turn_right(135)
motorleft.set_degrees_counted(0)
move_forward_pid_until(go_distance, 14, color_sensor2)
turn_left(90)
move_forward_pid_until(go_distance, 7.75, color_sensor2)
turn_left(90)
move_forward_pid_until(go_distance, 5.5, color_sensor2)