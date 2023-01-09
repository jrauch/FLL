from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from spike.operator import equal_to
from math import *

hub = PrimeHub()
color_sensor1 = ColorSensor('E')
color_sensor2 = ColorSensor('F')
counter = 0

def align_to_line(color, sensor1, sensor2):
    motor_pair.set_default_speed(10)

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

# line following "until" functions - use the PID version
def follow_line_pid_until(fun, value=False, line_sensor=color_sensor1):
    Kp = 0.3
    Ki = 0.001
    Kd = 1.0

    I = 0
    previous_error = 0
    base_power = 40
    #motor_pair.set_default_speed(10)
    mixed_average = int((black_reflected + white_reflected)/2)

    while (fun(value)):
        light_sensor_value = line_sensor.get_reflected_light()

        error = light_sensor_value - mixed_average #50
        P = error
        I = I + error
        D = error - previous_error
        previous_error = error

        correction = int((P * Kp) + (I * Ki) + (D * Kd))
        left_motor = base_power + correction
        right_motor = base_power - correction
        #print(correction)
        motor_pair.start_tank_at_power(left_motor, right_motor)
    motor_pair.stop()

def follow_line_until(fun, value):
    mixed_average = int((black_reflected + white_reflected)/2)
    motor_pair.set_default_speed(10)
    #print(hub.motion_sensor.get_yaw_angle())
    while (fun(value)):
        motor_pair.start((color_sensor1.get_reflected_light() - mixed_average)*3)


# Robot Turn Functions using the gyro
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

def move_forward(dist):
    motor_pair.move(dist, "in")

hub = PrimeHub()
color_sensor1 = ColorSensor('E') # right sensor
color_sensor2 = ColorSensor('F') # left sensor

#hub.left_button.wait_until_pressed()


# this function will count up to "val" times it sees "black"
count = 0
def go_until_black(val):
    global count
    color = color_sensor1.get_color()

    if color == 'black':
        count += 1
        print("trig")
        if count >= val:
            return False
    return True

# this function will transition between a bunch of colors on the sensor before stopping motion
trigger_color = ["black", "white", "black"]
def go_on_transitions(val):
    global trigger_color, index
    color = color_sensor1.get_color()
    if color == val[index]:
        print("trigger ")
        index += 1
        if index == len(val):
            return False
    return True


motor_pair = MotorPair('D', 'C')
motor_pair.set_default_speed(10)
black_reflected = 20 # calibrated in jeremy's office 1/4
white_reflected = 99 # calibrated in jeremy's office 1/4

align_to_line("white", color_sensor1, color_sensor2)
move_forward(2) # move forward a little since there's a lot going on at that line that can confuse the sensors

#follow_line_pid_until(go_until_black, 2, line_sensor = color_sensor2) # follow the line until you see black on the sensor 2x
index=0
follow_line_pid_until(go_on_transitions, ["green","white", "black"], line_sensor = color_sensor2) # follow the line until you see a transition from green to white to black to white to black again on the sensor

move_forward(2) # scoot the robot forward a little so we get the correct (left) sensor for line following
turn_right(90)

index=0
follow_line_pid_until(go_on_transitions, ["black", "white", "black"], line_sensor = color_sensor2) # follow the line until you see a transition from black to white to black again on the sensor
turn_right(90)
