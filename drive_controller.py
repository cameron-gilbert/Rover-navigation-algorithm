# the below function is the code provided. It will do one step of the navigation. Below I have an example of how to use the function. 

import math

# You can change these values if you want the rover behaviour to change
# for example if your points are always very close together but you want 
# the rover to go faster between them you could increase the forward_gain 
# or increase the min_fwd_vel
turn_gain = 1.0
forward_gain = 0.05
angular_linear_weight = 1.0
min_fwd_vel = 0.4
max_fwd_vel = 2.

def traverse(rover, target_x, target_y, sqr_error):
    curr_x = rover.x
    curr_y = rover.y
    curr_heading = math.radians(rover.heading)

    delta_x = target_x - curr_x
    delta_y = target_y - curr_y
    target_heading = math.atan2(delta_y, delta_x)

    delta_heading = (target_heading - curr_heading + 3.14159) % (2.0*3.14159) - 3.14159
    if(delta_heading < -3.14159):
        delta_heading += (3.14159 * 2.)
    delta_dist = delta_x **2. + delta_y ** 2.

    if(delta_dist < sqr_error): 
        rover.send_command(0,0)
        return True
    
    turn_cmd = delta_heading * turn_gain
    if(abs(delta_heading) < 3.1415/2.):
        fwd_cmd =  delta_dist * forward_gain - angular_linear_weight * delta_heading
        if(fwd_cmd > max_fwd_vel):
            fwd_cmd = max_fwd_vel
    else:
        fwd_cmd = 0

    right_cmd = fwd_cmd + turn_cmd
    left_cmd = fwd_cmd - turn_cmd

    if(abs(turn_cmd) < 0.1):
        right_cmd += min_fwd_vel
        left_cmd += min_fwd_vel
    print("Sending Command left: ", left_cmd, " right: ", right_cmd, " linear_error: ", delta_dist, " angular_error ", delta_heading)
    rover.send_command(left_cmd, right_cmd)
    return False


# below is the example code where I send the rover to a given x,y. 
# you should be updating x and y to be the next point in the path that 
# your planning algo gives you. sqr_error is the sqare of the position error
# you should avoid giving this the direct next point in the path and you should
# instead give it a point 2 or 3 meters away from the rover along the path.
# If you do that this is called a "Pure Pursuit Controller" you can research the
# pros and cons of the controller if you want

from qset_lib import Rover
import time
import signal


rover = Rover()
x = 100
y = 100
error = 0.1
sqr_error = 0.1 ** 2

time.sleep(1)

signal.signal(signal.SIGINT, signal.default_int_handler)
try:
    while True:
        if traverse(rover, x,y, sqr_error):
            break
        time.sleep(0.01)

except KeyboardInterrupt:
    pass

rover.send_command(0, 0)
