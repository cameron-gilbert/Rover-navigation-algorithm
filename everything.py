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
    # print("Sending Command left: ", left_cmd, " right: ", right_cmd, " linear_error: ", delta_dist, " angular_error ", delta_heading)
    rover.send_command(left_cmd, right_cmd)
    return False

x_goal = 0
y_goal = 30
k = 10

def fields(rover, k, y_goal, x_goal):
    
    fieldTotal = 0
    fieldAngle = 0

    # LiDAR goes here
    lidar_array = []

    for dist in rover.laser_distances:
    # add the current value to the array
        if dist < 15:
            lidar_array.append(dist)
        else:
            lidar_array.append(0)
   

# print the resulting array if needed to test lol
#print(len(lidar_array)
#obstacles = len(lidar_array)
 
    obstaclesArrayX = []
    angle = -90

    for x in lidar_array:
        if x != 0:
            new_value = x * math.sin(angle*(math.pi)/180)
            obstaclesArrayX.append(new_value)
        angle += 6

    angle = -90
    obstaclesArrayY = []
    
    for y in lidar_array:
        if y != 0:
            new_value = y * math.cos(angle*(math.pi)/180)
            obstaclesArrayY.append(new_value)
        angle += 6
    

    obstacles = len(obstaclesArrayY)
    #coordinates of the rover
    x = rover.x
    y = rover.y
    

    # calculating distance to target
    d_goal = math.sqrt(math.pow(x_goal - x, 2)*math.pow(y_goal - y, 2))
    theta_goal = math.atan((y_goal - y)/(x_goal - x))

    # test
    print(d_goal, theta_goal)
    #test array coordinates
    for elements in obstaclesArrayX:
        print(elements)

    lidar_array = []

    obstaclesArrayDistance = [0] * obstacles
    obstaclesArrayAngle = [0] * obstacles
    ob = 0
    while ob < obstacles:
        obstaclesArrayDistance[ob] = math.sqrt(
            math.pow(obstaclesArrayX[ob] - x, 2)*math.pow(obstaclesArrayY[ob] - y, 2))
        obstaclesArrayAngle[ob] = math.atan(
            (obstaclesArrayY[ob] - y)/(obstaclesArrayX[ob] - x))
        print("loop 1 working?")
        ob += 1

    ob = 0
    fieldXTotal = 0
    fieldYTotal = 0
    while ob < obstacles:
        fieldX = (k/math.pow(obstaclesArrayDistance[ob], 2))*(
            math.cos(obstaclesArrayAngle[ob]))
        fieldXTotal = fieldX+fieldXTotal
        fieldY = (k/math.pow(obstaclesArrayDistance[ob], 2))*(
            math.sin(obstaclesArrayAngle[ob]))
        fieldYTotal = fieldX+fieldYTotal
        print("loop 2 working?")
        ob += 1

    #fieldXGoal = ((100000*1)/math.pow(d_goal, 2))*(math.cos(theta_goal))
    #fieldYGoal = ((100000*1)/math.pow(d_goal, 2))*(math.sin(theta_goal))

    fieldXGoal = 10*x_goal
    fieldYGoal = 10*y_goal

    fieldXTotal = fieldXGoal - fieldXTotal
    fieldYTotal = fieldYGoal + fieldYTotal

    fieldTotal = math.sqrt(math.pow(fieldXTotal - x, 2)
                       * math.pow(fieldYTotal - y, 2))
    fieldAngle = math.atan(fieldYTotal/fieldXTotal)

    # test
    print("X field: " + str(fieldXTotal) + ", Y field: " + str(fieldYTotal))
    print("resultant field: " + str(fieldTotal) + ", angle: " + str(fieldAngle))

    return(fieldXTotal, fieldYTotal)


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

def main():
    rover = Rover()
    time.sleep(1)

    signal.signal(signal.SIGINT, signal.default_int_handler)

    roverError = math.sqrt((rover.x - x_goal) ** 2. + (rover.y - y_goal)**2.)
    try:

        while(roverError > 0.5):
            coords = fields(rover, k, y_goal, x_goal)
            go_x = coords[0]
            go_y = coords[1]
            print("go_x: " + str(go_x) + " Currently at: " + str(rover.x))
            print("go_y: " + str(go_y) + " Currently at: " + str(rover.y))
            error = 0.1
            sqr_error = 0.1 ** 2


            traverse(rover, go_x,go_y, sqr_error)
            time.sleep(0.01)

            roverError = math.sqrt((rover.x - x_goal) ** 2. + (rover.y - y_goal)**2.)
    except KeyboardInterrupt:
        pass
    rover.send_command(0,0)
    print("Target Reached")
main()