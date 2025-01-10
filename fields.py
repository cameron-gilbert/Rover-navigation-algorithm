import math

# variables (give these actual values later)
x_goal = 2
x = 1
y_goal = 2
y = 1
obstacles = 4
fieldTotal = 0
k = 2
fieldAngle = 0

# calculating distance to target
d_goal = math.sqrt(math.pow(x_goal - x, 2)*math.pow(y_goal - y, 2))
theta_goal = math.atan((y_goal - y)/(x_goal - x))

# test
print(d_goal, theta_goal)

obstaclesArrayX = [2, 2, 3, 4]
obstaclesArrayY = [5, 6, 7, 8]

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

fieldXGoal = ((k*1)/math.pow(d_goal, 2))*(math.cos(theta_goal))
fieldYGoal = ((k*1)/math.pow(d_goal, 2))*(math.sin(theta_goal))

fieldXTotal = fieldXGoal - fieldXTotal
fieldYTotal = fieldYGoal + fieldYTotal

fieldTotal = math.sqrt(math.pow(fieldXTotal - x, 2)
                       * math.pow(fieldYTotal - y, 2))
fieldAngle = math.atan(fieldYTotal/fieldXTotal)

# test
print("X field: " + str(fieldXTotal) + ", Y field: " + str(fieldYTotal))
print("resultant field: " + str(fieldTotal) + ", angle: " + str(fieldAngle))
