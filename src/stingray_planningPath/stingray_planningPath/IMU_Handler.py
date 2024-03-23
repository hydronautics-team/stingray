
import time

lastTime = 0

speed = [0, 0, 0]
lastAcceleration = [0, 0, 0]
lastAngularSpeed = [0, 0, 0]

def calcPositionFromIMU(position, acceleration, angularSpeed, currentTime=None) :
    global speed, lastAcceleration, lastAngularSpeed
    a = acceleration
    g = angularSpeed

    c = coordinates = position[0]
    r = rotation = position[1]

    if currentTime is None: currentTime = time.time()
    ct = currentTime

    coordinates[0] += speed[0] * (currentTime - lastTime)
    coordinates[1] += speed[1] * (currentTime - lastTime)
    coordinates[2] += speed[2] * (currentTime - lastTime)

    speed[0] += lastAcceleration[0] * (currentTime - lastTime)
    speed[1] += lastAcceleration[1] * (currentTime - lastTime)
    speed[2] += lastAcceleration[2] * (currentTime - lastTime)

    rotation[0] += lastAngularSpeed[0] * (currentTime - lastTime)
    rotation[1] += lastAngularSpeed[1] * (currentTime - lastTime)
    rotation[2] += lastAngularSpeed[2] * (currentTime - lastTime)

    lastAcceleration = acceleration[:]
    lastAngularSpeed = angularSpeed[:]

    return position