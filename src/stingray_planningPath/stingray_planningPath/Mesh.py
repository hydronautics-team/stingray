
from stingray_planningPath import AStar
import random
import math
from PIL import Image, ImageDraw

# граница перехода к следующей точке
DISTANCE_EPSILON = 0.5

# коды состояний, системные, не трогать
STATE_ERROR    = 0
STATE_OK       = 1
STATE_COMPLETE = 2
STATE_FIND     = 3
STATE_NO_PATH  = 4

# радиус красной дуги
ANGLE_DRAW_RADIUS = 20

# максимальная скорость
MAX_LINEAR_SPEED_L = 15
MAX_LINEAR_SPEED_A = 9
MAX_ANGULAR_SPEED_L = 80
MAX_ANGULAR_SPEED_A = 80
MAX_VERTICAL_SPEED = 9
# максимальный угол отклонение движения
MAX_MOVE_ANGLE_L = 15
MAX_MOVE_ANGLE_A = 35
# стандартный коэффициент замедления
LINEAR_SPEED_SLOW_COEF_L = 0.3
LINEAR_SPEED_SLOW_COEF_A = 0.3
ANGULAR_SPEED_SLOW_COEF_L = 0.3
ANGULAR_SPEED_SLOW_COEF_A = 0.3

def cos(x) :
    return math.cos(x/360*2*math.pi)

def sin(x) :
    return math.sin(x/360*2*math.pi)

LEFT = 0
RIGHT = 1

def calcAngleBeetwen(angle1, angle2) :
    x1 = cos(angle1)
    y1 = sin(angle1)
    x2 = cos(angle2)
    y2 = sin(angle2)

    #print('x1, y1, x2, y2 =', x1, y1, x2, y2)

    vm = x1*y2 - x2*y1
    angle = min(abs(angle1-angle2), 360-abs(angle1-angle2))

    if (vm > 0) : return angle, LEFT
    return angle, RIGHT

def distance(x1, y1, x2, y2) :
    return ((x1-x2)**2 + (y1-y2)**2)**0.5

def getPolarAngle(x1, y1, x2, y2):
    x = x2 - x1
    y = y2 - y1

    angle = math.acos(x / (x ** 2 + y ** 2) ** 0.5) / 2 / math.pi * 360

    if y < 0: angle = 360 - angle

    return angle

def relativeAngle(selfPolarAngle, targetPolarAngle) :
    angle, turn = calcAngleBeetwen(selfPolarAngle, targetPolarAngle)

    if turn == LEFT : return -angle
    return angle


def calcMove(circles, startPosition, selfAngleAll, endPoint, needToDraw, targetDeep) :
    selfAngle = selfAngleAll[2]
    startPoint = startPosition
    astar = AStar.AStar()
    path, image = astar.main(startPoint, endPoint, circles, needToDraw, False)
    draw = None
    if needToDraw : draw = ImageDraw.Draw(image)

    selfAngle %= 360

    print(path)

    #print('pathLenth =', len(path))
    if len(path) == 0 :
        #print("Error: no path")
        return STATE_NO_PATH, defaultVector(selfAngleAll, targetDeep), image

    if isinstance(path[0], AStar.Line):
        if distance(path[0].x1, path[0].y1, path[0].x2, path[0].y2) < DISTANCE_EPSILON :
            path.pop(0)

    elif isinstance(path[0], AStar.Arcc):
        x1 = cos(path[0].p1)*path[0].r
        y1 = sin(path[0].p1)*path[0].r
        x2 = cos(path[0].p2)*path[0].r
        y2 = sin(path[0].p2)*path[0].r
        if distance(x1, y1, x2, y2) < DISTANCE_EPSILON :
            path.pop(0)

    needToStop = False

    print(path, endPoint[2])

    if len(path) == 0 :
        #print("\nCOMPLETE\n")
        return STATE_COMPLETE, defaultVector(selfAngleAll, targetDeep), image

    AStar.AStar.drawCircle(draw, endPoint[0], endPoint[1], endPoint[2], None,'green', 2)

    if distance(startPosition[0], startPosition[1], endPoint[0], endPoint[1]) < endPoint[2]:
        # print("\nCOMPLETE\n")
        return STATE_COMPLETE, defaultVector(selfAngleAll, targetDeep), image
    if len(path) == 1 :
        needToStop = True

    if isinstance(path[0], AStar.Line):
        #print('Line:', selfAngle, getPolarAngle(path[0].x1, path[0].y1, path[0].x2, path[0].y2))
        angle, turn = calcAngleBeetwen(selfAngle, getPolarAngle(path[0].x1, path[0].y1, path[0].x2, path[0].y2))
        #print('angle, turn =', angle, turn)
        length = distance(path[0].x1, path[0].y1, path[0].x2, path[0].y2)

        return STATE_OK, moveToLine(startPosition, length, angle, turn,
                                    getPolarAngle(path[0].x1, path[0].y1, path[0].x2, path[0].y2),
                                    needToStop, targetDeep, selfAngleAll), image

    if isinstance(path[0], AStar.Arcc):
        dr = path[0].dr
        if path[0].isInvert : tangent = (path[0].p1 - 90) % 360
        else : tangent = (path[0].p1 + 90) % 360

        #print('polar, tangent =', [path[0].p1, path[0].p2], tangent)

        #AStar.AStar.drawLine(draw, startPoint[0], startPoint[1],
        #                     startPoint[0] + cos(tangent) * 100, startPoint[1] + sin(tangent) * 100, 'black', 5)

        angle, turn = calcAngleBeetwen(selfAngle, tangent)
        polarLenght = min(abs(path[0].p1-path[0].p2), abs(360-path[0].p1-path[0].p2))

        return STATE_OK, moveAlongArcc(startPosition, dr, path[0].p1, path[0].r,
                                       angle, turn, polarLenght, tangent, selfAngle, needToStop, targetDeep, selfAngleAll), image

    #print("Error: unknow error")
    return STATE_ERROR, defaultVector(selfAngleAll, targetDeep), image

def defaultVector(selfAngleAll, targetDeep) :
    return [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], selfAngleAll, [0, 0, targetDeep]]

def moveAlongArcc(startPosition, dr, polarAngle, r, angle, turn, polarLenght, tanget, selfAngle, needToStop, targetDeep, selfAngleAll) :
    # движемся по дуге окружности
    #print('ARCC')

    # длинна дуги окружности, по которой нужно пройти
    lenth = 2 * math.pi * r * polarLenght / 360

    # Сумма скоростей по косательным и от центра окружности
    vx = lenth * cos(tanget) + dr * cos(polarAngle)
    vy = lenth * sin(tanget) + dr * sin(polarAngle)
    vz = targetDeep - startPosition[2]

    # угол скорости
    angle = getPolarAngle(0, 0, vx, vy)

    # угол, сторона - на которые нужно повернуть аппарат
    angle, turn = calcAngleBeetwen(selfAngle, angle)

    surge = lenth * cos(relativeAngle(selfAngle, tanget)) + dr * cos(relativeAngle(selfAngle, polarAngle))
    pitch = lenth * sin(relativeAngle(selfAngle, tanget)) + dr * sin(relativeAngle(selfAngle, polarAngle))
    depth = targetDeep
    speedFinal = [surge, pitch, depth]

    if turn == RIGHT: return normVectorFromArcc([[vx, vy, vz], [0, 0, angle]], needToStop, selfAngleAll, speedFinal)
    return normVectorFromArcc([[vx, vy, vz], [0, 0, -angle]], needToStop, selfAngleAll, speedFinal)

def moveToLine(startPosition, length, angle, turn, angleToPoint, needToStop, targetDeep, selfAngleAll) :
    # движемся по линии
    #print('LINE')

    # скорость по проекциям
    vx = length * cos(angleToPoint)
    vy = length * sin(angleToPoint)
    vz = targetDeep - startPosition[2]

    surge = length * cos(angle)
    pitch = length * sin(angle)
    depth = targetDeep
    speedFinal = [surge, pitch, depth]

    if turn == RIGHT: return normVectorFromLine([[vx, vy, vz], [0, 0, angle]], needToStop, selfAngleAll, speedFinal)
    return normVectorFromLine([[vx, vy, vz], [0, 0, -angle]], needToStop, selfAngleAll, speedFinal)

def linearSlowFunction(value, coef) :
    #print("SLOW befor:", value)
    #print("SLOW after:", math.e**(coef*value - 2))
    return math.e**(coef*value - 0)

def angularSlowFunction(value, coef) :
    return value
    return math.e**(coef*value - 0)

def normVectorFromArcc(vector, needToStop, selfAngleAll, speedFinal) :
    maxLinearSpeed = MAX_LINEAR_SPEED_A
    maxAngularSpeed = MAX_ANGULAR_SPEED_A
    maxMoveAngle = MAX_MOVE_ANGLE_A
    linearSlowCoef = LINEAR_SPEED_SLOW_COEF_A
    angularSlowCoef = LINEAR_SPEED_SLOW_COEF_A

    # вектор [2][3] по состовляющим
    oldSpeed = vector[0][:]
    oldAngle = vector[1][:]
    speed = vector[0]
    angularSpeed = vector[1]

    speed[2] = min(speed[2], MAX_VERTICAL_SPEED)

    # длинна пути
    pathLenth = speedAbs = (speed[0] ** 2 + speed[1] ** 2) ** 0.5

    # замедляемся, если нужно
    if needToStop :
        maxLinearSpeed = min(linearSlowFunction(speedAbs, linearSlowCoef), maxLinearSpeed)
        maxAngularSpeed = min(angularSlowFunction(angularSpeed[2], angularSlowCoef), maxAngularSpeed)

    # запрет движения при расхождении угла
    if abs(angularSpeed[2]) > maxMoveAngle : speed = [0, 0, speed[2]]

    # изменение модуля скорости
    if speedAbs > maxLinearSpeed or (speedAbs != 0 and needToStop == False):
        # print("Speed before:", speed)
        speed[0] = speed[0] / speedAbs * maxLinearSpeed
        speed[1] = speed[1] / speedAbs * maxLinearSpeed
    # print("Speed after:", speed)

    # изменение модуля угловой скорости
    if angularSpeed[2] > maxAngularSpeed:
        angularSpeedAbs = abs(angularSpeed[2])
        angularSpeed[2] = angularSpeed[2] / angularSpeedAbs * maxAngularSpeed

    angle = [selfAngleAll[0], selfAngleAll[1], selfAngleAll[2] - oldAngle[2]]

    # возврат результата
    vector = [speed, angularSpeed, oldSpeed, oldAngle, angle, speedFinal]
    return vector

def normVectorFromLine(vector, needToStop, selfAngleAll, speedFinal) :
    maxLinearSpeed = MAX_LINEAR_SPEED_L
    maxAngularSpeed = MAX_ANGULAR_SPEED_L
    maxMoveAngle = MAX_MOVE_ANGLE_L
    linearSlowCoef = LINEAR_SPEED_SLOW_COEF_L
    angularSlowCoef = LINEAR_SPEED_SLOW_COEF_L

    # вектор [2][3] по состовляющим
    oldSpeed = vector[0][:]
    oldAngle = vector[1][:]
    speed = vector[0]
    angularSpeed = vector[1]

    speed[2] = min(speed[2], MAX_VERTICAL_SPEED)

    # длинна пути
    pathLenth = speedAbs = (speed[0] ** 2 + speed[1] ** 2) ** 0.5

    # замедляемся, если нужно
    if needToStop:
        maxLinearSpeed = min(linearSlowFunction(speedAbs, linearSlowCoef), maxLinearSpeed)
        maxAngularSpeed = min(angularSlowFunction(angularSpeed[2], angularSlowCoef), maxAngularSpeed)

    # запрет движения при расхождении угла
    if abs(angularSpeed[2]) > maxMoveAngle: speed = [0, 0, speed[2]]

    # изменение модуля скорости
    if speedAbs > maxLinearSpeed or (speedAbs != 0 and needToStop == False):
        # print("Speed before:", speed)
        speed[0] = speed[0] / speedAbs * maxLinearSpeed
        speed[1] = speed[1] / speedAbs * maxLinearSpeed
    # print("Speed after:", speed)

    # изменение модуля угловой скорости
    if angularSpeed[2] > maxAngularSpeed:
        angularSpeedAbs = abs(angularSpeed[2])
        angularSpeed[2] = angularSpeed[2] / angularSpeedAbs * maxAngularSpeed

    angle = [selfAngleAll[0], selfAngleAll[1], selfAngleAll[2] - oldAngle[2]]

    # возврат результата
    vector = [speed, angularSpeed, oldSpeed, oldAngle, angle, speedFinal]
    return vector

def drawSpeedVector(draw, vector, startPoint, angle) :
    if draw == None : return

    speed = vector[0]
    angularSpeed = vector[1]

    angle1 = 360 - angle
    angle2 = (360 - angle) + angularSpeed[2]
    if (angularSpeed[2] < 0): angle1, angle2 = angle2, angle1

    AStar.AStar.drawLine(draw, startPoint[0], startPoint[1],
                         startPoint[0] + cos(angle) * ANGLE_DRAW_RADIUS,
                         startPoint[1] + sin(angle) * ANGLE_DRAW_RADIUS, 'red', 1)
    AStar.AStar.drawArc(draw, startPoint[0], startPoint[1], ANGLE_DRAW_RADIUS, angle1, angle2, 'red')

    AStar.AStar.drawLine(draw, startPoint[0], startPoint[1], startPoint[0] + speed[0], startPoint[1] + speed[1],
                         'yellow', 2)


if __name__ == "__main__":

    startPoint = [random.randint(0, 450), random.randint(0, 210), 20] #[50, 50]
    endPoint = [random.randint(0, 450), random.randint(0, 210), 10] #[450, 210]
    angle = random.randint(0, 360)

    circles = []
    for i in range(2):
        circles.append([random.randint(0, 499), random.randint(0, 249), random.randint(1, 40)])
    circles.append([startPoint[0]+random.randint(1, 20), startPoint[1]+random.randint(1, 20), 30])

    startPoint = [50, 50, 25]
    endPoint = [130, 130, 10]
    circles = [[70, 70, 40]]

    # ОТРИЦАТЕЛЬНЫЙ УГОЛ - ЭТО НАЛЕВО!!!
    state, vector, image = calcMove(circles, startPoint, [0, 0, angle], endPoint, True, 25)

    draw = ImageDraw.Draw(image)
    drawSpeedVector(draw, vector, startPoint, angle)

    print('Vector:', vector)
    print("State is", state)

    image.show()
