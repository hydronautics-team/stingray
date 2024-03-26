import random
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from stingray_interfaces.msg import Bbox, BboxArray
from stingray_core_interfaces.srv import SetTwist
from stingray_core_interfaces.msg import UVState

from stingray_planningPath import AStar
from stingray_planningPath import ImageHandler
from stingray_planningPath import Mesh

# растояние в приделах которого, объекты остаются дольше активными
REMEMBER_DISTANCE = 2

# множитель для радиуса в случае конечной точки
COMPLETE_DISTANT_COEF = 1

# целевая глубина
POOL_TARGET_DEEP = 25
# кадров для инакцивации объектов
FRAME_COUNT_TO_INACTIVE = 10
# коэффициент для REMEMBER_DISTANCE
FRAME_COUNT_COEF = 20

# логи
LOG_SAVE = False
LOG_SAVE_FILENAME = 'NTB_log.txt'

# наш радиус
SELF_RADIUS = 1.2

# если не знаем радиус объекта
DEFAULT_OBJECT_RADIUS = 2

# дистанция на который происходит полное преоблаание новых данных над старами
FILL_CHANGE_DISTANCE = 5

# растояние до центра при поиске
FIND_CENTER_DISTANCE = 3

class Vector :

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __abs__(self):
        return (self.x**2 + self.y**2 + self.x**2)**0.5

    def __str__(self):
        return '(' + str(self.x) + ', ' + str(self.y) + ', ' + str(self.z) + ')'


class Object :

    def __init__(self, name) :
        self.name = name
        #self.pos = Vector(0, 0, 0)
        self.x = 0
        self.y = 0
        self.z = POOL_TARGET_DEEP
        self.isActive = False
        self.frameCount = 0

    def __str__(self):
        return '[' + str(self.x) + ', ' + str(self.y) + '] (' + str(self.isActive) + ')'

    def __repr__(self):
        return str(self)


def weightValue(a, b, value) :
    if value < -1: value = -1
    if value > 1 : value = 1

    if b < a :
        a, b = b, a
        value = 1 - value

    return a + (b - a) * value


class Map :

    def __init__(self, position, angle) :
        self.objects = dict()
        self.x = position[0]
        self.y = position[1]
        self.z = position[2]
        self.angle = angle
        self.frameCount = 0
        self.totalPath = 0
        self.target = None

        self.findFlag_1 = False
        self.findVector = 0

        if LOG_SAVE:
            self.file_imageData = open(LOG_SAVE_FILENAME, 'a')
            self.file_posData = open(LOG_SAVE_FILENAME + '2', 'a')
            self.file_imageData.write('\nSTART on ' + str(round(time.time(), 2)))
            self.file_posData.write('\nSTART on ' + str(round(time.time(), 2)))


    @staticmethod
    def sendVector(vector):
        speed = vector[4][:]
        angle = vector[3][:]
        pass

    @staticmethod
    def updateMap(map, data, image):
        #print(data)
        #input()
        objectData = ImageHandler.calcDistanceAndAngle(ImageHandler.parseData(data), image)

        if LOG_SAVE:
            map.file_imageData.write('\n' + '\n'.join(['new img'] + data))
            #file = open(LOG_SAVE_FILENAME, 'a')
            #file.write('\n'.join(data))
            #file.close()

        map.updateObject(objectData)

    @staticmethod
    def updateMap2(map, data, image):
        objectData = data

        if LOG_SAVE:
            map.file_imageData.write('\n\nStart\n' + str(data))
            #file = open(LOG_SAVE_FILENAME, 'a')
            #file.write('\n'.join(data))
            #file.close()

        map.updateObject(objectData)

    @staticmethod
    def updatePositionMap(map, pos, angle3D):

        if LOG_SAVE:
            #file = open(LOG_SAVE_FILENAME + '_2', 'a')
            map.file_posData.write(','.join(list(map(lambda x : str(x), pos))) + ' ' + str(map.frameCount) + '\n')
            #file.close()

        map.updatePosition(pos, angle3D)

    def updatePosition(self, pos, angle3D):

        self.totalPath += ((pos[0] - self.x) ** 2 + (pos[1] - self.y) ** 2 + (pos[2] - self.z) ** 2) ** 0.5

        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]
        self.angle = angle3D

    def updateObject(self, objectList) :

        #print(self.objects)

        self.frameCount += 1

        for obj in objectList :
            #distance, relativeAngle, name
            self.__updateObject(obj[0], obj[1], obj[2])

        for obj in self.objects :
            obj = self.objects[obj]

            distance = Mesh.distance(self.x, self.y, obj.x, obj.y)
            objFC = obj.frameCount + FRAME_COUNT_TO_INACTIVE
            objFC_RD = obj.frameCount + FRAME_COUNT_TO_INACTIVE * FRAME_COUNT_COEF

            if ((objFC < self.frameCount and distance > REMEMBER_DISTANCE)
                    or objFC_RD  < self.frameCount ) :
                obj.isActive = False

    def __updateObject(self, distance, relativeAngle, name) :

        if name not in self.objects : self.objects[name] = Object(name)

        polarAngle = self.angle[2] - relativeAngle
        #self.objects[name].x = self.x + distance * Mesh.cos(polarAngle)
        #self.objects[name].y = self.y + distance * Mesh.sin(polarAngle)

        value = 1
        if self.x != 0 and self.y != 0 :
            value = 0.3 + (self.frameCount-1 - self.objects[name].frameCount)/5
            if self.totalPath != 0 :
                value = 0.3 + (self.totalPath - self.objects[name].totalPath) / FILL_CHANGE_DISTANCE

        self.objects[name].x = (weightValue(self.objects[name].x, self.x + distance * Mesh.cos(polarAngle), value))
        self.objects[name].y = (weightValue(self.objects[name].y, self.y + distance * Mesh.sin(polarAngle), value))

        self.objects[name].frameCount = self.frameCount
        self.objects[name].isActive = True

    @staticmethod
    def calcObjectRadius(obj) :
        #print(ImageHandler.objectData.get(obj.name, DEFAULT_OBJECT_RADIUS)[2] + SELF_RADIUS)
        return ImageHandler.objectData.get(obj.name, DEFAULT_OBJECT_RADIUS)[2] + SELF_RADIUS

    def find(self, targetName):
        if self.x == 0 and self.y == 0 : return [self.x, self.y, 0]

        x = AStar.POOL_SIZE_X / 2
        y = AStar.POOL_SIZE_Y / 2
        r = ImageHandler.objectData[targetName][2] * COMPLETE_DISTANT_COEF + SELF_RADIUS

        if Mesh.distance(self.x, self.y, x, y) > FIND_CENTER_DISTANCE :
            self.findFlag_1 = True
            return [x, y, r]

        if self.findFlag_1:
            self.findVector = random.randint(0, 100)
            self.findFlag_1 = False

        print('self.findVector ', self.findVector)
        if self.findVector % 2 == 0 :
            return [AStar.POOL_SIZE_X, y, r]

        return [0, y, r]

    def updateTarget(self, targetName):
        if targetName not in ImageHandler.objectData or targetName is None : return
        self.target = targetName

    def calcPath(self, needToDraw=False, targetName=None) :

        if targetName is not None:
            self.target = targetName
        targetName = self.target

        circles = []
        for obj in self.objects :
            obj = self.objects[obj]
            if obj.isActive and obj.name != targetName :
                circles.append([obj.x, obj.y, Map.calcObjectRadius(obj)])

        startPoint = [self.x, self.y, self.z]

        if targetName not in self.objects or not self.objects[targetName].isActive:
            endPoint = self.find(targetName)
        else :
            r = ImageHandler.objectData[targetName][2] * COMPLETE_DISTANT_COEF + SELF_RADIUS
            endPoint = [self.objects[targetName].x, self.objects[targetName].y, r]

        isFind = False
        state, vector, image = Mesh.calcMove(circles, startPoint, self.angle, endPoint, needToDraw, POOL_TARGET_DEEP)
        if targetName not in self.objects : isFind = True

        return state, isFind, vector, image


map2 = Map([0, 0, 0], [0, 0, 90])

def updateImage(map, data, image, targetName) :
    objectData = ImageHandler.calcDistanceAndAngle(data, image)

    if LOG_SAVE :
        file = open(LOG_SAVE_FILENAME, 'a')
        file.write('#imageData#' + str(data) + '\n')
        file.write('#objectData#' + str(objectData) + '\n')
        file.close()

    map.updateObject(objectData)

    state, vector = map.calcPath(targetName)

    if state != Mesh.STATE_ERROR and state != Mesh.STATE_COMPLETE : Map.sendVector(vector)

    return state

def updatePosition(map, newPosition, targetName) :
    map.updatePosition(newPosition[0], newPosition[1], newPosition[2])


class LocationMsg(Float32MultiArray):
    def __init__(self, surge, pitch, depth, a1, a2, a3):
        super().__init__()
        self.data = [surge, pitch, depth, a1, a2, a3]

class DroneNode(Node):

    def __init__(self):
        super().__init__('Kazemaru')
        #self.publisher_ = self.create_publisher(Float32MultiArray, 'vector_topic', 10)

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(BboxArray, "objects_array_topic", self.listener_callback, 10)

        # Create the service for to transmit the motion vector
        self.client_vectorMovement = self.create_client(SetTwist, '/stingray/services/set_twist')
        while not self.client_vectorMovement.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service SetTwist not available, waiting again...')
        self.req_vectorMovement = SetTwist.Request()

        # Create the subscriber.
        self.sub_uvStateTop = self.create_subscription(UVState, "uv_state_topic", self.callback_uvStateTop, 10)

        # Create the action.
        


        

        


    def listener_callback(self, data):
        
        #__НЕ_ТРОГАТЬ_ОТ_СЮДА__
        global map2
        # data -> BboxArray
        # data2 -> ImageParsedDataBboxArray format
        # data3 -> calculatedFinishData
        data2 = []
        for i in data.bboxes :
            data2.append(ImageHandler.Object(i.name, i.confidence, i.top_left_x, i.top_left_y, i.bottom_right_x, i.bottom_right_y))
        data3 = ImageHandler.calcDistanceAndAngle(data2, None)

        Map.updateMap2(map2, data3, None)

        state, isFind, vector, imageMap = map2.calcPath(False, 'gate')

        if state == 1 :
            self.get_logger().info("vector: " + str(vector[5]+ vector[4]))
        #__НЕ_ТРОГАТЬ_ДО_СЮДА__




        speed = vector[5][:]
        #speed = [surge (скорость по маршу),
        #         sway (скорость по лагу),
        #         depth (значение глубины)]
        
        angle = vector[4][:]
        #angle = [roll (всегда 0),
        #         pitch (всегда 0),
        #         yaw (значение курса)]
        
        # Передай их куда нужно
        self.send_vectorMovement(speed, angle)


    # передача вектора движения + ожидание ответа
    def send_vectorMovement(self, speed, angle):
        self.req_vectorMovemenе.surge  = speed[0]
        self.req_vectorMovemenе.sway = speed[1]
        self.req_vectorMovemenе.depth = speed[2]
        self.req_vectorMovemenе.roll = angle[0]
        self.req_vectorMovemenе.pitch = angle[1]
        self.req_vectorMovemenе.yaw = angle[2]
        self.responce_vectorMovemenе = self.client_vectorMovement.call_async(self.req_vectorMovemenе)
        self.responce_vectorMovemenе.add_done_callback(self.callback_vectorMovement)

    # колбэк при получении ответа, после отправки вектора
    def callback_vectorMovement(self, responce_vectorMovemenе):
        self.get_logger().info("responce srv SetTwist:" + responce_vectorMovemenе)

    # колбэк подписчика состояний аппарата
    def callback_uvStateTop(self, msg):
        # нужно заполнить
        pos = [0, 0, 0]

        angle3D = [msg.roll, msg.pitch, msg.yaw]
        
        Map.updatePositionMap(map2, pos, angle3D)
        




def main(args=None):
    rclpy.init(args=args)

    drone_node = DroneNode()

    rclpy.spin(drone_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drone_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    

    main()