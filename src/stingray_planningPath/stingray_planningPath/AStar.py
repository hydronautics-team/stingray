#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from PIL import Image, ImageDraw
import random
import time
import math
import pickle

# Разрешение изображения
IMAGE_SIZE_X = 3000 # 5000
IMAGE_SIZE_Y = 1500  # 2500

# Размер бассейна
POOL_SIZE_X = 500
POOL_SIZE_Y = 250

# Если нужно уменшить окружность, то уменьшаем до MIN_RADIUS
MIN_RADIUS = 8
# Относительный размер точки на карте
POINT_SIZE = 1
# И так понятно, не трогать
COLLISION_EPSILON = 0.00001

# Цвето фона
BACKGROUND_COLOR = '#EAE5E9'
# Цвет линий графов
GRAPH_LINE_COLOR = 'black'
# Толщина линий графов
GRATH_LINE_WIDTH = 1
# Цвет вершин графа
GRATH_VERTEX_COLOR = '#404BE3'
# Радиус вершин графа
GRATH_VERTEX_RADIUS = 1

# Цвет линий пути
PATH_LINE_COLOR = '#FF4E4E'
# Толщина линий пути
PATH_LINE_WIDTH = 8

# Цвет просмотренных вершин графа
GRATH_VIEWED_VERTEX_COLOR = '#E3404B'
# Радиус просмотренных вершин графа
GRATH_VIEWED_VERTEX_RADIUS = 1.4
# Цвет удаленной окружности
DIED_CIRCLE_COLOR = '#D2CED1'
# Цвет границ удаленной окружности
DIED_CIRCLE_BORDER_COLOR = BACKGROUND_COLOR
# Цвет окружностей
CIRCLES_COLOR = '#29272D'
# цвет границ окружностей
CIRCLES_BORDER_COLOR = BACKGROUND_COLOR
# Толщина границ окружностей
CIRCLES_BORDER_WIDTH = 1


# Цвет начальной точки
START_POINT_COLOR = '#E3D840'
# Размер начальной точки
START_POINT_RADIUS = 3
# Цвет конечной точки
END_POINT_COLOR = '#E3D840'
# Размер конечной точки
END_POINT_RADIUS = 3


# Цвет уменьшенной окружности
VOID_CIRCLE_COLOR = '#D2CED1'
# Цвет границ уменьшенной окружности
VOID_CIRCLE_BORDER_COLOR = BACKGROUND_COLOR
# Цвет границ вершин графа
GRATH_VERTEX_BORDER_COLOR = BACKGROUND_COLOR

class Circle:
    
    def __init__(self, x, y, r, name='UNDEFINED'):
        self.name = name
        self.x = x
        self.y = y
        self.r = r
        self.nodes = []
        self.antinodes = []
        self.saw = False


class Line:

    def __init__(self, x1, y1, x2, y2) :
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def __str__(self):
        return 'Line->' + str(((self.x1-self.x2)**2 + (self.y1-self.y2)**2)**0.5)

    def __repr__(self):
        return str(self)

class Arcc:

    def __init__(self, centerX, centerY, r, dr, p1, p2, isInvert) :
        self.centerX = centerX
        self.centerY = centerY
        self.r = r
        self.dr = dr
        self.p1 = p1
        self.p2 = p2
        self.isInvert = isInvert

    def __str__(self):
        return 'Arc->' + str(self.r)

    def __repr__(self):
        return str(self)

class Node:

    def __init__(self, x, y, circleIndex, polarAngle):

        self.x = x 
        self.y = y
        self.polarAngle = polarAngle
        self.f = 0
        self.g = 0
        self.h = 0
        self.previous = None
        self.path = None
        self.circleIndex = circleIndex
        self.pathLenth = 0
        self.saw = False

    def __str__(self) :
        return '[' + str(self.x) + ' ' + str(self.y) + ']'

        
class AStar:

    def __init__(self):
        pass

    @staticmethod
    def drawLine(draw, x1, y1, x2, y2, color='pink', width=1):
        if (draw == None): return
        # return
        draw.line((x1 * IMAGE_SIZE_X / POOL_SIZE_X, IMAGE_SIZE_Y - y1 * IMAGE_SIZE_Y / POOL_SIZE_Y,
                   x2 * IMAGE_SIZE_X / POOL_SIZE_X, IMAGE_SIZE_Y - y2 * IMAGE_SIZE_Y / POOL_SIZE_Y), color, width)

    @staticmethod
    def drawCircle(draw, x, y, r, color, outline='black', width=0):
        if (draw == None) : return
        # return
        x = x * IMAGE_SIZE_X / POOL_SIZE_X
        r = r * IMAGE_SIZE_X / POOL_SIZE_X
        y = y * IMAGE_SIZE_Y / POOL_SIZE_Y
        draw.ellipse((x - r, IMAGE_SIZE_Y - y - r, x + r, IMAGE_SIZE_Y - y + r), color, outline, width)

    @staticmethod
    def drawArc(draw, x, y, r, start, end, color, width=1):
        if (draw == None): return
        # return
        x = x * IMAGE_SIZE_X / POOL_SIZE_X
        r = r * IMAGE_SIZE_X / POOL_SIZE_X
        y = y * IMAGE_SIZE_Y / POOL_SIZE_Y
        draw.arc((x - r, IMAGE_SIZE_Y - y - r, x + r, IMAGE_SIZE_Y - y + r), start, end, color, width)

    @staticmethod
    def binFind(v, c):
        y, x = -1, len(v)
        
        while y + 1 != x:
            m = (x+y)//2
            if v[m] <= c : y = m
            else : x = m

        return y

    @staticmethod
    def clean_open_set(open_set, current_node):

        for i in range(len(open_set)):
            if open_set[i] == current_node:
                open_set.pop(i)
                break

        return open_set

    @staticmethod
    def checkLine(circles, x1, y1, x2, y2, circleIndexes, draw) :
        #x1, y1, x2, y2 = x2, y2, x1, y1

        #print('line:', x1, y1, '   ', x2, y2)
            
        for i in range(len(circles)) :
            if i in circleIndexes : continue
            circle = circles[i]
            
            #print('circle\'s i x y:', i, circle.x, circle.y, circle.r)
            v1x = x2-x1
            v1y = y2-y1

            v2x = circle.x-x1
            v2y = circle.y-y1

            if (v1x**2 + v1y**2) == 0 :
                #AStar.drawCircle(draw, x1, y1, 2, 'orange')
                if ((x1-circle.x)**2 + (y1-circle.y)**2)**0.5 < circle.r - COLLISION_EPSILON : return False
                #else : return True
                else : continue
                
            u = (v1x*v2x + v1y*v2y)/(v1x**2 + v1y**2)
            u = max(min(1, u), 0)
                
            x = x1 + u*(x2 - x1)
            y = y1 + u*(y2 - y1)

            #AStar.drawCircle(draw, x, y, 2, 'blue')

            #print('u, x, y:', u, x, y)

            if ((circle.x - x)**2 + (circle.y - y)**2)**0.5 < circle.r - COLLISION_EPSILON  : return False

        AStar.drawLine(draw, x1, y1, x2, y2, GRAPH_LINE_COLOR, GRATH_LINE_WIDTH)
        
        return True

    @staticmethod
    def getPolarAngle(x1, y1, x2, y2):
        x = x2-x1
        y = y2-y1
        
        angle = math.acos(x/(x**2 + y**2)**0.5)/2/math.pi*360

        if y < 0 : angle = 360 - angle

        return angle

    @staticmethod
    def getPolarAngle2(x1, y1, c):
        x = x1-c.x
        y = y1-c.y
        
        angle = math.acos(x/(x**2 + y**2)**0.5)/2/math.pi*360

        if y < 0 : angle = 360 - angle

        return angle

    @staticmethod
    def isPointInsideCircle(node, circle):
        return ((node.x-circle.x)**2 + (node.y-circle.y)**2)**0.5 <= circle.r
    
    @staticmethod
    def findNeighborsFromPoint(circles, node, draw, points, n=False, n2=False):

        neighbors = []
        x = node.x
        y = node.y

        #print("aaaa", x, y)
        #AStar.drawCircle(draw, x, y, 7, 'green', 'black', 1)
        #AStar.drawCircle(draw, 100, 100, 5.5, 'green', 'black', 1)
        #AStar.drawCircle(draw, 200, 100, 5.5, 'green', 'black', 1)
        #AStar.drawCircle(draw, 10, 10, 6.5, 'green', 'black', 1)

        for i in range(len(circles)) :
            circle = circles[i]
            
            #if AStar.isPointInsideCircle(node, circle) : continue
            #if circle.r <= 0 : continue


            if ((circle.x-x)**2 + (circle.y-y)**2)**0.5 <= circle.r : continue

            x2 = (x + circle.x)/2
            y2 = (y + circle.y)/2
            r = ((x-x2)**2 + (y-y2)**2)**0.5
            d = r

            a = circle.r**2/2/d
            h = (circle.r**2 - a**2)**0.5

            x3 = circle.x + a/d*(x2-circle.x)
            y3 = circle.y + a/d*(y2-circle.y)

            x4 = x3 + h/d*(y2-circle.y)
            y4 = y3 - h/d*(x2-circle.x)

            x5 = x3 - h/d*(y2-circle.y)
            y5 = y3 + h/d*(x2-circle.x)

            #print('I = ', i)

            circleIndexes = [i]
            #if node.circleIndex != None and node.circleIndex != -1 :
                #circleIndexes.append(node.circleIndex)

            if AStar.checkLine(circles, x4, y4, x, y, circleIndexes, draw):
                node1 = Node(x4, y4, i, AStar.getPolarAngle2(x4, y4, circle))
                #print('AAAAAAAA:', x4, y4, circle.x, circle.y, i, AStar.getPolarAngle(x4, y4, circle.x, circle.y))
                node1.pathLenght = ((x-x4)**2 + (y-y4)**2)**0.5
                if n : node1.path = node
                neighbors.append(node1)
                if n2 : circle.nodes.append(node1)

            if AStar.checkLine(circles, x5, y5, x, y, circleIndexes, draw):
                node2 = Node(x5, y5, i, AStar.getPolarAngle2(x5, y5, circle))
                #print('AAAAAAAA:', x5, y5, i, AStar.getPolarAngle2(x5, y5, circle))
                node2.pathLenght = ((x-x5)**2 + (y-y5)**2)**0.5
                if n : node2.path = node
                neighbors.append(node2)
                if n2 : circle.nodes.append(node2)
        #print(len(neighbors))

        for p in points :
            if AStar.checkLine(circles, p.x, p.y, x, y, [], draw):
                neighbors.append(p)

        return neighbors

    @staticmethod
    def normalizeAngle(angle):
        if angle >= 0 : return angle % 360
        return 360 - ((-angle) % 360)

    @staticmethod
    def pointFromAngle(x, y, r, angle):
        x2 = x + r*math.cos(angle/360*2*math.pi)
        y2 = y + r*math.sin(angle/360*2*math.pi)
        return x2, y2

    @staticmethod
    def TwoCirclesIntersection(c1, c2):

        d = ((c1.x - c2.x)**2 + (c1.y - c2.y)**2)**0.5

        if d > c1.r + c2.r : return []
        if d < abs(c1.r - c2.r) : return []
        if d == 0 and c1.r == c2.r : return []

        a = (c1.r**2 - c2.r**2 + d**2)/2/d
        angle = math.acos(a/c1.r)/2/math.pi*360

        pa = AStar.getPolarAngle(c1.x, c1.y, c2.x, c2.y)

        #x1 = c1.x + c1.r*math.cos((pa-angle)/360*2*math.pi)
        #y1 = c1.y + c1.r*math.sin((pa-angle)/360*2*math.pi)
        #x2 = c1.x + c1.r*math.cos((pa+angle)/360*2*math.pi)
        #y2 = c1.y + c1.r*math.sin((pa+angle)/360*2*math.pi)
        #AStar.drawCircle(draw, x1, y1, 1, 'orange')

        return [AStar.normalizeAngle(pa-angle), AStar.normalizeAngle(pa+angle)]
        
    @staticmethod
    def findNeighborsForCircle(circles, circleIndex, draw):

        if circleIndex == None : return []

        circle = circles[circleIndex]
        neighbors = []
        
        if circle.saw :
            n = []
            for i in circle.nodes :
                if i.saw == False : n.append(i)
            return n

        for i in range(len(circles)) :
            c = circles[i]

            if i == circleIndex or circles[i].saw : continue
            if circle.x == c.x and circle.y == c.y: continue

            #print(circleIndex, i)

            a1 = (circle.r + c.r) / ((circle.x-c.x)**2 + (circle.y-c.y)**2)**0.5
            if -1 <= a1 <= 1 :
                a1 = math.acos(a1)/2/math.pi*360

                #print(circle.x, circle.y, c.x, c.y)
                ap = AStar.getPolarAngle(circle.x, circle.y, c.x, c.y)

                x1 = circle.x + circle.r*math.cos((ap+a1)/360*2*math.pi)
                y1 = circle.y + circle.r*math.sin((ap+a1)/360*2*math.pi)

                x2 = circle.x + circle.r*math.cos((ap-a1)/360*2*math.pi)
                y2 = circle.y + circle.r*math.sin((ap-a1)/360*2*math.pi)

                x3 = c.x + c.r*math.cos((ap+a1+180)/360*2*math.pi)
                y3 = c.y + c.r*math.sin((ap+a1+180)/360*2*math.pi)

                x4 = c.x + c.r*math.cos((ap-a1+180)/360*2*math.pi)
                y4 = c.y + c.r*math.sin((ap-a1+180)/360*2*math.pi)

                if AStar.checkLine(circles, x1, y1, x3, y3, [i, circleIndex], draw):
                    node1 = Node(x1, y1, circleIndex, AStar.getPolarAngle2(x1, y1, circle))
                    node3 = Node(x3, y3, i, AStar.getPolarAngle2(x3, y3, c))
                    
                    node1.path = node3
                    node3.path = node1
                    
                    node1.pathLenght = ((x1-x3)**2 + (y1-y3)**2)**0.5
                    node3.pathLenght = node1.pathLenght
                    
                    neighbors.append(node1)
                    neighbors.append(node3)
                    circle.nodes.append(node1)
                    c.nodes.append(node3)

                    AStar.drawCircle(draw, x1, y1, GRATH_VERTEX_RADIUS, GRATH_VERTEX_COLOR, GRATH_VERTEX_BORDER_COLOR, 1)
                    AStar.drawCircle(draw, x3, y3, GRATH_VERTEX_RADIUS, GRATH_VERTEX_COLOR, GRATH_VERTEX_BORDER_COLOR, 1)

                if AStar.checkLine(circles, x2, y2, x4, y4, [i, circleIndex], draw):
                    node2 = Node(x2, y2, circleIndex, AStar.getPolarAngle2(x2, y2, circle))
                    node4 = Node(x4, y4, i, AStar.getPolarAngle2(x4, y4, c))
                    
                    node2.path = node4
                    node4.path = node2

                    node2.pathLenght = ((x2-x4)**2 + (y2-y4)**2)**0.5
                    node4.pathLenght = node2.pathLenght
                    
                    neighbors.append(node2)
                    neighbors.append(node4)
                    circle.nodes.append(node2)
                    c.nodes.append(node4)

                    AStar.drawCircle(draw, x2, y2, GRATH_VERTEX_RADIUS, GRATH_VERTEX_COLOR, GRATH_VERTEX_BORDER_COLOR, 1)
                    AStar.drawCircle(draw, x4, y4, GRATH_VERTEX_RADIUS, GRATH_VERTEX_COLOR, GRATH_VERTEX_BORDER_COLOR, 1)

            
            a2 = (circle.r - c.r) / ((circle.x-c.x)**2 + (circle.y-c.y)**2)**0.5
            
            if -1 <= a2 <= 1 :
                a2 = math.acos(a2)/2/math.pi*360

                #print(circle.x, circle.y, c.x, c.y)
                if circle.r > c.r :
                    ap = AStar.getPolarAngle(circle.x, circle.y, c.x, c.y)
                    #print('aaaB', circle.x, circle.y, c.x, c.y, ap)
                    #print(a2)
                else :
                    ap = AStar.getPolarAngle(c.x, c.y, circle.x, circle.y)
                    #print('aaaa', c.x, c.y, circle.x, circle.y, ap)
                    #print(a2)

                if a2 > 90 : a2 = 180 - a2

                #print(ap, a2)

                x5 = circle.x + circle.r*math.cos((ap+a2)/360*2*math.pi)
                y5 = circle.y + circle.r*math.sin((ap+a2)/360*2*math.pi)

                x6 = circle.x + circle.r*math.cos((ap-a2)/360*2*math.pi)
                y6 = circle.y + circle.r*math.sin((ap-a2)/360*2*math.pi)

                x7 = c.x + c.r*math.cos((ap+a2)/360*2*math.pi)
                y7 = c.y + c.r*math.sin((ap+a2)/360*2*math.pi)

                x8 = c.x + c.r*math.cos((ap-a2)/360*2*math.pi)
                y8 = c.y + c.r*math.sin((ap-a2)/360*2*math.pi)

                if AStar.checkLine(circles, x5, y5, x7, y7, [i, circleIndex], draw):
                    node5 = Node(x5, y5, circleIndex, AStar.getPolarAngle2(x5, y5, circle))
                    node7 = Node(x7, y7, i, AStar.getPolarAngle2(x7, y7, c))
                    
                    node5.path = node7
                    node7.path = node5

                    node5.pathLenght = ((x5-x7)**2 + (y5-y7)**2)**0.5
                    node7.pathLenght = node5.pathLenght
                    
                    neighbors.append(node5)
                    neighbors.append(node7)
                    circle.nodes.append(node5)
                    c.nodes.append(node7)

                    AStar.drawCircle(draw, x5, y5, GRATH_VERTEX_RADIUS, GRATH_VERTEX_COLOR, GRATH_VERTEX_BORDER_COLOR, 1)
                    AStar.drawCircle(draw, x7, y7, GRATH_VERTEX_RADIUS, GRATH_VERTEX_COLOR, GRATH_VERTEX_BORDER_COLOR, 1)

                if AStar.checkLine(circles, x6, y6, x8, y8, [i, circleIndex], draw):
                    node6 = Node(x6, y6, circleIndex, AStar.getPolarAngle2(x6, y6, circle))
                    node8 = Node(x8, y8, i, AStar.getPolarAngle2(x8, y8, c))
                    
                    node6.path = node8
                    node8.path = node6

                    node6.pathLenght = ((x6-x8)**2 + (y6-y8)**2)**0.5
                    node8.pathLenght = node6.pathLenght
                    
                    neighbors.append(node6)
                    neighbors.append(node8)
                    circle.nodes.append(node6)
                    c.nodes.append(node8)

                    AStar.drawCircle(draw, x6, y6, GRATH_VERTEX_RADIUS, GRATH_VERTEX_COLOR, GRATH_VERTEX_BORDER_COLOR, 1)
                    AStar.drawCircle(draw, x8, y8, GRATH_VERTEX_RADIUS, GRATH_VERTEX_COLOR, GRATH_VERTEX_BORDER_COLOR, 1)

        for c in circles :
            #if c.saw : continue
            circle.antinodes += AStar.TwoCirclesIntersection(circle, c)
        circle.antinodes.sort()

        circle.saw = True

        n = []
        for i in circle.nodes :
            if i.saw == False : n.append(i)
        return n
        #return circle.nodes

    @staticmethod
    def findNeighbors(circles, node, draw, points, isStartPoint=False):
        #print('\nfindNeighbors')
        #if node.path == None :
            #print('from:', node.circleIndex, node.x, node.y, 'None')
        #else :
            #print('from:', node.circleIndex, node.x, node.y, node.path.circleIndex)
        #input()
        if isStartPoint :
            return [AStar.findNeighborsFromPoint(circles, node, draw, points, False, True), AStar.findNeighborsForCircle(circles, node.circleIndex, draw)]


        if node.circleIndex == None or node.circleIndex == -1 : return [AStar.findNeighborsFromPoint(circles, node, draw, points, False, True), []]

        return [[], AStar.findNeighborsForCircle(circles, node.circleIndex, draw)]

    @staticmethod
    def hordD(circles, node1, node2):
        #print('\nhordD')
        
        if node1.circleIndex == None : return AStar.lineD(node1, node2)

        if node1.circleIndex != node2.circleIndex :
            print("Error: diferent circles in hordD!")
            return 0, 0
            #raise SystemExit
        
        c = circles[node1.circleIndex]
        r = c.r
        x, y = c.x, c.y

        l = 2*math.pi*r

        v1x = node1.x-x
        v1y = node1.y-y
        v2x = node2.x-x
        v2y = node2.y-y

        #print(v1x, v1y, v2x, v2y, (v1x*v2x + v1y*v2y)/(v1x**2 + v1y**2)**0.5/(v2x**2 + v2y**2)**0.5)
      
        a = math.acos(round((v1x*v2x + v1y*v2y)/(v1x**2 + v1y**2)**0.5/(v2x**2 + v2y**2)**0.5, 14))/2/math.pi*360

        antinodes = c.antinodes
        #print(node1.polarAngle, node2.polarAngle, antinodes)
        if antinodes :
            
            #AStar.drawCircle(draw, node1.x, node1.y, 3, 'orange')
            #AStar.drawCircle(draw, node2.x, node2.y, 3, 'orange')

            if node1.polarAngle > node2.polarAngle : node1, node2 = node2, node1

            a1 = node1.polarAngle
            a2 = node2.polarAngle

            #direction = -1 # against clock arrow

            #d1 = node1.polarAngle
            #d2 = node1.polarAngle + 180

            #if d2 < 360 :
            #    if d1 <= node2.polarAngle <= d2 : direction = 1
            #else :
            #    d2 = d2 % 360
            #    if d2 > node2.polarAngle or d1 < node2.polarAngle : direction = 1
            
            v1 = AStar.binFind(antinodes, node1.polarAngle)
            v2 = AStar.binFind(antinodes, node2.polarAngle)

            #a1 = node1.polarAngle
            #a2 = node2.polarAngle
            
            #if direction == 1 :
            #    if a1 > a2 :
            #        if v1 == len(antinodes)-1 and

            
            if v1 != v2 and (v2 != len(antinodes)-1 or v1 > 0) :
                #print(10**6)
                return 10**6, 0
            if v1 != v2 and a1 < a2 < a1+180 :
                #print('1:', 360 - a, a1 < a2 < a1+180)
                return l - a/360*l, 0
            if (v2 != len(antinodes)-1 or v1 > 0) and not a1 < a2 < a1+180 :
                #print('2:', 360 - a, a1 < a2 < a1+180)
                return l - a/360*l, 0

        #print('3:', a, a1 < a2 < a1+180)
        return a/360*l, 0

    @staticmethod
    def lineD(node1, node2):
        v1x = node1.x-node2.x
        v1y = node1.y-node2.y

        return (v1x**2 + v1y**2)**0.5

    @staticmethod
    def PointToPointDistance(x1, y1, x2, y2):
        return ((x1-x2)**2 + (y1-y2)**2)**0.5

    @staticmethod
    def start_path(circles, open_set, current_node, end, draw, dr, isStartPoint=False):
        #print('\nstart_path')

        best_way = 0
        for i in range(len(open_set)):
            if open_set[i].f < open_set[best_way].f: best_way = i

        current_node = open_set[best_way]
        final_path = []

        # print('\nopenset len:', len(open_set))
        # for neighbor in open_set:
        #     print('\nnode.circleIndex =', neighbor.circleIndex, ' x, y:', neighbor.x, neighbor.y)
        #     print('ADR =', neighbor)
        #     print('neighbor.path =', neighbor.path)
        #     print('PREV =', neighbor.previous)
        #     print('G =', neighbor.g)
        #     print('H =', neighbor.h)
        #     print('F =', neighbor.f)
        #
        # print('\nCurrent_node.circleIndex =', current_node.circleIndex, ' x, y:', current_node.x, current_node.y)
        # print('ADR =', current_node)
        # print('PREV =', current_node.previous)
        # print('G =', current_node.g)
        # print('H =', current_node.h)
        # print('F =', current_node.f)

        if current_node.circleIndex == -1: # == end
            temp = current_node
            q = 0
            path = []
            while temp.previous:

                if (temp.circleIndex != temp.previous.circleIndex) : #It's a line
                    path.append(Line(temp.previous.x, temp.previous.y, temp.x, temp.y))

                if (temp.circleIndex == temp.previous.circleIndex) : #It's a arcc

                    angle1 = AStar.getPolarAngle2(temp.previous.x, temp.previous.y, circles[temp.circleIndex])
                    angle2 = AStar.getPolarAngle2(temp.x, temp.y, circles[temp.circleIndex])

                    v1x = circles[temp.circleIndex].x - temp.x
                    v1y = circles[temp.circleIndex].y - temp.y
                    v2x = circles[temp.circleIndex].x - temp.previous.x
                    v2y = circles[temp.circleIndex].y - temp.previous.y

                    vecMul = v1x*v2y - v2x*v1y
                    isInvert = vecMul > 0

                    path.append(Arcc(circles[temp.circleIndex].x,
                                     circles[temp.circleIndex].y,
                                     circles[temp.circleIndex].r,
                                     dr,
                                     angle1,
                                     angle2,
                                     isInvert))

                temp = temp.previous

            path.reverse()

            for object in path :
                if isinstance(object, Line):
                    #print(object.x1, object.y1, object.x2, object.y2)
                    AStar.drawLine(draw, object.x1, object.y1, object.x2, object.y2, PATH_LINE_COLOR, PATH_LINE_WIDTH)
                if isinstance(object, Arcc):
                    angle1 = 360 - object.p2
                    angle2 = 360 - object.p1
                    if object.isInvert :
                        angle1, angle2 = angle2, angle1

                    AStar.drawArc(draw, object.centerX, object.centerY, object.r, angle1, angle2, PATH_LINE_COLOR, PATH_LINE_WIDTH)


            return open_set, current_node, path

        open_set = AStar.clean_open_set(open_set, current_node)
        current_node.saw = True

        
        neighborsLine, neighborsCircle = AStar.findNeighbors(circles, current_node, draw, [end], isStartPoint)
        #if isStartPoint :
            #print(len(neighbors))
            #for i in neighbors :
                #print(i.x, i.y)
                #AStar.drawCircle(draw, i.x, i.y, POINT_SIZE * 1.2, 'red', 'black', 1)
        #print(neighbors)
        #AStar.drawCircle(draw, current_node.x, current_node.y, 5.5, 'pink', 'black', 1)
        AStar.drawCircle(draw, current_node.x, current_node.y,
                         GRATH_VIEWED_VERTEX_RADIUS, GRATH_VIEWED_VERTEX_COLOR, GRATH_VERTEX_BORDER_COLOR, 1)

        #print('\nneighbors len:', len(neighbors))
        #for neighbor in neighbors:
            #print('\nneighbor.circleIndex =', neighbor.circleIndex, ' x, y:', neighbor.x, neighbor.y)
            #print('ADR =', neighbor)
            #print('neighbor.path =', neighbor.path)
            #print('PREV =', neighbor.previous)
            #print('SAW =', neighbor.saw)
            #print('G =', neighbor.g)
            #print('H =', neighbor.h)
            #print('F =', neighbor.f)

        for neighbor in neighborsLine :

            if neighbor.saw == True: continue
            temp_g = current_node.g + AStar.lineD(current_node, neighbor)
            neighbor.g = temp_g
            neighbor.h = AStar.lineD(neighbor, end)
            neighbor.f = neighbor.g + neighbor.h
            neighbor.previous = current_node

            if neighbor not in open_set:
                open_set.append(neighbor)

        for neighbor in neighborsCircle:

            if neighbor.saw == True: continue
            if neighbor.path == None: continue

            temp_g = current_node.g + AStar.hordD(circles, current_node, neighbor)[0]
            if neighbor.g < temp_g and neighbor.g != 0: continue

            neighbor.g = temp_g
            neighbor.h = AStar.lineD(neighbor, neighbor.path) + AStar.lineD(neighbor.path, end)
            neighbor.f = neighbor.g + neighbor.h
            neighbor.previous = current_node

            if (neighbor.path.g > neighbor.g + AStar.lineD(neighbor, neighbor.path) or neighbor.path.g == 0):
                neighbor.path.previous = neighbor
                neighbor.path.g = neighbor.g + AStar.lineD(neighbor, neighbor.path)
                neighbor.path.h = AStar.lineD(neighbor.path, end)
                neighbor.path.f = neighbor.path.g + neighbor.path.h

                open_set.append(neighbor.path)

        # print('\nneighbors len:', len(neighborsLine))
        # for neighbor in neighborsLine:
        #     print('\nneighbor.circleIndex =', neighbor.circleIndex, ' x, y:', neighbor.x, neighbor.y)
        #     print('ADR =', neighbor)
        #     print('neighbor.path =', neighbor.path)
        #     print('PREV =', neighbor.previous)
        #     print('SAW =', neighbor.saw)
        #     print('G =', neighbor.g)
        #     print('H =', neighbor.h)
        #     print('F =', neighbor.f)
        #
        # print('\nneighbors len:', len(neighborsCircle))
        # for neighbor in neighborsCircle:
        #     print('\nneighbor.circleIndex =', neighbor.circleIndex, ' x, y:', neighbor.x, neighbor.y)
        #     print('ADR =', neighbor)
        #     print('neighbor.path =', neighbor.path)
        #     print('PREV =', neighbor.previous)
        #     print('SAW =', neighbor.saw)
        #     print('G =', neighbor.g)
        #     print('H =', neighbor.h)
        #    print('F =', neighbor.f)

        return open_set, current_node, final_path

    def main(self, start, end, circles_, needToDraw, needToDrawAllLines):

        self.start = start
        self.end = end
        self.circles = []

        im, draw = None, None
        if needToDraw:
            im = Image.new('RGB', (IMAGE_SIZE_X, IMAGE_SIZE_Y), BACKGROUND_COLOR)
            draw = ImageDraw.Draw(im)

        endNode = Node(end[0], end[1], 0, 0)
        startNode_ = Node(start[0], start[1], 0, 0)
        startNone = None

        endNode = Node(end[0], end[1], -1, 0)

        circles = []
        max_deep = 0
        maxDeepIndex = len(circles_)
        for i in range(len(circles_)) :
            circle_ = circles_[i]

            distanceToCenter = AStar.PointToPointDistance(startNode_.x, startNode_.y, circles_[i][0], circles_[i][1])
            distanceToCenterEnd = AStar.PointToPointDistance(endNode.x, endNode.y, circles_[i][0], circles_[i][1])

            if circles_[i][2] <= 0 : continue
            if distanceToCenter <= MIN_RADIUS:
                #print(AStar.PointToPointDistance(10, 50, 10, circles_[i][1]))
                #print(distanceToCenter, circle_[0], circle_[1], circle_[2], startNode_.x, startNode_.y)
                AStar.drawCircle(draw, circle_[0], circle_[1], circle_[2], DIED_CIRCLE_COLOR, DIED_CIRCLE_COLOR, 1)
                continue
            if distanceToCenterEnd <= MIN_RADIUS:
                AStar.drawCircle(draw, circle_[0], circle_[1], circle_[2], DIED_CIRCLE_COLOR, DIED_CIRCLE_COLOR, 1)
                continue

            circles.append([circles_[i][0], circles_[i][1], circles_[i][2]])

            deep = circles_[i][2] - distanceToCenter
            if deep > max_deep :
                max_deep = deep
                maxDeepIndex = len(circles)-1

        #print('max_deep: ', max_deep, maxDeepIndex)
        for i in range(len(circles)) :
            circle = circles[i]

            if AStar.isPointInsideCircle(endNode, Circle(circle[0], circle[1], circle[2])) :
                AStar.drawCircle(draw, circle[0], circle[1], circle[2], VOID_CIRCLE_COLOR , VOID_CIRCLE_BORDER_COLOR, 1)
                circle[2] = MIN_RADIUS
                #continue

            if AStar.isPointInsideCircle(startNode_, Circle(circle[0], circle[1], circle[2])) :

                AStar.drawCircle(draw, circle[0], circle[1], circle[2], VOID_CIRCLE_COLOR , VOID_CIRCLE_BORDER_COLOR, 1)

                if i == maxDeepIndex :
                    #print('circle parametrs: ', max_deep, i, maxDeepIndex, circle[2])
                    circle[2] = circle[2] - max_deep
                    #print('new radius:', circle[2])
                    startNone = maxDeepIndex
                else :

                    circle[2] = MIN_RADIUS

            self.circles.append(Circle(circle[0], circle[1], circle[2]))
            #print('Add a circle: ', circle[0], circle[1], circle[2])

        for circle in self.circles :
            #print(circle.x, circle.y, circle.r)
            AStar.drawCircle(draw, circle.x, circle.y, circle.r, CIRCLES_COLOR, CIRCLES_BORDER_COLOR, CIRCLES_BORDER_WIDTH)

        if startNone == None :
            AStar.drawCircle(draw, start[0], start[1], START_POINT_RADIUS, START_POINT_COLOR)
            AStar.drawCircle(draw, end[0], end[1], END_POINT_RADIUS, END_POINT_COLOR)

            startNode = Node(start[0], start[1], None, 0)

            n1 = AStar.findNeighborsFromPoint(self.circles, startNode, draw, [endNode])
            n2 = AStar.findNeighborsFromPoint(self.circles, endNode, draw, [], True, True)

            for i in n1 : AStar.drawCircle(draw, i.x, i.y, GRATH_VERTEX_RADIUS, GRATH_VERTEX_COLOR,
                                           GRATH_VERTEX_BORDER_COLOR, 1)
            for i in n2 : AStar.drawCircle(draw, i.x, i.y, GRATH_VERTEX_RADIUS, GRATH_VERTEX_COLOR,
                                           GRATH_VERTEX_BORDER_COLOR, 1)

        else :
            #print("AAA")
            angle = AStar.getPolarAngle2(start[0], start[1], self.circles[maxDeepIndex])
            startNode = Node(start[0], start[1], maxDeepIndex, angle)
            self.circles[maxDeepIndex].nodes.append(startNode)

            AStar.drawCircle(draw, start[0], start[1], START_POINT_RADIUS, START_POINT_COLOR)
            AStar.drawCircle(draw, end[0], end[1], END_POINT_RADIUS, END_POINT_COLOR)

            n1 = AStar.findNeighborsFromPoint(self.circles, startNode, draw, [endNode])
            n2 = AStar.findNeighborsFromPoint(self.circles, endNode, draw, [], True, True)

            for i in n1: AStar.drawCircle(draw, i.x, i.y, GRATH_VERTEX_RADIUS, GRATH_VERTEX_COLOR,
                                          GRATH_VERTEX_BORDER_COLOR, 1)
            for i in n2: AStar.drawCircle(draw, i.x, i.y, GRATH_VERTEX_RADIUS, GRATH_VERTEX_COLOR,
                                          GRATH_VERTEX_BORDER_COLOR, 1)

        if needToDrawAllLines :
            for i in range(len(self.circles)) : AStar.findNeighborsForCircle(self.circles, i, draw)
        

        open_set  = []
        #closed_set  = []
        current_node = None
        final_path  = []
        open_set.append(startNode)
        self.end = endNode

        #im.show()
        if (maxDeepIndex < len(circles)) :
            #print(maxDeepIndex, len(circles))
            open_set, current_node, final_path = AStar.start_path(self.circles, open_set, current_node, endNode, draw, max_deep,True)
        while len(open_set) > 0:
            open_set, current_node, final_path = AStar.start_path(self.circles, open_set, current_node, endNode, draw, max_deep)
            if len(final_path) > 0: break
        #AStar.drawCircle(draw, startNode.x, startNode.y, 1, 'yellow', 'black', 1)

        return final_path, im



if __name__ == "__main__":

    a_star = AStar()

    #startPoint = [50, 50]
    #endPoint = [450, 210]
    startPoint = [random.randint(0, 499), random.randint(0, 249)]
    endPoint  = [random.randint(0, 499), random.randint(0, 249)]


    circles = []
    for i in range(60) :
        circles.append([random.randint(0, 499), random.randint(0, 249), random.randint(1, 40)])
    print(circles)

    time1 = time.time()
    final_path, im = a_star.main(startPoint, endPoint, circles, False, False)
    print("Time to make a path and draw:", time.time() - time1)

    time1 = time.time()
    final_path, im = a_star.main(startPoint, endPoint, circles, False, True)
    print("Time to make a path and draw:", time.time() - time1)

    time1 = time.time()
    final_path, im = a_star.main(startPoint, endPoint, circles, True, False)
    print("Time to make a path and draw:", time.time()-time1)
    im.show()
    im.save('draw-arc.png')

    time1 = time.time()
    final_path, im = a_star.main(startPoint, endPoint, circles, True, True)
    print("Time to make a path and draw:", time.time() - time1)
    im.show()
    im.save('draw-arc2.png')
        

 


