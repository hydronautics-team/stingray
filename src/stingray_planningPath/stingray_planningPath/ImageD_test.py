import sys

import ImageHandler
import cv2
from PIL import Image, ImageDraw
import AStar
import Mesh
import numpy
import NeedToBelieve
import sys


def parseFile(fileName) :

    videoData = []
    frameData = []

    file = open(fileName, 'r')
    data = file.read().splitlines()
    file.close()

    for i in range(len(data)) :
        i = data[i]

        if i == 'new img' :
            videoData.append(frameData)
            frameData = []
            continue

        q = i.split()

        name = q[0]
        p = float(q[1])
        x1 = int(q[2])
        y1 = int(q[3])
        x2 = int(q[4])
        y2 = int(q[5])

        frameData.append(ImageHandler.Object(name, p, x1, y1, x2, y2))

    return videoData

if __name__ == '__main__' :

    print(sys.argv[0])

    videoData = parseFile('/home/nearl/PycharmProjects/Akame/data.txt')

    cap = cv2.VideoCapture("/home/nearl/PycharmProjects/Akame/video.avi")
    while not cap.isOpened():
        cap = cv2.VideoCapture("/home/nearl/PycharmProjects/Akame/video.avi")
        cv2.waitKey(1000)
        print("Wait for the header")

    AStar.IMAGE_SIZE_X = 1000
    AStar.IMAGE_SIZE_Y = 500
    AStar.POOL_SIZE_X = 50
    AStar.POOL_SIZE_Y = 25
    AStar.MIN_RADIUS = 0.8
    # Относительный размер точки на карте
    AStar.POINT_SIZE = 0.1
    # Толщина линий графов
    AStar.GRATH_LINE_WIDTH = 0
    # Радиус вершин графа
    AStar.GRATH_VERTEX_RADIUS = 0.1
    # Толщина линий пути
    AStar.PATH_LINE_WIDTH = 3
    # Радиус просмотренных вершин графа
    AStar.GRATH_VIEWED_VERTEX_RADIUS = 0.14
    # Толщина границ окружностей
    AStar.CIRCLES_BORDER_WIDTH = 1
    # Размер начальной точки
    AStar.START_POINT_RADIUS = 0.5
    # Размер конечной точки
    AStar.END_POINT_RADIUS = 0.8


    selfX = AStar.POOL_SIZE_X / 2
    selfY = 1

    map = NeedToBelieve.Map([selfX, selfY, 10], [0, 0, 90])

    pos_frame = cap.get(cv2.CAP_PROP_POS_FRAMES)
    q = 0
    while True:
        flag, frame = cap.read()
        if flag:
            # The frame is ready and already captured

            frameData = videoData[q]

            for object in frameData :

                if object.p < 0.75 : continue

                font = cv2.FONT_HERSHEY_SIMPLEX
                org = (object.x1, object.y2)
                cv2.putText(frame, object.name, org, font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.rectangle(frame, (object.x1, object.y1), (object.x2, object.y2), color=(255, 0, 0), thickness=2)

            positionData = ImageHandler.calcDistanceAndAngle(frameData, None)
            map.y += 0.01
            map.x += 0.006
            #map.angle[2] += 0.1
            map.updateObject(positionData)
            #NeedToBelieve.Map.updateMap(map, frameData, None)

            state, isFind, vector, imageMap = map.calcPath(True, 'gate')

            print('state:', state, isFind)



            image = Image.new('RGB', (AStar.IMAGE_SIZE_X, AStar.IMAGE_SIZE_Y), AStar.BACKGROUND_COLOR)
            draw = ImageDraw.Draw(image)

            AStar.AStar.drawCircle(draw, selfX, selfY, 0.6, AStar.CIRCLES_COLOR, AStar.CIRCLES_BORDER_COLOR,
                             AStar.CIRCLES_BORDER_WIDTH)

            for object in positionData:
                polarAngle = 90 - object[1]
                d = object[0]
                name = object[2]
                r = ImageHandler.objectData[name][2]
                color =  ImageHandler.objectData[name][3]
                #print('r =', r)
                x = d * Mesh.cos(polarAngle) + selfX
                y = d * Mesh.sin(polarAngle) + selfY

                AStar.AStar.drawCircle(draw, x, y, r, color, AStar.CIRCLES_BORDER_COLOR,
                                       AStar.CIRCLES_BORDER_WIDTH)

            open_cv_image = numpy.array(image)
            # Convert RGB to BGR
            open_cv_image = open_cv_image[:, :, ::-1].copy()

            open_cv_image2 = numpy.array(imageMap)
            # Convert RGB to BGR
            open_cv_image2 = open_cv_image2[:, :, ::-1].copy()

            cv2.imshow('video2', open_cv_image)

            cv2.imshow('video3', open_cv_image2)

            cv2.imshow('video', frame)
            pos_frame = cap.get(cv2.CAP_PROP_POS_FRAMES)
            q += 1

            print(str(pos_frame) + " frames")
        else:
            # The next frame is not ready, so we try to read it again
            cap.set(cv2.CAP_PROP_POS_FRAMES, pos_frame - 1)
            print("frame is not ready")
            # It is better to wait for a while for the next frame to be ready
            cv2.waitKey(1000)

        if cv2.waitKey(10) == 27:
            break
        if cap.get(cv2.CAP_PROP_POS_FRAMES) == cap.get(cv2.CAP_PROP_FRAME_COUNT):
            # If the number of captured frames is equal to the total number of frames,
            # we stop
            break
