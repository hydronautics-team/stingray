
import AStar
import random
import math
import tkinter
from tkinter import *
from PIL import Image, ImageDraw, ImageTk
import Mesh

# создать разброс по скорости
RANDOM_SPEED = True
# коеффициент на которой умножается скосроть
LINEAR_SPEED_TIME_COEF = 1
# его приделы при разбросе
LINEAR_SPEED_TIME_MAX_COEF = 0.5
LINEAR_SPEED_TIME_MIN_COEF = 0.1
# аналогично для угловой скорсоти
ANGULAR_SPEED_TIME_COEF = 1
ANGULAR_SPEED_TIME_MAX_COEF = 0.5
ANGULAR_SPEED_TIME_MIN_COEF = 0.1
# разрешение окна приложения
SCREEN_RESOLUTION_X = 1600
SCREEN_RESOLUTION_Y = 800

# глобальные переменные
systemStates = []
circles = []
imageTk = None
image_id = None
selfPosition = [0, 0, 0]
selfDirection = [0, 0, 0]
endPoint = [0, 0, 0]

def randomMinMax(min, max) :
    return random.random() * (max - min) + min

# изменяем случайный коэффициент скорости
def updateRandomCoef() :
    global LINEAR_SPEED_TIME_COEF, ANGULAR_SPEED_TIME_COEF
    if RANDOM_SPEED:
        LINEAR_SPEED_TIME_COEF = randomMinMax(LINEAR_SPEED_TIME_MIN_COEF, LINEAR_SPEED_TIME_MAX_COEF)
        ANGULAR_SPEED_TIME_COEF = randomMinMax(ANGULAR_SPEED_TIME_MIN_COEF, ANGULAR_SPEED_TIME_MAX_COEF)

# обновляем позицию аппарата
def updatePosition(vector) :
    global selfPosition, selfDirection
    # здесь домножаем на случайный коэффициент скорсоти
    selfPosition[0] += vector[0][0] * LINEAR_SPEED_TIME_COEF
    selfPosition[1] += vector[0][1] * LINEAR_SPEED_TIME_COEF
    selfDirection[2] -= vector[1][2] * ANGULAR_SPEED_TIME_COEF

# обновляем картинку в GUI
def updateImage(image) :
    global image_id, imageTk

    image = image.resize((SCREEN_RESOLUTION_X, SCREEN_RESOLUTION_Y))
    imageTk = ImageTk.PhotoImage(image)

    if image_id == None :
        image_id = canvas.create_image(SCREEN_RESOLUTION_X//2, SCREEN_RESOLUTION_Y//2, image=imageTk)

    canvas.itemconfig(image_id, image=imageTk)

# считает следующую позицию во времени и двигает аппарат
def stepHandler(needLog) :
    # получаем текущее состояние, вектор скоростей и картинку
    state, vector, image = Mesh.calcMove(circles, selfPosition, selfDirection, endPoint, True, 25)

    # если нужно что-то выводить
    if needLog :
        print('Selfposition:', selfPosition)
        print('selfDirection:', selfDirection)
        print('Vector:', vector)

    # если вектор сокрости успешно посчитан
    if state == Mesh.STATE_OK :

        # рисуем вектор скорсоти на карте
        draw = ImageDraw.Draw(image)
        Mesh.drawSpeedVector(draw, vector[2:], selfPosition, selfDirection[2])

        # обновляем соучайные коэффициенты скорсоти
        updateRandomCoef()

        # изменяем положение аппарата, по вектору скорости
        updatePosition(vector)

        if needLog:
            print('Speed abs:', (vector[0][0] ** 2 + vector[0][1] ** 2) ** 0.5)
            print('State is OK\n')

    # обновляем картинку в GUI
    updateImage(image)

# следующая позиция аппарата (следующий момент времени)
def next(event):
    # запоминаем текущее состояние
    systemStates.append(
        [circles, selfPosition[:], selfDirection[:], endPoint[:], LINEAR_SPEED_TIME_COEF, ANGULAR_SPEED_TIME_COEF])

    # обработчик следующего шага аппарата
    stepHandler(False)

# возвращяет в прошлую позицию (шаг назад во времени для аппарата)
def prev(event):
    global systemStates, selfPosition, selfDirection, endPoint, LINEAR_SPEED_TIME_COEF, ANGULAR_SPEED_TIME_COEF

    # удаляем текущее состояние
    if len(systemStates) <= 1: return
    systemStates.pop()

    # возвращяем одно состояние назад
    circles, selfPosition, selfDirection, endPoint, LINEAR_SPEED_TIME_COEF, ANGULAR_SPEED_TIME_COEF = systemStates[-1]
    selfPosition, selfDirection, endPoint = selfPosition[:], selfDirection[:], endPoint[:]

    # обработчик следующего шага аппарата
    stepHandler(False)

if __name__ == '__main__' :

    # определение случайного начального положения
    selfPosition = [random.randint(10, 450), random.randint(10, 210), 25]  # [50, 50]
    endPoint = [random.randint(10, 450), random.randint(10, 210), 10]  # [450, 210]
    selfDirection = [0, 0, random.randint(0, 360)]

    # генерация препятствий
    circles = []
    for i in range(60):
        circles.append([random.randint(0, 499), random.randint(0, 249), random.randint(1, 40)])

    # создание окна
    root = Tk()
    root.bind("<Right>", next)
    root.bind("<Left>", prev)

    # создание canvas
    canvas = Canvas(root, height=SCREEN_RESOLUTION_Y, width=SCREEN_RESOLUTION_X)
    canvas.pack(side=TOP, expand=True, fill=BOTH)

    # нулевая обработка движения
    stepHandler(False)

    # запуск GUI
    root.mainloop()
