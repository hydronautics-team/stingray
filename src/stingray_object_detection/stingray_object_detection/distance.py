
import math

class DistanceCalculator:
    # Y, X
    def __init__(self,
                 imgsz=(480, 640),
                 fov=60,
                 object_attrs=None,
                 ):
        self.imgsz = imgsz
        self.fov = (fov * (imgsz[0] / imgsz[1]), fov)
        self.object_attrs = object_attrs

    def _tg(self, x) :
        return math.tan(x/360*2*math.pi)

    def _calcDistance(self, angleSize, realSize) :
        return realSize / (2 * self._tg(angleSize / 2))

    def calcDistanceAndAngle(self, xyxy, cls) :

        if cls in self.object_attrs:
            realSizeX, realSizeY = self.object_attrs[cls][0], self.object_attrs[cls][1]

            sizeX = (xyxy[2] - xyxy[0]) / self.imgsz[1]
            sizeY = (xyxy[3] - xyxy[1]) / self.imgsz[0]
            angleSizeX = sizeX * self.fov[1]
            angleSizeY = sizeY * self.fov[0]

            DX = self._calcDistance(angleSizeX, realSizeX)
            DY = self._calcDistance(angleSizeY, realSizeY)

            D = DX
            if (sizeY != 0 and sizeY != 1): D = DY
            if (sizeX != 0 and sizeX != 1): D = DX
            if (sizeX != 0 and sizeX != 1 and sizeY != 0 and sizeY != 1) : D = (DX + DY) / 2

            positionX = xyxy[0] + (xyxy[2] - xyxy[0]) / 2
            angleX = (positionX / self.imgsz[1]) * self.fov[1]
            angle = angleX - self.fov[1] / 2

            return D, angle
