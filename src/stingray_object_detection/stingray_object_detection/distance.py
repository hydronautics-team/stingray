
import math
from sensor_msgs.msg import CameraInfo


class DistanceCalculator:
    # Y, X
    def __init__(self,
                 object_attrs: dict,
                 ):
        self.object_attrs = object_attrs

    def _tg(self, x):
        return math.tan(x/360*2*math.pi)

    def _calcDistance(self, angleSize, realSize):
        return realSize / (2 * self._tg(angleSize / 2))

    def calcDistanceAndAngle(self, xyxy, label_name: str, camera_info: CameraInfo):

        if label_name in self.object_attrs:
            # camera intrinsics
            ppx = camera_info.k[2]
            ppy = camera_info.k[5]
            fx = camera_info.k[0]
            fy = camera_info.k[4]

            # distance
            obj_width = xyxy[2] - xyxy[0]
            obj_height = xyxy[3] - xyxy[1]
            obj_center_x = (xyxy[2] + xyxy[0]) / 2
            obj_center_y = (xyxy[3] + xyxy[1]) / 2
            if self.object_attrs[label_name]["reference_dim"] == "width":
                pos_z = self.object_attrs[label_name]["real_size"] * fx / obj_width
            elif self.object_attrs[label_name]["reference_dim"] == "height":
                pos_z = self.object_attrs[label_name]["real_size"] * fy / obj_height
            else:
                raise ValueError("reference_dim must be width or height")
            
            pos_x = (obj_center_x - ppx) * pos_z / fx
            pos_y = (obj_center_y - ppy) * pos_z / fy

            horizontal_angle = math.atan2(pos_x, pos_z) * 57.3
            vertical_angle = math.atan2(pos_y, pos_z) * 57.3

            return pos_x, pos_y, pos_z, horizontal_angle, vertical_angle
