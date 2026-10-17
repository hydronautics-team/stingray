import math

from sensor_msgs.msg import CameraInfo


class DistanceCalculator:
    def __init__(self, object_attrs: dict):
        self.object_attrs = object_attrs

    def calcDistanceAndAngle(
            self, xyxy, label_name: str, camera_info: CameraInfo):
        """Estimate object position from its known size and camera intrinsics."""
        if label_name not in self.object_attrs:
            return None

        ppx = camera_info.k[2]
        ppy = camera_info.k[5]
        fx = camera_info.k[0]
        fy = camera_info.k[4]

        obj_width = xyxy[2] - xyxy[0]
        obj_height = xyxy[3] - xyxy[1]
        if obj_width <= 0 or obj_height <= 0 or fx <= 0 or fy <= 0:
            return None

        obj_center_x = (xyxy[2] + xyxy[0]) / 2
        obj_center_y = (xyxy[3] + xyxy[1]) / 2
        object_attrs = self.object_attrs[label_name]
        if object_attrs["reference_dim"] == "width":
            pos_z = object_attrs["real_size"] * fx / obj_width
        elif object_attrs["reference_dim"] == "height":
            pos_z = object_attrs["real_size"] * fy / obj_height
        else:
            raise ValueError("reference_dim must be width or height")

        pos_x = (obj_center_x - ppx) * pos_z / fx
        pos_y = (obj_center_y - ppy) * pos_z / fy
        horizontal_angle = math.degrees(math.atan2(pos_x, pos_z))
        vertical_angle = math.degrees(math.atan2(pos_y, pos_z))

        return pos_x, pos_y, pos_z, horizontal_angle, vertical_angle
