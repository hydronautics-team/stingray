from ultralytics import YOLO
import cv2
from ultralytics.utils.plotting import Annotator, colors
# from stingray_object_detection_msgs.msg import Object, ObjectsArray
from stingray_interfaces.msg import Object, ObjectsArray
import numpy as np

model = YOLO('../weights/best.pt')
cap = cv2.VideoCapture('/home/shakuevda/Desktop/SAUVC/stingray_testDetect/front_camera_image_raw_2022_09_24_06_18_09.avi')

cap.set(3, 640)
cap.set(4, 480)

f = open('log.txt', 'w')


while True:
    _, img = cap.read()
    
    # BGR to RGB conversion is performed under the hood
    # see: https://github.com/ultralytics/ultralytics/issues/2575
    results = model.predict(img)

    for r in results:
        
        annotator = Annotator(img)
        # print("r = ", r)
        boxes = r.boxes
        # print("boxes = ", boxes)

        for box in boxes:
            
            b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
            c = box.cls
            label = model.names[int(c)]

            annotator.box_label(b, label, color=colors(int(c), True))
            # print(model.names[int(c)], " = ", box.xyxy[0])


            left, top, right, bottom = np.array(box.xyxy.cpu(), dtype=int).squeeze()
            width = right - left
            height = bottom - top
            center = (left + int((right-left)/2), top + int((bottom-top)/2))
            confidence = float(box.conf.cpu())
            
            object_msg = Object()
            object_msg.name = label
            object_msg.confidence = confidence
            object_msg.top_left_x = int(left)
            object_msg.top_left_y = int(top)
            object_msg.bottom_right_x = int(right)
            object_msg.bottom_right_y = int(bottom)
            print(object_msg)
            str_name = str(object_msg.name)
            str_confidence = str(object_msg.confidence)
            str_top_left_x = str(object_msg.top_left_x)
            str_top_left_y = str(object_msg.top_left_y)
            str_bottom_right_y = str(object_msg.bottom_right_y)
            str_bottom_right_x = str(object_msg.bottom_right_x)
            string = str_name + ' ' + str_confidence + ' ' + str_top_left_x + ' ' + str_top_left_y + ' ' + str_bottom_right_x + ' ' + str_bottom_right_y + '\n'
            f.write(string)


            
    img = annotator.result()  
    cv2.imshow('YOLO V8 Detection', img)     
    if cv2.waitKey(1) & 0xFF == ord(' '):
        break
    
cap.release()
cv2.destroyAllWindows()
