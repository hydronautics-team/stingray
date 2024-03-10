from ultralytics import YOLO
import cv2
from ultralytics.utils.plotting import Annotator, colors
from stingray_object_detection_msgs.msg import Object, ObjectsArray


model = YOLO('../weights/best.pt')
cap = cv2.VideoCapture('/home/shakuevda/Desktop/SAUVC/stingray_testDetect/front_camera_image_raw_2022_09_24_06_18_09.avi')

cap.set(3, 640)
cap.set(4, 480)

while True:
    _, img = cap.read()
    
    # BGR to RGB conversion is performed under the hood
    # see: https://github.com/ultralytics/ultralytics/issues/2575
    results = model.predict(img)

    for r in results:
        
        annotator = Annotator(img)
        
        boxes = r.boxes
        for box in boxes:
            
            b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
            c = box.cls
            label = model.names[int(c)]

            annotator.box_label(b, label, color=colors(int(c), True))
            print(model.names[int(c)], " = ", box.xyxy[0])

            # object_msg = Object()
            # object_msg.name = label + ' ' + str(int(id))
            # object_msg.top_left_x = int(xyxy[0])
            # object_msg.top_left_y = int(xyxy[1])
            # object_msg.bottom_right_x = int(xyxy[2])
            # object_msg.bottom_right_y = int(xyxy[3])
          
    img = annotator.result()  
    cv2.imshow('YOLO V8 Detection', img)     
    if cv2.waitKey(1) & 0xFF == ord(' '):
        break

cap.release()
cv2.destroyAllWindows()
