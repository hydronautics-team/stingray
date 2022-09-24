#!/opt/bin/python

import rospy
import serial
from acoustic_listener.msg import AcousticListener as amp_msg

def form_message(amp) -> amp_msg:
    msg = amp_msg()
    msg.name = "pinger_amplitude"
    msg.amplitude = int(amp)
    return msg

if __name__ == '__main__':
    rospy.init_node('acoustic_data_publisher')
    amplitude_output_topic = rospy.get_param("~topic_name")
    com_port = rospy.get_param("~com_port_name")
    baudrate = rospy.get_param("~baudrate")
    
    amp_pub = rospy.Publisher(amplitude_output_topic, amp_msg, queue_size=1)
    ser = serial.Serial(str(com_port), int(baudrate), timeout=0.1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if (ser.inWaiting() > 0):
            try:
                data = ser.read(ser.inWaiting())
                amp_pub.publish(form_message(int.from_bytes(data, "big")))
            except:
                pass
        rate.sleep() 
