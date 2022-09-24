#!/opt/bin/python
#!/usr/bin/env python

import rospy
import serial
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolRequest
from std_srvs.srv import SetBoolResponse

class AcousticsListener:
    def __init__(self, com_port, baudrate) -> None:
        self.toggle_srv = rospy.Service("toggle_flare_buckets", SetBool, self.toggle_frequency)
        
        self.frequency_flare_or_bucket = False
        self.ser = serial.Serial(str(com_port), int(baudrate), timeout=0.1)
        self.amplitude = 0 
        
    def toggle_frequency(self, request: SetBoolRequest) -> SetBoolResponse:
        self.frequency_flare_or_bucket = request.data
        msg = b'0'
        if self.frequency_flare_or_bucket:
            msg = b'1'
        
        try:
            self.ser.write(msg)
        except:
            rospy.logerr("Unable to send message")
        
        response = SetBoolResponse()
        response.success = True
        if msg:
            msg_txt = "Now listening to flare"
        else:
            msg_txt = "Now listening to buckets"
        response.message =  msg_txt
        return response      

if __name__ == '__main__':
    rospy.init_node('acoustics_control_node')
    com_port = rospy.get_param("~com_port_name")
    baudrate = rospy.get_param("~baudrate")
    
    try:
        al = AcousticsListener(com_port, baudrate)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Shutting down {} node".format(rospy.get_name()))
