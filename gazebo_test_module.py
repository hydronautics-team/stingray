#!/usr/bin/env python

import rospy
from src.stingray_pilot.scripts.top_level_actions import AUV


def main():
    rospy.init_node("action_test")

    auv = AUV()

    auv.forward(1000, 0.3)      # initial move from base position
    auv.dive(95)

    for i in range(6):          # making a hexagonal "circle". TODO test this in water
        auv.rotate(60)
        auv.forward(1000, 0.35)
    auv.rotate(60)

    auv.rotate(-180)            # return to base position
    auv.forward(1000, 0.3)
    auv.rotate(180)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
