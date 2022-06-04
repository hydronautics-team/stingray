#!/usr/bin/env python

from stingray_pilot.top_level_actions import AUV
import rospy


def main():
    rospy.init_node("action_test")

    auv = AUV()

    auv.forward_locked(1000, 0.3)      # initial move from base position
    auv.dive(95)

    # making a hexagonal "circle". TODO test this in water
    auv.circle(1500)

    auv.rotate(-180)            # return to base position
    auv.forward_locked(1000, 0.3)
    auv.rotate(180)

    # making a dodecagon "circle"
    # auv.execute_pattern("circle", 1000, 0.4, True, False)
    #auv.circle(radius=2000)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
