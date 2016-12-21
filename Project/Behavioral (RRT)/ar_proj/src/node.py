#!/usr/bin/env python

import roslib; roslib.load_manifest('ar_proj')
import rospy

from driver import driver

if __name__ == '__main__':
    try:
        pilot = driver()
        pilot.load_goals()
        pilot.drive()
    except rospy.ROSInterruptException:
        pass
