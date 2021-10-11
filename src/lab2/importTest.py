#!/usr/bin/env python

import rospy
from shifty_common.TrackedBot import TrackedBot

if __name__ == '__main__':
    rospy.init_node('test_node')
    print(TrackedBot.tracking_headers)

