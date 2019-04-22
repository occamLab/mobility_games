#!/usr/bin/env python

import rospy
from keyboard.msg import Key

class KeyboardHack(object):
    def __init__(self):
        rospy.init_node('keyboardhack')
        self.pub = rospy.Publisher('/keyboard/keydown', Key, queue_size=10)

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            msg = Key(code=97)
            self.pub.publish(msg)
            r.sleep()


if __name__ == "__main__":
    node = KeyboardHack()
    node.run()
