#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Point

def talker():
    pub = rospy.Publisher('blueberry/translationRelative', Point, queue_size=10)
    rospy.init_node('blueberry_controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        p = Point(x = 0.0, y = 0.0, z = 0.0)
        print("input x relative: ")
        p.x = input()
        print("input y relative: ")
        p.y = input()
        print("input z relative: ")
        p.z = input()
        pub.publish(p)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass