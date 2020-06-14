#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import math
def talker():
    pub = rospy.Publisher('/vision', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(3) # 10hz
    x=7.0
    y=7.0
    r= 7
    th=0
    while not rospy.is_shutdown():
        mesg= Odometry()
        rospy.loginfo("Publishing Odometryee")
        mesg.pose.pose.position.x= x + r*math.cos(th);
        mesg.pose.pose.position.y= y + r*math.sin(th);
        pub.publish(mesg)
        th= th+0.05
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass