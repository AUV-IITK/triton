#!/usr/bin/env python
# license removed for brevity
import rospy
from math import sin, cos, pi
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from underwater_sensor_msgs.msg import DVL
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
odom_broadcaster = tf.TransformBroadcaster()
   
x = 0.0
y=0.0
z=0.0
r=0.0
p=0.0
yw=0.0
vx=0.0
vy=0.0
vz=0.0
vr=0.0
vp=0.0
vyw=0.0
al=0
be=0.0
ga=0.0  
orientation = Quaternion() 
def callback(data):
    
    global vx
    vx = -data.bi_x_axis
    global vy
    vy = -data.bi_y_axis
    global vz
    vz=data.bi_z_axis


def callback2(data):
    
    global vr
    vr= data.angular_velocity.x
    global vp
    vp =data.angular_velocity.y
    global vyw
    vyw = data.angular_velocity.z
    global orientation
    orientation = data.orientation
    global orientation_covariance 
    orientation_covariance= data.orientation_covariance


    

def talker():
    pub = rospy.Publisher('example/odom', Odometry, queue_size=10)
    rospy.init_node('odom_msg_generator', anonymous=True)
    rate = rospy.Rate(10) # 10h

    last_time = rospy.Time.now()
    while not rospy.is_shutdown():

        current_time= rospy.Time.now()
        
        global vx
        global vy
        global vz

        rospy.Subscriber('/g500/imu', Imu, callback2)
        rospy.Subscriber('/g500/dvl', DVL, callback)

        dt = (current_time-last_time).to_sec()
        delta_x = ((vx*cos(ga)*cos(be)) - (vy*sin(ga)*cos(be)) + vz*sin(be))*dt
        delta_y = ((vx*(cos(al)*sin(ga)+ sin(al)*sin(be)*cos(ga))) + vy*((cos(al)*cos(ga))- sin(al)*sin(be)*sin(ga)) - vz*(sin(al)*cos(be)))*dt
        delta_z = ((vx*(sin(al)*sin(ga)- cos(al)*sin(be)*cos(ga))) + vy*(sin(al)*cos(ga) + cos(al)*sin(be)*sin(ga)) + vz*(cos(al)*cos(be)))*dt
        delta_al = vr * dt
        delta_be = vp * dt
        delta_ga = vyw *dt

        global x
        x += delta_x
        global y 
        y+= delta_y
        global z
        z += delta_z
        global al
        al += delta_al
        global be
        be += delta_be
        global ga
        ga += delta_ga

        odom_quat = tf.transformations.quaternion_from_euler(vyw, vp, vr, 'rzyx')

        odom_broadcaster.sendTransform(
        (x, y, z),
        odom_quat,
        current_time,
        "girona500/base_link",
        "/world"
        )

        
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "world"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        
        odom.pose.pose.orientation.x = orientation.x
        odom.pose.pose.orientation.y = orientation.y
        odom.pose.pose.orientation.z = orientation.z
        odom.pose.pose.orientation.w = orientation.w
        #odom_quat.pose.pose.orientation_covariance = orientation_covariance

        #odom.pose.pose = Pose(Point(x, y, z), Quaternion(*odom_quat))
        
        odom.child_frame_id = "girona500/base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.x = vr
        odom.twist.twist.angular.y = vp
        odom.twist.twist.angular.z = vyw
        
        rospy.loginfo('%s ', odom.pose.pose.orientation )
        rospy.loginfo('%s ', orientation )
        pub.publish(odom)

        last_time=current_time


        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
