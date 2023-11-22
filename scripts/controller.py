#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

home_x, home_y = 0,0
velocity = 0.1
k = 1
gamma = 1
h = 1


def controller(X):
    # X stores e,phi,theta of a bot
    e = X[0]
    phi = X[1]
    theta = X[2]
    alpha = theta - phi
    omega = k*alpha + gamma * (np.cos(alpha)*np.sin(alpha)/alpha)*(alpha+h*theta)
    new_vel = gamma * np.cos(alpha)*e
    return omega, new_vel


class control_handle():

    def __init__(self):
        self.node       = rospy.init_node('controller', anonymous=True)
        self.odom  = rospy.Subscriber('tb3_odom', Odometry, self.OdomCallback)
        self.imu       = rospy.Subscriber('tb3_imu', Imu, self.ImuCallback)
        self.vel_pub    = rospy.Publisher('cmd_vel', Twist, queue_size= 1000)
        self.velocity   = velocity
        self.w  = 0
        self.X_home     = np.empty(3)
        self.rate       = rospy.Rate(10) # 10hz
        self.engaged    = False
        self.time       = 0
        print("Starting controller")

    def run(self):
        
        omega, new_vel  = controller(self.X_home)

        vel = Twist()
        vel.linear.x   = new_vel
        vel.angular.z  = omega
        self.vel_pub.publish(vel)  

    def OdomCallback(self, data):

        pos = data.pose.pose
        position = pos.position

        # global x and y positions
        x, y = position.x, position.y
        #x and y positions relative to home
        x_rel = x - home_x
        y_rel = y - home_y

        #phi relative to axis parallel to global x
        quaternion = pos.orientation
        _, __, phi = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])         #phi \in [-pi,pi]
        if phi<0:
            phi = 2*np.pi + phi       #phi \in [0,2pi]

        #theta relative to axis parallel to global x and origin as home
        theta = np.arctan2(y_rel,x_rel)
        if theta<0:
            theta = 2*np.pi + theta    #theta \in [0, 2pi]

        e = np.sqrt(x_rel^2+y_rel^2)
        ### IMPORTANT: X stores e,phi,theta of a bot
        self.X_home = np.array([e, phi, theta])


        #velocity
        self.velocity = data.twist.twist.linear.x

    def ImuCallback(self, data):
        self.w = data.angular_velocity.x




if __name__ == '__main__':
    con_h = control_handle()
    try:
        con_h.run()
    except rospy.ROSInterruptException:
        pass