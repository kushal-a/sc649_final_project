#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sc649_final_project.msg import TrajectoryPoints
from std_msgs.msg import Float64


velocity = 0.1
k = 0.6
gamma = 0.15
h = 1.5


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
        self.odom  = rospy.Subscriber('/odom', Odometry, self.OdomCallback)
        self.imu       = rospy.Subscriber('/imu', Imu, self.ImuCallback)
        self.vel_pub    = rospy.Publisher('/cmd_vel', Twist, queue_size= 1000)
        self.error_pub    = rospy.Publisher('/error', Float64, queue_size= 1000)
        self.points_sub = rospy.Subscriber('/points', TrajectoryPoints, self.PointsCallback)
        self.velocity   = velocity
        self.home_X  = np.empty(3)
        self.w  = 0
        self.X     = np.empty(3)
        self.rate       = rospy.Rate(10) # 10hz
        self.engaged    = False
        self.time       = 0
        print("Starting controller")

    def run(self):
        while not rospy.is_shutdown():
            # print(self.X, self.velocity, self.w)
            if self.X[0]<1 and (self.X[1]<0.17 or self.X[1]>6.1) and (self.X[2]<0.17 or self.X[2]>6.1):
                omega = 0
                new_vel = 0
            else:
                omega, new_vel  = controller(self.X)
            vel = Twist()
            vel.linear.x   = new_vel
            vel.angular.z  = omega
            self.vel_pub.publish(vel)
            error = self.X[0] + self.X[1] +self.X[2]  
            self.error_pub.publish(error)

    def OdomCallback(self, data):
        
        pos = data.pose.pose
        position = pos.position

        # global x and y positions
        x, y = position.x, position.y
        #x and y positions relative to home
        x_rel = self.home_X[0] - x
        y_rel = self.home_X[1] - y

        #phi relative to axis parallel to global x
        quaternion = pos.orientation
        _, __, phi = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])         #phi \in [-pi,pi]
        if phi<0:
            phi = 2*np.pi + phi       #phi \in [0,2pi]

        #theta relative to axis parallel to global x and origin as home
        theta = np.arctan2(y_rel,x_rel)
        if theta<0:
            theta = 2*np.pi + theta    #theta \in [0, 2pi]

        e = np.sqrt(np.power(x_rel,2)+np.power(y_rel,2))
        ### IMPORTANT: X stores e,phi,theta of a bot
        self.X = np.array([e, phi, theta])


        #velocity
        self.velocity = data.twist.twist.linear.x

    def ImuCallback(self, data):
        self.w = data.angular_velocity.x

    def PointsCallback(self, data):
        self.home_X[0] = data.X
        self.home_X[1] = data.Y
        self.home_X[2] = data.YAW
        print("POINT RECIEVED %f %f %f"%(data.X,data.Y,data.YAW))
        

if __name__ == '__main__':
    con_h = control_handle()
    try:
        con_h.run()
    except rospy.ROSInterruptException:
        pass 
