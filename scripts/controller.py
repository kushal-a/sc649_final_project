#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import numpy as np
import csv
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sc649_final_project.msg import TrajectoryPoints
from sc649_final_project.msg import ErrorAngles
import numpy as np


velocity = 0.1
k = 0.2
gamma = 0.2
h = 2

def controller(X):
    # X stores e,phi,theta of a bot
    e = X[0]
    phi = X[1]
    theta = X[2]
    alpha = theta - phi
    if abs(alpha)<0.04:
        omega = k*alpha + gamma * (alpha+h*theta)
    else:
        omega = k*alpha + gamma * (np.cos(alpha)*np.sin(alpha)/alpha)*(alpha+h*theta)
    new_vel = gamma * np.cos(alpha)*e
    return omega, new_vel

class control_handle():

    def __init__(self):
        self.node       = rospy.init_node('controller', anonymous=True)
        self.odom  = rospy.Subscriber('/odom', Odometry, self.OdomCallback)
        self.imu       = rospy.Subscriber('/imu', Imu, self.ImuCallback)
        self.vel_pub    = rospy.Publisher('/cmd_vel', Twist, queue_size= 1000)
        self.error_pub    = rospy.Publisher('/error', ErrorAngles, queue_size= 1000)
        self.points_sub = rospy.Subscriber('/points', TrajectoryPoints, self.PointsCallback)
        self.velocity   = velocity
        self.home_X  = np.empty(3)
        self.w  = 0
        self.state_data = np.empty(6)
        self.X     = np.empty(3)
        self.rate       = rospy.Rate(10) # 10hz
        self.x_state_home = np.empty(6)
        self.engaged    = False
        self.time       = 0
        print("Starting controller")

    def run(self):
        while not rospy.is_shutdown():
            if self.X[0]<0.03:
                if abs(self.X[1])<0.04 and abs(self.X[2])<0.04:
                    omega = 0
                    new_vel = 0 
                else:
                    new_vel = 0
                    omega, _ = controller(self.X)                
            else:
                omega, new_vel  = controller(self.X)
            #print(self.X, self.velocity, self.w, new_vel, omega)
            vel = Twist()
            vel.linear.x   = new_vel
            vel.angular.z  = omega
            self.vel_pub.publish(vel)
            error = ErrorAngles()
            error.E, error.PHI, error.THETA = np.abs(self.X)
            self.error_pub.publish(error)
            self.state_data = np.vstack((self.state_data,self.x_state_home))
        else:
            np.savetxt("/home/kushal/sc649_ws/src/sc649_final_project/data/states.csv", self.state_data, delimiter=',')


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
        phi_rel = phi - self.home_X[2]
        # if phi<0:
        #     phi = 2*np.pi + phi       #phi \in [0,2pi]

        #theta relative to axis parallel to global x and origin as home
        theta = np.arctan2(y_rel,x_rel)
        theta_rel = theta - self.home_X[2]

        # if theta<0:
        #     theta = 2*np.pi + theta    #theta \in [0, 2pi]

        e = np.sqrt(np.power(x_rel,2)+np.power(y_rel,2))
        ### IMPORTANT: X stores e,phi,theta of a bot
        self.X = np.array([e, phi_rel, theta_rel])


        #velocity
        self.velocity = data.twist.twist.linear.x
        self.x_state_home = np.array([x,y,phi,self.home_X[0], self.home_X[1], self.home_X[2]])

    def ImuCallback(self, data):
        self.w = data.angular_velocity.x

    def PointsCallback(self, data):
        self.home_X[0] = data.X
        self.home_X[1] = data.Y
        self.home_X[2] = data.YAW
        

if __name__ == '__main__':
    con_h = control_handle()
    try:
        con_h.run()
    except rospy.ROSInterruptException:
        pass 
