#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sc649_final_project.msgs import TrajectoryPoints
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from IPython.display import HTML


velocity = 0.1
k = 0.5
gamma = 0.15
h = 2
csv = np.empty((0, 3))
stop_positions = []

# Lists to store the lines connecting the dots
lines = []

# Function to update the plot in each animation frame
def update(frame):
    # Clear the previous plot
    plt.clf()

    # Get the current data point
    current_point = csv[frame]

    # Plot the arrow at the specified coordinates and angle
    plt.arrow(current_point['x'], current_point['y'],
              0.1 * np.cos(np.radians(current_point['angle'])),
              0.1 * np.sin(np.radians(current_point['angle'])),
              head_width=5, head_length=5, fc='red', ec='red')

    # Store the position where the arrow stops
    stop_positions.append((current_point['x'], current_point['y']))

    # Plot dots at stopping points
    plt.scatter(*zip(*stop_positions), color='blue', s=10, marker='o')

    # Plot lines connecting the dots
    if len(stop_positions) > 1:
        lines.append(plt.plot(*zip(*stop_positions), color='blue', linestyle='-', marker='o'))

    # Set plot limits
    plt.xlim(0, 150)
    plt.ylim(0, 150)

    # Set plot title
    plt.title(f"Frame {frame + 1}/{len(csv)}")


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
        self.points_pub    = rospy.Publisher('/points', Float64, queue_size= 1000)
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
            print(self.X, self.velocity, self.w)
            if self.X[0]<1.6 and (self.X[1]<0.17 or self.X[1]>6.1) and (self.X[2]<0.17 or self.X[2]>6.1):
                omega = 0
                new_vel = 0
            else:
                omega, new_vel  = controller(self.X)
            vel = Twist()
            vel.linear.x   = new_vel
            vel.angular.z  = omega
            self.vel_pub.publish(vel)
            error = self.X[0] + self.X[1] +self.X[2]  
            self.error_pub(error)
            csv=np.append(csv,self.X,axis=0)
        else:
            fig, ax = plt.subplots()

# Create an animation
            animation = FuncAnimation(fig, update, frames=len(csv), interval=1000)

# Display the animation in the notebook
            HTML(animation.to_jshtml())
            


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
        

if __name__ == '__main__':
    con_h = control_handle()
    try:
        con_h.run()
    except rospy.ROSInterruptException:
        pass 
