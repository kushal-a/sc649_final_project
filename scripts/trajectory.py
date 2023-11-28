#!/usr/bin/env python3
import rospy
from sc649_final_project.msg import ErrorAngles
import numpy as np
from sc649_final_project.msg import TrajectoryPoints
import csv


class trajectory_node():

    def __init__(self):
        
        file_path = '/home/kushal/sc649_ws/src/sc649_final_project/data/data.csv'  # Replace 'your_file.csv' with the path to your CSV file

        # Read the CSV file and store data in a list
        data_list = []
        with open(file_path, 'r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                data_list.append(row)

        # Convert the list to a NumPy array
        self.trajectory_points = np.array(data_list, dtype=np.float64)
        self.num_of_points = np.shape(self.trajectory_points)[0]
        self.node       = rospy.init_node('trajectory', anonymous=True)
        self.error_sub       = rospy.Subscriber('/error', ErrorAngles, self.ErrorCallback)
        self.points_pub    = rospy.Publisher('/points', TrajectoryPoints, queue_size= 1000)
        self.current_line = -1
        self.error = np.array([9000,9000,9000])

    def run(self):
        rospy.sleep(4)
        while not rospy.is_shutdown():
            print(self.error)
            if np.all(self.error<np.array([0.3,0.1,0.1])) and self.current_line+1<=self.num_of_points-1:
                print(self.current_line)
                points = TrajectoryPoints()
                points.X = self.trajectory_points[self.current_line+1][0]
                points.Y = self.trajectory_points[self.current_line+1][1]
                points.YAW = self.trajectory_points[self.current_line+1][2]
                self.current_line = self.current_line+1
                self.points_pub.publish(points)
                print("POINT PUBLISHED")
                rospy.sleep(2)
    
    def ErrorCallback(self, data):
        self.error = np.array([data.E, data.PHI, data.THETA])
            


if __name__ == '__main__':
    traj_n = trajectory_node()
    try:
        traj_n.run()
    except rospy.ROSInterruptException:
        pass 
