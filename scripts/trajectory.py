#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import numpy as np
from sc649_final_project.msgs import TrajectoryPoints
import csv


class trajectory_node():

    def __init__(self):
        
        file_path = 'your_file.csv'  # Replace 'your_file.csv' with the path to your CSV file

        # Read the CSV file and store data in a list
        data_list = []
        with open(file_path, 'r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                data_list.append(row)

        # Convert the list to a NumPy array
        self.trajectory_points = np.array(data_list)
        self.node       = rospy.init_node('trajectory', anonymous=True)
        self.error_sub       = rospy.Subscriber('/error', Float64, self.ErrorCallback)
        self.points_pub    = rospy.Publisher('/points', TrajectoryPoints, queue_size= 1000)
        self.current_line = 0
        self.error = 9000

    def run(self):
        while not rospy.is_shutdown():
            if self.error<0.3:
                points = TrajectoryPoints()
                points.X = self.trajectory_points[self.current_line+1][0]
                points.Y = self.trajectory_points[self.current_line+1][1]
                points.YAW = self.trajectory_points[self.current_line+1][2]
                self.current_line = self.current_line+1
                self.points_pub(points)
    
    def ErrorCallback(self, data):
        self.error = data.data
            


if __name__ == '__main__':
    traj_n = trajectory_node()
    try:
        traj_n.run()
    except rospy.ROSInterruptException:
        pass 
