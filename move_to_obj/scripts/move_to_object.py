#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from move_to_obj.cfg import moveToObjConfig
import numpy as np

# Tested robot field of view is from 90 degrees to 270 degrees. 

class RobotMover:
    def __init__(self):
        rospy.init_node('move_to_obj', anonymous=True)
        
        self.move_command_pub = rospy.Publisher("/prizm/twist_controller/twist_cmd", Twist, queue_size=10)
        rospy.Subscriber("/scan_filtered", LaserScan, self.laser_callback)
        
        # move command to control the robot
        self.move_command = Twist()

        # once the robot finds the closest object this can be turned to true using Dynamic recon to move the robot
        self.target_aquired = False

        #once the robot reaches the object it will stop
        self.stop_flag = False

        # Initialize the dynamic reconfigure server
        self.config_server = Server(moveToObjConfig, self.dynamic_reconfigure_callback)

        rospy.spin()

    # dynamic recon getting updated value of target_aquired
    def dynamic_reconfigure_callback(self, config, level):
        self.target_aquired = config.target_aquired
        #rospy.loginfo(self.target_aquired)
        return config
    
    # Calculate the np_range, min_range, and angle_to_obj
    def laser_callback(self, data: LaserScan):

        np_ranges = np.array(data.ranges)
        np_ranges[np.isnan(np_ranges)] = np.inf # filtering out out the nans

        np_ranges[(np_ranges < 0.1) | (np_ranges > 2.0)] = 1000 # only reading the range value from 0.1 to 2.0. mainly filtering the 0.0 values out

        min_range_index = np.argmin(np_ranges) # returning the min index
        min_range = np_ranges[min_range_index] # finding the min distance 

        angle_to_obj = data.angle_min + min_range_index * data.angle_increment # calculating the angle to the closest odject
        angle_to_degrees = angle_to_obj * (180/3.14159) # converting to degrees
        

        #print(min_range) #Used for debugging 
        #print(angle_to_degrees) #Used for debugging 

        # Checking state of robot if it needs to move forward
        if self.target_aquired:
            self.robot_forward(angle_to_degrees, min_range)
        else:
            self.robot_turn(angle_to_degrees)
    
    def robot_forward(self, angle_to_degrees, min_range):

        if angle_to_degrees > 185.00 and min_range > 0.2 and self.stop_flag == False: # moves forward and corrects angle to object
            self.move_command.linear.x = 0.4 
            self.move_command.angular.z = 0.2
        elif angle_to_degrees < 165.00 and min_range > 0.2 and self.stop_flag == False: # moves forward and corrects angle to object
            self.move_command.linear.x = 0.4
            self.move_command.angular.z = -0.2 
        elif 165.00 <= angle_to_degrees <= 185.00 and min_range > 0.2 and self.stop_flag == False: # moves forward 
            self.move_command.linear.x = 0.4
            self.move_command.angular.z = 0.0
        elif min_range <= 0.2: # stops once closes to odject
            self.move_command.linear.x = 0.0
            self.move_command.angular.z = 0.0
            self.stop_flag = True

        self.move_command_pub.publish(self.move_command)

    def robot_turn(self, angle_to_degrees):

        # checks if odject is inside of the field of veiw.
        if angle_to_degrees < 90 or angle_to_degrees > 260:
            self.move_command.linear.x = 0.0
            self.move_command.angular.z = 0.0
            self.move_command_pub.publish(self.move_command)
            return

        if angle_to_degrees > 190.00: # turns robot if the object is not in front of robot
            self.move_command.linear.x = 0.0
            self.move_command.angular.z = 0.2
        elif angle_to_degrees < 165.00: # turns robot if the object is not in front of robot
            self.move_command.linear.x = 0.0
            self.move_command.angular.z = -0.2 
        elif 165.00 <= angle_to_degrees <= 190.00: # object is in front of robot
            self.move_command.linear.x = 0.0
            self.move_command.angular.z = 0.0
            
        self.move_command_pub.publish(self.move_command)


if __name__ == '__main__':
    robot_mover = RobotMover()



