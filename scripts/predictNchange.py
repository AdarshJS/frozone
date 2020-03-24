#!/usr/bin/env python

#  Code to check which pedestrians are relevant, construct forbidden zone and change velocity
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import time
import math
import random
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist

class Frozone:

    def __init__(self):
        self.ROB_POS_FLAG = False
        self.rob_pos = np.array([])
        self.ped_pos = np.array([])
        self.rob_theta = 0
        self.ped_theta = 0
        self.relevant_names = []
        self.relevant_positions = []
        self.relevant_vectors = []
        self.ped_vel = 1 # m/s
        self.prediction_time = 1 # in seconds
        self.turn_phi = 0
        self.reaction_time = 0.5 # in seconds
        self.COMFORT_DIST_THRESH = 3.0 # meters
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_modelstates)
        rospy.Subscriber("/cmd_vel", Twist, self.callback_twist)
        self.velocity_publisher = rospy.Publisher('/jackal/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)

    def check_relevancy(self, rel_x_rot, rel_y_rot, rel_ped_vec):
        # print(rel_x_rot, rel_y_rot)
        # print(rel_ped_vec)

        # Checking relevancy conditions
        # Check if obstacle is in front of the robot within sensing range
        if (rel_x_rot > 0.5 and rel_x_rot < 5.5 and rel_y_rot > -2 and rel_y_rot < 2):
            if (rel_y_rot > 0 and rel_ped_vec[0] <= 0.7071 and rel_ped_vec[0] >= -0.7071 and rel_ped_vec[1] <= -0.7071 and rel_ped_vec[1] >= -1): # Obstacle approaching from left

                return True
            elif (rel_y_rot < 0 and rel_ped_vec[0] <= 0.7071 and rel_ped_vec[0] >= -0.7071 and rel_ped_vec[1] >= 0.7071 and rel_ped_vec[1] <= 1): # Obstacle approaching from right

                return True
            elif (rel_ped_vec[0] < 0 and rel_ped_vec[1] >= -0.5 and rel_ped_vec[1] <= 0.5 and rel_y_rot < 0.5 and rel_y_rot > -0.5): # Head-on collision
                return True
            elif (rel_ped_vec[0] > 0 and rel_ped_vec[1] >= -0.5 and rel_ped_vec[1] <= 0.5 and rel_y_rot < 0.5 and rel_y_rot > -0.5): # Straight ahead and moving away
                return True
            else:
                # print("Irrelevant")
                return False

        else:
            # print("Irrelevant")
            return False

    def calc_distance(self, rob_pos, ped_pos):
        dist = math.sqrt((rob_pos[0] - ped_pos[0])**2 + (rob_pos[1] - ped_pos[1])**2)
        # print("Distance from predicted position = %f"% dist)
        return dist

    def callback_modelstates(self, msg):

        #  Relevant pedestrian positions and walking directions
        self.relevant_names = []
        self.relevant_positions = []
        self.relevant_vectors = []

        for i in range(len(msg.name)):
            if (msg.name[i][0:6] == "jackal"):
                x = msg.pose[i].orientation.x
                y = msg.pose[i].orientation.y
                z = msg.pose[i].orientation.z
                w = msg.pose[i].orientation.w
                orientation_list = [x,y,z,w]

                (_, _, self.rob_theta) = euler_from_quaternion (orientation_list)
                # rob_vec = np.array([math.cos(rob_theta), math.sin(rob_theta)])

                x = msg.pose[i].position.x
                y = msg.pose[i].position.y
                self.rob_pos = np.array([x, y])

                if(self.ROB_POS_FLAG == False):
                    self.ROB_POS_FLAG = True
                    print ("Obtained Robot's position")


            # Get all pedestrian vectors and positions
            if(self.ROB_POS_FLAG == True):
                # if (msg.name[i][0:2] == "r2"):
                if (msg.name[i][0:5] == "actor"):

                    # Right now we get only orientation. Ideally we should get direction of velocities
                    # x = msg.pose[i].orientation.x
                    # y = msg.pose[i].orientation.y
                    # z = msg.pose[i].orientation.z
                    # w = msg.pose[i].orientation.w
                    x = msg.pose[i].orientation.z
                    y = msg.pose[i].orientation.x
                    z = msg.pose[i].orientation.y
                    w = msg.pose[i].orientation.w
                    orientation_list = [x,y,z,w]

                    # NOTE: Always check which side of the gazebo model is defined as front. For person_walking front of model is actually its backside
                    (_, _, self.ped_theta) = euler_from_quaternion (orientation_list)
                    self.ped_theta = self.ped_theta - math.pi/2
                    # print("Pedestrian heading direction %f" % (self.ped_theta))
                    ped_vec = np.array([math.cos(self.ped_theta), math.sin(self.ped_theta)]) # unit vector

                    # Get pedestrian position wrt global coordinates
                    x = msg.pose[i].position.x
                    y = msg.pose[i].position.y
                    self.ped_pos = np.array([x, y])

                    # Compute relative position of ped wrt unrotated robot
                    rel_x = self.ped_pos[0] - self.rob_pos[0]
                    rel_y = self.ped_pos[1] - self.rob_pos[1]

                    # Compute relative position of ped wrt rotated robot
                    rel_x_rot = rel_x * math.cos(self.rob_theta) + rel_y * math.sin(self.rob_theta)
                    rel_y_rot = -rel_x * math.sin(self.rob_theta) + rel_y * math.cos(self.rob_theta)
                    rel_ped_pos = np.array([rel_x_rot, rel_y_rot])

                    # Compute relative yaw angle (orientation) of ped wrt robot
                    rel_theta = self.ped_theta - self.rob_theta

                    # compute unit vector of pedestrian relative to rotated robot
                    rel_ped_vec = np.array([math.cos(rel_theta), math.sin(rel_theta)])

                    # Checking relevancy
                    relevant_result = self.check_relevancy(rel_x_rot, rel_y_rot, rel_ped_vec)

                    if (relevant_result == True):
                        # Add current location to an array
                        self.relevant_names.append(msg.name[i])
                        self.relevant_positions.append(rel_ped_pos)
                        self.relevant_vectors.append(rel_ped_vec)



    def headon_overtake(self, direction):

        new_vel_data = Twist()
        num = (self.turn_phi * 6) / (math.pi/2)
        # print(int(num))

        if (direction == "left"):
            new_vel_data.angular.z = -(self.turn_phi / self.reaction_time)
        elif (direction == "right"):
            new_vel_data.angular.z = (self.turn_phi / self.reaction_time)

        for pub_i in range(int(num)):
        # while(len(self.relevant_vectors) == 1): # might have to change this
            new_vel_data.linear.x = 0.7
            self.velocity_publisher.publish(new_vel_data)


    def make_turn(self):
        new_vel_data = Twist()
        num = (self.turn_phi * 7) / (math.pi/2)

        # if(self.turn_phi > 0): # left turn
        #     new_vel_data.angular.z = (self.turn_phi / self.reaction_time)
        # else:
        #     new_vel_data.angular.z = (self.turn_phi / self.reaction_time)

        for pub_i in range(int(num)):
            new_vel_data.linear.x = 0.5
            new_vel_data.angular.z = (self.turn_phi / self.reaction_time)
            self.velocity_publisher.publish(new_vel_data)



    def callback_twist(self, vel_data):
        # rospy.loginfo("Received a /cmd_vel message!")
        new_vel_data = Twist()
        pred_vector = []
        pred_pos = []
        if (len(self.relevant_vectors) == 0):
            print("None")
            # Publish same velocity as obtained from final_sim.py
            new_vel_data.linear = vel_data.linear
            new_vel_data.angular = vel_data.angular
            self.velocity_publisher.publish(new_vel_data)


        else:

            # Special case: n = 1
            if (len(self.relevant_vectors) == 1):

                # Calculate predicted positions of pedestrian
                pred_vector = self.relevant_vectors[0] * self.ped_vel # (pedestrial velocity is currently set to 1 m/s)
                pred_pos = [self.relevant_positions[0][0] + pred_vector[0]*self.prediction_time, self.relevant_positions[0][1] + pred_vector[1]*self.prediction_time]

                # Head-on collision case
                if (self.relevant_positions[0][1] <= 0.5 and self.relevant_positions[0][1] >= -0.5 and
                    self.relevant_vectors[0][0] < 0 and self.relevant_vectors[0][1] >= -0.5 and self.relevant_vectors[0][1] <= 0.5):
                    # Pedestrian is less than confortable distance
                    if (self.calc_distance([0, 0], pred_pos) < self.COMFORT_DIST_THRESH):
                        # change velocity for head on case
                        print("changing velocity for head-on collision")
                        # Calculate turn phi Angle
                        self.turn_phi = math.atan(3 / self.calc_distance([0, 0], pred_pos)) # in radians
                        if (self.relevant_positions[0][1] <= 0.5 and self.relevant_positions[0][1] >= 0.0): # slightly towards left
                            self.headon_overtake("left")
                        else: # slightly towards right
                            self.headon_overtake("right")


                # Straight ahead but moving away
                elif (self.relevant_positions[0][1] <= 0.5 and self.relevant_positions[0][1] >= -0.5 and
                    self.relevant_vectors[0][0] > 0 and self.relevant_vectors[0][1] >= -0.5 and self.relevant_vectors[0][1] <= 0.5):
                    if (self.calc_distance(self.rob_pos, pred_pos) < self.COMFORT_DIST_THRESH):
                        # change velocity similar to previous case
                        print("changing velocity to overtake")
                        self.turn_phi = math.atan(4.08/self.calc_distance([0, 0], pred_pos)) # in radians
                        self.headon_overtake()

                else:
                    if (self.calc_distance(self.rob_pos, self.relevant_positions[0]) < self.COMFORT_DIST_THRESH):
                        # Change velocity towards pedestrian's current position
                        print("Moving in a pedestrian friendly way")
                        # temp_vec = self.relevant_positions[0];
                        self.turn_phi = math.acos(self.relevant_positions[0][0] / math.sqrt(self.relevant_positions[0][0]**2 + self.relevant_positions[0][1]**2))
                        self.make_turn()





if __name__ == '__main__':
    rospy.init_node('Model_State_Sub')
    # spin() simply keeps python from exiting until this node is stopped

    try:
        frozone = Frozone()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass