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
        self.rob_pos = []
        self.ped_pos = []
        self.rob_theta = 0.0
        self.ped_theta = 0.0
        self.relevant_names = []
        self.relevant_positions = []
        self.relevant_vectors = []
        self.ped_vel = 1.0 # m/s
        self.prediction_time = 1.0 # in seconds

        self.turn_phi = 0.0
        self.turn_dist = 0.0
        self.avoid_point = []

        self.reaction_time = 0.5 # in seconds
        self.COMFORT_DIST_THRESH = 3.0 # meters
        self.COMFORT_SIDE_DIST = 1.5 # meters

        # Proportional gains
        self.P_lin = 0.3
        self.P_ang = 1.0

        # Crowdmove velocities
        self.crowdmove_vx = 0.0
        self.crowdmove_wz = 0.0

        # counter for model Subscriber
        self.counter = 0
        self.before = False

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


    def callback_modelstates(self, msg):

        #  Relevant pedestrian positions and walking directions
        if (self.counter % 100 == 0 and self.before == False):
            # print("Inside model states callback")
            self.counter = self.counter + 1

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
                    self.rob_pos = [x, y]

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
                        self.ped_pos = [x, y]

                        # Compute relative position of ped wrt unrotated robot
                        rel_x = self.ped_pos[0] - self.rob_pos[0]
                        rel_y = self.ped_pos[1] - self.rob_pos[1]

                        # Compute relative position of ped wrt rotated robot
                        rel_x_rot = rel_x * math.cos(self.rob_theta) + rel_y * math.sin(self.rob_theta)
                        rel_y_rot = -rel_x * math.sin(self.rob_theta) + rel_y * math.cos(self.rob_theta)
                        rel_ped_pos = [rel_x_rot, rel_y_rot]

                        # Compute relative yaw angle (orientation) of ped wrt robot
                        rel_theta = self.ped_theta - self.rob_theta

                        # compute unit vector of pedestrian relative to rotated robot
                        rel_ped_vec = [math.cos(rel_theta), math.sin(rel_theta)]

                        # Checking relevancy
                        relevant_result = self.check_relevancy(rel_x_rot, rel_y_rot, rel_ped_vec)
                        # relevant_result = True

                        if (relevant_result == True):
                            # Add current location to an array
                            self.relevant_names.append(msg.name[i])
                            self.relevant_positions.append(rel_ped_pos)
                            self.relevant_vectors.append(rel_ped_vec)

        else:
            self.counter = self.counter + 1
            if (self.counter > 999):
                self.counter = 0

    def calc_distance(self, rob_pos, ped_pos):
        dist = math.sqrt((rob_pos[0] - ped_pos[0])**2 + (rob_pos[1] - ped_pos[1])**2)
        return dist

    def local_to_global_tf(self, point): # pass the point in robot frame and return global coordinates
        x_global = point[0] * math.cos(self.rob_theta) - point[1] * math.sin(self.rob_theta) + self.rob_pos[0]
        y_global = point[0] * math.sin(self.rob_theta) + point[1] * math.cos(self.rob_theta) + self.rob_pos[1]
        point_global = [x_global, y_global]
        return point_global

    def callback_twist(self, vel_data):
        # rospy.loginfo("Received a /cmd_vel message!")
        self.crowdmove_vx = vel_data.linear.x
        self.crowdmove_wz = vel_data.angular.z


    def use_crowdmove_vel(self):
        new_vel_data = Twist()
        new_vel_data.linear.x = self.crowdmove_vx
        new_vel_data.angular.z = self.crowdmove_wz
        self.velocity_publisher.publish(new_vel_data)


    def headon_overtake(self):
        new_vel_data = Twist()

        # Assign avoid point wrt robot frame
        if(len(self.relevant_positions) >= 1):
            if (self.relevant_positions[0][1] >= 0.0 and self.relevant_positions[0][1] <= 0.5): # slightly towards left
                self.avoid_point = [(self.relevant_positions[0][0])/2, (self.relevant_positions[0][1] - self.COMFORT_SIDE_DIST)]
            else: # slightly towards right
                self.avoid_point = [(self.relevant_positions[0][0])/2, (self.relevant_positions[0][1] + self.COMFORT_SIDE_DIST)]

            # Transform avoid point to be wrt global frame
            self.avoid_point = self.local_to_global_tf(self.avoid_point) # avoid point wrt global coordinates

            self.turn_dist = self.calc_distance(self.rob_pos, self.avoid_point)
            print("Distance to avoid point")
            print(self.turn_dist)

            flag = 0
            turn_dist_0 = 0.0
            while(self.turn_dist > 1.0):
                self.turn_dist = self.calc_distance(self.rob_pos, self.avoid_point)
                if(flag == 0):
                    turn_dist_0 = self.turn_dist
                    flag = 1

                if(flag == 1):
                    if(self.turn_dist > turn_dist_0):
                        print("Current distance is greater than initial distance")
                        return

                self.turn_phi = math.atan((self.avoid_point[1] - self.rob_pos[1])/ (self.avoid_point[0] - self.rob_pos[0])) - self.rob_theta

                new_vel_data.linear.x = self.P_lin * self.turn_dist
                new_vel_data.angular.z = self.P_ang * self.turn_phi
                if (new_vel_data.linear.x > 0.75):
                    new_vel_data.linear.x = 0.5
                if (new_vel_data.angular.z > 0.75):
                    new_vel_data.angular.z = 0.35
                if (new_vel_data.angular.z < -0.75):
                    new_vel_data.angular.z = -0.35

                print("Head-on function")
                print(new_vel_data.linear.x)
                print(new_vel_data.angular.z)
                self.velocity_publisher.publish(new_vel_data)
        else:
            return


    def make_turn(self, i):
        new_vel_data = Twist()

        # Assign avoid point wrt robot frame
        self.avoid_point = [self.relevant_positions[i][0], self.relevant_positions[i][1]]

        # Transform avoid point to be wrt global frame
        self.avoid_point = self.local_to_global_tf(self.avoid_point) # avoid point wrt global coordinates

        self.turn_dist = self.calc_distance(self.rob_pos, self.avoid_point)
        print("Distance to avoid point")
        print(self.turn_dist)

        exit_counter = 0
        flag = 0
        turn_dist_0 = 0.0
        while(self.turn_dist > 1.5):

            self.turn_dist = self.calc_distance(self.rob_pos, self.avoid_point)
            if(flag == 0):
                turn_dist_0 = self.turn_dist
                flag = 1

            if(flag == 1):
                if(self.turn_dist > turn_dist_0):
                    print("Current distance is greater than initial distance")
                    return
            self.turn_phi = math.atan((self.avoid_point[1] - self.rob_pos[1])/ (self.avoid_point[0] - self.rob_pos[0])) - self.rob_theta

            new_vel_data.linear.x = self.P_lin * self.turn_dist
            new_vel_data.angular.z = self.P_ang * self.turn_phi
            if (new_vel_data.linear.x > 0.75):
                new_vel_data.linear.x = 0.5
            if (new_vel_data.angular.z > 0.75):
                new_vel_data.angular.z = 0.35
            if (new_vel_data.angular.z < -0.75):
                new_vel_data.angular.z = -0.35

            print("Make turn function")
            print(new_vel_data.linear.x)
            print(new_vel_data.angular.z)
            self.velocity_publisher.publish(new_vel_data)

            # exit_counter = exit_counter + 1
            # if (exit_counter > 10):
            #     print("-----------------Breaking------------------------------------------------------------------")
            #     break

        if(self.turn_dist < 1.5):
            self.use_crowdmove_vel()

        return

    def loop(self):
        while not rospy.is_shutdown():
            pred_vector = []
            pred_pos = []
            rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
            rospy.wait_for_message("/cmd_vel", Twist, timeout=None)

            if (len(self.relevant_vectors) == 0):
                # print("No Relevant Obstacles")
                # Publish same velocity as obtained from final_sim.py
                self.use_crowdmove_vel()

            else:
                # Special case: n = 1
                self.before = True

                if(len(self.relevant_names) == len(self.relevant_positions) and len(self.relevant_positions) == len(self.relevant_vectors)):
                    if (len(self.relevant_vectors) == 1 and len(self.relevant_positions) == 1):
                        print("Length of relevant positions = %d"% len(self.relevant_positions))
                        print("Length of relevant vectors = %d"% len(self.relevant_vectors))

                        # Calculate predicted positions of pedestrian
                        # pred_vector = self.relevant_vectors[0] * self.ped_vel # (pedestrial velocity is currently set to 1 m/s)
                        pred_vector = [elem * self.ped_vel for elem in self.relevant_vectors[0]]

                        pred_pos = [self.relevant_positions[0][0] + pred_vector[0]*self.prediction_time,
                                    self.relevant_positions[0][1] + pred_vector[1]*self.prediction_time]

                        self.before = False

                        # Head-on collision case
                        if (self.relevant_positions[0][1] <= 0.5 and self.relevant_positions[0][1] >= -0.5 and
                            self.relevant_vectors[0][0] < 0 and self.relevant_vectors[0][1] >= -0.5 and self.relevant_vectors[0][1] <= 0.5):

                            # Pedestrian is less than confortable distance
                            if (self.calc_distance([0, 0], pred_pos) < self.COMFORT_DIST_THRESH):
                                print("changing velocity for head-on collision")
                                self.headon_overtake()

                            else:
                                print("Using Crowdmove velocity")
                                self.use_crowdmove_vel()


                        # Straight ahead but moving away
                        elif (self.relevant_positions[0][1] <= 0.5 and self.relevant_positions[0][1] >= -0.5 and
                            self.relevant_vectors[0][0] > 0 and self.relevant_vectors[0][1] >= -0.5 and self.relevant_vectors[0][1] <= 0.5):
                            if (self.calc_distance(self.rob_pos, pred_pos) < self.COMFORT_DIST_THRESH):
                                # change velocity similar to previous case
                                print("changing velocity to overtake")
                                self.headon_overtake()

                            else:
                                print("Using Crowdmove velocity")
                                self.use_crowdmove_vel()

                        else:
                            if (self.calc_distance(self.rob_pos, self.relevant_positions[0]) < self.COMFORT_DIST_THRESH):
                                # Change velocity towards pedestrian's current position
                                print("Moving in a pedestrian friendly way")
                                # temp_vec = self.relevant_positions[0];

                                self.make_turn(0)

                            else:
                                print("Using Crowdmove velocity")
                                self.use_crowdmove_vel()


                    # More than 1 pedestrian
                    elif (len(self.relevant_vectors) >= 2 and len(self.relevant_positions) >= 2):
                        print("Length of relevant positions = %d"% len(self.relevant_positions))
                        print("Length of relevant vectors = %d"% len(self.relevant_vectors))
                        pred_pos_list = []
                        ped_dist_list = []
                        # get predicted poses for the pedestrians
                        for ind in range(len(self.relevant_positions)):
                            pred_vector = [elem * self.ped_vel for elem in self.relevant_vectors[ind]]

                            pred_pos = [self.relevant_positions[ind][0] + pred_vector[0]*self.prediction_time,
                                        self.relevant_positions[ind][1] + pred_vector[1]*self.prediction_time]

                            # Check predicted pedestrian distance from Robot
                            dist = self.calc_distance([0, 0], pred_pos)
                            ped_dist_list.append(dist)

                        # Find the index of pedestrian with least Distance
                        if (min(ped_dist_list) <=  self.COMFORT_DIST_THRESH): # Change velocity
                            print("Making a turn towards the nearest pedestrian")
                            self.make_turn(ped_dist_list.index(min(ped_dist_list)))
                        else:
                            print("Using Crowdmove velocity")
                            self.use_crowdmove_vel()

                    else:
                        print("Using Crowdmove velocity")
                        self.use_crowdmove_vel()


if __name__ == '__main__':
    rospy.init_node('Model_State_Sub')
    # spin() simply keeps python from exiting until this node is stopped

    try:
        frozone = Frozone()
        frozone.loop()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
