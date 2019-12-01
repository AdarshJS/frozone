import rospy
import time
import math

import numpy as np
import tensorflow as tf
import tensorlayer as tl

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Hyper parameters
MAX_LINEAR_VELOCITY = 0.5  #0.5
MAX_ANGULAR_VELOCITY = 0.75 #1.0


class MLSHAgent(object):
    def __init__(self):
        self.sess = tf.InteractiveSession()

        self._build_ph()

        # Subpolicies are defined later as classes
        self.subpolicies = []
        self.subpolicies.append(PPOAgent(self.sess, self.scan_ph, self.goal_ph, self.vel_ph))
        self.subpolicies.append(ClassController())
        self.subpolicies.append(FailedSafeController(self.subpolicies[0]))

        self.sess.run(tf.global_variables_initializer())

    def _build_ph(self):
        self.scan_ph = tf.placeholder(tf.float32,
            [None, 512, 3], 'scan_ph')
        self.goal_ph = tf.placeholder(tf.float32,
            [None, 2], 'goal_ph')
        self.vel_ph = tf.placeholder(tf.float32,
            [None, 2], 'vel_ph')

    def act(self, obs, terminated):
        if terminated:
            return np.zeros(2) # Stop the robot

        scan = obs[0]
        goal = obs[1]
        if np.min(scan) > 0.4 or (np.min(scan) * 4. > goal[0]): # Distance to goal is less than 4 times the least scan data
            action = self.subpolicies[1].act(obs) # Use PD controller


        else:
            action = self.subpolicies[0].act(obs) # Use PPO
            print("Using PPO")

        if action[0] < 0:
            action[0] = 0
        elif action[0] > MAX_LINEAR_VELOCITY:
            action[0] = MAX_LINEAR_VELOCITY

        if action[1] > MAX_ANGULAR_VELOCITY:
            action[1] = MAX_ANGULAR_VELOCITY
        elif action[1] < -MAX_ANGULAR_VELOCITY:
            action[1] = -MAX_ANGULAR_VELOCITY
        return action

    # Loads the trained policy params
    def load(self):
        self.subpolicies[0].load()

# Class for the P-D controller (It is actually only a Proportional controller)
class ClassController(object):
    def __init__(self):
        self.kp_x = 0.1
        self.kd_x = 0.0
        self.kp_z = 1.
        self.kd_z = 0.0

    def act(self, obs):
        print("Using P-controller")
        goal = obs[1]
        vels = obs[2]
        action = np.zeros(2) # init with zero velocities

        action[0] = self.kp_x * goal[0]
        action[1] = self.kp_z * goal[1] # goal[1] is in radians
        # if abs(goal[1]) < np.pi / 6.: # Angle to goal < pi/6, linear vel is zero
        #     action[0] = 0
        # elif abs(goal[1]) > np.pi / 2.:
        #     action[0] = 1. # Although it is given as 1 m/s, this is reduced in act() in MLSHAgent class

        return action

# Not used at all
class FailedSafeController(object):
    def __init__(self, policy):
        self.policy = policy

    def act(self, obs):
        scan = obs[0]
        obs[0] = obs[0] / 1.25
        action = self.policy.act(obs)
        return action


class PPOAgent(object):
    def __init__(self, sess, scan, goal, vel):
        self.sess = sess
        self.scan_ph = scan
        self.goal_ph = goal
        self.vel_ph = vel

        self.model, self.means = self._build_net()

    def _build_net(self):
        net = tl.layers.InputLayer(self.scan_ph, name='scan_input')
        net = tl.layers.Conv1dLayer(net, act=tf.nn.relu, shape=[5, 3, 32], stride=2,name='cnn1')
        net = tl.layers.Conv1dLayer(net, act=tf.nn.relu, shape=[3, 32, 16], stride=2,name='cnn2')
        net = tl.layers.FlattenLayer(net, name='fl')
        net = tl.layers.DenseLayer(net, n_units=256, act=tf.nn.relu, name='cnn_output')
        cnn_output = net.outputs

        act_net = tl.layers.InputLayer(tf.concat([self.goal_ph, self.vel_ph, cnn_output], axis=1), name='nav_input')
        act_net = tl.layers.DenseLayer(act_net, n_units=128, act=tf.nn.relu, name='act1')
        linear = tl.layers.DenseLayer(act_net, n_units=1, act=tf.nn.sigmoid, name='linear')
        angular = tl.layers.DenseLayer(act_net, n_units=1, act=tf.nn.tanh, name='angular')
        with tf.variable_scope('means'):
            means = tf.concat([MAX_LINEAR_VELOCITY *linear.outputs, MAX_ANGULAR_VELOCITY * angular.outputs], axis=1)

        return [net, act_net, linear, angular], means

    def act(self, obs):
        feed_dict = {
            self.scan_ph: [obs[0]],
            self.goal_ph: [obs[1]],
            self.vel_ph: [obs[2]]
        }

        action = self.sess.run(self.means, feed_dict=feed_dict)[0]
        return action

    def load(self):
        for i in range(len(self.model)):
            params = tl.files.load_npz(
                name='./model/last_act_{}.npz'.format(i))
            tl.files.assign_params(self.sess, params, self.model[i])

class Environment(object):
    def __init__(self, agent):
        import tf
        import tf.transformations
        rospy.init_node('ppo_agent')
        self.control_rate = rospy.Rate(10)
        self.agent = agent
        self.agent.load()
        self.scan = np.zeros([512, 3])
        self.uwb_pos = np.zeros(4)
        self.uwb_pos[3] = np.pi / 2.
        self.uwb_vel = np.zeros(3) # Not used anywhere
        self.vel = np.zeros(2)
        self.target_pos = np.zeros(3)
        self.odom_x = 0.
        self.odom_y = 0.
        self.odom_heading = 0.

        rospy.Subscriber('/jackal/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/jackal/odometry/filtered', Odometry, self.odom_callback)
        rospy.Subscriber('jackal/target/position', Twist, self.target_callback)

        self.command_pub = rospy.Publisher('/jackal/jackal_velocity_controller/cmd_vel', Twist, queue_size=1)

        time.sleep(1)

    def run(self):
        while not rospy.is_shutdown():
            goal, terminated = self.preprocess_nav()
            act = self.agent.act([self.scan, goal, self.vel], terminated) # agent is the object of MLSHAgent class

            #act[0] = 1
            # act[1] = -act[1] # Reversing angular velocity
            print 'linear: ', act[0], ' --- angular: ', act[1]
            self.send_command(act)
            self.control_rate.sleep()
            if terminated:
                #direct += 1
                time.sleep(1)


    def send_command(self, act):
        command = Twist()
        command.linear.x = act[0]
        command.angular.z = act[1]
        self.command_pub.publish(command)

    def preprocess_scan(self, scan):
        scan = np.asarray(scan)
        scan -= 0.1
        scan_list = scan.tolist()
        scan_list.reverse()
        scan = np.asarray(scan_list)
        scan = np.where(np.isinf(scan), 4., scan)
        scan = np.where(np.isnan(scan), 4., scan)
        scan = np.where(scan < 0., 0., scan)
        scan /= 4.
        return scan

    def preprocess_nav(self):
        goal = [self.compute_distance(), self.compute_angle()] # There was * 2 for compute_distance (maybe useful for practical purposes)
        print 'goal: ', goal[0], ' --- ', goal[1] * 57.3
        if goal[0] < 0.65:
            terminated = True
        else:
            terminated = False
        return goal, terminated

    def compute_distance(self):
        return np.hypot(self.uwb_pos[0] - self.target_pos[0], self.uwb_pos[1] - self.target_pos[1])

    def compute_angle(self):
        # angle = self.uwb_pos[3] - math.atan2(-self.uwb_pos[1]+self.target_pos[1], -self.uwb_pos[0]+self.target_pos[0])
        angle = math.atan2(self.target_pos[1], self.target_pos[0]) - self.target_pos[2]
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
        # print 'angle: ', angle * 57.3
        # print 'p_heading: ', 57.3 * math.atan2(-self.uwb_pos[1]+self.target_pos[1], -self.uwb_pos[0]+self.target_pos[0]), '  r_heading: ', -57.3 * self.uwb_pos[3]
        return angle

    def scan_callback(self, data):
        scan = self.preprocess_scan(data.ranges)/2 # There was / 2 here
        # print("max:{}/	min:{}".format(np.max(scan),np.min(scan)))
        self.scan = np.concatenate((self.scan[:, 1:], np.asarray([scan]).transpose()), axis=1)

    def odom_callback(self, data):
        import tf
        import tf.transformations
        self.vel[0] = data.twist.twist.linear.x
        self.vel[1] = data.twist.twist.angular.z/4.0

    def uwb_vel_callback(self, data):
        self.uwb_vel[0] = data.angular.x

    def target_callback(self, data):
        self.target_pos[0] = data.linear.x
        self.target_pos[1] = data.linear.y
        self.target_pos[2] = data.linear.z
        # print(self.target_pos[2])


if __name__ == '__main__':
    agent = MLSHAgent()
    env = Environment(agent)
    env.run()
