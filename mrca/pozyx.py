
#!/usr/bin/env python
"""
The Pozyx ready to localize tutorial (c) Pozyx Labs
Please read the tutorial that accompanies this sketch:
https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Python

This tutorial requires at least the contents of the Pozyx Ready to Localize kit. It demonstrates the positioning capabilities
of the Pozyx device both locally and remotely. Follow the steps to correctly set up your environment in the link, change the
parameters and upload this sketch. Watch the coordinates change as you move your device around!

"""
import rospy
import time
from statistics import mean
import numpy as np

from time import sleep
from math import  (pi, cos, sin)
from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_2D, POZYX_3D, Coordinates, EulerAngles, POZYX_SUCCESS, POZYX_ANCHOR_SEL_AUTO,
                     DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister)
from pythonosc.udp_client import SimpleUDPClient
from geometry_msgs.msg import Twist

def AverageInitEnd(x,front,back):
    t = x[front:10-back]
    y = mean(x)
    x.pop(0)
    return y

def AverageMinMax(x):
    minv = min(x)
    maxv = max(x)
    t = filter(lambda y: y>minv and y<maxv, x)
    value = mean(t)
    return(value)

def Exponentialfilter(x):
    ratio = 0.9
    y = ratio*x[0]+(1-ratio)*x[1]
    return y

class MultitagPositioning(object):
    """Continuously performs multitag positioning"""

    def __init__(self, pozyx, osc_udp_client, tags, anchors, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, remote_id=None):
        rospy.init_node("uwb_pub")
        self.uwb_target_pub = rospy.Publisher('/target/position', Twist, queue_size=1)
        self.uwb_pos_pub = rospy.Publisher('/uwb/position', Twist, queue_size=1)
        self.pozyx = pozyx
        self.osc_udp_client = osc_udp_client

        self.tags = tags
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.remote_id = remote_id

    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX MULTITAG POSITIONING V1.1 ------------")
        print("NOTES:")
        print("- Parameters required:")
        print("\t- Anchors for calibration")
        print("\t- Tags to work with")
        print()
        print("- System will manually calibration")
        print()
        print("System will auto start positioning")
        print()
        self.pozyx.printDeviceInfo(self.remote_id)
        print()
        print("------------POZYX MULTITAG POSITIONING V1.1 ------------")
        print()


        self.setAnchorsManual()
        self.printPublishAnchorConfiguration()

    def loop(self):
        """Performs positioning and prints the results."""
        for i, tag in enumerate(self.tags):
            position = Coordinates()
            angles = EulerAngles()
            status = self.pozyx.doPositioning(
                position, self.dimension, self.height, self.algorithm, remote_id=tag)

            if status == POZYX_SUCCESS:
                #BUFFERx.pop(0)
                #BUFFERx.append(int(position.x))
                #BUFFERy.pop(0)
                #BUFFERy.append(int(position.y))
                #tempy = AverageMinMax(BUFFERy)
                # tempx = AverageMinMax(BUFFERx)

                # BUFFERx[1] = BUFFERx[0]
                # BUFFERx[0] = position.x
                # BUFFERy[1] = BUFFERy[0]
                # BUFFERy[0] = position.y
                # tempx = Exponentialfilter(BUFFERx)
                # tempy = Exponentialfilter(BUFFERy)

                theta= -2*pi/3
                position.x = cos(theta)*position.x - sin(theta)*position.y
                position.y = sin(theta)*position.x + cos(theta)*position.y


                uwb_pos = Twist()
                uwb_pos.linear.x = int(position.x) / 1000.
                uwb_pos.linear.y = int(position.y) / 1000.
                uwb_pos.linear.z = int(position.z) / 1000.
                self.printPublishPosition(position, tag)
                self.uwb_target_pub.publish(uwb_pos)
                return False
            else:
                self.printPublishErrorCode("positioning", tag)
                return True 

    def printPublishPosition(self, position, network_id):
        """Prints the Pozyx's position and possibly sends it as a OSC packet"""
        if network_id is None:
            network_id = 0
        s = "POS ID: {}, x(mm): {}, y(mm): {}, z(mm): {}".format("0x%0.4x" % network_id, position.x, position.y, position.z)
        print(s)
        if self.osc_udp_client is not None:
            self.osc_udp_client.send_message(
                "/position", [network_id, position.x, position.y, position.z])

    def setAnchorsManual(self):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        for tag in self.tags:
            status = self.pozyx.clearDevices(tag)
            for anchor in self.anchors:
                status &= self.pozyx.addDevice(anchor, tag)
            if len(anchors) > 4:
                status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(anchors), remote_id=tag)
            # enable these if you want to save the configuration to the devices.
            # self.pozyx.saveAnchorIds(tag)
            # self.pozyx.saveRegisters([POZYX_ANCHOR_SEL_AUTO], tag)
            self.printPublishConfigurationResult(status, tag)

    def printPublishConfigurationResult(self, status, tag_id):
        """Prints the configuration explicit result, prints and publishes error if one occurs"""
        if tag_id is None:
            tag_id = 0
        if status == POZYX_SUCCESS:
            print("Configuration of tag %s: success" % tag_id)
        else:
            self.printPublishErrorCode("configuration", tag_id)

    def printPublishErrorCode(self, operation, network_id):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code, self.remote_id)
        if network_id is None:
            network_id = 0
        if status == POZYX_SUCCESS:
            print("Error %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/error_%s" % operation, [network_id, error_code[0]])
        else:
            # should only happen when not being able to communicate with a remote Pozyx.
            self.pozyx.getErrorCode(error_code)
            print("Error % s, local error code %s" % (operation, str(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error_%s" % operation, [0, error_code[0]])

    def printPublishAnchorConfiguration(self):
        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.pos)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [anchor.network_id, anchor.pos.x, anchor.pos.y, anchor.pos.z])
                sleep(0.025)


def function(Invalue):

    return Outvalue
                
if __name__ == "__main__":
    # shortcut to not have to find out the port yourself
    #serial_port = '/dev/ttyACM0'
    serial_port = get_first_pozyx_serial_port()

    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    remote_id = 0x602b                    # remote device network ID
    remote = False                         # whether to use a remote device
    if not remote:
        remote_id = None

    use_processing = False               # enable to send position data through OSC
    ip = "127.0.0.1"                       # IP for the OSC UDP
    network_port = 8888                    # network port for the OSC UDP
    osc_udp_client = None
    if use_processing:
        osc_udp_client = SimpleUDPClient(ip, network_port)

    # the tag on the target person
    tags = [0x6716]
    # the anchors on the turtlebot
    anchors = [ DeviceCoordinates(0x676a, 1, Coordinates(0, 500, 500)),
                DeviceCoordinates(0x6720, 1, Coordinates(-500, 0,500)),
                DeviceCoordinates(0x6f29, 1, Coordinates(0,-500, 500)),
                DeviceCoordinates(0x675e, 1, Coordinates(500, 0, 500))]
    

    algorithm = POZYX_POS_ALG_UWB_ONLY  # positioning algorithm to use
    dimension = POZYX_2D               # positioning dimension
    height = 300                      # height of device, required in 2.5D positioning

    BUFFERx = [0]*10
    BUFFERy = [0]*10

    while not rospy.is_shutdown():
        pozyx = PozyxSerial(serial_port)
        r = MultitagPositioning(pozyx, osc_udp_client, tags, anchors,
                                algorithm, dimension, height, remote_id)
        r.setup()
        restart_flag = False

        while not rospy.is_shutdown() and restart_flag is False:

            restart_flag = r.loop()
            time.sleep(0.005)
