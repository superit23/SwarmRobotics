#!/usr/bin/python

import rospy
from swarmflock.srv import *
import wifiutils
import hotspot
import mrssi
import sys
import os

class WiFiTrilatSrv:

  def handle_Trilat(self, req):
    mRSSI = mrssi.MacRSSI(self.interface, req.mac_address, self.freq)
    mRSSI.run()
    rospy.sleep(1)

    rssis = mRSSI.hist[:, 1]
    relevant = rssis[rssis != -100]

    distance = wifiutils.calcDistance(sum(relevant) / len(relevant), self.freq)
    return WiFiTrilatResponse(distance)


  def __init__(self, interface, freq=0):

    self.freq = freq
    self.interface = interface
    robotName = os.getenv('HOSTNAME')

    # Start ROS integration
    rospy.init_node(robotName + "_wifitrilat_server")
    #robotName = rospy.get_param("/robot/name")
    self.service = rospy.Service("/" + robotName + "/WiFiTrilat", WiFiTrilat, self.handle_Trilat)

    if self.freq != 0:
      hotspot.main("start")

    rospy.on_shutdown(self.shutdown)    

    # [DEPRECATED] This will create the ad-hoc network
    # https://gist.github.com/meeuw/5765413

    rospy.spin()


  def shutdown(self):

    if self.freq != 0:
      hotspot.main("stop")


if __name__ == "__main__":

  freq = 0

  if len(sys.argv) > 2:
    freq = sys.argv[2]


  WiFiTrilatSrv(sys.argv[1], freq)
