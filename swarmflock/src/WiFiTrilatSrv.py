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
    print "Interface: " + self.interface + "\nMAC Address: " + req.mac_address + "\nFrequency: " + str(self.freq)
    mRSSI = mrssi.MacRSSI(self.interface, req.mac_address, self.freq)
    mRSSI.run()
    rospy.sleep(1)

    #rssis = mRSSI.hist[:, 1]
    rssis = [x[0] for x in mRSSI.hist]
    #relevant = rssis[rssis != -100]
    relevant = [x for x in rssis if x != -100]

    if len(rssis) == 0:
      print "No packets received from " + req.mac_address
      return WiFiTrilatResponse(-1)
    elif len(relevant) == 0:
      print "Packets received from " + req.mac_address + ", but nothing 'relevant'"
      return WiFiTrilatResponse(-2)

    distance = wifiutils.calcDistance(sum(relevant) / len(relevant), self.freq)
    return WiFiTrilatResponse(distance)


  def __init__(self, interface, startAP=False, freq=2412):

    self.freq = freq
    self.interface = interface
    self.startAP = startAP
    robotName = os.getenv('HOSTNAME')

    # Start ROS integration
    rospy.init_node(robotName + "_wifitrilat_server")
    #robotName = rospy.get_param("/robot/name")
    self.service = rospy.Service("/" + robotName + "/WiFiTrilat", WiFiTrilat, self.handle_Trilat)

    if self.startAP:
      hotspot.main("start")

    rospy.on_shutdown(self.shutdown)    

    # [DEPRECATED] This will create the ad-hoc network
    # https://gist.github.com/meeuw/5765413

    rospy.spin()


  def shutdown(self):

    if self.startAP:
      hotspot.main("stop")


if __name__ == "__main__":

  startAP = False

  if len(sys.argv) > 3:
    startAP = sys.argv[3]


  WiFiTrilatSrv(sys.argv[1], startAP, sys.argv[2])
