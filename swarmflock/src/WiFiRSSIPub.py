#!/usr/bin/python

import rospy
#from swarmflock.msg import WiFiRSSIMsg
from swarmflock.srv import *
import wifiutils
import hotspot
import sys
import os, time
from scapy.all import sniff, Dot11
from pythonwifi.iwlibs import Wireless, Iwrange
from netaddr import OUI
from itertools import groupby

class WiFiRSSIPub:

  def handler(self, packet):
    if packet.haslayer(Dot11):
      # Check to make sure this is a management frame (type=0) and that
      # the subtype is one of our management frame subtypes indicating a
      # a wireless client
      #if packet.addr2 and packet.addr2.lower() == self.mac:
      if packet.addr2:
        rssi = self.siglevel(packet)# if self.siglevel(packet)!=-256 else -100
        self.msgs.append((packet.addr2.lower(), rssi))

        #if packet.addr2.lower() == "7c:e9:d3:f5:c4:e5":
        #  print "RECEIVED"
        #rospy.spinOnce()



  def patience_call(self, event):
    self.distances = []
    self.msgs.sort()
    byMAC = groupby(self.msgs, lambda x: x[0])

    for key, group in byMAC:
      mac = key
      rssis = [p[1] for p in group if p[1] > -230]

      if len(rssis) == 0:
        continue

      avg = sum(rssis) / len(rssis)
      self.distances.append((mac, wifiutils.calcDistance(avg, self.freq), time.time()))

      #rssiMsg = WiFiRSSIMsg()
      #rssiMsg.mac_address = mac
      #rssiMsg.distance = wifiutils.calcDistance(avg, self.freq)
      #self.rssiPub.publish(rssiMsg)


    self.msgs = []


  def handle_Trilat(self, req):
    distances = [x for x in self.distances if x[0] == req.mac_address]

    if len(distances) > 0:
      return WiFiTrilatResponse(distances[0][1], distances[0][2])
    else:
      return WiFiTrilatResponse(-1, -1)


  def __init__(self, interface, freq):
    self.robotName = os.getenv('HOSTNAME')

    rospy.init_node(self.robotName + "_wifiRSSIPub")

    #self.rssiPub = rospy.Publisher('/' + self.robotName + '/WiFiRSSI', WiFiRSSIMsg, queue_size=10)
    self.service = rospy.Service("/" + self.robotName + "/WiFiTrilat", WiFiTrilat, self.handle_Trilat)

    self.interface = interface
    #self.mac = mac.lower()
    self.freq = int(freq)

    self.msgs = []
    self.distances = []

    self.wifi = Wireless(self.interface).setFrequency("%.3f" % (float(self.freq) / 1000))

    self.patience = rospy.Timer(rospy.Duration(2), self.patience_call)

    sniff(iface=self.interface, prn=self.handler, store=0)
    rospy.spin()


  def siglevel(self, packet):
    return -(256-ord(packet.notdecoded[-4:-3]))


if __name__ == "__main__":
  WiFiRSSIPub(sys.argv[1], sys.argv[2])
