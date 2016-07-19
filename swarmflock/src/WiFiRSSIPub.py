#!/usr/bin/python

import rospy
from swarmflock.msg import WiFiRSSIMsg
import wifiutils
import hotspot
import sys
import os
from scapy.all import sniff, Dot11
from pythonwifi.iwlibs import Wireless, Iwrange
from netaddr import OUI

class WiFiRSSIPub:

  def handler(self, packet):
    if packet.haslayer(Dot11):
      # Check to make sure this is a management frame (type=0) and that
      # the subtype is one of our management frame subtypes indicating a
      # a wireless client
      #if packet.addr2 and packet.addr2.lower() == self.mac:
      rssi = self.siglevel(packet)# if self.siglevel(packet)!=-256 else -100
      now = time.time()
      self.msgs.append((packet.addr2.lower(), now, rssi))

      if rssi != -256:
        rssiMsg = WiFiRSSIMsg()
        rssiMsg.mac_address = packet.addr2.lower()
        rssiMsg.distance = wifiutils.calcDistance(rssi, self.freq)

        self.rssiPub.publish(rssiMsg)
        rospy.spinOnce()



  def patience_call(self):
    self.msgs = []


  def __init__(self, interface, mac, freq):
    self.robotName = os.getenv('HOSTNAME')
    self.rssiPub = rospy.Publisher('/' + self.robotName + '/WiFiRSSI', WiFiRSSIMsg, queue_size=10)

    self.interface = interface
    self.mac = mac.lower()
    self.freq = int(freq)

    self.msgs = []
    self.wifi = Wireless(self.interface).setFrequency("%.3f" % (float(self.freq) / 1000))

    self.patience = rospy.Timer(rospy.Rate(2), self.patience_call)

    sniff(iface=self.interface, prn=self.handler, store=0)
    rospy.spin()


  def siglevel(self, packet):
    return -(256-ord(packet.notdecoded[-4:-3]))

