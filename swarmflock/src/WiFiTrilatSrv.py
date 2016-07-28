#!/usr/bin/python

import rospy
#from swarmflock.msg import WiFiRSSIMsg
from swarmflock.srv import *
import wifiutils, statutils
#import hotspot
import sys, cli
import os, time, math
from scapy.all import sniff, Dot11, get_if_hwaddr, IP, ICMP, sr
from pythonwifi.iwlibs import Wireless, Iwrange
from itertools import groupby
from operator import itemgetter
from WiFiTrilatClient import WiFiTrilatClient


class WiFiTrilatSrv:

  def handler(self, packet):

    if packet.haslayer(Dot11):
      # Check to make sure this is a management frame (type=0) and that
      # the subtype is one of our management frame subtypes indicating a
      # a wireless client
      #if packet.addr2 and packet.addr2.lower() == self.mac:

      if packet.addr2:
        rssi = self.siglevel(packet)# if self.siglevel(packet)!=-256 else -100
        self.msgs.append((packet.addr2.lower(), rssi, time.time()))

        #if "7c:e9:d3" in packet.addr3.lower():
        #  rospy.loginfo("ADDR3: %s > %s ? %s" % (packet.addr2, packet.addr1, packet.addr3))
        #elif "7c:e9:d3" in packet.addr2.lower():
        #  rospy.loginfo("ADDR2: %s > %s" % (packet.addr2, packet.addr2))



  def msgPurge(self, event):
    #rospy.loginfo("Purging messages. Currently have " + str(len(self.msgs)))

    now = time.time()


    self.msgs = [x for x in self.msgs if x[2] + self.tolerance > now]
    self.msgs = sorted(self.msgs, key=itemgetter(2))

    rospy.loginfo("Messages purged! Now have " + str(len(self.msgs)))




  def handle_Trilat(self, req):
    rospy.loginfo("Handling distance request for %s" % req.mac_address)
    msgs = [x for x in self.msgs if x[0] == req.mac_address and math.fabs(req.time - x[2]) <= req.tolerance and x[1] > -230]    

    numElem = len(msgs)

    if numElem > 0:
      sigs = [msg[1] for msg in msgs]

      mad = statutils.mad(sigs)
      sigs = statutils.remOutliers(sigs, mad)

      avgSig = sum(sigs) / len(sigs)
      distance = wifiutils.calcDistance(avgSig, self.freq)

      times = [msg[2] for msg in msgs]
      avgTime = sum(times) / numElem

      return WiFiTrilatResponse(self.robotName, distance, avgTime, self.x, self.y)
    else:
      return WiFiTrilatResponse(self.robotName, -1, -1, self.x, self.y)



  # This function will attempt to find this server's position relative to the other servers.
  def findSelfPos(self, event):
    mac = get_if_hwaddr(self.interface)
    servers = self.client.discover()

    while(len(servers) < 3):
      rospy.loginfo("Found only " + str(len(servers)) + " servers!")
      return

    otherServers = [x for x in servers if x.find(self.robotName) == -1]

    # This will call the other servers' WiFiTrilatServices. To do this, we enter our own MAC address,
    # then take two servers that are NOT ours.
    responses = self.client.getDistances(mac, otherServers, time.time(), 50)
    #goodServers = [x.srv_name for x in responses if x.distance != -1]
    goodResponses = [x for x in responses if x.distance != -1]

    for x in responses:
      print x.srv_name
      print x.distance
    goodServers = [x.srv_name for x in goodResponses]


    if len(goodServers) < 2:
      rospy.loginfo("Not enough servers!")
      return

    srvToUse = []

    for x in goodServers:
      srvToUse.append(x)

    srvToUse.append(self.robotName)
    srvToUse = sorted(srvToUse)

    #serverNames = sorted([s[1:s.find('/WiFi')] for s in servers])
    index = srvToUse.index(self.robotName)

    # Next we need to find the distance between the two other servers

    print "Host 0 is " + goodServers[0]
    print "Host 1 is " + goodServers[1]
    goodResponses.append(self.client.getDistances(self.client.IPtoMAC(self.client.hostToIP(goodServers[0])), ["/" + goodServers[1] + "/WiFiTrilat"])[0])

    # We translate the indices, so each server will end up building the same triangle.
    indexTrans = [(0, 2, 1), (0, 1, 2), (2, 1, 0)]
    myInd = indexTrans[index]

    # We take our relative position based on alphabetical order.
    try:
      [self.x, self.y] = wifiutils.calcFrameOfRef(goodResponses[myInd[0]].distance, goodResponses[myInd[1]].distance, goodResponses[myInd[2]].distance)[index]
    except ValueError as e:
      rospy.logwarn(str(e))
      return

    print self.x
    print self.y

    if self.discoverOnce:
      rospy.loginfo("Position resolved; shutting down heartbeat/findpos")
      self.discoverTimer.shutdown()
      self.heartbeat.shutdown()



  def __init__(self, listenInt, interface, freq, essid, psswd, ip, nm, discoverOnce=True):
    self.robotName = os.getenv('HOSTNAME')
    self.tolerance = 20
    self.x = 0
    self.y = 0
    self.client = WiFiTrilatClient()

    self.discoverOnce = discoverOnce

    rospy.init_node(self.robotName + "_wifitrilat_server")

    #self.rssiPub = rospy.Publisher('/' + self.robotName + '/WiFiRSSI', WiFiRSSIMsg, queue_size=10)
    self.service = rospy.Service("/" + self.robotName + "/WiFiTrilat", WiFiTrilat, self.handle_Trilat)

    self.listenInt = listenInt
    self.interface = interface
    #self.mac = mac.lower()
    self.freq = int(freq)

    self.msgs = []

    cli.execute_shell("ifconfig %s down" % self.listenInt)
    #self.wifi = Wireless(self.interface).setFrequency("%.3f" % (float(self.freq) / 1000))
    self.connectToNet(essid, psswd,ip, nm)
    cli.execute_shell("ifconfig %s up" % self.listenInt)

    self.purge = rospy.Timer(rospy.Duration(2), self.msgPurge)
    self.heartbeat = rospy.Timer(rospy.Duration(1), self.heartbeat_call)
    self.discoverTimer = rospy.Timer(rospy.Duration(20), self.findSelfPos)


    sniff(iface=self.listenInt, prn=self.handler, store=0)
    rospy.spin()




  def heartbeat_call(self, event):
    # Send a packet to every WiFiTrilat server.
    for host in [s[1:s.find('/WiFi')] for s in self.client.discover()]:
      #sr(IP(dst=host)/ICMP(), iface=self.interface)
      cli.execute(command='ping -c 4 %s' % host, wait=False, shellexec=True)


  def connectToNet(self, essid, psswd, ip, nm):
    rospy.loginfo("Attempting to manually connect...")
    cli.execute_shell("pkill -f wpa_supplicant")
    cli.execute_shell('wpa_passphrase %s \"%s\" > ~/wifitrilat.conf' % (essid, psswd))
    cli.execute_shell("wpa_supplicant -B -i %s -c ~/wifitrilat.conf " % self.interface)
    cli.execute_shell("ifconfig %s %s netmask %s" % (self.interface, ip, nm))




  def siglevel(self, packet):
    try:
      return -(256-ord(packet.notdecoded[-4:-3]))
    except TypeError as e:
      #rospy.logwarn(str(e))
      return -256


if __name__ == "__main__":
  WiFiTrilatSrv(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7])
