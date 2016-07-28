#!/usr/bin/python
import cli, os
import socket, re, time
import rospy
from swarmflock.srv import *
import wifiutils

class WiFiTrilatClient:

  def __init__(self):
    return


  def discover(self):
    servers = [x for x in cli.execute_shell('rosservice list | grep WiFiTrilat').split('\n') if x != '']
    return servers


  def hostToIP(self, host):
    return repr(socket.gethostbyname(host))


  def IPtoMAC(self, IP):
    arpCache = cli.execute_shell('arp -n ' + IP)
    mac = re.search(r"(([a-f\d]{1,2}\:){5}[a-f\d]{1,2})", arpCache).groups()[0]
    return mac


  def getDistances(self, mac, servers, timeframe=-1, tolerance=5):
    for service in servers:
      print service

    if timeframe == -1:
      timeframe = time.time()

    services = [rospy.ServiceProxy(service, WiFiTrilat) for service in servers]

    return [service(mac, timeframe, tolerance) for service in services]



  def trilaterate(self, mac, servers, timeframe=-1, tolerance=5):
    responses = self.getDistances(mac, servers[:3], timeframe, tolerance)
    return wifiutils.trilaterate([responses[0].x, responses[0].y], responses[0].distance, [responses[1].x, responses[1].y], responses[1].distance, [responses[2].x, responses[2].y], responses[2].distance)
