import rospy
import dbus
from swarmflock.srv import *
import wifiutils
import hotspot
import mrssi
import sys

class WiFiTrilatSrv:

  def handle_Trilat(self, req):
    mRSSI = mrssi.MacRSSI("wlan0", req.mac_address, self.freq)
    mRSSI.run()
    rospy.sleep(1)

    rssis = mRSSI.hist[:, 1]

    distance = wifiutils.calcDistance(sum(rssis[rssis != -100]), self.freq)
    return WiFiTrilatResponse(distance)


  def __init__(self, interface, freq):

    self.freq = freq
    self.interface = interface

    # Start ROS integration
    rospy.init_node("wifitrilat_server")
    self.service = rospy.Service("WiFiTrilat", WiFiTrilatRequest, self.handle_Trilat)

    hotspot.main("start")

    rospy.on_shutdown(self.shutdown)    

    # This will create the ad-hoc network
    # https://gist.github.com/meeuw/5765413

    #bus = dbus.SystemBus()
    #wpas_obj = bus.get_object("fi.w1.wpa_supplicant1",
    #                      "/fi/w1/wpa_supplicant1")

    #wpas = dbus.Interface(wpas_obj, "fi.w1.wpa_supplicant1")
    #path = wpas.CreateInterface({'Ifname':"swarm_wlan0"})
    #if_obj = bus.get_object("fi.w1.wpa_supplicant1", path)

    #path = if_obj.AddNetwork({
    #   'ssid':robotName + "-adhoc",
    #   'mode':1,
    #   'frequency':self.freq,
    #   'proto':"WPA",
    #   'key_mgmt':"NONE",
    #   'pairwise':"NONE",
    #   'group':'TKIP',
    #   'psk':"verysecret123",
    #}, dbus_interface="fi.w1.wpa_supplicant1.Interface")

    #self.network = bus.get_object("fi.w1.wpa_supplicant1", path)
    #self.network.Set("fi.w1.wpa_supplicant1.Network", "Enabled", True, dbus_interface=dbus.PROPERTIES_IFACE)
    #print self.network.GetAll("fi.w1.wpa_supplicant1.Network", dbus_interface=dbus.PROPERTIES_IFACE)

    rospy.spin()


   def shutdown(self):
     hotspot.main("stop")


if __name__ == "__main__":
  WiFiTrilatSrv(sys.argv[1], sys.argv[2])
