import rospy
import dbus
from swarmflock.srv import *
import wifiutils
import hotspot

class WiFiTrilatSrv:

  def handle_Trilat(self, req):
    distance = wifiutils.calcDistance()
    return WiFiTrilatResponse(30)


  def __init__(self, robotName):

    # Start ROS integration
    rospy.init_node("wifitrilat_server")
    self.service = rospy.Service("WiFiTrilat", WiFiTrilat, self.handle_Trilat)

    self.freq = 2412
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
  WiFiTrilatSrv("test")
