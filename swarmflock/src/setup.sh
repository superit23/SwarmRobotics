#!/bin/sh
pip install scapy
pip install netaddr
pip install python-wifi

apt-get install -y hostapd ros-indigo-multimaster-fkie aircrack-ng

iptables -A INPUT -p tcp --dport 11311 -j ACCEPT
