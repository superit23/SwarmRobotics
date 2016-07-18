#!/bin/sh
pip install scapy
pip install netaddr
pip install python-wifi

apt-get install -y hostapd ros-indigo-multimaster-fkie

iptables INPUT A -p tcp --dport 11311 -j ACCEPT
