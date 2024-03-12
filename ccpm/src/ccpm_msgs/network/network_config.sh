#!/bin/bash

# Set the IP address of the ethernet port to 

#ifconfig eth0 down
#ifconfig eth0 192.168.4.12
#ifconfig eth0 up
rfkill unblock all

# Set the IP address of the hotspot port to
#iw phy phy0 interface add hotspot type __ap
#rfkill unblock all
#sleep 1
#ifconfig hotspot 192.168.12.1 up


#Add dns server
echo "nameserver 8.8.8.8" >> /etc/resolv.conf
