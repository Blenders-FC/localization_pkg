#!/bin/bash
network_name=$(iwconfig wlan0 | grep 'ESSID:' | awk -F'ESSID:"' '{print $2}' | awk -F'"' '{print $1}')
echo $network_name
nmcli con mod $network_name ipv4.addresses 192.168.13.3/24
#nmcli con mod $network_name ipv4.gateway 192.168.0.1
nmcli con mod $network_name ipv4.dns "8.8.8.8 1.1.1.1"
nmcli con mod $network_name ipv4.method manual
