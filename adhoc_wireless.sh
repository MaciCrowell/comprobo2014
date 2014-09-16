#!/bin/bash

IPSUFFIX=`echo  $1 | cut -d'.' -f 4`
IPSUFFIX=`expr $IPSUFFIX - 100`

echo "Starting up adhoc wireless"
echo "Make sure to use sudo when executing"
echo ""

stop network-manager
ifconfig wlan0 down
sleep 2
iwconfig wlan0 mode ad-hoc
iwconfig wlan0 channel 1
iwconfig wlan0 key aaaaa11111
iwconfig wlan0 essid RPi
ifconfig wlan0 192.168.17.$IPSUFFIX netmask 255.255.255.0 up

ping $1
