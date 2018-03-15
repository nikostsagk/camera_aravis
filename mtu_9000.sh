#!/bin/bash
ip link set dev eth2 mtu 9000
ip link set dev eth3 mtu 9000
ip link set dev eth5 mtu 9000
echo "Ran MTU script"
