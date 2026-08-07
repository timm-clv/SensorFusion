#!/bin/bash


# Has been lunched one time at the begining
# Better to not lunch it again


# script for configuring a persistent static IP 


# Checking root privileges
if [[ $EUID -ne 0 ]]; then
   echo "[ERROR] This script must be run as root (sudo)"
   exit 1
fi

echo "-> Disabling Reverse Path Filtering for Asymmetric DDS Routing..."
sysctl -w net.ipv4.conf.all.rp_filter=2
sysctl -w net.ipv4.conf.default.rp_filter=2
sysctl -w net.ipv4.conf.enx_olive_cam.rp_filter=2
sysctl -w net.ipv4.conf.enx_olive_imu.rp_filter=2

echo "-> optimization of receive and transmit buffers (FastDDS / Video Stream)..."
sysctl -w net.core.rmem_max=8388608
sysctl -w net.core.rmem_default=8388608
sysctl -w net.core.wmem_max=8388608
sysctl -w net.core.wmem_default=8388608

# Interface Configuration 
IF_CAMERA="enx_olive_cam" #"enx0a575b00dfce"
IF_IMU1="enx_olive_imu" #"enxa21011e2ae4e"
# Maybe in the futur
#IF_IMU2="enx_olive_imu2"
#IF_IMU3="enx_olive_imu3"

# static IP addresses assigned to the Jetson on these subnets
IP_CAMERA="192.168.7.101/24"
IP_IMU1="192.168.8.102/24"
#IP_IMU2="192.168.7.103/24"
#IP_IMU3="192.168.7.104/24"



echo "Starting the network configuration for the sensor platform..."

configure_interface() {
    local iface=$1
    local ip=$2
    local con_name="Olive_${iface}"

    echo "-> Configuration of $iface with IP $ip..."

    #  Delete the old connection, if it exists, to avoid conflicts
    if nmcli con show "$con_name" >/dev/null 2>&1; then
        nmcli con delete "$con_name" >/dev/null
    fi

    # Create the new static connection
    nmcli con add type ethernet ifname "$iface" con-name "$con_name" ipv4.method manual ipv4.addresses "$ip" >/dev/null

    # Disable IPv6 (not needed for local DDS traffic from sensors)
    nmcli con modify "$con_name" ipv6.method disabled

    #Apply and activate the connection
    nmcli con up "$con_name" >/dev/null
    
    ip link set dev "$iface" mtu 8000

    echo "   [OK] $iface configured ($con_name)."
}

echo "-> Adding multicast routes (ROS 2) for subnets..."
ip route add 224.0.0.0/4 dev enx_olive_cam metric 100
ip route add 224.0.0.0/4 dev enx_olive_imu metric 101

# Application on sensors
configure_interface $IF_CAMERA $IP_CAMERA
configure_interface $IF_IMU1 $IP_IMU1
#configure_interface $IF_IMU2 $IP_IMU2
#configure_interface $IF_IMU3 $IP_IMU3

echo "=================================================="
echo "Verification:"
ip -4 addr show $IF_CAMERA
ip -4 addr show $IF_IMU1
#ip -4 addr show $IF_IMU2
#ip -4 addr show $IF_IMU3
echo "=================================================="
