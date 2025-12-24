# Run this script to initialize the Hunter Drone through MAVSDK and MAVProxy
# Tested on Ubuntu 20.04 and 22.04
# Make sure that the drone is powered on and connected to the same network as your computer [Through HM30]

gnome-terminal --tab --title="mavproxy" -- bash -c 'mavproxy.py --master udpin:192.168.144.50:14542 --out udpout:192.168.144.50:14540 --out udp:192.168.144.50:14543'
gnome-terminal --tab --title="mavsdk" -- bash -c 'cd ~/RC3
sudo ./mavsdk_server_musl_x86_64 udpin://192.168.144.50:14543 -p 50060'
