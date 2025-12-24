gnome-terminal --tab --title 'mqtt' -e "bash -c 'sudo ip a add dev wlp0s20f3 192.168.1.1/24
sudo systemctl stop mosquitto
sudo snap stop mosquitto
./setupmqtt.sh'" 
gnome-terminal --tab --title 'mavsdk' -e "bash -c 'cd RC3
sudo ./mavsdk_server_musl_x86_64 udpin://192.168.144.50:14540'"
