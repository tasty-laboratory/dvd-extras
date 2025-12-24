# Run this script to start the CV and MQTT scripts for the Hunter Drone
# Tested on Ubuntu 20.04 and 22.04
# Tested with Python 3.8, 3.10, Tensorrt 8.5, 10.3 and OpenCV 4.6, 4.10, 4.11
# Requires OpenCV with CUDA support, Refer to build_opencv.sh for building OpenCV with CUDA support
# Make sure that you have read the README.md and installed all dependencies

# To rebuild the engine file from the onnx file run from the folder containing onnx:
# trtexec --onnx=drone_cpu.onnx --saveEngine=detect_drone.engine --fp16 --minShapes=images:1x3x640x640 --optShapes=images:1x3x640x640 --maxShapes=images:1x3x640x640
source .venv/bin/activate
cd rookMQTT

#python3 run.py main_cv.db --manual 1-2_2-3_3-4_4-5_5-6_6-8_7-10 # ORIGINAL CV #
python3 run.py main_cv.db --manual 1-1_2-11_3-4_4-5_5-6_6-8_7-10 # YY CV #

###################### LEGACY ######################
#python3 run.py main.db ## Manual running of the script ##
#python3 run.py main_cv.db --manual 1-1_2-14_3-4_4-5_5-6_6-8_7-10 # NO CV #
#python3 run.py main_cv.db --manual 1-1_2-12_3-4_4-5_5-6_6-8_7-9 # XN CV #
#python3 run.py main_cv.db --manual 1-1_2-13_3-4_4-5_5-6_6-8_7-9 # Gab CV #
#python3 run.py main_cv.db --manual 1-1_2-15_3-4_4-5_5-6_6-8_7-9 # Rach CV #
