# Object detection
## Install dependencies
### Install ROS (if you haven't already)
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
# ROS_DISTRO=melodic # ubuntu18
ROS_DISTRO=noetic # ubuntu20
sudo apt-get install ros-$ROS_DISTRO-desktop
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
source /opt/ros/$ROS_DISTRO/setup.bash
# sudo apt-get install catkin python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential # ubuntu18
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin # ubuntu20
sudo rosdep init
rosdep update
```

### Install dependencies and compile
```
git clone https://github.com/ea520/object-detection && cd object-detection
```
```bash
./install_dependencies.sh
rm ./install_dependencies.sh # you can now remove the file if the install was successful
```
```bash
source setupvars.bash 
catkin_make -DCMAKE_BUILD_TYPE=RELEASE
```

## Run the code

### If you have a camera, you can check the object detection/QR code viewer
```bash
source ./devel/setup.bash
WEIGHTS=./resources
rosrun object_detection detect2d --source 0 --target-fps 10 --conf-thres 0.8 --GPU --bin $WEIGHTS/best.bin --xml $WEIGHTS/best.xml --classes  $WEIGHTS/classes.txt --no-qr --output-path path/to/save/video
# "rosrun object_detection detect2d --help" will show the command line options
```

### If you have a couple Gb storage:
Download [this rosbag file](https://drive.google.com/drive/u/1/folders/1Y2u8pNS8XX3paCsEkHHC_YGhx59B44ql) and save it.
This is a 1-minute recording of the sensor streams

```bash
rosbag decompress recording.bag
```
```bash
roslaunch object_detection detect_objects.launch rosbag_path:=/full/path/to/recording.bag gpu:=True
```

### If you have access to the RealSense cameras:
```bash
roslaunch object_detection detect_objects.launch  gpu:=True #rviz:=False
# add the rviz:=False to just produce the output csv file and don't visualise it.
# positions in the file are relative to the tracking camera's coordinate system
# you can include the arguments to rs_d400_and_t265.launch here as well
```

With any luck, you'll see the following screen:

![Object detection visualization](resources/visualisation.png)
The contents of src/object_detection/output.csv:

![Output](resources/output.png)
