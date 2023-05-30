This is a class project involving a robot chess arm. The robot is programmed using ROS and Python, and uses the ROS packages rospy, std_msgs, kortex_driver, kortex_gazebo, cv_bridge, message_generation, usb_cam, image_view, and actionlib_msgs. The robot arm is a model from Kortex, and the ROS packages are used for control and interfacing with the robot.

## Prerequisites

- ROS Noetic: You must have ROS Noetic installed on your system. If you don't have it, you can install it from [here](http://wiki.ros.org/noetic/Installation).

- Python3: The project uses Python3 and pip for installing necessary libraries.

- Conan: Conan is a package manager for C++ and is required for installing ros_kortex. If not already installed, you can install it using pip.

- OpenCV: Used in the vision code 

- Numpy: Used for various calculations

## Installation

This is the installation for ros_kortex, however in the given zip file it is already installed. But if you run into any issues this may help.

```bash
sudo apt install python3 python3-pip
sudo python3 -m pip install conan==1.59
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default
mkdir -p catkin_workspace/src
cd catkin_workspace/src
git clone -b devel-noetic https://github.com/Kinovarobotics/ros_kortex.git
cd ../
rosdep install --from-paths src --ignore-src -y
```
These commands will install Python3 and pip, configure Conan, clone the ros_kortex repository, and install the necessary ROS packages using rosdep.

## Running the Program
To run the program, you need to launch the chess_arm.launch file and the kinova_chess_control.py script in separate terminals.

### Terminal 1:
Navigate to your catkin workspace and run:

```
source devel/setup.bash
roslaunch kinova_5551_chess_arm chess_arm.launch
```
### Terminal 2:
In a new terminal, navigate to the scripts directory and run the python control script:

```
cd ~/kinova_5551_chess_arm/src/scripts/
source ../../devel/setup.bash
python kinova_chess_control.py __ns:=my_gen3_lite
```
Here, my_gen3_lite is the namespace of your robot. Make sure to replace my_gen3_lite with the appropriate namespace for your robot.