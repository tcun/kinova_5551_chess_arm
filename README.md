# 5551_project

## Install ros_kinova

```bash
sudo apt install python3 python3-pip
sudo python3 -m pip install conan==1.59
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default
mkdir -p catkin_workspace/src
cd catkin_workspace/src
git clone -b <branch-name> https://github.com/Kinovarobotics/ros_kortex.git
cd ../
rosdep install --from-paths src --ignore-src -y
```

## Helpful Commands
```bash
catkin_make
source devel/setup.bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -y
roslaunch kinova_5551_chess_arm aruco_detect.launch
rostopic list
rospack list
```
