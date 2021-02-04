
echo "Binpicking: Setup your sources list"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
echo "Binpicking: Gettign key"
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo add-apt-repository ppa:eugenesan/ppa
echo "Binpicking: Updating"
sudo apt update
echo "Binpicking: Installing ros melodic desktop full"
sudo apt-get install ros-melodic-desktop-full -y
echo "Binpicking: Setting up setup.bash standard"
sudo echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo "Binpicking: Installing additional "
export ROS_VER=melodic 	
echo "Installing realsence ros"
sudo apt-get install ros-$ROS_VER-realsense2-camera -y
sudo apt-get install ros-$ROS_VER-realsense2-description -y
sudo apt-get install ros-melodic-catkin -y
sudo apt-get install python-catkin-tools -y
sudo apt-get install ros-melodic-moveit -y
sudo apt-get install ros-melodic-gazebo-ros-control -y
sudo apt-get install ros-melodic-ros-control -y
sudo apt-get install ros-melodic-ros-controllers -y
sudo apt-get install ros-$ROS_DISTRO-flexbe-behavior-engine -y

echo "getting camera fix"

cd /etc/udev/rules.d/
sudo wget https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
cd
cd BinPicking
cp 99_usbdevices.rules -t /etc/udev/rules.d/

echo "unziping master for linken lib"
cd src/binpicking
sudo wget https://github.com/JimEverest/magician_dobot/archive/master.zip
unzip -n master.zip

"echo installing rosdep"
sudo apt-get install python-pip
sudo pip install -U rosdep
sudo rosdep init
sudo rosdep update

echo "installing flexbe"
cd ~/BinPicking/src

git clone https://github.com/FlexBE/flexbe_app.git


cd ~/BinPicking
catkin build binpicking_vision
catkin build binpicking_vision
catkin build

sudo echo "source ~/BinPicking/devel/setup.bash" >> ~/.bashrc

reboot
#http://philserver.bplaced.net/fbe/download.php
