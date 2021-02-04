#!/bin/bash
source /opt/ros/melodic/setup.bash

echo "Build"
cd ~/BinPicking 
catkin build
cd ..

cd flexbe_app
catkin build
cd ..

echo "Source ros setup"


source ~/BinPicking/devel/setup.bash
source ~/flexbe_app/devel/setup.bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/BinPicking/devel/setup.bash" >> ~/.bashrc
echo "source ~/flexbe/devel/setup.bash" >> ~/.bashrc



