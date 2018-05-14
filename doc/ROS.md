# Installing ROS

Instructions found at: [Installing ROS Raspberry Pi 3 with Strech](http://www.venelinpetkov.com/how-to-install-ros-robot-operating-system-on-raspberry-pi-3-with-raspbian-stretch/)

## Instructions

Install a missing certificate management service.
```
$ sudo apt-get install dirmngr
```

Add the ROS repository
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Add the public key
```
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

Update the PI
```
$ sudo apt-get update
$ sudo apt-get upgrade
```

Install the bootstrap dependencies
```
$ sudo apt-get install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential
```

Initilialize the rosdep package manager. **Do Not Use SUDO**
```
$ sudo rosdep init
$ rosdep update
```

Create your catkin workspace
```
$ mkdir ~/catkin_ws
$ cd ~/catkin_ws
```

Install ROS
```
$ rosinstall_generator perception --rosdistro kinetic --deps --wet-only --tar > kinetic-perception-wet.rosinstall
```

Download the ROS source code (Use the second one)
```
$ wstool init -j8 src kinetic-perception-wet.rosinstall
```

**If the download fails**, it can be resumed with
```
$ wstool update -j4 -t src
```

Check that the src folder exists
```
ls ~/catkin_ws/src
```

Check if all the dependencies have been installed, if not install them.
```
$ rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
```

Make your user the owner of the ros folder.
```
$ sudo mkdir -p /opt/ros/kinetic
$ sudo chown pi:pi /opt/ros/kinetic
```

We need to change the swap memory to allow more memory. To do this open up the file which declares the swap memory
```
$ sudo nano /etc/dphys-swapfile
```

Find the line which states how much memory you have and change it to 1024MB
```
CONF_SWAPSIZE=1024
```

Stop and start the service which manages the swap files.
```
$ sudo /etc/init.d/dphys-swapfile stop
$ sudo /etc/init.d/dphys-swapfile start
```

Check to make sure you have updated the swap memory to 1024MB
```
$ free -m
```

Build ROS. If it fails restart the pi and rerun the command. (Go enjoy your life -+ 6 hours)
```
$ ./src/catkin/bin/catkin_make_isolated -j1 --install --install-space /opt/ros/kinetic -DCMAKE_BUILD_TYPE=Release
```

Source the installation.
```
$ source /opt/ros/kinetic/setup.bash
```

Add the setup to your bash file
```
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
```

Test that you can launch Ros
```
$ roscore
```

You can then compile Ros
```
$ catkin_make_isolated -j1
```

We need to change the swap memory, so that it doesnt burn out SD card.
```
$ sudo nano /etc/dphys-swapfile
```

Find the line which states how much memory you have and change it to 100MB
```
CONF_SWAPSIZE=100
```

Stop and start the service which manages the swap files.
```
$ sudo /etc/init.d/dphys-swapfile stop
$ sudo /etc/init.d/dphys-swapfile start
```

Check to make sure you have updated the swap memory to 1024MB
```
$ free -m
```

Create a new catkin workspace
```
$ mkdir ~/catkin_ws2/src
$ cd ~/catkin_ws2/src
$ git clone <clustrhat>
```

Clone ClusterHat into the src folder and run catkin_make
```
$ cd ~/catkin_ws2
$ catkin_make -j1
```

