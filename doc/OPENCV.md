# Installing OpenCV

Instructions found at: [Raspbian Stretch: Install OpenCV 3 + Python on your Raspberry Pi](https://www.pyimagesearch.com/2017/09/04/raspbian-stretch-install-opencv-3-python-on-your-raspberry-pi/)

Supplementary instructions found at: [OpenCV in Ubuntu 17.04](https://stackoverflow.com/questions/43484357/opencv-in-ubuntu-17-04/44488374#44488374)

OpenCV is now installed during the ROS installation. Thus these instructions do not need to be followed anymore.

## Instructions

Make sure everything is up to date
```
$ sudo apt-get update && sudo apt-get upgrade
```

check you have all the developer tools you need installed.
```
$ sudo apt-get install build-essential cmake pkg-config
```

Tell the pi to look into custom repositories (if you dont do this you cant find the packages you need)
```
echo "deb http://us.archive.ubuntu.com/ubuntu/ yakkety universe" | sudo tee -a /etc/apt/sources.list
```

Install image I/O packages that allow loading of various image file formats from disk.
```
$ sudo apt-get install build-essential cmake pkg-config
```

Install video I/O packages that allow loading of various video file formats from disk.
```
$ sudo apt-get install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
$ sudo apt-get install libxvidcore-dev libx264-dev
```

Install the modules needed to display images and build opencv guis.
```
$ sudo apt-get install libgtk2.0-dev libgtk-3-dev
```

Install matrix operation dependencies used by openCV
```
$ sudo apt-get install libatlas-base-dev gfortran
```

Make sure you have python installed
```
$ sudo apt-get install python2.7-dev python3-dev
```

Download and extract opencv
```
$ cd ~
$ wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.3.0.zip
$ unzip opencv.zip
```

Download and extract the extra featurers OpenCV needs for a full instalation
```
$ wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/3.3.0.zip
$ unzip opencv_contrib.zip
```


Install pythons package manager PIP
```
$ wget https://bootstrap.pypa.io/get-pip.py
$ sudo python get-pip.py
$ sudo python3 get-pip.py
```

Install numpy
```
$ pip install numpy
```

Setup the build for OpenCV
```
$ cd ~/opencv-3.3.0/
$ mkdir build
$ cd build
$ cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.3.0/modules \
    -D BUILD_EXAMPLES=ON ..
```

Confirm that you have an Interperter, library, numpy and package paths for both python 2 and 3. You will see this on the output of the previous command and it should look something like:
```
--   Python 2:
--     Interpreter:                 /usr/bin/python2.7 (ver 2.7.13)
--     Libraries:                   /usr/lib/arm-linux-gnueabihf/libpython2.7.so (ver 2.7.13)
--     numpy:                       /usr/lib/python2.7/dist-packages/numpy/core/include (ver 1.12.1)
--     packages path:               lib/python2.7/dist-packages
--
--   Python 3:
--     Interpreter:                 /usr/bin/python3 (ver 3.5.3)
--     Libraries:                   /usr/lib/arm-linux-gnueabihf/libpython3.5m.so (ver 3.5.3)
--     numpy:                       /usr/lib/python3/dist-packages/numpy/core/include (ver 1.12.1)
--     packages path:               lib/python3.5/dist-packages
--
--   Python (for build):            /usr/bin/python2.7
```

Compile OpenCV. We are only going to use 2 cores, this is slower, but it means we dont have to mess with the swap memory on the PI (Go and make yourself a coffee, or 10)
```
$ make -j2
```

Install OpenCV (Repeat the coffee getting step).
```
$ sudo make install
$ sudo ldconfig
```

Check Python is Installed (Each should have roughly arround a total of 3800)
```
$ ls -l /usr/local/lib/python2.7/dist-packages/
$ ls -l /usr/local/lib/python3.5/dist-packages/
```

A final test to check if python is working would be to import the library.
```
$ python3
>>> import cv2
```

If you got no errors you are done.







