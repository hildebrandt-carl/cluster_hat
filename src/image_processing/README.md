# Image Processing Test

## About
This is a c++ implementation of an image processing test . It takes an image and does Canny edge detection on it. It then takes the orignal image and trys to create another Canny image with less detail. To do this it does watershed segmentation, which relys on a seris of other image processing techniques such as errosion, binary thresholding and segmentation.

## Prerequisits

The following packages are required to run this program. Instructions how to obtain them bellow in the installation instructions:
* OpenCV


## Installation

One can compile the program using the following commands:
```
$ make
```

You can then run the program using:
```
$ ./main "image.jpg"
```

Where image is the name of an image inside the same folder as your main program.

### Installing OPENCV
These instructions have been modified from the [Installing opencv](https://askubuntu.com/questions/334158/installing-opencv/647791) question on stack overflow.

Install the dependencies
```
$ sudo apt-get install build-essential
$ sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
$ sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
```

Install opencv.
```
sudo apt-get install libopencv-dev
```

## Authors

* **Carl Hildebrandt** - *Initial work* - [hildebrandt.carl](https://github.com/hildebrandt-carl)
* **William Turner** - *Initial work* - [william.turner](https://git.unl.edu/william.turner)
* **Molly Lee** - *Group Member*
* **Casey Lafferty** - *Group Member*