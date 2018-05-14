# Custom software for testing the Raspberry Pi Cluster

## About

The following are programs used for testing the Raspberry Pi. All programs output a CPU and wall clock time. The CPU time is the amount of time the CPU physically spent processing the data. The wall clock time is the amount of time taken for the program to run. Three seperate tests were created for testing the software. A description for each can be found below.

## Programs

* [Image Processing](https://github.com/hildebrandt-carl/cluster_hat/tree/master/src/image_prcocessing/) – This is a stand alone C++ program which does numerous image processing techniques on an image.
* [Image Processing ROS](https://github.com/hildebrandt-carl/cluster_hat/tree/master/src/image_prcocessing_ros/) – This implements the same image processing techniques used in the image processing program inside a ROS environment. The program is developed in such a way that the original image is split up onto mutliple processor nodes.
* [Speed Test](https://github.com/hildebrandt-carl/cluster_hat/tree/master/src/speed_test/) – This is a program which uses the same ROS architecture as the image processing ROS implementation. However instread of doing image processing tasks, this one does many floating point operations. 

## Authors

* **Carl Hildebrandt** - *Initial work* - [hildebrandt.carl](https://github.com/hildebrandt-carl)
* **William Turner** - *Initial work* - [william.turner](https://git.unl.edu/william.turner)
* **Molly Lee** - *Group Member*
* **Casey Lafferty** - *Group Member*


