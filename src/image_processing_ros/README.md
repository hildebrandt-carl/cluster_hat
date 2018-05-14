# ROS Distributed Image Processing Test

## About
This is a ROS implementation of robotic friendly image processing. The package creates a controller node, which takes a large image and distributes it to multiple processing nodes. The multiple nodes run the image processing operations in parallel. After each of the iamges are processed, the processing node returns the resultant image to the controller. Once a number of images have been processed the program saves the processed images and terminates.

The images are sent back and forth using cv bridge and image transport. The tutorials used are the ROS turtorial on [publishing images]{http://wiki.ros.org/image_transport/Tutorials/PublishingImages} and [subscribing to images]{http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages}. 

## Running the program

First compile the program using:
```
$ catkin_make
```

Then run the launch file
```
$ roslaunch image_processing test.launch
```

## Controller Params

* **total_processors:** This tells the program how many processor nodes you are going to launch. You need to launch 1 more than what you type. Thus if you launch 16 nodes, you need to launch from node 0 - 16 (17 nodes in total). It should also be noted this node has to be square rootable.
* **save_dir:** This is where the final processed images will be saved. It nees to be a full path.
* **send_delay:** The program bugs out if images are sent to quickly. To solve this, this delay was added.
* **namespace:** This should not be changed by the user, it is automatically set to use the default environment path.
* **image_name:** This describes which image should be used for processing. If it is not of the correct dimensions it will crash the program. (Correct dimensions in this case mean perfectly divisible by the square root of the **total_processors**).

## Authors

* **Carl Hildebrandt** - *Initial work* - [hildebrandt.carl](https://github.com/hildebrandt-carl)
* **William Turner** - *Initial work* - [william.turner](https://git.unl.edu/william.turner)
* **Molly Lee** - *Group Member*
* **Casey Lafferty** - *Group Member*
