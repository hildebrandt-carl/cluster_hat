# ROS Speed Test

## About
This project allows for testing of a system through the use of ROS. The package creates multiple nodes which all run expensive operations. After each operation is complete, the node sends a message to a controller node. The controller node then adds up how many messages it has received. Once it has received a certain number of messages it terminates the program.

## Running the program

First compile the program using:
```
$ catkin_make
```

Then run the launch file
```
$ roslaunch speed_test test.launch
```

## Authors

* **Carl Hildebrandt** - *Initial work* - [hildebrandt.carl](https://git.unl.edu/hildebrandt.carl)
* **William Turner** - *Initial work* - [william.turner](https://git.unl.edu/william.turner)
* **Molly Lee** - *Initial work* - [TODO](NA)
* **Casey Lafferty** - *Initial work* - [TODO](NA)