# Cluster Computing Performance Testing Using Raspberry Pi's

## About
This project contains the work used in putting together a [ClusterHat](https://clusterhat.com/). The [ClusterHat](https://clusterhat.com/) combines 4 Raspberry Pi 0's such that they can all communicate. This we used as a representation of a cluster computer made up of multiple cores. Benchmarks and real-life applications were then run using this cluster. 

The cluster was setup instructions can be found here: [Setup](https://clusterhat.com/setup-overview)

## Project Plan

* Run benchmarking tests found at: http://climbers.net/sbc/clusterhat-review-raspberry-pi-zero/
* We hopefully will find that the pi 0 is the slowest, followed by 4 pi zeros, and the pi 3 is the fastest.
* The second stage will be to create a ROS node which we can duplicate many times. This will be an intermediate step between benchmarking and real-life applications. 
* The ROS node will then be duplicated many times and run, to see how long it takes. They will be structured in such a way that they simulated multithreaded applications. This will hopefully be very difficult for the pi 0 to run, decent for the cluster to run, and difficult for the pi 3 to run.
* We will then run a real life ROS application. I am thinking the drone catching demo. We can record a ball throw. We then run that same ball throw with the application running on the pi 0, cluster hat and pi 3. We record the time the drone begins to move. This should show us speedup.

## Documents

* [CPU SPEC 2006](https://github.com/hildebrandt-carl/cluster_hat/blob/master/doc/CPUSPEC.md) – The initial benchmark tested. (Not complete)
* [HPC Challenge Benchmark](https://github.com/hildebrandt-carl/cluster_hat/blob/master/doc/HPC.md) – The final benchmark used.
* [ROS Installation](https://github.com/hildebrandt-carl/cluster_hat/blob/master/doc/ROS.md) – The software environment used for real life testing.

## Authors

* **Carl Hildebrandt** - *Initial work* - [hildebrandt.carl](https://github.com/hildebrandt-carl)
* **William Turner** - *Initial work* - [william.turner](https://git.unl.edu/william.turner)
* **Molly Lee** - *Group Member*
* **Casey Lafferty** - *Group Member*


