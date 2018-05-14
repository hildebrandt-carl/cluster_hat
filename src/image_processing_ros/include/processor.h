#ifndef PROCESSOR_H
#define PROCESSOR_H

#include <ros/ros.h>
#include <image_processing_ros/comm.h>
#include <image_processing_ros/control.h>
#include <image_processing_ros/split_img.h>
#include <image_processing_ros/return_img.h>
#include <signal.h>
#include <thread>
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class Processor
{
	private:
		// Node handle
		ros::NodeHandle n ;
		ros::NodeHandle private_n ;

		// Private variables
		float my_counter ;	
		bool doing_calculations ;

		cv::Mat rec_img ;
		cv::Mat proc1_img ;
		cv::Mat proc2_img ;

	public:

		static bool shutdown_requested ;
		static bool alive ;

		// Subscriber to the incoming ROS messages
		ros::Subscriber incoming_msg_sub ;

		// Subscriber to the incoming picture messages
		ros::Subscriber image_sub ;
		//image_transport::Subscriber image_sub ;

		// Publisher to send out the new calculated message
		ros::Publisher outgoing_msg_pub ;

		// Constructor
		Processor() ;
		// TODO
		void beginProcessor() ;
		// TODO
		void expensiveOperation() ; 
		// TODO
		// TODO
		void controlCallback(const image_processing_ros::control::ConstPtr &msg) ;
		// TODO
		//void imageCallback(const sensor_msgs::ImageConstPtr &msg) ;
		void imageCallback(const image_processing_ros::split_img::ConstPtr &msg) ;

		static void signal_handler(int) ;		
};
#endif
