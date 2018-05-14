#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <image_processing_ros/comm.h>
#include <image_processing_ros/control.h>
#include <image_processing_ros/split_img.h>
#include <image_processing_ros/return_img.h>
#include <signal.h>
#include <thread>
#include <ctime>
#include <chrono>
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


class Controller
{
	private:
		// Node handle
		ros::NodeHandle n ;
		ros::NodeHandle private_n ;

		// Private variables
		float my_counter ;	
		clock_t cpu_begin_time ;
		clock_t cpu_end_time ;

		std::chrono::time_point<std::chrono::system_clock> wall_begin_time ;
		std::chrono::time_point<std::chrono::system_clock> wall_end_time ;

		std::string img_name ;
		int tot_pros ;

		std::vector<cv::Mat> final_processed_images ;
		float sendDelay ;
		std::string	SaveDir ;

	public:

		static bool shutdown_requested ;

		// Subscriber to the incoming picture messages
		ros::Subscriber processed_image_sub ;

		// Publisher to send out the new calculated message
		ros::Publisher out_control_messages ;

		//image_transport::Publisher image_pub ;
		ros::Publisher image_pub ;

		// Constructor
		Controller() ;

		static void signal_handler( int ) ;

		void processedImageCallback(const image_processing_ros::return_img::ConstPtr &msg) ;
		

		void startAllNodes() ;
		void stopAllNodes() ;
	
};
#endif
