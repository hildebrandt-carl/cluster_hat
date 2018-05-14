#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <speed_test/comm.h>
#include <speed_test/control.h>
#include <signal.h>
#include <thread>
#include <ctime>
#include <chrono>

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

	public:

		static bool shutdown_requested;

		// Subscriber to the incoming ROS messages
		ros::Subscriber in_number_messages;

		// Publisher to send out the new calculated message
		ros::Publisher out_control_messages;

		// Constructor
		Controller() ;

		static void signal_handler( int ) ;

		void numberCallback(const speed_test::comm::ConstPtr &msg) ;

		void startAllNodes() ;
		void stopAllNodes() ;
	
};
#endif
