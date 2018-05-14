#ifndef CALCULATION_H
#define CALCULATION_H

#include <ros/ros.h>
#include <speed_test/comm.h>
#include <speed_test/control.h>
#include <signal.h>
#include <thread>

class Processor
{
	private:
		// Node handle
		ros::NodeHandle n ;
		ros::NodeHandle private_n ;

		// Private variables
		float my_counter ;	
		bool doing_calculations ;

	public:

		static bool shutdown_requested ;
		static bool alive ;

		// Subscriber to the incoming ROS messages
		ros::Subscriber incoming_msg_sub ;

		// Publisher to send out the new calculated message
		ros::Publisher outgoing_msg_pub ;

		// Constructor
		Processor() ;
		// TODO
		void beginProcessor() ;
		// TODO
		float expensiveOperation() ; 
		// TODO
		void sendNumber( float num_in ) ;
		// TODO
		void controlCallback( const speed_test::control::ConstPtr &msg ) ;

		static void signal_handler( int ) ;		
};
#endif
