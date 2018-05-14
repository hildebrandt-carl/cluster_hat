#include "calculation.h"

Processor::Processor() : n(), private_n()
{
	ROS_INFO_STREAM(ros::this_node::getName() << ": Started ") ;
	
	// If we get a control-c send it to signal_handler
	signal( SIGINT, Processor::signal_handler ) ;

	// TODO
	incoming_msg_sub = n.subscribe( "/control", 1000, &Processor::controlCallback, this ) ;

    // TODO
    outgoing_msg_pub = n.advertise<speed_test::comm>( "/numbers", 1000, true );

	Processor::doing_calculations = false ;
}

void Processor::signal_handler( int sig )
{
	alive = false ;
	exit(0) ;
}

void Processor::beginProcessor()
{
	while(( Processor::doing_calculations ) && ( ros::ok() ))
	{
		float answer = Processor::expensiveOperation() ;
		Processor::sendNumber( answer ) ;
	}
}

float Processor::expensiveOperation()
{
	float ans = 0 ;
	
	ROS_INFO_STREAM(ros::this_node::getName() << ": Expensive Operation" ) ;
	for(int i = 0; i < 1000; i++)
	{
		for(int j = 0; j < 1000; j++)
		{
			for(int k = 0; k < 1000; k++)
			{
				ans = ans + 1.1 ;
			}
		}
	}

	return ans ;
}

void Processor::sendNumber( float num_in )
{
	speed_test::comm out_msg;
	out_msg.number = num_in ;
	out_msg.header.stamp = ros::Time::now();

	outgoing_msg_pub.publish( out_msg );
}

void Processor::controlCallback( const speed_test::control::ConstPtr &msg )
{
	if((msg->turn_on) && (msg->turn_off))
	{
		ROS_ERROR("Ignoring Message") ;
		return ;
	}

	if(msg->turn_on)
	{
		ROS_INFO_STREAM(ros::this_node::getName() << ": Addition starting") ;
		doing_calculations = true ;
		std::thread t1( &Processor::beginProcessor, this ) ;
		t1.detach() ;
	}

	if(msg->turn_off)
	{
		ROS_INFO_STREAM(ros::this_node::getName() << ": Stopping the addition") ;
		doing_calculations = false ;
		ROS_INFO_STREAM(ros::this_node::getName() << ": Exiting") ;
		exit(0) ;
	}
}	

bool Processor::alive = true;
int main( int argc, char **argv )
{
	std::string ROS_NODE_NAME = argv[1] ;
	//ROS_INFO("Starting ROS node : " + ROS_NODE_NAME) ;

	// Start the node, and dont allow siginthandler to shutdown the node
	ros::init( argc, argv, ROS_NODE_NAME, ros::init_options::NoSigintHandler ) ;

	Processor calc_obj ;

	ros::spin() ;

	return 0 ;
}
