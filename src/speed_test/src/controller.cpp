#include "controller.h"

Controller::Controller() : n(), private_n()
{
	ROS_INFO_STREAM(ros::this_node::getName() << ": Started ") ;

	// If we get a control-c send it to signal_handler
	signal( SIGINT, Controller::signal_handler ) ;

	// TODO
	in_number_messages = n.subscribe( "/numbers", 2, &Controller::numberCallback, this ) ;

    // TODO
    out_control_messages = n.advertise<speed_test::control>( "/control", 1000, true);

	std::thread t1( &Controller::startAllNodes, this ) ;
	t1.detach() ;

	Controller::my_counter = 0;
}

void Controller::signal_handler( int sig )
{	
	ros::shutdown() ;
	exit(0) ;
}

void Controller::numberCallback(const speed_test::comm::ConstPtr &msg)
{
	ROS_INFO_STREAM(ros::this_node::getName() << ": Got Message = " << Controller::my_counter) ;
	//ROS_INFO_STREAM( msg->number ) ;
	Controller::my_counter++ ;
	if(Controller::my_counter >= 100)
	{
		Controller::stopAllNodes() ;
		ros::Duration five_seconds(5.0) ;
		five_seconds.sleep() ;
		
		double cpu_elapsed_secs = double(Controller::cpu_end_time - Controller::cpu_begin_time) / CLOCKS_PER_SEC;
        std::cout<<"CPU Time taken: "<<cpu_elapsed_secs<<std::endl;

		std::chrono::duration<double> wall_elapsed_seconds = Controller::wall_end_time - Controller::wall_begin_time;
        std::cout<<"Wall Time taken: "<<wall_elapsed_seconds.count()<<std::endl;

		ros::shutdown() ;
		exit(0) ;
	}
}

void Controller::startAllNodes()
{
	

	ROS_INFO_STREAM(ros::this_node::getName()<<  ": Starting Nodes in:" ) ;
	
	for (int k = 5; k > 0; k--)
	{
		ROS_INFO_STREAM(ros::this_node::getName() << ": " << k) ;
		ros::Duration five_seconds(1.0) ;
		five_seconds.sleep() ;
	}

	speed_test::control out_msg;
	out_msg.turn_on = true ;
	out_msg.turn_off = false ;
	out_msg.header.stamp = ros::Time::now();

	out_control_messages.publish(out_msg) ;

	ROS_INFO_STREAM(ros::this_node::getName()<<  ": Starting Timer" ) ;
	Controller::cpu_begin_time = clock();
	Controller::wall_begin_time = std::chrono::system_clock::now();
}

void Controller::stopAllNodes()
{
	ROS_INFO_STREAM(ros::this_node::getName() << ": Stopping Addition Nodes" ) ;

	speed_test::control out_msg;
	out_msg.turn_on = false ;
	out_msg.turn_off = true ;
	out_msg.header.stamp = ros::Time::now();

	out_control_messages.publish(out_msg) ;

	Controller::cpu_end_time = clock();
	Controller::wall_end_time = std::chrono::system_clock::now();
}

int main( int argc, char **argv )
{
	std::string ROS_NODE_NAME = argv[1] ;

	// Start the node, and dont allow siginthandler to shutdown the node
	ros::init( argc, argv, ROS_NODE_NAME, ros::init_options::NoSigintHandler ) ;

	Controller controller_obj ;

	ros::spin() ;

	return 0 ;
}
