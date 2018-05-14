#include "controller.h"

Controller::Controller() : n(), private_n()
{
	image_transport::ImageTransport it(n);
	ROS_INFO_STREAM(ros::this_node::getName() << ": Started ") ;

	std::string namespace_param ; 
	std::string imagename_param ;
	std::string totpross_param ;
	std::string senddelay_param ;
	std::string savedir_param ;

	n.param<std::string>("/" + ros::this_node::getName() + "/namespace", namespace_param, "childebrandt") ;
	n.param<std::string>("/" + ros::this_node::getName() + "/image_name", imagename_param, "example1.jpg") ;
	n.param<std::string>("/" + ros::this_node::getName() + "/total_processors", totpross_param, "1") ;
	n.param<std::string>("/" + ros::this_node::getName() + "/send_delay", senddelay_param, "0.1") ;
	n.param<std::string>("/" + ros::this_node::getName() + "/save_dir", savedir_param, "/home/") ;
	
	ROS_INFO_STREAM("The namespace has been set to: " << namespace_param) ;
	ROS_INFO_STREAM("The image name has been set to: " << imagename_param) ;
	ROS_INFO_STREAM("Total processors launched: " << totpross_param) ;
	ROS_INFO_STREAM("The send delay will be: " << senddelay_param) ;
	ROS_INFO_STREAM("The save directory will be: " << savedir_param) ;

	Controller::SaveDir = savedir_param ;
	Controller::img_name = imagename_param ;
	Controller::tot_pros = std::stoi(totpross_param) ;
	Controller::sendDelay = std::stof(senddelay_param) ;

	double sqrt_total = sqrt(Controller::tot_pros) ;

	double intpart ;
	if(modf(sqrt_total, &intpart) != 0.0)
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<  ": Please pass in a total number of processor which can be square rooted." ) ;
		exit(0) ;
	}

	// If we get a control-c send it to signal_handler
	signal( SIGINT, Controller::signal_handler ) ;

	// TODO
	//processed_image_sub = it.subscribe("/finished_image", 1, &Processor::imageCallback, this) ;

    // TODO
    out_control_messages = n.advertise<image_processing_ros::control>( "/control", 1000, true);

	image_pub = n.advertise<image_processing_ros::split_img>( "/small_image", 1000, true);
	//image_pub = it.advertise("/small_image", 1);

	processed_image_sub = n.subscribe("/processed_Images", 1, &Controller::processedImageCallback, this) ;

	std::thread t1( &Controller::startAllNodes, this ) ;
	t1.detach() ;

	Controller::my_counter = 0 ;
}

void Controller::startAllNodes()
{
	ros::Duration one_second(1.0) ;

	ROS_INFO_STREAM(ros::this_node::getName()<<  ": Loading Image to Process:" ) ;
	cv::Mat image ;
    image = cv::imread(Controller::img_name, CV_LOAD_IMAGE_COLOR) ;

	ROS_INFO_STREAM(ros::this_node::getName()<<  ": Starting Processing Nodes in:" ) ;
	
	for (int k = 3; k > 0; k--)
	{
		ROS_INFO_STREAM(ros::this_node::getName() << ": " << k) ;
		one_second.sleep() ;
	}

	image_processing_ros::control out_msg ;
	out_msg.turn_on = true ;
	out_msg.turn_off = false ;
	out_msg.header.stamp = ros::Time::now() ;

	out_control_messages.publish(out_msg) ;

	ROS_INFO_STREAM(ros::this_node::getName()<<  ": Getting Ready to Prime CV_Bridges in:" ) ;

	for (int k = 3; k > 0; k--)
	{
		ROS_INFO_STREAM(ros::this_node::getName() << ": " << k) ;
		one_second.sleep() ;
	}

	ROS_INFO_STREAM(ros::this_node::getName()<<  ": Priming CV_Bridges") ;

	// DUMMY START THE BELLOW ONE WILL FAIL NO ONE KNOWS WHY
	// ---------------------------------------
	for (int send_imgs = 0; send_imgs <= 1; send_imgs++)
	{
		sensor_msgs::ImagePtr converted_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg() ;

		image_processing_ros::split_img msg ;
		msg.im = *converted_img ;
		std::string rec_name = "/processor" ;
		rec_name = rec_name + std::to_string(send_imgs) ;
		msg.node_name = rec_name ;
		msg.image_number = 1 ;
		ROS_INFO_STREAM(ros::this_node::getName()<<  ": Sent image to " << rec_name ) ;
		image_pub.publish(msg) ;
		ros::Duration send_msg_delay(5) ;
		send_msg_delay.sleep() ;

		ROS_INFO_STREAM(ros::this_node::getName()<<  ": Sending Image in:" ) ;
		
		for (int k = 3; k > 0; k--)
		{
			ROS_INFO_STREAM(ros::this_node::getName() << ": " << k) ;
			one_second.sleep() ;
		}
	}
	
	ROS_INFO_STREAM(ros::this_node::getName()<<  ": Starting Timer" ) ;
	Controller::cpu_begin_time = clock();
	Controller::wall_begin_time = std::chrono::system_clock::now();

	double sqrt_total = sqrt(Controller::tot_pros) ;
	cv::Size smallSize(image.cols/sqrt_total,image.rows/sqrt_total) ;
	std::vector<cv::Mat> smallImages ;

	for (int y = 0; y < image.rows; y += smallSize.height)
	{
		for (int x = 0; x < image.cols; x += smallSize.width)
		{
			cv::Rect rect =  cv::Rect(x,y, smallSize.width, smallSize.height) ;
			smallImages.push_back(cv::Mat(image, rect)) ;
		}
	}

	sensor_msgs::ImagePtr converted_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", smallImages[0]).toImageMsg() ;
	
	for (int send_imgs = 2; send_imgs <= Controller::tot_pros+1; send_imgs++)
	{
		ROS_INFO_STREAM(ros::this_node::getName()<<  ": Attempting to send image number " << send_imgs ) ;
		if(send_imgs == 2)
		{
			converted_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", smallImages[send_imgs-2]).toImageMsg() ;
		}
		else
		{
			converted_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", smallImages[send_imgs-2]).toImageMsg() ;
		}

		image_processing_ros::split_img msg ;
		msg.im = *converted_img ;
		std::string rec_name = "/processor" ;
		rec_name = rec_name + std::to_string(send_imgs) ;
		msg.node_name = rec_name ;
		msg.image_number = 1 ;
		ROS_INFO_STREAM(ros::this_node::getName()<<  ": Sent image to " << rec_name ) ;
		image_pub.publish(msg) ;
		ros::Duration send_msg_delay(sendDelay) ;
		send_msg_delay.sleep() ;
	}

	//ROS_INFO_STREAM(ros::this_node::getName()<<  ": Showing Image" ) ;
	//cv::imshow("view", image) ;
	//cv::waitKey(10000);
	//ROS_INFO_STREAM(ros::this_node::getName() << ": Displayed Image") ;
}

void Controller::signal_handler( int sig )
{	
	ros::shutdown() ;
	exit(0) ;
}

void Controller::stopAllNodes()
{
	ROS_INFO_STREAM(ros::this_node::getName() << ": Stopping Addition Nodes" ) ;

	image_processing_ros::control out_msg;
	out_msg.turn_on = false ;
	out_msg.turn_off = true ;
	out_msg.header.stamp = ros::Time::now();

	out_control_messages.publish(out_msg) ;

	ROS_INFO_STREAM(ros::this_node::getName()<<  ": Stopping Timer" ) ;
	Controller::cpu_end_time = clock();
	Controller::wall_end_time = std::chrono::system_clock::now();
}

void Controller::processedImageCallback(const image_processing_ros::return_img::ConstPtr &msg)
{

	if(msg->node_name.compare("/processor0") != 0)
	{
		ROS_INFO_STREAM(ros::this_node::getName()<< ": Received message from " << msg->node_name << " with image number " << msg->image_number );

		cv::Mat proc_img = cv_bridge::toCvShare(msg->im, msg, "mono8")->image ;

		// Some pointer cazzate
		cv::Mat deepCopyImage ;
		proc_img.copyTo(deepCopyImage) ;

		Controller::final_processed_images.push_back(deepCopyImage) ;

		ROS_INFO_STREAM(Controller::final_processed_images.size()) ;

		ROS_INFO_STREAM(ros::this_node::getName()<< ": Pushed image into vector");

		//if(Controller::final_processed_images.size() > (unsigned)Controller::tot_pros - 1)
		if(Controller::final_processed_images.size() > (unsigned)Controller::tot_pros - 2)
		{
			Controller::stopAllNodes() ;

			//for(int show_imgs = 0; show_imgs < Controller::tot_pros; show_imgs++)
			//{	
			//	cv::Mat poped_img = Controller::final_processed_images.back() ;
			//	Controller::final_processed_images.pop_back() ;
			//	std::string view_name = "processedimage" + std::to_string(show_imgs) + ".jpg" ;
			//	cv::imwrite(Controller::SaveDir + view_name, poped_img) ;
			//	cv::waitKey(0.5) ;
			//} 

			ros::Duration send_msg_delay(3) ;
			send_msg_delay.sleep() ;

			double cpu_elapsed_secs = double(Controller::cpu_end_time - Controller::cpu_begin_time) / CLOCKS_PER_SEC;
			std::cout<<"CPU Time taken: "<<cpu_elapsed_secs<<std::endl;

			std::chrono::duration<double> wall_elapsed_seconds = Controller::wall_end_time - Controller::wall_begin_time;
			std::cout<<"Wall Time taken: "<<wall_elapsed_seconds.count()<<std::endl;

			ros::shutdown() ;
			exit(0) ;
		}
	}
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
