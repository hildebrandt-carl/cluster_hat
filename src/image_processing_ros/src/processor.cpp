#include "processor.h"

class WatershedSegmenter{
private:
    cv::Mat markers;
public:
    void setMarkers(cv::Mat& markerImage)
    {
        markerImage.convertTo(markers, CV_32S);
    }

    cv::Mat process(cv::Mat &image)
    {
        cv::watershed(image, markers);
        markers.convertTo(markers, CV_8U);
        return markers;
    }
};

Processor::Processor() : n(), private_n()
{
	ROS_INFO_STREAM(ros::this_node::getName() << ": Started ") ;

	image_transport::ImageTransport it(n);

	// If we get a control-c send it to signal_handler
	signal( SIGINT, Processor::signal_handler ) ;

	// TODO
	incoming_msg_sub = n.subscribe( "/control", 1000, &Processor::controlCallback, this ) ;

	// TODO
	image_sub = n.subscribe("/small_image", 1, &Processor::imageCallback, this) ;

    // TODO
    outgoing_msg_pub = n.advertise<image_processing_ros::return_img>( "/processed_Images", 1000, true );

	Processor::doing_calculations = false ;
}

void Processor::imageCallback(const image_processing_ros::split_img::ConstPtr &msg)
{
	if(Processor::doing_calculations)
	{
		if(ros::this_node::getName().compare(msg->node_name) == 0)
		{
			ROS_INFO_STREAM(ros::this_node::getName()<< ": Received message for " << msg->node_name << " with image number " << msg->image_number );

			Processor::rec_img = cv_bridge::toCvShare(msg->im, msg, "bgr8")->image ;

			std::thread t1( &Processor::beginProcessor, this ) ;
			t1.detach() ;
		}
	}
	else
	{
		ROS_INFO_STREAM(ros::this_node::getName() << ": Got an image but am currently not in processing mode") ;
	}
}

void Processor::controlCallback( const image_processing_ros::control::ConstPtr &msg )
{
	if((msg->turn_on) && (msg->turn_off))
	{
		ROS_ERROR("Ignoring Message") ;
		return ;
	}

	if(msg->turn_on)
	{
		ROS_INFO_STREAM(ros::this_node::getName() << ": Processor Active") ;
		doing_calculations = true ;
	}

	if(msg->turn_off)
	{
		ROS_INFO_STREAM(ros::this_node::getName() << ": Stopping the addition") ;
		doing_calculations = false ;
		ROS_INFO_STREAM(ros::this_node::getName() << ": Exiting") ;
		exit(0) ;
	}
}

void Processor::signal_handler( int sig )
{
	alive = false ;
	exit(0) ;
}

void Processor::beginProcessor()
{
	Processor::expensiveOperation() ;
	ROS_INFO_STREAM(ros::this_node::getName() << ": Finished Processing Image") ;

	sensor_msgs::ImagePtr converted_img = cv_bridge::CvImage(std_msgs::Header(), "mono8", Processor::proc1_img).toImageMsg() ;

	image_processing_ros::return_img msg5 ;
	msg5.im = *converted_img ;
	msg5.node_name = ros::this_node::getName() ;
	msg5.image_number = 1 ;
	ROS_INFO_STREAM(ros::this_node::getName()<<  ": Returned image from " << msg5.node_name ) ;

	outgoing_msg_pub.publish(msg5) ;

	//cv::imshow("viewx", Processor::proc1_img) ;
	//cv::waitKey(0) ;
}

void Processor::expensiveOperation()
{
	//cv::namedWindow("Canny");
	//cv::imshow("Canny",Processor::rec_img);

	//cv::waitKey(100000) ;


	ROS_INFO_STREAM(ros::this_node::getName() << ": Processing Image" ) ;

	// Create the image objects you will need	
	cv::Mat contours;
	cv::Mat contours2;
	cv::Mat gray_image;
	cv::Mat binary;
	cv::Mat fg;

	// Convert the original image to grayscale
	cvtColor( Processor::rec_img, gray_image, CV_RGB2GRAY );

	// Do Canny Edge detection on the image
	cv::Canny(Processor::rec_img,contours,10,350);

	// Create a binary image from the grayscale image
	cv::threshold(gray_image, binary, 100, 255, cv::THRESH_BINARY);

	// Eliminate noise and smaller objects
	cv::erode(binary,fg,cv::Mat(),cv::Point(-1,-1),2);
	
	// Identify image pixels without objects
	cv::Mat bg;
	cv::dilate(binary,bg,cv::Mat(),cv::Point(-1,-1),3);
	cv::threshold(bg,bg,1, 128,cv::THRESH_BINARY_INV);

	// Create markers image
	cv::Mat markers(binary.size(),CV_8U,cv::Scalar(0));
	markers= fg+bg;

	// Create watershed segmentation object
	WatershedSegmenter segmenter;
	segmenter.setMarkers(markers);

	cv::Mat result = segmenter.process(Processor::rec_img);
	result.convertTo(result,CV_8U);

	// Do Canny Edge detection on the image
	cv::Canny(result,contours2,10,350);
	
	
	//cv::namedWindow("Canny");
	//cv::imshow("Canny",contours);

	//cv::namedWindow("Canny2") ;
	//cv::imshow("Canny2",contours2) ;

	Processor::proc1_img = contours ;
	Processor::proc2_img = contours2 ;

	return ;
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

	cv::destroyWindow("view1");
}
