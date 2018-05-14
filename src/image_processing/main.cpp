//Apated From : https://stackoverflow.com/questions/11435974/watershed-segmentation-opencv-xcode/11441676#11441676
//What I am doing : https://stackoverflow.com/questions/11294859/how-to-define-the-markers-for-watershed-in-opencv/11438165#11438165

#include "opencv2/opencv.hpp"
#include <string>
#include <ctime>
#include <chrono>

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

int main(int argc, char* argv[])
{
    
    clock_t cpu_begin_time ;
    clock_t cpu_end_time ;

    std::chrono::time_point<std::chrono::system_clock> wall_begin_time ;
    std::chrono::time_point<std::chrono::system_clock> wall_end_time ;

    if (argc != 2)
    {
        std::cout<<"Please enter 1 argument, the image name"<<std::endl ;
        return 0;
    }
    else
    {
        // Read the image whose name is passed as the first arugment
        cv::Mat image = cv::imread(argv[1]);

        // Start the timer
        cpu_begin_time = clock();
	    wall_begin_time = std::chrono::system_clock::now();

        // Create the image objects you will need
        cv::Mat contours;
        cv::Mat contours2;
        cv::Mat gray_image;
        cv::Mat binary;
        cv::Mat fg;

        // Convert the original image to grayscale
        cvtColor( image, gray_image, CV_RGB2GRAY );

        // Do Canny Edge detection on the image
        cv::Canny(image,contours,10,350);

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

        cv::Mat result = segmenter.process(image);
        result.convertTo(result,CV_8U);

        // Do Canny Edge detection on the image
        cv::Canny(result,contours2,10,350);

        // End the timer
        cpu_end_time = clock();
	    wall_end_time = std::chrono::system_clock::now();
        
        // Display all images
        cv::namedWindow("Image");
        cv::resize(image, image, cv::Size(), 0.25, 0.25);
        cv::imshow("Image",image);

        cv::namedWindow("Gray");
        cv::resize(gray_image, gray_image, cv::Size(), 0.25, 0.25);
        cv::imshow("Gray",gray_image);

        cv::namedWindow("Binary");
        cv::resize(binary, binary, cv::Size(), 0.25, 0.25);
        cv::imshow("Binary",binary);

        cv::namedWindow("Markers");
        cv::resize(markers, markers, cv::Size(), 0.25, 0.25);
        cv::imshow("Markers",markers);

        cv::namedWindow("Eroded");
        cv::resize(fg, fg, cv::Size(), 0.25, 0.25);
        cv::imshow("Eroded",fg);

        cv::namedWindow("BG");
        cv::resize(bg, bg, cv::Size(), 0.25, 0.25);
        cv::imshow("BG",bg);

        cv::namedWindow("Segmented");
        cv::resize(result, result, cv::Size(), 0.25, 0.25);
        cv::imshow("Segmented",result);

        cv::namedWindow("Canny");
        cv::resize(contours, contours, cv::Size(), 0.25, 0.25);
        cv::imshow("Canny",contours);

        cv::namedWindow("Canny2");
        cv::resize(contours2, contours2, cv::Size(), 0.25, 0.25);
        cv::imshow("Canny2",contours2);

        // Display the time taken
        double cpu_elapsed_secs = double(cpu_end_time - cpu_begin_time) / CLOCKS_PER_SEC;
        std::cout<<"CPU Time taken: "<<cpu_elapsed_secs<<std::endl;

		std::chrono::duration<double> wall_elapsed_seconds = wall_end_time - wall_begin_time;
        std::cout<<"Wall Time taken: "<<wall_elapsed_seconds.count()<<std::endl;

        cv::waitKey(0);
    }
    
    return 0;
}

