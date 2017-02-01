#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ueye_cam/ueye_cam_driver.hpp"
#include <sstream>   
#include "uEye.h"
                                             

static const std::string OPENCV_WINDOW = "Image window";


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  int image_count = 0;	//number of images you want to take


public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed 
    image_sub_ = it_.subscribe("/camera/image_rect_color", 1, &ImageConverter::imageCb, this);

  }

  ~ImageConverter()
  {
    //~ cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
                      
		
                          			
    if (image_count != 2) {	
		std::stringstream sstream; 
		sstream << "my_image" << image_count << ".png" ; 
		//std::cout << sstream.str() << std::endl;               
		cv::imwrite(sstream.str(),cv_ptr->image); 
		image_count++; 
		 
		//ros::Duration(1).sleep(); // sleep for x seconds between images                            
	}
    
    	  
	if (image_count == 2) {
		std::cout << "Imagenes capturadas" << std::endl;
	    ros::shutdown();	//put the camera in standby mode
	}
	    
	return;
	    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
