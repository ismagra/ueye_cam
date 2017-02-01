#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>                                              

int count_;	//number of times you want to take images	

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller");
  ros::topic::waitForMessage<sensor_msgs::Image>("/camera/image_rect_color");  //wait for the correct images
  
  for (count_=0; count_<2; count_=count_+1) {	//change the value of count_
	  system ("rosrun ueye_cam take_image");
	  //CHANGE PARAMETERS WITH ROSPARAM AND YAML FILES
	  //ros::Duration(5).sleep();	//seconds between times
  }
  
  if (count_ == 2) {
	  std::cout << "Captura terminada" << std::endl;
  }
    
  ros::spin();
  return 0;
}
