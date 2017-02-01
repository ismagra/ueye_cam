#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "ueye_cam/ueye_cam_driver.hpp"
#include <sstream>   
#include "uEye.h"


using namespace ueye_cam;
using namespace cv;

class TriggerImage
{
	
public:

    TriggerImage(int width, int height, int camNumber)
    {
    // Variables of camera's initialization
    hCam = camNumber;
    int BITS_PER_PIXEL = 24;
    pWidth = width;
    pHeight = height;
    SENSORINFO sensor_info;
    CAMINFO camera_info;
    //Memory
    int m_lMemoryId;
    //Free memory
    if (hCam != 0){
        is_FreeImageMem (hCam, m_pcImageMemory, m_lMemoryId);
        is_ExitCamera(hCam);
    }
    //Initialization of camera
    int initcamera = is_InitCamera(&hCam, NULL);
    if(initcamera != IS_SUCCESS)
    {
        std::cout<< "Impossible to initializate camera"<<std::endl;
        exit(-1);
    }
    // Retrieve information of camera
    int camerainfo = is_GetCameraInfo (hCam, &camera_info);
    if(camerainfo != IS_SUCCESS)
    {
        printf("Impossible to retrieve information of camera");
        exit(-1);
    }
    // Retrieve information of camera's sensor
    int sensorinfo = is_GetSensorInfo (hCam, &sensor_info);
    if(sensorinfo != IS_SUCCESS)
    {
        printf("Impossible retrieve information of camera's sensor");
        exit(-1);
    }
    //Output informazioni camera/sensore
    std::cout<< std::endl <<"<<< Properties of the camera>>>"
            <<std::endl;
    std::cout<<"Serial Number: " << camera_info.SerNo << std::endl;
    std::cout << "ID Product: " << camera_info.ID << std::endl;
    std::cout << "ID " << camera_info.Select << std::endl;
    std::cout << "Model: " << sensor_info.strSensorName << std::endl;
    std::cout << "Maximun dimension of image: " << sensor_info.nMaxWidth
            << "x" << sensor_info.nMaxHeight << std::endl << std::endl;
    //Set color mode
    int colormode = is_SetColorMode(hCam, IS_CM_BGR8_PACKED);
    //int colormode = is_SetColorMode(hCam, IS_SET_CM_RGB24);
    if(colormode != IS_SUCCESS)
    {
        printf("Impossible to set color mode");
        exit(-1);
    }
	// Set image size
    int pXPos = (sensor_info.nMaxWidth);
    int pYPos = (sensor_info.nMaxHeight);
    
    //Initialization camera's memory
    int rit = is_AllocImageMem (hCam,pXPos,pYPos, 24, &m_pcImageMemory, &m_lMemoryId);
    if(rit != IS_SUCCESS)
    {
        std::cout << std::endl << "Imposible to initialize camera's memory"
                << std::endl;
        system("PAUSE");
        exit(-1);
    }
    std::cout << std::endl << "Memoria inicializada"
            << std::endl;
    //Allocate memory
    int rat = is_SetImageMem (hCam, m_pcImageMemory, m_lMemoryId);
    if(rat != IS_SUCCESS)
    {
        std::cout << std::endl << "Impossible to allocate camera's memory"
                << std::endl;
        system("PAUSE");
        exit(-1);
    }
    std::cout<<std::endl << "Activated Memory" << std::endl;
    //Set color correction
    double strenght_factor = 1.0;
    int colorcorrection = is_SetColorCorrection(hCam, IS_CCOR_ENABLE, &strenght_factor);
    //Set auto white balance
    double pval = 1;
    int whiteb = is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &pval, 0);
    //Set auto gain
    double gval = 1;
    int gains = is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_GAIN, &gval, 0);
    //Set trigger and get image
    is_SetExternalTrigger(hCam, IS_SET_TRIGGER_SOFTWARE);
	is_FreezeVideo(hCam, IS_WAIT);
    }
    
    
    
    ~TriggerImage()
    {
    int en = is_ExitCamera(hCam);
    if (en == IS_SUCCESS){
        std::cout << std::endl << "Camera closed" << std::endl;
    }
    }





    void GetFrame(cv::Mat& frameOut)
    {
	std::cout << std::endl << "checkpoint1"
                << std::endl;	
    cv::Mat frame(pHeight, pWidth, CV_8UC3);
    //inicio fase de capturar imagenes
    int dummy;
    char *pMem, *pLast;
    int sho = is_FreezeVideo(hCam, IS_WAIT);
    if(sho != IS_SUCCESS)
    {
        std::cout<< std::endl <<"IMPOSIBLE ADQUIRIR IMAGEN"
            << std::endl;
        system("PAUSE");
        exit(-1);
    }
    if (sho == IS_SUCCESS){
        int m_Ret = is_GetActiveImageMem(hCam, &pLast, &dummy);
        int n_Ret = is_GetImageMem(hCam, (void**)&pLast);
    }
    IplImage* tmpImg = cvCreateImageHeader(cvSize (pWidth, pHeight),
                                            IPL_DEPTH_8U,3);
    tmpImg->imageData = m_pcImageMemory;
    frame = cv::cvarrToMat(tmpImg);
    frameOut = frame.clone();
    //añadido//
    namedWindow("Imagen", WINDOW_NORMAL);
	imshow("Imagen", frameOut );
    std::cout << std::endl << "checkpoint2"
                << std::endl;
                
                
	//~ //guardar la imagen
	//~ sensor_msgs::ImageConstPtr& msg;
	//~ cv_bridge::CvImagePtr cv_ptr;
	//~ std::cout << std::endl << "checkpoint3"
                //~ << std::endl;
	//~ cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	//~ std::cout << std::endl << "checkpoint4"
                //~ << std::endl;
    //~ std::stringstream sstream; 
	//~ sstream << "my_image" << image_count << ".png" ; 
	//~ cv::imwrite(sstream.str(),cv_ptr->image);
	//~ std::cout << std::endl << "checkpoint5"
                //~ << std::endl;
	
    }
    
    

private:
    HIDS hCam;
    int pWidth, pHeight;
    char* m_pcImageMemory;
};









int main(int argc, char** argv)
{
	int a=1280,b=720,c;
	cv::Mat* frm;
	cv::Mat ppp = cv::Mat(a,b,CV_8UC3);
	frm = &ppp;
		

	//~ //Open camera with ID 1
	//~ HIDS hCam = 1;
	//~ is_InitCamera (&hCam, NULL);
	//~ if (nRet != IS_SUCCESS)
	//~ {
		//~ printf("error");
	//~ }
	//~ is_CameraStatus (hCam, IS_STANDBY, IS_GET_STATUS);
	

  ros::init(argc, argv, "TriggerImage");
  TriggerImage ti(a,b,c);
  ti.GetFrame(*frm);
  //cv::Mat pp = ti.GetFrame(*frm);
  ros::spin();
  return 0;


	
	

  //~ ros::init(argc, argv, "image_triggered");
  //~ ros::NodeHandle n; 
  
  //~ HIDS hCam = 0;
  //~ //n.getParam("camera_id", hCam);
  //~ //ros::param::get("/ueye_cam_nodelet/camera_id", hCam);

  //~ is_SetExternalTrigger(hCam, IS_SET_TRIGGER_SOFTWARE);
  //~ is_FreezeVideo(hCam, IS_WAIT);
  
  //~ // Fit image to window and display it upside down:
  //~ //is_RenderBitmap (hCam, nMemID, hwnd, IS_RENDER_FIT_TO_WINDOW | IS_RENDER_MIRROR_UPDOWN);

  //~ ros::spin();
  return 0;
}
































