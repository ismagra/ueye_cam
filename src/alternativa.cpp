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


#define CAPTURE_WIDTH  4192		//1920
#define CAPTURE_HEIGHT 3104		//1080


using namespace ueye_cam;
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "Alternativa");
	
	//Asignacion de matriz de la imagen
	Mat frame(CAPTURE_HEIGHT, CAPTURE_WIDTH,CV_8UC3);

	//Variables de inicializacion de la camara

	HIDS hCam = 1; //Abre Camera con ID 1
	int BITS_PER_PIXEL = 24;
	int pWidth = CAPTURE_WIDTH;
	int pHeight = CAPTURE_HEIGHT; 
	SENSORINFO sensor_info;
	CAMINFO camera_info;

	//Punteros a memoria
	char* m_pcImageMemory;
	int m_lMemoryId;

	//Limpieza de fotos anteriores en la memoria
	if (hCam != 0){
		is_FreeImageMem (hCam,m_pcImageMemory,m_lMemoryId);
		is_ExitCamera(hCam);
	}

	//Inicializacion de la camara 
	int initcamera = is_InitCamera(&hCam, NULL);
	if(initcamera != IS_SUCCESS)
	{
		cout<<endl<<"Imposible inicializar la camara"<<endl;
		exit(-1);
	}

	//Adquisicion de informacion de la camara
	int camerainfo = is_GetCameraInfo (hCam, &camera_info);
	if(camerainfo != IS_SUCCESS)
	{
		printf("Imposible adquirir informacion de camara");
		exit(-1);
	} 
	//Adquisicion de informacion del sensor
	int sensorinfo = is_GetSensorInfo (hCam, &sensor_info);
	if(sensorinfo != IS_SUCCESS)
	{
		printf("Imposible adquirir informacion de sensor");
		exit(-1);
	}
	//Mostrar las informaciones
	cout<<endl<<"<<< Caracteristicas de la camara >>>"<<endl;
	cout<<"Numero serie: " << camera_info.SerNo << endl;
	cout << "Producto: " << camera_info.ID << endl;
	cout << "Modelo: " << sensor_info.strSensorName << endl;
	cout << "Dimension maxima de la imagen: " << sensor_info.nMaxWidth << "x" << sensor_info.nMaxHeight << endl << endl;
	
	
	//-------------------------------------------------------------------------------------------------------------------//
	//---------------------------------------------------------------------------------------------------------------------//
	//---------------------------------------------------------------------------------------------------------------------//
	//---------------------------------------------------------------------------------------------------------------------//
	//---------------------------------------------------------------------------------------------------------------------//
	
	//Cargar parametros de la camara
	//int paraset = is_ParameterSet(hCam, IS_PARAMETERSET_CMD_LOAD_FILE,(void*)"/home/juan/Escritorio/camera.ini", NULL);
	//char* p = "/home/juan/Escritorio/gggg.ini";
	//~ int paraset = is_ParameterSet(hCam, IS_PARAMETERSET_CMD_SAVE_FILE, NULL, NULL);
	//~ if (paraset != IS_SUCCESS)
	//~ {
		//~ printf("no se han cargado");
		//~ exit(-1);
	//~ }
	
	// Check if parameter set in the user memory is supported by the camera
	UINT nSupportedHWParameterSet;
	INT nRet = is_ParameterSet(hCam, IS_PARAMETERSET_CMD_GET_HW_PARAMETERSET_AVAILABLE,
							  (void*)&nSupportedHWParameterSet, sizeof(nSupportedHWParameterSet));
	if (nRet == IS_SUCCESS)
	{
	if (nSupportedHWParameterSet == 1)
	{
	  cout << "soportado" << endl;
	}
	}
		
		
	// Load parameters from specified file using a relative path
	INT nRet3 = is_ParameterSet(hCam, IS_PARAMETERSET_CMD_LOAD_FILE, (void*)L"configuracion.ini", NULL);
	if (nRet3 == IS_SUCCESS)
	{
	  cout << "cargado en archivo" << endl;
	}
	
	
	//~ INT nRet1 = is_ParameterSet(hCam, IS_PARAMETERSET_CMD_SAVE_EEPROM, NULL, NULL);
	//~ if (nRet1 == IS_SUCCESS)
	//~ {
	  //~ cout << "salvado en la memoria" << endl;
	//~ }

	
	
	//~ // Save parameters to specified file
	//~ INT nRet2 = is_ParameterSet(hCam, IS_PARAMETERSET_CMD_SAVE_FILE, (void*)L"PRUEBA6.ini", NULL);
	//~ if (nRet2 == IS_SUCCESS)
	//~ {
	  //~ cout << "salvado en archivo" << endl;
	//~ }
	
	
	//---------------------------------------------------------------------------------------------------------------------//
	//---------------------------------------------------------------------------------------------------------------------//
	//---------------------------------------------------------------------------------------------------------------------//
	//---------------------------------------------------------------------------------------------------------------------//
	//---------------------------------------------------------------------------------------------------------------------//
	
	
	
	//Establecer el modo de color BGR8 
	int colormode = is_SetColorMode(hCam, IS_CM_BGR8_PACKED);
	//int colormode = is_SetColorMode(hCam, IS_SET_CM_RGB24);
	if(colormode != IS_SUCCESS)
	{
		printf("Imposible establecer el modo de color");
		exit(-1);
	}

	//Establece el tamaño de las imagenes que se quieran capturar
	//~ int pXPos = (sensor_info.nMaxWidth);
	//~ int pYPos = (sensor_info.nMaxHeight);
	int pXPos = 4192;
	int pYPos = 3104;

	

	//Inicializacion de la memoria de la camara
	//int rit = is_AllocImageMem (hCam,pXPos,pYPos, 24, &m_pcImageMemory, &m_lMemoryId);
	int rit = is_AllocImageMem (hCam,pXPos,pYPos, 24, &m_pcImageMemory, &m_lMemoryId);
	if(rit != IS_SUCCESS)
	{
		cout<<endl<<"Imposible inicializar la memoria"<<endl;
		system("PAUSE");
		exit(-1);
	}
	cout<<endl<<"Memoria inicializada"<<endl;

	//Activacion de la posicion en memoria
	int rat = is_SetImageMem (hCam, m_pcImageMemory, m_lMemoryId);
	if(rat != IS_SUCCESS)
	{
		cout<<endl<<"Imposible activar la memoria"<<endl;
		system("PAUSE");
		exit(-1);
	}
	cout<<endl<<"Memoria activada"<<endl;

	//Ajustes de correccion de color
	double strenght_factor = 1.0;
	int colorcorrection = is_SetColorCorrection(hCam, IS_CCOR_ENABLE, &strenght_factor);

	//Ajustes de correcion de blancos
	double pval = 1;
	int whiteb = is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &pval, 0);

	//Establecer la correccion de ganancia
	double gval = 1;
	int gains = is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_GAIN, &gval, 0);


	//Inicio de la fase de captura
	int dummy;
	char *pMem, *pLast;
	
	//Set trigger 
    int extt= is_SetExternalTrigger(hCam, IS_SET_TRIGGER_SOFTWARE);
    if (extt != IS_SUCCESS)
		cout<<"Imposible establecer modo trigger"<<endl;
	 
    
	//Captura de imagen
	//~ int sho = is_FreezeVideo(hCam, IS_WAIT);		//No pasa de aqui.
	//~ if(sho != IS_SUCCESS)
	//~ {
		//~ cout<<endl<<"Imposible adquirir imagen de la camara"<<endl;
		//~ system("PAUSE");
		//~ exit(-1);
	//~ }
	//~ if (sho == IS_SUCCESS)
	//~ {
		//~ is_GetActiveImageMem(hCam, &pLast, &dummy);		//(int m_Ret =)
		//~ is_GetImageMem(hCam, (void**)&pLast);		//(int n_Ret=)
	//~ }

	//~ IplImage* tmpImg = cvCreateImageHeader(cvSize (pXPos, pYPos), IPL_DEPTH_8U,3); 
	//~ tmpImg->imageData = m_pcImageMemory;
	//~ frame = cv::cvarrToMat(tmpImg);
	//~ imshow("Prueba",frame);
	//~ waitKey(0);
	 
	//Salvar fotos en tres modos
	IMAGE_FILE_PARAMS ImageFileParams;
	ImageFileParams.pwchFileName = L"./snap_BGR8.png";
	ImageFileParams.pnImageID = NULL;
	ImageFileParams.ppcImageMem = NULL;
	ImageFileParams.nQuality = 100;		//0 pone el valor por defecto = 75
	ImageFileParams.nFileType = IS_IMG_PNG;

	INT nRet4 = is_ImageFile(hCam, IS_IMAGE_FILE_CMD_SAVE, (void*) &ImageFileParams, sizeof(ImageFileParams));
	printf("Status is_ImageFile %d\n",nRet4);

	ImageFileParams.pwchFileName = L"./snap_BGR8.bmp";
	ImageFileParams.pnImageID = NULL;
	ImageFileParams.ppcImageMem = NULL;
	ImageFileParams.nQuality = 0;		//Ignorado para el metodo bmp
	ImageFileParams.nFileType = IS_IMG_BMP;

	INT nRet5 = is_ImageFile(hCam, IS_IMAGE_FILE_CMD_SAVE, (void*) &ImageFileParams, sizeof(ImageFileParams));
	printf("Status is_ImageFile %d\n",nRet5);

	ImageFileParams.pwchFileName = L"./snap_BGR8.jpg";
	ImageFileParams.pnImageID = NULL;
	ImageFileParams.ppcImageMem = NULL;
	ImageFileParams.nQuality = 100;		//100 es el modo de maxima calidad
	ImageFileParams.nFileType = IS_IMG_JPG;

	INT nRet6 = is_ImageFile(hCam, IS_IMAGE_FILE_CMD_SAVE, (void*) &ImageFileParams, sizeof(ImageFileParams));
	printf("Status is_ImageFile %d\n",nRet6);


	//Cierre de la camara y limpieza
	int en = is_ExitCamera(hCam);
	if (en == IS_SUCCESS)
	{
		cout<<endl<<"Camara cerrada correctamente"<<endl;
	} 
	 
	 
	ros::spin();
	return 0;
}
