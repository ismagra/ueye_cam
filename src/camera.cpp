#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sstream>   
#include "uEye.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

int main(int argc, char** argv)
{
		
	ros::init(argc, argv, "camera");
	
//--------------------------------------------------------------------//
//-----------------INICIALIZACION DE LA CAMARA------------------------//
//--------------------------------------------------------------------//

	//Variables de inicializacion de la camara
	HIDS hCam = 1; //Abre Camera con ID 1
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

//--------------------------------------------------------------------//
//-----------------INFORMACION DE LA CAMARA---------------------------//
//--------------------------------------------------------------------//

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
	cout << endl <<"<<< Caracteristicas de la camara >>>"<< endl;
	cout << "Numero serie: " << camera_info.SerNo << endl;
	cout << "Producto: " << camera_info.ID << endl;
	cout << "Modelo: " << sensor_info.strSensorName << endl;
	cout << "Dimension maxima de la imagen: " << sensor_info.nMaxWidth << "x" << sensor_info.nMaxHeight << endl << endl;
	
//--------------------------------------------------------------------//
//------------------CARGAR ARCHIVO DE CONFIGURACION-------------------//
//--------------------------------------------------------------------//

	
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

//--------------------------------------------------------------------//
//---------------------ESTABLECER PARAMETROS--------------------------//
//--------------------------------------------------------------------//
		
	//Establecer el modo de color BGR8 
	int colormode = is_SetColorMode(hCam, IS_CM_BGR8_PACKED);
	if (colormode != IS_SUCCESS)
	{
		printf("Imposible establecer el modo de color");
		exit(-1);
	}

	//Set trigger 
    int extt= is_SetExternalTrigger(hCam, IS_SET_TRIGGER_SOFTWARE);
    if (extt != IS_SUCCESS)
		cout<<"Imposible establecer modo trigger"<<endl;
	 
//--------------------------------------------------------------------//
//---------------ESTABLECER POSICION DE MEMORIA------------------------//
//--------------------------------------------------------------------//

	//Establece el tamaño de las imagenes que se quieran capturar
	int pXPos = 4192;
	int pYPos = 3104;

	//Inicializacion de la memoria de la camara
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
	
//--------------------------------------------------------------------//
//------------------------CAPTURAR IMAGEN-----------------------------//
//--------------------------------------------------------------------//

	//Captura de imagen
	int sho = is_FreezeVideo(hCam, IS_WAIT);		
	if(sho != IS_SUCCESS)
	{
		cout<<endl<<"Imposible adquirir imagen de la camara"<<endl;
		exit(-1);
	}
	 
	cout<<endl<<"checkpoint"<<endl;
	
	//Salvar fotos en png
	IMAGE_FILE_PARAMS ImageFileParams;
	ImageFileParams.pwchFileName = L"./snap_BGR8.png";
	ImageFileParams.pnImageID = NULL;
	ImageFileParams.ppcImageMem = NULL;
	ImageFileParams.nQuality = 100;		//0 pone el valor por defecto = 75
	ImageFileParams.nFileType = IS_IMG_PNG;

	INT nRet4 = is_ImageFile(hCam, IS_IMAGE_FILE_CMD_SAVE, (void*) &ImageFileParams, sizeof(ImageFileParams));
	printf("Status is_ImageFile %d\n",nRet4);
	
	
//--------------------------------------------------------------------//
//-------------------------PUBLICAR IMAGEN----------------------------//
//--------------------------------------------------------------------//

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera11/image11", 1);

	cv::Mat image = cv::imread("snap_BGR8.png", CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

	ros::Rate loop_rate(5);
	while (nh.ok()) {
	  pub.publish(msg);
	  ros::spinOnce();
	  loop_rate.sleep();
	}
	
//~ //--------------------------------------------------------------------//
//~ //--------------------------CERRAR CAMARA-----------------------------//
//~ //--------------------------------------------------------------------//

	//~ //Cierre de la camara y limpieza
	//~ int en = is_ExitCamera(hCam);
	//~ if (en == IS_SUCCESS)
	//~ {
		//~ cout<<endl<<"Camara cerrada correctamente"<<endl;
	//~ } 
	 
	 
	//~ ros::spin();
	//~ return 0;
}
