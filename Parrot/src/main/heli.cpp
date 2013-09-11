#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "SDL/SDL.h"
/*
 * A simple 'getting started' interface to the ARDrone, v0.2 
 * author: Tom Krajnik
 * The code is straightforward,
 * check out the CHeli class and main() to see 
 */
#include <stdlib.h>
#include "CHeli.h"
#include <unistd.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

int freeze = 0;
int deltaClick;
int valBclick,valGclick,valRclick,valBin;
int limInfB = 0, limSupB = 0, limInfG = 0, limSupG = 0, limInfR = 0, limSupR = 0;
Mat currentImage = Mat(240, 320, CV_8UC3);
int analysis = 0;
int RTemp = 0, GTemp = 0, BTemp = 0;
int redClick = 0, greenClick = 0, blueClick = 0;
vector<Point> points;
int coordinateX, coordinateY;
bool stop = false;
CRawImage *image;
CHeli *heli;
float pitch, roll, yaw, height;
int hover;
// Joystick related
SDL_Joystick* m_joystick;
bool useJoystick;
int joypadRoll, joypadPitch, joypadVerticalSpeed, joypadYaw;
bool navigatedWithJoystick, joypadTakeOff, joypadLand, joypadHover;

Mat highlightObject(Mat sourceImage);
void flipImageEfficient(const Mat &sourceImage, Mat &destinationImage);
void mouseCoordinates(int event, int x, int y, int flags, void* param);
void luminosity (Mat &sourceImage, Mat &bwImage, int umbral);
void rawToMat(Mat &destImage, CRawImage* sourceImage);
void rgb2yiq(Mat &sourceImage, Mat &destImage);
void rgb2hsv(Mat &sourceImage, Mat &hsvImage);
void generateRedHistogram (Mat &sourceImage, Mat &redHistogram);
void generateBlueHistogram (Mat &sourceImage, Mat &blueHistogram);
void generateGreenHistogram (Mat &sourceImage, Mat &greenHistogram);



int main(int argc,char* argv[])
{
	//establishing connection with the quadcopter
	heli = new CHeli();
	Mat imgFiltrada = Mat(240, 320, CV_8UC3);
	//this class holds the image from the drone	
	image = new CRawImage(320,240);


	// Initial values for control	
    pitch = roll = yaw = height = 0.0;
    joypadPitch = joypadRoll = joypadYaw = joypadVerticalSpeed = 0.0;

	// Destination OpenCV Mat	
	Mat snapshot = Mat(240, 320, CV_8UC3);
	Mat bwImage = Mat(240, 320, CV_8UC3);
	Mat filteredImage = Mat(240, 320, CV_8UC3);
	Mat yiqImage = Mat(240, 320, CV_8UC3);
	Mat hsvImage = Mat(240, 320, CV_8UC3);
	Mat flippedImage;
	Mat redHistogram = Mat(256,256,CV_8UC3);
	Mat blueHistogram = Mat(256,256,CV_8UC3);
	Mat greenHistogram = Mat(256,256,CV_8UC3);

	namedWindow("ParrotCam");
    setMouseCallback("ParrotCam", mouseCoordinates);
	// Show it	
	imshow("ParrotCam", currentImage);

    // Initialize joystick
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);
    useJoystick = SDL_NumJoysticks() > 0;
    if (useJoystick)
    {
        SDL_JoystickClose(m_joystick);
        m_joystick = SDL_JoystickOpen(0);
    }

    while (stop == false)
    {

        // Clear the console
        printf("\033[2J\033[1;1H");

        if (useJoystick)
        {
            SDL_Event event;
            SDL_PollEvent(&event);

            joypadRoll = SDL_JoystickGetAxis(m_joystick, 2);
            joypadPitch = SDL_JoystickGetAxis(m_joystick, 3);
            joypadVerticalSpeed = SDL_JoystickGetAxis(m_joystick, 1);
            joypadYaw = SDL_JoystickGetAxis(m_joystick, 0);
            joypadTakeOff = SDL_JoystickGetButton(m_joystick, 1);
            joypadLand = SDL_JoystickGetButton(m_joystick, 2);
            joypadHover = SDL_JoystickGetButton(m_joystick, 0);
        }

        // prints the drone telemetric data, helidata struct contains drone angles, speeds and battery status
        printf("===================== Parrot Basic Example =====================\n\n");
        //fprintf(stdout, "Angles  : %.2lf %.2lf %.2lf \n", helidata.phi, helidata.psi, helidata.theta);
        //fprintf(stdout, "Speeds  : %.2lf %.2lf %.2lf \n", helidata.vx, helidata.vy, helidata.vz);
        fprintf(stdout, "Battery : %.0lf \n", helidata.battery);
        //fprintf(stdout, "Hover   : %d \n", hover);
        //fprintf(stdout, "Joypad  : %d \n", useJoystick ? 1 : 0);
        //fprintf(stdout, "  Roll    : %d \n", joypadRoll);
        //fprintf(stdout, "  Pitch   : %d \n", joypadPitch);
        //fprintf(stdout, "  Yaw     : %d \n", joypadYaw);
        //fprintf(stdout, "  V.S.    : %d \n", joypadVerticalSpeed);
        //fprintf(stdout, "  TakeOff : %d \n", joypadTakeOff);
        //fprintf(stdout, "  Land    : %d \n", joypadLand);
        //fprintf(stdout, "Navigating with Joystick: %d \n", navigatedWithJoystick ? 1 : 0);
        fprintf(stdout, "Click coordinates: (%d, %d) \n", coordinateX, coordinateY);
        fprintf(stdout, "R: %d, G: %d, B: %d\n", redClick, greenClick, blueClick);

		//image is captured
		heli->renewImage(image);

		// Copy to OpenCV Mat
		rawToMat(currentImage, image);
		imshow("ParrotCam", currentImage);

        char key = waitKey(5);
		switch (key) {
			case 'a': yaw = -20000.0; break;
			case 'd': yaw = 20000.0; break;
			case 'w': height = -20000.0; break;
			case 's': height = 20000.0; break;
			//case 'q': heli->takeoff(); break;
			case 'e': heli->land(); break;
			case 'z': heli->switchCamera(0); break;
			case 'x': heli->switchCamera(1); break;
			case 'c': heli->switchCamera(2); break;
			case 'v': heli->switchCamera(3); break;
			case 'j': roll = -20000.0; break;
			case 'l': roll = 20000.0; break;
			case 'i': pitch = -20000.0; break;
			case 'k': pitch = 20000.0; break;
            case 'h': hover = (hover + 1) % 2; break;
            case 'f': freeze ^= 1;  break;
            case 27: stop = true; break;
            default: pitch = roll = yaw = height = 0.0;
		}

		if(!freeze){
			filteredImage = highlightObject(currentImage);
			imshow("Filtered image", filteredImage);
			luminosity(filteredImage, bwImage, atoi(argv[1]));
	        imshow("Black and White", bwImage);
	        flipImageEfficient(currentImage, flippedImage);
	        imshow("Flipped", flippedImage);
	        rgb2hsv(currentImage, hsvImage);
	        imshow("HSV", hsvImage);
	        generateRedHistogram(currentImage, redHistogram);
	        generateGreenHistogram(currentImage,greenHistogram);
	        generateBlueHistogram(currentImage,blueHistogram);
	        imshow("Red Histogram",redHistogram);
	        imshow("Green Histogram",greenHistogram);
	        imshow("Blue Histogram",blueHistogram);
	        rgb2yiq(currentImage, yiqImage);
	        imshow("YIQ", yiqImage)
    	}
/*
        if (joypadTakeOff) {
            heli->takeoff();
        }
        if (joypadLand) {
            heli->land();
        }
        hover = joypadHover ? 1 : 0;

        //setting the drone angles
        if (joypadRoll != 0 || joypadPitch != 0 || joypadVerticalSpeed != 0 || joypadYaw != 0)
        {
            heli->setAngles(joypadPitch, joypadRoll, joypadYaw, joypadVerticalSpeed, hover);
            navigatedWithJoystick = true;
        }
        else
        {
            heli->setAngles(pitch, roll, yaw, height, hover);
            navigatedWithJoystick = false;
        }
*/
        usleep(15000);
	}

	heli->land();
    SDL_JoystickClose(m_joystick);
    delete heli;
	delete image;
	return 0;
}

void flipImageEfficient(const Mat &sourceImage, Mat &destinationImage)
{
	if (destinationImage.empty())
		destinationImage = Mat(sourceImage.rows, sourceImage.cols, sourceImage.type());

	int channels = sourceImage.channels();

	for (int y = 0; y < sourceImage.rows; ++y) 
	{
		uchar* sourceRowPointer = (uchar*) sourceImage.ptr<uchar>(y);
		uchar* destinationRowPointer = (uchar*) destinationImage.ptr<uchar>(y);
		for (int x = 0; x < sourceImage.cols / 2; ++x)
			for (int i = 0; i < channels; ++i)
			{
				destinationRowPointer[x * channels + i] = sourceRowPointer[(sourceImage.cols - 1 - x ) * channels + i];
				destinationRowPointer[(sourceImage.cols - 1 - x) * channels + i] = sourceRowPointer[x * channels + i];
			}
	}
}

void mouseCoordinates(int event, int x, int y, int flags, void* param)
{
    switch (event)
    {

	  /*CV_EVENT_MOUSEMOVE - when the mouse pointer moves over the specified window
		CV_EVENT_LBUTTONDOWN - when the left button of the mouse is pressed on the specified window
		CV_EVENT_RBUTTONDOWN - when the right button of the mouse is pressed on the specified window
		CV_EVENT_MBUTTONDOWN - when the middle button of the mouse is pressed on the specified window
		CV_EVENT_LBUTTONUP - when the left button of the mouse is released on the specified window
		CV_EVENT_RBUTTONUP - when the right button of the mouse is released on the specified window
		CV_EVENT_MBUTTONUP - when the middle button of the mouse is released on the specified window */

        case CV_EVENT_LBUTTONDOWN:
            coordinateX = x;
            coordinateY = y;
            redClick = currentImage.at<Vec3b>(y, x)[2];
	        greenClick = currentImage.at<Vec3b>(y, x)[1];
	        blueClick = currentImage.at<Vec3b>(y, x)[0];

			deltaClick = 20;
			limInfR = redClick - (deltaClick * 255) / 100;
			limSupR = redClick + (deltaClick * 255) / 100;
			limInfG = greenClick - (deltaClick * 255) / 100;
			limSupG = greenClick+ (deltaClick * 255) / 100;
			limInfB = blueClick - (deltaClick * 255) / 100;
			limSupB = blueClick + (deltaClick * 255) / 100;
			break;

            /*  Draw a point */
            //points.push_back(Point(x, y));
            break;
        case CV_EVENT_RBUTTONDOWN:
            break;
    }
}
void luminosity (Mat &sourceImage,  Mat &destImage, int umbral)
{
	int channels = sourceImage.channels(); 	// Numero de canales
	// int average;
	int pond;
	for(int y = 0; y < sourceImage.rows; ++y)
	{
		for(int x = 0; x < sourceImage.cols; ++x)
		{
			for(int i = 0; i < channels; ++i)
			{
				//average = (sourceImage.at<Vec3b>(y, x)[0] + sourceImage.at<Vec3b>(y, x)[1] + sourceImage.at<Vec3b>(y, x)[2]) / 3;
				pond = 0.1 * sourceImage.at<Vec3b>(y, x)[0] + 0.3 * sourceImage.at<Vec3b>(y, x)[1] + 0.6 * sourceImage.at<Vec3b>(y, x)[2];

				if(pond > umbral)
					pond = 255;
				else
					pond = 0;

				switch(i)
				{
					case 0: destImage.at<Vec3b>(y, x)[i] = pond; // Blue
					break;
					case 1: destImage.at<Vec3b>(y, x)[i] = pond; // Green
					break;
					case 2: destImage.at<Vec3b>(y, x)[i] = pond; // Red
					break;
					default:
					break;
				}
				/*
				switch(i)
				{
					case 0: sourceImage.at<Vec3b>(y, x)[i] = average; // Blue
					break;
					case 1: sourceImage.at<Vec3b>(y, x)[i] = average; // Green
					break;
					case 2: sourceImage.at<Vec3b>(y, x)[i] = average; // Red
					break;
					default:
					break;
				}*/

			}
		}
	}
}

void rgb2yiq(Mat &sourceImage, Mat &destImage)
{
	for (int y = 0; y < sourceImage.rows; ++y) 
	{
		for (int x = 0; x < sourceImage.cols; ++x){
			destImage.at<Vec3b>(y, x)[2] = 0.299 * sourceImage.at<Vec3b>(y, x)[2] + 0.587 * sourceImage.at<Vec3b>(y, x)[2] + 0.144 * sourceImage.at<Vec3b>(y, x)[2];
			destImage.at<Vec3b>(y, x)[1] = 0.596 * sourceImage.at<Vec3b>(y, x)[2] - 0.275 * sourceImage.at<Vec3b>(y, x)[2] - 0.321 * sourceImage.at<Vec3b>(y, x)[2];
			destImage.at<Vec3b>(y, x)[0] = 0.212 * sourceImage.at<Vec3b>(y, x)[2] - 0.528 * sourceImage.at<Vec3b>(y, x)[2] + 0.311 * sourceImage.at<Vec3b>(y, x)[2]; 
		}

	}
}

void rgb2hsv(Mat &sourceImage, Mat &hsvImage)
{
	int channels = sourceImage.channels(); 	// Numero de canales
	double vector_r_lineal = 0;
	double vector_g_lineal = 0;
	double vector_b_lineal = 0;
	char max='n';
	char min='n';
	int max_value = 0;
	int min_value = 255;
	for(int y = 0; y < sourceImage.rows; ++y)
	{
		for(int x = 0; x < sourceImage.cols; ++x)
		{
			vector_r_lineal = (double)sourceImage.at<Vec3b>(y, x)[2] / (double)255;
			vector_g_lineal = (double)sourceImage.at<Vec3b>(y, x)[1] / (double)255;
			vector_b_lineal = (double)sourceImage.at<Vec3b>(y, x)[0] / (double)255;
			//if(x<10)
			//cout<<"r "<<vector_r_lineal<<"g "<<vector_g_lineal<<"b "<<vector_b_lineal;
			//Encuentra maximo y minimo
			for (int i = 0; i < channels; ++i)
			{
				if (sourceImage.at<Vec3b>(y, x)[i] >= max_value)
				{
					//cout<<"ENTRA_MAX_ENTRA"<<endl;
					max_value = sourceImage.at<Vec3b>(y, x)[i];
					switch(i)
					{
						  case 0: max = 'b'; // Blue
						  break;
						  case 1: max = 'g'; // Green
						  break;
						  case 2: max = 'r'; // Red
						  break;
						  default:
						  break;
					}
				}
				if(sourceImage.at<Vec3b>(y, x)[i] < min_value)
				{
					//cout<<"ENTRA_MIN_ENTRA"<<endl;
					min_value = sourceImage.at<Vec3b>(y, x)[i];
					switch(i)
					{
						case 0: min = 'b'; // Blue
						break;
						case 1: min = 'g'; // Green
						break;
						case 2: min = 'r'; // Red
						break;
						default:
						break;
					}
				}
			}
			/*DEBUG
			if(x<10)
			{
			  cout<<" SALIDASALIDA "<<max<<max_value<<min<<min_value<<endl;
			}*/

			double max_value_lineal = (double)max_value/(double)255;
			double min_value_lineal = (double)min_value/(double)255;
			double v = max_value_lineal;
			double s = 0; //Default si v = 0
			double h = 180; //Default si v = 0
			if(v != 0) //Si el máximo no es cero
			{
				s = (max_value_lineal - min_value_lineal) / max_value_lineal;
				switch(max)
				{
					case 'r': h=(vector_g_lineal-vector_b_lineal)*60/(max_value_lineal-min_value_lineal);
					case 'g': h=((vector_b_lineal-vector_r_lineal)*60/(max_value_lineal-min_value_lineal))+120;
					case 'b': h=((vector_r_lineal-vector_g_lineal)*60/(max_value_lineal-min_value_lineal))+240;
				}
			}
			/*DEBUG
			if(x<10)
			{
			  cout<<"VALORES";
			  cout<<"H"<<h<<" "<<(int)h/2<<"S"<<s<<" "<<(int)s*255<<"V"<<v<<" "<<(int)v*255<<endl;
			}*/

				//se sustituyen los valores hsv en cada canal de 8 bits
			hsvImage.at<Vec3b>(y, x)[0]=(int)(v*255);
			hsvImage.at<Vec3b>(y, x)[1]=(int)(s*255);
			hsvImage.at<Vec3b>(y, x)[2]=(int)h/2;//Se divide para que se pueda representar en 8 bits
			/*DEBUG
			if(x<10)
			{
			  cout<<"V IMAGEN"<<hsvImage.at<Vec3b>(y, x)[0];
			  cout<<"S IMAGEN"<<hsvImage.at<Vec3b>(y, x)[1];
			  cout<<"H IMAGEN"<<hsvImage.at<Vec3b>(y, x)[2];
			}
			*/

			//Hacer todo default
			max='n';
			min='n';
			max_value = 0;
			min_value = 255;

			//cvSplit(hsvImage, h/2, s, v, 0);
		}
	}

}

void generateRedHistogram (Mat &sourceImage, Mat &redHistogram)
{
	//int channels = sourceImage.channels(); 	// Numero de canales 
	int colorSat;

	//Creo un arreglo de 256 localidades y despues se llena con 0's
	int arr[256];

	for (int i = 0; i < 256; i++)
	{
		arr[i] = 0;
	}

	for(int y = 0; y < sourceImage.rows; ++y)
	{
		for(int x = 0; x < sourceImage.cols; ++x)
		{
				//Saco la intensidad del rojo y la guardo en colorSat
				colorSat = sourceImage.at<Vec3b>(y,x)[2];
				//busco la casilla dond esta casa valor
				arr[colorSat] = arr[colorSat] + 1;
				arr[colorSat - 1] = arr[colorSat - 1] + 1;
				arr[colorSat - 2] = arr[colorSat - 2] + 1;
				arr[colorSat - 3] = arr[colorSat - 3] + 1;
				arr[colorSat - 4] = arr[colorSat - 4] + 1;
				arr[colorSat - 5] = arr[colorSat - 5] + 1;
				arr[colorSat - 6] = arr[colorSat - 6] + 1;
				arr[colorSat - 7] = arr[colorSat - 7] + 1;
		}

	}

	//Pinto toda la imagen de blanco
	for(int y = 0; y < redHistogram.rows; ++y)
		for(int x = 0; x < redHistogram.cols; ++x)
		{
			redHistogram.at<Vec3b>(y, x)[0] = 255;
			redHistogram.at<Vec3b>(y, x)[1] = 255;
			redHistogram.at<Vec3b>(y, x)[2] = 255;
		}

		int maxDelArreglo = 0;

	for(int i = 0; i<256; i++)
	{
		if(arr[i]>maxDelArreglo)
			maxDelArreglo = arr[i];
	}

	for(int i = 0; i < 256; i++){

			arr[i] = ((arr[i]*255)/maxDelArreglo);
	}

	for(int x = 1; x < redHistogram.cols; ++x)
		for(int y = arr[x]; y>0;--y)//for(int y = 0; y < arr[x]; ++y)
		{
			redHistogram.at<Vec3b>(y, x)[0] = 0;
			redHistogram.at<Vec3b>(y, x)[1] = 0;

		}



}

Mat highlightObject(Mat sourceImage)
{
	Mat highlightedImg;
	highlightedImg = sourceImage.clone();
	unsigned char *source, *dest;
	for(int i = 0; i < sourceImage.rows; ++i)
	{
		source = (unsigned char*)(sourceImage.ptr<uchar>(i));
		dest = (unsigned char*)(highlightedImg.ptr<uchar>(i));

		for(int j = 0; j < sourceImage.cols; ++j)
		{
			if (source[j * 3 + 0] < limInfB || source[j * 3 + 0] > limSupB || source[j * 3 + 1] < limInfG || source[j * 3 + 1] > limSupG || source[j * 3 + 2] < limInfR || source[j * 3 + 2] > limSupR)
			{
				dest[j * 3 + 0] = 0;
				dest[j * 3 + 1] = 0;
				dest[j * 3 + 2] = 0;
			}
			else
			{
				dest[j * 3 + 0] = source[j * 3 + 0];
				dest[j * 3 + 1] = source[j * 3 + 1];
				dest[j * 3 + 2] = source[j * 3 + 2];
			}
		}
	}

	return highlightedImg;
}
void rawToMat(Mat &destImage, CRawImage* sourceImage)
{	
	uchar *pointerImage = destImage.ptr(0);

	for (int i = 0; i < 240*320; i++)
	{
		pointerImage[3*i] = sourceImage->data[3*i+2];
		pointerImage[3*i+1] = sourceImage->data[3*i+1];
		pointerImage[3*i+2] = sourceImage->data[3*i];
	}
}
void generateGreenHistogram (Mat &sourceImage, Mat &greenHistogram)
{
	int channels = sourceImage.channels(); 	// Numero de canales 
	int colorSat;

	//Creo un arreglo de 256 localidades y despues se llena con 0's
	int arr[256];

	for (int i = 0; i < 256; i++)
	{
		arr[i] = 0;
	}

	for(int y = 0; y < sourceImage.rows; ++y)
	{
		for(int x = 0; x < sourceImage.cols; ++x)
		{
				//Saco la intensidad del rojo y la guardo en colorSat
				colorSat = sourceImage.at<Vec3b>(y,x)[1];
				//busco la casilla dond esta casa valor
				arr[colorSat] = arr[colorSat] + 1;
				arr[colorSat - 1] = arr[colorSat - 1] + 1;
				arr[colorSat - 2] = arr[colorSat - 2] + 1;
				arr[colorSat - 3] = arr[colorSat - 3] + 1;
				arr[colorSat - 4] = arr[colorSat - 4] + 1;
				arr[colorSat - 5] = arr[colorSat - 5] + 1;
				arr[colorSat - 6] = arr[colorSat - 6] + 1;
				arr[colorSat - 7] = arr[colorSat - 7] + 1;
		}

	}

//Pinto toda la imagen de blanco
	for(int y = 0; y < greenHistogram.rows; ++y)
		for(int x = 0; x < greenHistogram.cols; ++x)
		{
			greenHistogram.at<Vec3b>(y, x)[0] = 255;
			greenHistogram.at<Vec3b>(y, x)[1] = 255;
			greenHistogram.at<Vec3b>(y, x)[2] = 255;
		}


		int maxDelArreglo = 0;

	for(int i = 0; i<256; i++)
	{
		if(arr[i]>maxDelArreglo)
			maxDelArreglo = arr[i];
	}

	for(int i = 0; i < 256; i++){

			arr[i] = ((arr[i]*255)/maxDelArreglo);
	}



	for(int x = 1; x < greenHistogram.cols; ++x)
		for(int y = arr[x]; y>0;--y)//for(int y = 0; y < arr[x]; ++y)
		{
			greenHistogram.at<Vec3b>(y, x)[0] = 0;
			greenHistogram.at<Vec3b>(y, x)[2] = 0;

		}



}

void generateBlueHistogram (Mat &sourceImage, Mat &blueHistogram)
{
	int channels = sourceImage.channels(); 	// Numero de canales 
	int colorSat;

	//Creo un arreglo de 256 localidades y despues se llena con 0's
	int arr[256];

	for (int i = 0; i < 256; i++)
	{
		arr[i] = 0;
	}

	for(int y = 0; y < sourceImage.rows; ++y)
	{
		for(int x = 0; x < sourceImage.cols; ++x)
		{
				//Saco la intensidad del rojo y la guardo en colorSat
				colorSat = sourceImage.at<Vec3b>(y,x)[0];
				//busco la casilla dond esta casa valor
				arr[colorSat] = arr[colorSat] + 1;
				arr[colorSat - 1] = arr[colorSat - 1] + 1;
				arr[colorSat - 2] = arr[colorSat - 2] + 1;
				arr[colorSat - 3] = arr[colorSat - 3] + 1;
				arr[colorSat - 4] = arr[colorSat - 4] + 1;
				arr[colorSat - 5] = arr[colorSat - 5] + 1;
				arr[colorSat - 6] = arr[colorSat - 6] + 1;
				arr[colorSat - 7] = arr[colorSat - 7] + 1;
		}

	}

//Pinto toda la imagen de blanco
	for(int y = 0; y < blueHistogram.rows; ++y)
		for(int x = 0; x < blueHistogram.cols; ++x)
		{
			blueHistogram.at<Vec3b>(y, x)[0] = 255;
			blueHistogram.at<Vec3b>(y, x)[1] = 255;
			blueHistogram.at<Vec3b>(y, x)[2] = 255;
		}


		int maxDelArreglo = 0;

	for(int i = 0; i<256; i++)
	{
		if(arr[i]>maxDelArreglo)
			maxDelArreglo = arr[i];
	}

	for(int i = 0; i < 256; i++){

			arr[i] = ((arr[i]*255)/maxDelArreglo);
	}



	for(int x = 1; x < blueHistogram.cols; ++x)
		for(int y = arr[x]; y>0;--y)//for(int y = 0; y < arr[x]; ++y)
		{
			blueHistogram.at<Vec3b>(y, x)[1] = 0;
			blueHistogram.at<Vec3b>(y, x)[2] = 0;

		}



}