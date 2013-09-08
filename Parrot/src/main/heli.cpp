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

int RTemp = 0, GTemp = 0, BTemp = 0;
int RED = 0, GREEN = 0, BLUE = 0;
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

void flipImageEfficient(const Mat &sourceImage, Mat &destinationImage);
//void blackNwhite (Mat &sourceImage, int umbral);
void mouseCoordinates(int event, int x, int y, int flags, void* param);
void luminosity (Mat &sourceImage, Mat &bwImage, int umbral);
void rawToMat(Mat &destImage, CRawImage* sourceImage);

int main(int argc,char* argv[])
{
	//establishing connection with the quadcopter
	heli = new CHeli();
	
	//this class holds the image from the drone	
	image = new CRawImage(320,240);


	// Initial values for control	
    pitch = roll = yaw = height = 0.0;
    joypadPitch = joypadRoll = joypadYaw = joypadVerticalSpeed = 0.0;

	// Destination OpenCV Mat	
	Mat currentImage = Mat(240, 320, CV_8UC3);
	Mat snapshot = Mat(240, 320, CV_8UC3);
	Mat bwImage = Mat(240, 320, CV_8UC3);
	Mat flippedImage;

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
        fprintf(stdout, "Angles  : %.2lf %.2lf %.2lf \n", helidata.phi, helidata.psi, helidata.theta);
        fprintf(stdout, "Speeds  : %.2lf %.2lf %.2lf \n", helidata.vx, helidata.vy, helidata.vz);
        fprintf(stdout, "Battery : %.0lf \n", helidata.battery);
        fprintf(stdout, "Hover   : %d \n", hover);
        fprintf(stdout, "Joypad  : %d \n", useJoystick ? 1 : 0);
        fprintf(stdout, "  Roll    : %d \n", joypadRoll);
        fprintf(stdout, "  Pitch   : %d \n", joypadPitch);
        fprintf(stdout, "  Yaw     : %d \n", joypadYaw);
        fprintf(stdout, "  V.S.    : %d \n", joypadVerticalSpeed);
        fprintf(stdout, "  TakeOff : %d \n", joypadTakeOff);
        fprintf(stdout, "  Land    : %d \n", joypadLand);
        fprintf(stdout, "Navigating with Joystick: %d \n", navigatedWithJoystick ? 1 : 0);
        fprintf(stdout, "Click coordinates: (%d, %d) \n", coordinateX, coordinateY);
        if (currentImage.data) 
		{
	        RTemp = currentImage.at<Vec3b>(coordinateY, coordinateX)[2];
	        GTemp = currentImage.at<Vec3b>(coordinateY, coordinateX)[1];
	        BTemp = currentImage.at<Vec3b>(coordinateY, coordinateX)[0];
    	}
    	RED = RTemp;
    	GREEN = GTemp; 
    	BLUE = BTemp;
        fprintf(stdout, "R: %d, G: %d, B: %d", RED, GREEN, BLUE);

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
            // ***************Punto 1***************
            case 'f': 
            snapshot = currentImage;
            imshow("Frozen image", snapshot);
            break;
            // ***************Punto 2***************
            case 'b': 
            snapshot = currentImage;
            //blackNwhite(snapshot, atoi(argv[1]) );
            luminosity(snapshot, bwImage, atoi(argv[1]));
            imshow("Black and White", bwImage);
            break;
            // ***************Invertir**************
            case 'm': 
            snapshot = currentImage;
            flipImageEfficient(snapshot, flippedImage);
            imshow("Original", snapshot);
            imshow("Flipped", flippedImage);
            break;
            // ***************Punto 1***************
            case 27: stop = true; break;
            default: pitch = roll = yaw = height = 0.0;
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
/*
void blackNwhite (Mat &sourceImage, int umbral)
{
	int channels = sourceImage.channels();
	for(int y = 0; y < sourceImage.rows; ++y)
	{
		for(int x = 0; x < sourceImage.cols; ++x)
		{
			for(int i = 0; i < channels; ++i)
			{
				switch(i)
				{
					case 0: sourceImage.at<Vec3b>(y, x)[i] = 0.1 * sourceImage.at<Vec3b>(y, x)[i]; // Blue
					break;
					case 1: sourceImage.at<Vec3b>(y, x)[i] = 0.3 * sourceImage.at<Vec3b>(y, x)[i]; // Green
					break;
					case 2: sourceImage.at<Vec3b>(y, x)[i] = 0.6 * sourceImage.at<Vec3b>(y, x)[i]; // Red
					break;
					default:
					break;
				}				 
			}
			if(sourceImage.at<Vec3b>(y, x)[i] > umbral)
			{	
				sourceImage.at<Vec3b>(y, x)[i] = 255;
			}
			else
			{
				sourceImage.at<Vec3b>(y, x)[i] = 0;
			}
		}
	}
}
*/

void mouseCoordinates(int event, int x, int y, int flags, void* param)
{
    switch (event)
    {
        case CV_EVENT_LBUTTONDOWN:
            //cout << "  Mouse X, Y: " << x << ", " << y ;
            //cout << endl;
            coordinateX = x;
            coordinateY = y;
            /*  Draw a point */
            points.push_back(Point(x, y));
            break;
        case CV_EVENT_MOUSEMOVE:
            break;
        case CV_EVENT_LBUTTONUP:
            break;
    }
}

void luminosity (Mat &sourceImage, Mat &bwImage, int umbral)
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
					case 0: bwImage.at<Vec3b>(y, x)[i] = pond; // Blue
					break;
					case 1: bwImage.at<Vec3b>(y, x)[i] = pond; // Green
					break;
					case 2: bwImage.at<Vec3b>(y, x)[i] = pond; // Red
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

