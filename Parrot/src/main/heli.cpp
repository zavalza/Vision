<<<<<<< HEAD
#include "definitions.h"
typedef list <Segmento *> Segmentos;
map <int, Tabla *> tablaColores;
double thresh = 0;
bool freeze = false;
float pitch;
float roll;
float yaw;
//float heightParrot; 
int deltaClick;
int valBclick,valGclick,valRclick,valBin;
int limInfB = 0, limSupB = 0, limInfG = 0, limSupG = 0, limInfR = 0, limSupR = 0;
Mat currentImage = Mat(height, width, CV_8UC3);
Mat hsvImage = Mat(height, width, CV_8UC3);
Mat new_hsvImage = Mat(height, width, CV_8UC3);
Mat filteredImage = Mat(height, width, CV_8UC3);
Mat coloredImage = Mat(height, width, CV_8UC3);
Mat userInterface = Mat(400, 600, CV_8UC3);
Mat mapa = Mat(480, 640, CV_8UC3, Scalar(255, 255, 255));
Mat ruta = Mat(480, 640, CV_8UC3, Scalar(255, 255, 255));
vector<Mat> hsv_planes;
int RTemp = 0, GTemp = 0, BTemp = 0;
int redClick = 0, greenClick = 0, blueClick = 0;
int hClick = 0, sClick = 0, vClick = 0;
vector<Point> points;
vector<double> phi1Training;
vector<double> phi2Training;
int coordinateX, coordinateY;
bool stop = false;
bool newValues = false;
bool ready = false;
int joypadRoll, joypadPitch, joypadVerticalSpeed, joypadYaw;
bool navigatedWithJoystick, joypadTakeOff, joypadLand, joypadHover;
int acumRoll = 0;
int change;
char rollAnterior = 'z';
int acumPitch = 0;
char pitchAnterior = 'z';
bool seguroAltura =false;
bool reconocer=true;
int alturaActual = 0;
int contadorObjetos = 0;
CRawImage *image;
CHeli *heli;
bool useJoystick;
float heightParrot;
int hover;
bool firstTime = true;
Mat highlightObject(Mat sourceImage);
void flipImageEfficient(const Mat &sourceImage, Mat &destinationImage);
void mouseCoordinates(int event, int x, int y, int flags, void* param);
void veillon(Mat &srcImage);
void tablaPrint();
void matClean(Mat &srcImage);
void pintaSegmento(int y, int xStart, int xEnd, int objectValue);
void objectValue2BGR(Mat &srcImage, Mat &destImage);
void llenaColores(unsigned char Colores[][3], int numeroDeColores);
void training();
void drawColor();
void rawToMat(Mat &destImage, CRawImage* sourceImage);
char comparaObjeto();
void graficaPhis();
void rutinaDespega();
void mediaVuelta();
void mueveHaciaDerecha();
void mueveHaciaDerecha2();
void mueveHaciaDerecha3();
void mueveHaciaIzquierda();
void mueveHaciaIzquierda2();
void mueveHaciaIzquierda3();
void aterrizaSuave();
void rutina1();
void mueveHaciaAtras();
void mueveHaciaAtras2();
void mueveHaciaDerechaAterriza();
void mueveHaciaIzquierdaAterriza();
void dibujaMapa();
unsigned char Colores[50][3];
char objetos[2];
char objetoTemporal='n';
Point centrosDeCelda[3][5];
int distanciaCuatro[3][5];

int main(int argc, char* argv[])
{
	// Required variables
	Moments moments; 
	double hu[7];


	//SDL_Joystick* m_joystick;
	//bool useJoystick;
	//pitch = roll = yaw = height = 0.0;
    //joypadPitch = joypadRoll = joypadYaw = joypadVerticalSpeed = 0.0;

    //Draws the map used for trajectory planning
    dibujaMapa();

    //Elements used later in filtering
	Mat element = getStructuringElement(MORPH_RECT, Size(7, 7), Point(1,1));
	Mat elementErode = getStructuringElement(MORPH_RECT, Size(5, 5), Point(1,1));

	//establishing connection with the quadcopter
	heli = new CHeli();

	Mat imgFiltrada = Mat(height, width, CV_8UC3);
	//this class holds the image from the drone	
	image = new CRawImage(320,240);

	//Use these lines if you want to try the code using a Webcam
	//VideoCapture camera;  
	//camera.open(0);

	//Use these lines if you want to try the code using a source image
	//Windows:
	//Mat testImage = imread("C:/test640.jpg",CV_LOAD_IMAGE_COLOR);
	//Linux:
	//Mat testImage = imread("/media/OS/test3Y.jpg",CV_LOAD_IMAGE_COLOR);


	// Initial values for control	
    pitch = roll = yaw = heightParrot = 0.0;
    joypadPitch = joypadRoll = joypadYaw = joypadVerticalSpeed = 0;

	// Destination OpenCV Mat	
	Mat snapshot = Mat(height, width, CV_8UC3);
	Mat bwImage = Mat(height, width, CV_8UC3);
	Mat yiqImage = Mat(height, width, CV_8UC3);
	//Mat histImage = Mat(240,320, CV_8UC3);
	namedWindow("ParrotCam");
    setMouseCallback("ParrotCam", mouseCoordinates);
	// Show it	
	imshow("ParrotCam", currentImage);


    // Initialize joystick
    //SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);
    //useJoystick = SDL_NumJoysticks() > 0;
    //if (useJoystick)
    //{
    //    SDL_JoystickClose(m_joystick);
    //    m_joystick = SDL_JoystickOpen(0);
    //}

    while (stop == false)
    {

        // Clear the console on Linux
        printf("\033[2J\033[1;1H");

        //Clear the console on Windows
		//system("cls");

        //if (useJoystick)
        //{
        //    SDL_Event event;
        //    SDL_PollEvent(&event);

        //    joypadRoll = SDL_JoystickGetAxis(m_joystick, 2);
        //    joypadPitch = SDL_JoystickGetAxis(m_joystick, 3);
        //    joypadVerticalSpeed = SDL_JoystickGetAxis(m_joystick, 1);
        //    joypadYaw = SDL_JoystickGetAxis(m_joystick, 0);
        //    joypadTakeOff = SDL_JoystickGetButton(m_joystick, 1);
        //    joypadLand = SDL_JoystickGetButton(m_joystick, 2);
        //    joypadHover = SDL_JoystickGetButton(m_joystick, 0);
        //}

        // prints the drone telemetric data, helidata struct contains drone angles, speeds and battery status
        printf("===================== Proyecto Vision =====================\n\n");
        fprintf(stdout, "Angles  : %.2lf %.2lf %.2lf \n", helidata.phi, helidata.psi, helidata.theta);
        fprintf(stdout, "Speeds  : %.2lf %.2lf %.2lf \n", helidata.vx, helidata.vy, helidata.vz);
        fprintf(stdout, "Battery : %.0lf%% \n", helidata.battery);
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
        fprintf(stdout, "R: %d, G: %d, B: %d\n", redClick, greenClick, blueClick);
		fprintf(stdout, "H: %d, S: %d, V: %d\n", hClick, sClick, vClick);
		//namedWindow("Proyecto de Visión para Robots");
		
		//If we have already an object in the color Table, print it with the moments of OpenCV
		 if(!tablaColores.empty())
		{
			//tablaPrint();
			cout << endl << "Momentos correctos" << endl;
			cout << "m00 " << moments.m00 << endl;
			cout << "Centro(" << moments.m10 / moments.m00 << ", " << moments.m01 / moments.m00 << ")" <<  endl;
			cout << "m10 " << moments.m10 << endl;
			cout << "m01 " << moments.m01 << endl;
			cout << "m11 " << moments.m11 << endl;
			cout << "m20 " << moments.m20 << endl;
			cout << "m02 " << moments.m02 << endl;
			cout << "mu20 " << moments.mu20 << endl;
			cout << "mu02 " << moments.mu02 << endl;
			cout << "mu11 " << moments.mu11 << endl;
			cout << "nu20 " << moments.nu20 << endl;
			cout << "nu02 " << moments.nu02 << endl;
			cout << "nu11"  << moments.nu11 << endl;
			cout << "phi1 " << hu[0] << endl;
			cout << "phi2 " << hu[1] << endl;
			cout << "e " << moments.nu20 / moments.nu02 << endl;
		}

		//image is captured
		heli->renewImage(image);
        
		//In case of WebCam
		//camera >> currentImage;
		
		//In case of source file
		//currentImage = testImage;
		
		// Copy to OpenCV Mat
		rawToMat(currentImage, image);
		imshow("ParrotCam", currentImage);


		//These are the keyboard options in order to interact with the vision system. See the ReadMe File.
        char key = waitKey(100);
		switch (key) {
			case 'a': mueveHaciaDerecha(); break;
			case 'd': mueveHaciaIzquierda(); break;
			case 'w': mueveHaciaAtras(); break;
			// case 's': heightParrot = 20000.0; break;
			// case 'q': heli->takeoff(); break;
			// case 'e': heli->land(); break;
			case 'g': graficaPhis();break;
			case 'z': heli->switchCamera(0); break;
			case 'x': heli->switchCamera(1); break;
			case 'c': heli->switchCamera(2); break;
			case 'v': heli->switchCamera(3); break;
			case 'j': roll = -20000.0; break;
			case 'l': roll = 20000.0; break;
			case 'i': pitch = -20000.0; break;
			case 'k': pitch = 20000.0; break;
            case 'h': hover = (hover + 1) % 2; break;
            case 'f': freeze = !freeze;  break;
            case 't': training(); break;
            case 'r': rutinaDespega();break;
            case 'm': aterrizaSuave();break;
            case 27: stop = true; break;
            default: pitch = roll = yaw = heightParrot = 0.0;
            
		}

		if(!freeze){
			
			//In case you want ot highlight an object based on its RGB values
			//filteredImage = highlightObject(currentImage);
			//imshow("Filtered image", filteredImage);
			
			//Using a smooth filter in order to reduce noise
			//GaussianBlur(currentImage, currentImage, cv::Size (3, 3),0);
			medianBlur(currentImage, currentImage, 5);
			//Change the image to HSV and store it on hsvImage
			cvtColor(currentImage, hsvImage, CV_BGR2HSV);

			//cvtColor(currentImage, bwImage, CV_BGR2GRAY);
			//imshow("BW", bwImage);
			//imshow("HSV",hsvImage);


			/// Separate the HSV image in 3 planes
			split( hsvImage, hsv_planes );


			//threshold(bwImage, yiqImage, 30, 255, THRESH_BINARY);
			//imshow("Bin", yiqImage);
			
			
			Mat filter_mask = Mat(height, width, CV_8UC1);
			vector <Mat> dividedHSV(3);
			dividedHSV[0] = hsv_planes[0];
			dividedHSV[1] = hsv_planes[1];
			dividedHSV[2] = hsv_planes[2];
			//Threshold each of the hsv channels with the value obtained by the user's click plus or minus a constant
			int limInfH = 12;
			int limSupH = 12;
			if(hClick < limInfH)
			{
				limInfH = hClick;
			}
			else if(hClick + limSupH > 255)

			{
				limSupH = 255 - hClick;
			}  
			inRange(hsvImage, Scalar(hClick - limInfH, 40, 40), Scalar(hClick + limSupH, 255, 255), filter_mask);
				
			dividedHSV[2]=hsv_planes[2].mul(filter_mask / 255);
			
			//In case you want to see how the HSV were filtered
			merge(dividedHSV,new_hsvImage);
			//imshow("NEW HSV", new_hsvImage);
			
			//Use of an open morhphological filter
			morphologyEx(filter_mask, filter_mask, MORPH_OPEN, element);
			//erode(filter_mask,filter_mask,elementErode);


			if(!firstTime)
			{
				//imshow("Filter Mask", filter_mask);
				//imwrite("C:/Users/Paulz/Documents/8voSemestre/mask.jpg",filter_mask);
				matClean(filter_mask);
				llenaColores(Colores,10);
				coloredImage = Mat::zeros(height,width,CV_8UC3);
				userInterface = Mat::zeros(400,600,CV_8UC3);
				veillon(filter_mask);
				objectValue2BGR(coloredImage, coloredImage);
				
				//imwrite("C:/Users/Paulz/Documents/8voSemestre/colored.jpg",coloredImage);

				//We have now all the regions of the image, we will now process the regions.
				firstTime=true;
				moments = cv::moments(filter_mask, true);
				HuMoments(moments, hu);
				if(!points.empty())
				{
					for (unsigned int i = 0; i < points.size(); ++i) 
					{
						coloredImage.at<Vec3b>(points[i])[0]=127;
						coloredImage.at<Vec3b>(points[i])[1]=127;
						coloredImage.at<Vec3b>(points[i])[2]=127;

					}
					imshow("Colored Image", coloredImage);
					if(!points.empty())
					{
						for (unsigned int i = 0; i < points.size(); ++i)
						{
							//rectangle(coloredImage,points[i] - Point(2,10), points[i] + Point(2,10),Scalar(127,127,127),-1,CV_AA);
							//rectangle(coloredImage,points[i] - Point(10,2), points[i] + Point(10,2),Scalar(127,127,127),-1,CV_AA);
							circle(coloredImage, (Point) points[i], 5, Scalar(127, 127, 127), -1);
						}
						points.clear();
						drawColor();
						imshow("Estadisticos", userInterface);
						imshow("Colored Image", coloredImage);
					}

					//Looks for an interpretation of the region according to its phi1 and phi2 values
					 objetoTemporal = comparaObjeto();
					 if(objetoTemporal!='n')
					 	objetos[contadorObjetos-1] = objetoTemporal;
					//contadorObjetos--;

					if(contadorObjetos==2)
					{
						ruta = mapa;
						destroyWindow("Mapa");
						imshow("Ruta", ruta);
						Point posicionParrot;
						for(int i = 0; i < 2; i++)
						{
							switch(objetos[i])
							{
								case 'r': 
								line(ruta, centrosDeCelda[1][0], centrosDeCelda[0][0], Scalar(0,0, 255), 2 );
								mueveHaciaDerecha();usleep(1000);
								line(ruta,centrosDeCelda[0][0], centrosDeCelda[0][2],Scalar(0,0, 255), 2);
								mueveHaciaAtras();usleep(1000);
								heli -> setAngles(0, 0, 0, 0, 1);usleep(2000000);
								posicionParrot=centrosDeCelda[0][2];
								break; //rectánguldestr
								case 't': 
									if(objetos[i-1]=='r')
										{
											line(ruta, posicionParrot, centrosDeCelda[1][2],Scalar(0,0, 255), 2 );
											mueveHaciaIzquierda2();
										}
										
									else
										{
											line(ruta, posicionParrot, centrosDeCelda[1][2],Scalar(0,0, 255), 2 );
											mueveHaciaDerecha3();
										}
										
									aterrizaSuave();
									imshow("Ruta", ruta);
									break;  //letra T
								case 'h':
								line(ruta, centrosDeCelda[1][0], centrosDeCelda[2][0], Scalar(0,0, 255), 2 );
								mueveHaciaIzquierda();usleep(1000);
								line(ruta,centrosDeCelda[2][0], centrosDeCelda[2][2],Scalar(0,0, 255), 2);
								mueveHaciaAtras(); usleep(1000);
								heli -> setAngles(0, 0, 0, 0, 1);usleep(2000000);
								posicionParrot=centrosDeCelda[2][2];
								break;  //letra H
								case '3': 
								line(ruta, posicionParrot, Point(posicionParrot.x+m(2),posicionParrot.y), Scalar(0,0, 255), 2 );
								mueveHaciaAtras2(); usleep(1000);
									if(objetos[i-1]=='r')
									{
										line(ruta, Point(posicionParrot.x+m(2),posicionParrot.y), centrosDeCelda[1][4],Scalar(0,0, 255), 2 );
										mueveHaciaIzquierda3();
									}
										
									else
									{
										mueveHaciaDerecha2();
										line(ruta, Point(posicionParrot.x+m(2),posicionParrot.y), centrosDeCelda[1][4],Scalar(0,0, 255), 2 );
									}
										
									aterrizaSuave();
									imshow("Ruta", ruta);
									break;  //triangulo
								default: break;
							}
						}

					}
					
					points.clear();
				}
			}
			cvtColor(new_hsvImage,filteredImage,CV_HSV2BGR);


			
			//dilate(filteredImage,filteredImage, Mat());
			//firstTime?__nop():imshow("Filtered image", filteredImage);
			//newValues=false;
			//}
			//threshold(hsv_planes[1], hsv_planes[1],40, 255,THRESH_BINARY);
			//threshold(hsv_planes[2], hsv_planes[2],50, 255,THRESH_BINARY);
    	}
		//Sleep(15); Windows
        usleep(15000);
	}
	heli->land();
	//SDL_JoystickClose(m_joystick);
	delete heli;
	delete image;
	//camera.release();
	return 0;
}

//Indicates the center of the region with  a circle and writes the moments values in the user interface
void drawColor()
{
	estadisticos tM; //tempMoments
	int i = 0;
	double X, Y;
	for(map<int,Tabla *>::iterator it = tablaColores.begin(); it != tablaColores.end(); ++it)
	{
		tM = it->second->getEstadisticos();
		X = tM.centro.getX();
		Y = tM.centro.getY();
		// Escribe estadisticos
		putText(userInterface, format("centro  %.3f, %.3f", X, Y), Point(270 * i, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface,format("angle %.3f", (tM.theta)), Point(270 * i, 40), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface, format("m00  %.3f", (tM.m00)), Point(270 * i, 60), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface, format("m10  %.3f", (tM.m10)), Point(270 * i, 80), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface, format("m01  %.3f", (tM.m01)), Point(270 * i, 100), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface, format("m11  %.3f", (tM.m11)), Point(270 * i, 120), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface, format("m20  %.3f", (tM.m20)), Point(270 * i, 140), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface, format("m02  %.3f", (tM.m02)), Point(270 * i, 160), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface, format("mu20  %.3f", (tM.mu20)), Point(270 * i, 180), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface, format("mu02  %.3f", (tM.mu02)), Point(270 * i, 200), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface, format("mu11  %.3f", (tM.mu11)), Point(270 * i, 220), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface, format("nu20  %.3f", (tM.nu20)), Point(270 * i, 240), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface, format("nu02  %.3f", (tM.nu02)), Point(270 * i, 260), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface, format("nu11  %.3f", (tM.nu11)), Point(270 * i, 280), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface, format("phi1  %.3f", (tM.phi1)), Point(270 * i, 300), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface, format("phi2  %.3f", (tM.phi2)), Point(270 * i, 320), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		putText(userInterface, format("e  %.3f", (tM.nu20 / tM.nu02)), Point(270 * i, 340), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
		// Dibuja linea de orientacion
		//line(imgRegiones,Point(centroide.x-100*cos(theta),centroide.y-100*sin(theta)),Point(centroide.x+100*cos(theta),centroide.y+100*sin(theta)),	Scalar(0,255,0),2);
		//line(imgRegiones,Point(centroide.x-50*cos(PI/2+(double)theta),centroide.y-50*sin(PI/2+theta)),Point(centroide.x+50*cos(PI/2+(double)theta),centroide.y+50*sin(PI/2+theta)),Scalar(255,0,0),2);
		line(coloredImage, Point(X - 100 * cos(tM.theta),Y - 100 * sin(tM.theta)), Point(X + 100 * cos(tM.theta),Y + 100 * sin(tM.theta)), Scalar(0, 127, 255), 2);
		line(coloredImage, Point(X - 50 * cos(PI/2 + tM.theta),Y - 50 * sin(PI/2 + tM.theta)), Point(X + 50 * cos(PI/2 + tM.theta),Y + 50 * sin(PI/2 + tM.theta)), Scalar(255, 127, 0), 2);
		++i;
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
		
			hClick = hsvImage.at<Vec3b>(y, x)[0];
	        sClick = hsvImage.at<Vec3b>(y, x)[1];
	        vClick = hsvImage.at<Vec3b>(y, x)[2];
			firstTime=false;

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
        	phi1Training.pop_back();
        	phi2Training.pop_back();
        	coloredImage = Mat::zeros(height, width, CV_8UC3);
        	imshow("Colored Image", coloredImage);
            break;
        case CV_EVENT_MBUTTONDOWN:
        	phi1Training.clear();
        	phi2Training.clear();
        	coloredImage = Mat::zeros(height, width, CV_8UC3);
        	imshow("Colored Image", coloredImage);
        	break;

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

void dibujaMapa()
{

	// 640 * 480

	circle(mapa, Point(m(2.10), m(2.18)), m(0.365), Scalar(0, 0, 0), -1, 8, 0);
	circle(mapa, Point(m(2.10), m(2.18)), m(0.055), Scalar(0, 207, 31), -1, 8, 0);

	circle(mapa, Point(m(4.10), m(2.18)), m(0.365), Scalar(0, 0, 0), -1, 8, 0);
	circle(mapa, Point(m(4.10), m(2.18)), m(0.055), Scalar(0, 207, 31), -1, 8, 0);
	
	//Dibujo de celdas
	for(int celdaEnLargo = 0; celdaEnLargo < 5; celdaEnLargo++)
	{
		for(int celdaEnAncho = 0; celdaEnAncho<3;celdaEnAncho++)
		{	
			centrosDeCelda[celdaEnAncho][celdaEnLargo]=Point(m(1*celdaEnLargo)+m(1.2),m(1*celdaEnAncho)+m(1.18));
			rectangle(mapa, Rect(m(1*celdaEnLargo)+m(.7),m(1*celdaEnAncho)+m(.68),m(1), m(1)),Scalar(255, 0, 255), 2, 8, 0);
			if(celdaEnAncho!=1)
			{
				distanciaCuatro[celdaEnAncho][celdaEnLargo]=celdaEnLargo+1;
				putText(mapa, format("%i",celdaEnLargo+1),Point(m(1*celdaEnLargo)+m(1.2),m(1*celdaEnAncho)+m(1.18)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0,0,0));
			}
			else if (celdaEnAncho==1 && celdaEnLargo==2)
			{
				distanciaCuatro[celdaEnAncho][celdaEnLargo]=4;
				putText(mapa, "4",Point(m(1*celdaEnLargo)+m(1.2),m(1*celdaEnAncho)+m(1.18)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0,0,0));
			}
				
			else if (celdaEnAncho==1 && celdaEnLargo==4)
			{
				distanciaCuatro[celdaEnAncho][celdaEnLargo]=6;
				putText(mapa, "6",Point(m(1*celdaEnLargo)+m(1.2),m(1*celdaEnAncho)+m(1.18)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0,0,0));
			}
			else
				distanciaCuatro[celdaEnAncho][celdaEnLargo]=0;
				
		}
	}

	rectangle(mapa, Point(0, m(1.965)), Point(m(0.46), m(2.395)), Scalar(0, 255, 255), 2, 8, 0);

	rectangle(mapa, Point(m(0.7), m(1.88)), Point(m(1.26), m(2.48)), Scalar(0, 255, 255), 2, 8, 0);
	imshow("Mapa", mapa);
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


int sumaRecursiva(int iz, int der, int caso)
{
	int suma = 0;
	switch(caso)
	{
		case 1:
			for(int i = iz;i <= der; i++)
				suma += i;
			return suma;
			break;
		case 2:
			for(int i = iz;i <= der; i++)
				suma += i * i;
			return suma;
			break;
	}
	return 0;
}

double otraSumaRecursiva(int iz, int der, int y)
{
	double suma = 0;
	for(int i = iz; i <= der; i++)
	{
		suma += i * y;
	}
	return suma;
}
/*int sumaRecursiva(int iz, int der)
{
	if(iz == der)
	{
		return der;
	}
	else
	{
		return iz + sumaRecursiva(iz + 1, der);
	}
}
*/
void tablaPrint()
{
	for(map<int, Tabla *>::iterator it = tablaColores.begin(); it != tablaColores.end(); ++it)
	{
		it->second->printTabla();
		//fprintf(stdout, "counter = %d\n", it->second->getCount());
		//cout << "counter = " <<  it->second->getCount() << endl;
	}
}


void rutinaDespega() //Rutina para tomar primer objeto
{        
    //heli -> setAngles(0,0,0,0,0);
    heli -> setAngles(1000, -2800, 0, 0, 0); //calibracion pila vieja, elimina ladeo izq -2800 
    heli->takeoff();
    //cout << "Take Off" << endl;
    usleep(50000);
    heli -> setAngles(0, 0, 0, 0, 1);
}

void mueveHaciaDerecha()
{
	heli -> setAngles(0,4000,0,0,0);
	cout<<"Derecha";
	usleep(1500000);
}

void mueveHaciaDerecha2()
{
	heli -> setAngles(0,4000,0,0,0);
	cout<<"Derecha";
	usleep(300000);
}

void mueveHaciaDerecha3()
{
	heli -> setAngles(0,4000,0,0,0);
	cout<<"Derecha";
	usleep(300000);
}
void mueveHaciaDerechaAterriza()
{
	heli -> setAngles(0,4000,0,0,0);
	usleep(50000);
}

void mueveHaciaIzquierda()
{
	heli -> setAngles(0,-4000,0,0,0);
	cout<<"Izquierda";
	usleep(1500000);
}

void mueveHaciaIzquierda2()
{
	heli -> setAngles(0,-4000,0,0,0);
	cout<<"Izquierda";
	usleep(500000);
}

void mueveHaciaIzquierda3()
{
	heli -> setAngles(0,-4000,0,0,0);
	cout<<"Izquierda";
	usleep(500000);
}

void mueveHaciaIzquierdaAterriza()
{
	heli -> setAngles(0,-4000,0,0,0);
	usleep(50000);
}

void mueveHaciaAtras()
{
	heli -> setAngles(4000,0,0,0,0);
	usleep(2500000);
}
void mueveHaciaAtras2()
{
	heli -> setAngles(4000,0,0,0,0);
	usleep(1500000);
}
void mediaVuelta()
{
	//heli -> setAngles(pitch,roll,yaw,verticalSpeed,hover);
	heli -> setAngles(0,0,-6500,0,1);
	usleep(6000000);
}

void rutina1()
{
	//heli -> setAngles(pitch,roll,yaw,verticalSpeed,hover);
	heli -> setAngles(0,0,-6000,0,1);
	cout << "Giro" << endl;
	usleep(6500000);
	//heli->setAngles(-3000,0,0,0,0);
	//cout << "Adelante" << endl;
	//usleep(2500000);
}

void aterrizaSuave()
{
	//heli -> setAngles(0,0,0,0,1);
    //heli -> setAngles(0,0,10000,0,1); // calibracion pila vieja, elimina ladeo izq
    usleep(2000000);
    heli -> setAngles(-250,-4000,0,-4000,1); // calibracion pila vieja, elimina ladeo izq
    cout << "Se estabiliza 5 segundos" << endl;
    cout << "Aterriza" << endl;
    heli->land(); 	
}

//Segmentation Algorithm, receives an image and return the same image, but with a table(TablaColores) filled with the recognized regions.
void veillon(Mat &srcImage)
{
	list <Segmento> filas[height];
	Segmentos *ptrSegmentos = (Segmentos *) &filas[0];
	Segmentos *ptrActual = (Segmentos *) &filas[0];
	Segmentos *nextPtrSegmentos = (Segmentos *) &filas[1];
	list <Segmento*>::iterator itSegmentos = ptrSegmentos->begin();
	list <Segmento*>::iterator nextitSegmentos = nextPtrSegmentos->begin();
	list <Segmento*>::iterator itAuxSegmentos = ptrActual->begin();

	bool izquierda = true;
	bool analizaCiclos = false;
	Punto puntoAux(0,0);
	int id = 0;
	map<int, Tabla *>::iterator itMap;

	// Momentos estadisticos
	double m00, m01, m10, m11, m20, m02;
	double mu20, mu02, mu11;
	double nu20, nu02, nu11;
	double phi1, phi2;
	
	tablaColores.clear();
	for (int i = 0; i < height; ++i)
	{
		if(i == 1)
		{
			analizaCiclos = true;
		}
		izquierda = true;
		for (int j = 0; j < width; ++j)
		{
			if(i == 0 && j == 0 && srcImage.at<uchar>(i, j) != 0/* mat[i][j] == 1*/)	// Si esta en el origen
			{
				ptrSegmentos -> push_back(new Segmento(j, 0, 0));
				itSegmentos = ptrSegmentos->begin();
				izquierda = false;
				if(srcImage.at<uchar>(i,j+1) == 0 /*mat[i][j + 1] == 0*/) // Si es un solo pixel 
				{
					(*itSegmentos)->setRight(j);
					izquierda = true;
				}

			}

			else if(j == 0)	// Si esta en la primera columna no se revisa a la izquierda
			{
				if(srcImage.at<uchar>(i,j) != 0 /*mat[i][j] == 1*/)
				{
					ptrSegmentos -> push_back(new Segmento(j, 0, 0));
					itSegmentos = ptrSegmentos->end();
					--itSegmentos;
					izquierda = false;
					if(srcImage.at<uchar>(i,j+1) == 0 /*mat[i][j + 1] == 0*/)	// Si el ancho es un pixel
					{
						(*itSegmentos)->setRight(j);
						izquierda = true;
					}
				}
			}
			else if(j > 0)
			{

				if(izquierda && srcImage.at<uchar>(i,j) != 0 && srcImage.at<uchar>(i,j-1) == 0/*mat[i][j] == 1 && mat[i][j - 1] == 0*/ && j < width - 1)	// Limite izquierdo
				{
					ptrSegmentos -> push_back(new Segmento(j, 0, 0));
					itSegmentos = ptrSegmentos->end();
					--itSegmentos;
					izquierda = false;
					if(srcImage.at<uchar>(i,j+1) == 0 /*mat[i][j + 1] == 0*/)	//Si el ancho es un pixel
					{
						(*itSegmentos)->setRight(j);
						izquierda = true;
					}
				}
				else if(!izquierda && srcImage.at<uchar>(i,j) != 0 /*mat[i][j] == 1*/&& srcImage.at<uchar>(i,j+1) == 0 /*mat[i][j + 1] == 0*/ && j < width)	// // Limite derecho
				{
					(*itSegmentos)->setRight(j);
					izquierda = true;
				}

				if(j == width - 1)	// Caso ultima columna
				{
					if(izquierda && j == width - 1 && srcImage.at<uchar>(i,j) != 0 /*mat[i][j] == 1*/ && srcImage.at<uchar>(i,j-1) == 0 /*mat[i][j-1] == 0*/)	// Caso ultima columna
					{
						ptrSegmentos -> push_back(new Segmento(j, j, 0));
						itSegmentos = ptrSegmentos->end();
						--itSegmentos;
						izquierda = true;
					}
					else if(!izquierda && srcImage.at<uchar>(i,j) != 0 /*mat[i][j] == 1*/)	// Se completa el segmento en la ultima columna
					{
						(*itSegmentos)->setRight(j);
						izquierda = true;
					}
				}

			}
		}
		
		if(analizaCiclos)
		{
			// ptrSegmentos i + 1
			nextPtrSegmentos = ptrSegmentos;
			nextitSegmentos = nextPtrSegmentos->begin(); // i + 1
			ptrActual = nextPtrSegmentos;
			--ptrActual;
			//ptrActual = nextPtrSegmentos;
			itSegmentos = ptrActual->begin();		// i
			
			if(!(ptrActual->empty() && nextPtrSegmentos->empty()))
			{
				while(itSegmentos != ptrActual->end() || nextitSegmentos != nextPtrSegmentos->end())
				{
					//Inicio

					if(!(nextPtrSegmentos->empty() || nextitSegmentos == nextPtrSegmentos->end()) && (ptrActual->empty() || (!(ptrActual->empty()) && (*itSegmentos)->getLeft() > (*nextitSegmentos)->getRight())))
					{
						(*nextitSegmentos)->setObject(id);
						tablaColores.insert(pair <int, Tabla*> (id, new Tabla(id, (*nextitSegmentos)->getRight() - (*nextitSegmentos)->getLeft() + 1 )));
						++id; 
						// No se debe de eliminar i + 1 !
						++nextitSegmentos;
					}
					else if(!(nextPtrSegmentos->empty() || nextitSegmentos == nextPtrSegmentos->end()) && traslape)
					{
						if( ( (*nextitSegmentos)->getTouched() == false) && (*itSegmentos)->getTouched() == false) //Revisar si nadie esta tocado
						{ 
                        
							if((*nextitSegmentos)->getRight() > (*itSegmentos)->getRight()) //Posible Bloque cerradura
                            {
                                (*nextitSegmentos)->setTouched(true);
                                (*nextitSegmentos)->setObject((*itSegmentos)->getObject());
                                tablaColores[(*itSegmentos)->getObject()]->addArea((*nextitSegmentos)->getRight() - (*nextitSegmentos)->getLeft() + 1);
								tablaColores[(*itSegmentos)->getObject()]->addCentro(sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),1), ((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1)); //mx,my
								tablaColores[(*itSegmentos)->getObject()]->addm20((sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),2)));	//sum(x^2)
								tablaColores[(*itSegmentos)->getObject()]->addm02(((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1) * (i-1));	//sum(y^2)
								tablaColores[(*itSegmentos)->getObject()]->addm11(otraSumaRecursiva((*itSegmentos)->getLeft(), (*itSegmentos)->getRight(), i-1)); // sum(x*y)
								pintaSegmento(i, (*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),(*itSegmentos)->getObject());
								itAuxSegmentos = itSegmentos;
                                ++itSegmentos;
								ptrActual->erase(itAuxSegmentos);
								if(itSegmentos == ptrActual->end() || !traslape)
								{
									(*nextitSegmentos)->setTouched(false);
									++nextitSegmentos;
								}
								//posibleCerradura = true;
                            }
							else if((*itSegmentos)->getRight() > (*nextitSegmentos)->getRight()) //Posible Bloque apertura
							{ 
                                (*itSegmentos)->setTouched(true);
                                (*nextitSegmentos)->setObject((*itSegmentos)->getObject());
                                tablaColores[(*itSegmentos)->getObject()]->addArea((*nextitSegmentos)->getRight() - (*nextitSegmentos)->getLeft()+1);
                                ++nextitSegmentos;
								if(nextitSegmentos == nextPtrSegmentos->end() || !traslape)
								{
									tablaColores[(*itSegmentos)->getObject()]->addCentro(sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),1), ((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1));
									tablaColores[(*itSegmentos)->getObject()]->addm20((sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),2)));	//sum(x^2)
									tablaColores[(*itSegmentos)->getObject()]->addm02(((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1) * (i-1));	//sum(y^2)
									tablaColores[(*itSegmentos)->getObject()]->addm11(otraSumaRecursiva((*itSegmentos)->getLeft(), (*itSegmentos)->getRight(), i-1)); // sum(x*y)
									pintaSegmento(i, (*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),(*itSegmentos)->getObject());
									itAuxSegmentos = itSegmentos;
									++itSegmentos;
									ptrActual->erase(itAuxSegmentos);
								} 
								//posibleApertura = true;
                            }
				
							else // Seguimiento simple
							{
								(*nextitSegmentos)->setObject((*itSegmentos)->getObject());
								tablaColores[(*itSegmentos)->getObject()]->addArea((*nextitSegmentos)->getRight() - (*nextitSegmentos)->getLeft() + 1);
								tablaColores[(*itSegmentos)->getObject()]->addCentro(sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),1), ((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1));
								tablaColores[(*itSegmentos)->getObject()]->addm20((sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),2)));	//sum(x^2)
								tablaColores[(*itSegmentos)->getObject()]->addm02(((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1) * (i-1));	//sum(y^2)
								tablaColores[(*itSegmentos)->getObject()]->addm11(otraSumaRecursiva((*itSegmentos)->getLeft(), (*itSegmentos)->getRight(), i-1)); // sum(x*y)
								pintaSegmento(i, (*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),(*itSegmentos)->getObject());
								itAuxSegmentos = itSegmentos;
								++itSegmentos;
								ptrActual->erase(itAuxSegmentos);
								++nextitSegmentos;
							}
                        }
						else if ((*itSegmentos)->getTouched() == true && !((*nextitSegmentos)->getLeft() > (*itSegmentos)->getRight())) //Existe bloque apertura
						{ 
							tablaColores[(*itSegmentos)->getObject()]->incrementCount();
							(*nextitSegmentos)->setObject((*itSegmentos)->getObject());
                            tablaColores[(*itSegmentos)->getObject()]->addArea((*nextitSegmentos)->getRight() - (*nextitSegmentos)->getLeft()+1);
                            //Se propaga tocado
							if((*nextitSegmentos)->getRight() > (*itSegmentos)->getRight())
                            {
                                (*nextitSegmentos)->setTouched(true);
                                (*itSegmentos)->setTouched(false);
                                ++itSegmentos;
                            }
							else if((*nextitSegmentos)->getRight() == (*itSegmentos)->getRight())
							{
								(*nextitSegmentos)->setTouched(false);
								tablaColores[(*itSegmentos)->getObject()]->addCentro(sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),1), ((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1));
								tablaColores[(*itSegmentos)->getObject()]->addm20((sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),2)));	//sum(x^2)
								tablaColores[(*itSegmentos)->getObject()]->addm02(((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1) * (i-1));	//sum(y^2)
								tablaColores[(*itSegmentos)->getObject()]->addm11(otraSumaRecursiva((*itSegmentos)->getLeft(), (*itSegmentos)->getRight(), i-1)); // sum(x*y)
								pintaSegmento(i, (*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),(*itSegmentos)->getObject());
								itAuxSegmentos = itSegmentos;
								++itSegmentos;
								ptrActual->erase(itAuxSegmentos);
								++nextitSegmentos;
							}
                            else 
							{
								(*nextitSegmentos)->setTouched(false);
								++nextitSegmentos;   
							}
							
							if(itSegmentos == ptrActual->end() && nextitSegmentos == nextPtrSegmentos->end())
							{
								;
							}
							else if(nextitSegmentos == nextPtrSegmentos->end())
							{
								tablaColores[(*itSegmentos)->getObject()]->addCentro(sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),1), ((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1));
								tablaColores[(*itSegmentos)->getObject()]->addm20((sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),2)));	//sum(x^2)
								tablaColores[(*itSegmentos)->getObject()]->addm02(((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1) * (i-1));	//sum(y^2)
								tablaColores[(*itSegmentos)->getObject()]->addm11(otraSumaRecursiva((*itSegmentos)->getLeft(), (*itSegmentos)->getRight(), i-1)); // sum(x*y)
								pintaSegmento(i, (*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),(*itSegmentos)->getObject());
								itAuxSegmentos = itSegmentos;
								++itSegmentos;
								ptrActual->erase(itAuxSegmentos);
							} 
                        }
                        else //Existe bloque cerradura
						{
							if((*itSegmentos)->getObject() == (*nextitSegmentos)->getObject()) //Si hubo apertura antes
							{
								tablaColores[(*itSegmentos)->getObject()]->decrementCount();
							}
							else 
							{
								// Transfiere los momentos estadisticos calculados al nuevo objeto
								tablaColores[(*itSegmentos)->getObject()]->decrementCount(); // Cuando no hay apertura no se debe hacer
								itMap = tablaColores.find((*itSegmentos)->getObject());	// Iterador que apunta a localidad a borrar
								tablaColores[(*nextitSegmentos)->getObject()]->addArea(itMap->second->getArea());	// Transfiere el area calculada al objeto real
								tablaColores[(*nextitSegmentos)->getObject()]->addCentro(itMap->second->getCentro().getX(),itMap->second->getCentro().getY());
								tablaColores[(*nextitSegmentos)->getObject()]->addm11(itMap->second->getm11());
								tablaColores[(*nextitSegmentos)->getObject()]->addm02(itMap->second->getm02());
								tablaColores[(*nextitSegmentos)->getObject()]->addm20(itMap->second->getm20());
								tablaColores.erase(itMap);	// Borra el objeto falso
								
							}
							for(int c=0; c<3;c++)
							{
								Colores[(*nextitSegmentos)->getObject()+1][c]=Colores[(*itSegmentos)->getObject()+1][c];
							}
							(*itSegmentos)->setObject((*nextitSegmentos)->getObject());	//Pasa el Kolor al segmento de arriba. Se borra
							//Se propaga tocado
                            if((*itSegmentos)->getRight() > (*nextitSegmentos)->getRight())
                            {
                                (*itSegmentos)->setTouched(true);
                                (*nextitSegmentos)->setTouched(false);
                                ++nextitSegmentos; 
                            }
							else if((*itSegmentos)->getRight() == (*nextitSegmentos)->getRight())
							{
								(*nextitSegmentos)->setTouched(false);
								tablaColores[(*itSegmentos)->getObject()]->addCentro(sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),1), ((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1));
								tablaColores[(*itSegmentos)->getObject()]->addm20((sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),2)));	//sum(x^2)
								tablaColores[(*itSegmentos)->getObject()]->addm02(((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1) * (i-1));	//sum(y^2)
								tablaColores[(*itSegmentos)->getObject()]->addm11(otraSumaRecursiva((*itSegmentos)->getLeft(), (*itSegmentos)->getRight(), i-1)); // sum(x*y)
								pintaSegmento(i, (*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),(*itSegmentos)->getObject());
								itAuxSegmentos = itSegmentos;
								++itSegmentos;
								ptrActual->erase(itAuxSegmentos);
								++nextitSegmentos; 
							}
                            else
							{
								tablaColores[(*itSegmentos)->getObject()]->addCentro(sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),1), ((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1));
								tablaColores[(*itSegmentos)->getObject()]->addm20((sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),2)));	//sum(x^2)
								tablaColores[(*itSegmentos)->getObject()]->addm02(((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1) * (i-1));	//sum(y^2)
								tablaColores[(*itSegmentos)->getObject()]->addm11(otraSumaRecursiva((*itSegmentos)->getLeft(), (*itSegmentos)->getRight(), i-1)); // sum(x*y)
								pintaSegmento(i, (*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),(*itSegmentos)->getObject());
								itAuxSegmentos = itSegmentos;
								++itSegmentos;
								ptrActual->erase(itAuxSegmentos);
							}
							if(itSegmentos == ptrActual->end() && nextitSegmentos == nextPtrSegmentos->end())
							{
								;
							}
							else if(nextitSegmentos == nextPtrSegmentos->end())
							{
								tablaColores[(*itSegmentos)->getObject()]->addCentro(sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),1), ((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1));
								tablaColores[(*itSegmentos)->getObject()]->addm20((sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),2)));	//sum(x^2)
								tablaColores[(*itSegmentos)->getObject()]->addm02(((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1) * (i-1));	//sum(y^2)
								tablaColores[(*itSegmentos)->getObject()]->addm11(otraSumaRecursiva((*itSegmentos)->getLeft(), (*itSegmentos)->getRight(), i-1)); // sum(x*y)
								pintaSegmento(i, (*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),(*itSegmentos)->getObject());
								itAuxSegmentos = itSegmentos;
								++itSegmentos;
								ptrActual->erase(itAuxSegmentos);
							}
							else if(itSegmentos == ptrActual->end())
							{
								(*nextitSegmentos)->setTouched(false);
								++nextitSegmentos;
							}
                        }
					}

					// Fin
					else if((nextPtrSegmentos->empty() || nextitSegmentos == nextPtrSegmentos->end()) || ((*itSegmentos)->getRight() < (*nextitSegmentos)->getLeft()))
					{
						if(tablaColores[(*itSegmentos)->getObject()]->getCount() == 0) // Fin real
						{
							tablaColores[(*itSegmentos)->getObject()]->addCentro(sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),1), ((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1));
							tablaColores[(*itSegmentos)->getObject()]->addm20((sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),2)));	//sum(x^2)
							tablaColores[(*itSegmentos)->getObject()]->addm02(((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1) * (i-1));	//sum(y^2)
							tablaColores[(*itSegmentos)->getObject()]->addm11(otraSumaRecursiva((*itSegmentos)->getLeft(), (*itSegmentos)->getRight(), i-1)); // sum(x*y)
							// Calculo de centroide
							tablaColores[(*itSegmentos)->getObject()]->addm10(tablaColores[(*itSegmentos)->getObject()]->getCentro().getX());	//m10
							tablaColores[(*itSegmentos)->getObject()]->addm01(tablaColores[(*itSegmentos)->getObject()]->getCentro().getY());	//m01
							m00 = tablaColores[(*itSegmentos)->getObject()]->getArea();
							m10 = tablaColores[(*itSegmentos)->getObject()]->getm10();
							m01 = tablaColores[(*itSegmentos)->getObject()]->getm01();
							puntoAux.setX(m10 / m00);	//m10/m00
							puntoAux.setY(m01 / m00);	//m01/m00
							tablaColores[(*itSegmentos)->getObject()]->setCentro(puntoAux);
							// Calculo de mu
							m11 = tablaColores[(*itSegmentos)->getObject()]->getm11();
							m20 = tablaColores[(*itSegmentos)->getObject()]->getm20();
							m02 = tablaColores[(*itSegmentos)->getObject()]->getm02();
							tablaColores[(*itSegmentos)->getObject()]->setmu20(m20 - ((m10 * m10) / m00));
							tablaColores[(*itSegmentos)->getObject()]->setmu02(m02 - ((m01 * m01) / m00));
							tablaColores[(*itSegmentos)->getObject()]->setmu11(m11 - ((m01 * m10) / m00));
							// Calculo de nu
							
							mu20 = tablaColores[(*itSegmentos)->getObject()]->getmu20();
							mu02 = tablaColores[(*itSegmentos)->getObject()]->getmu02();
							mu11 = tablaColores[(*itSegmentos)->getObject()]->getmu11();
							tablaColores[(*itSegmentos)->getObject()]->setnu20(mu20 / (m00 * m00));
							tablaColores[(*itSegmentos)->getObject()]->setnu02(mu02 / (m00 * m00));
							tablaColores[(*itSegmentos)->getObject()]->setnu11(mu11 / (m00 * m00));
							// Calculo de phi 
							nu20 = tablaColores[(*itSegmentos)->getObject()]->getnu20();
							nu02 = tablaColores[(*itSegmentos)->getObject()]->getnu02();
							nu11 = tablaColores[(*itSegmentos)->getObject()]->getnu11();
							tablaColores[(*itSegmentos)->getObject()]->setphi1(nu20 + nu02);
							tablaColores[(*itSegmentos)->getObject()]->setphi2(pow((nu20 - nu02), 2) + 4 * pow(nu11, 2));
							// Calculo de theta
							tablaColores[(*itSegmentos)->getObject()]->setTheta();
							// Guarda informacion de entrenamiento
							phi1 = tablaColores[(*itSegmentos)->getObject()]->getphi1();
							phi2 = tablaColores[(*itSegmentos)->getObject()]->getphi2();
							phi1Training.push_back(phi1);
							phi2Training.push_back(phi2);
							// Calculo de fin 
							pintaSegmento(i, (*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),(*itSegmentos)->getObject());
							itAuxSegmentos = itSegmentos;
							++itSegmentos;
							ptrActual->erase(itAuxSegmentos);

							points.push_back(Point((int) puntoAux.getX(),(int) puntoAux.getY()));
							

							// Pendientes mas cosas de fin
						}
						else // Pseudo fin
						{
							tablaColores[(*itSegmentos)->getObject()]->addCentro(sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),1), ((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1));
							tablaColores[(*itSegmentos)->getObject()]->addm20((sumaRecursiva((*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),2)));	//sum(x^2)
							tablaColores[(*itSegmentos)->getObject()]->addm02(((*itSegmentos)->getRight() - (*itSegmentos)->getLeft() + 1) * (i-1) * (i-1));	//sum(y^2)
							tablaColores[(*itSegmentos)->getObject()]->addm11(otraSumaRecursiva((*itSegmentos)->getLeft(), (*itSegmentos)->getRight(), i-1)); // sum(x*y)
							tablaColores[(*itSegmentos)->getObject()]->decrementCount();
							pintaSegmento(i, (*itSegmentos)->getLeft(),(*itSegmentos)->getRight(),(*itSegmentos)->getObject());
							itAuxSegmentos = itSegmentos;
							++itSegmentos;
							ptrActual->erase(itAuxSegmentos);
						}
					}
				}
			}
		}

		++ptrSegmentos;
	}
	ptrSegmentos = (Segmentos *) &filas[0];
}

//Eliminates the borders of the image. These borders may cause problems with the Veillon's Algorithm for segmentation
void matClean(Mat &srcImage)
{
	for(int j = 0; j < srcImage.rows; j++)
	{
		srcImage.at<uchar>(j, 0) = 0;				//Left border
		srcImage.at<uchar>(j,srcImage.cols - 1) = 0; //Right border
	}
	for(int i = 0; i < srcImage.cols; i++)
	{
		srcImage.at<uchar>(0,i) = 0;				//Down Border
		srcImage.at<uchar>(srcImage.rows - 1, i) = 0; //Up Border
	}
	
}
//Takes the number of the object (id) that was stored on each channel of the image and gets the corresponding color (BGR value) of the Colores matrix
void objectValue2BGR(Mat &srcImage, Mat &destImage)
{
	for(int i = 0; i < srcImage.rows; i++)
	{
		for(int j=0;j<srcImage.cols;j++)
		{
			destImage.at<Vec3b>(i, j)[0] = Colores[srcImage.at<Vec3b>(i, j)[0]][0];
			destImage.at<Vec3b>(i, j)[1] = Colores[srcImage.at<Vec3b>(i, j)[1]][1];
			destImage.at<Vec3b>(i, j)[2] = Colores[srcImage.at<Vec3b>(i, j)[2]][2];
		}
	}

}
void pintaSegmento(int y, int xStart, int xEnd, int objectValue)
{
	//unsigned char *colores = &Colores[0][0];
	//unsigned char *datos = (unsigned char*)(coloredImage.data);
	unsigned char *renglon = coloredImage.ptr<unsigned char>(y);
	

	//line(coloredImage, Point(xStart,y), Point(xEnd,y),(Colores[objectValue][0], &Colores[objectValue][1], &Colores[objectValue][2]));
	for(int i = xStart; i <=xEnd; i++)
	{
		(renglon + i * 3)[0] = objectValue + 1;
		(renglon + i * 3)[1] = objectValue + 1;
		(renglon + i * 3)[2] = objectValue + 1;
	}
}

void training()
{
	ofstream statistics, samples;
	string fileName, sampleFileName;
	vector <double> distance;
	double phi1avg = 0;
	double phi2avg = 0;
	double phi1var = 0;
	double phi2var = 0;
	double maxdistance = 0;

	cout << "Enter file name: " << endl;
	cin >> fileName; 
	sampleFileName = fileName + "muestras.txt";
	fileName += ".txt";
	statistics.open(fileName.c_str());
	samples.open(sampleFileName.c_str());
	if(phi1Training.size() == phi2Training.size())
	{
		//cout << "Enter the object file name: " << endl;
		//scanf("%s", fileName);
		//statistics.open(fileName);
		unsigned int size = phi1Training.size();
		for (unsigned int numMuestras = 0; numMuestras < size; ++numMuestras)
		{
			phi1avg += phi1Training[numMuestras];
			phi2avg += phi2Training[numMuestras];
			samples << phi1Training[numMuestras] << "\t" << phi2Training[numMuestras] << endl;
		}
		phi1avg /= size;
		phi2avg /= size;
		statistics << phi1avg << endl;
		statistics << phi2avg << endl;
		for (unsigned int numMuestras = 0; numMuestras < size; ++numMuestras)
		{
			phi1var += pow(phi1Training[numMuestras] - phi1avg, 2);
			phi2var += pow(phi2Training[numMuestras] - phi2avg, 2);
		}
		phi1var /= size - 1;
		phi2var /= size - 1;

		maxdistance = sqrt(phi1var+phi2var);
		statistics << phi1var << endl;
		statistics << phi2var << endl;
		statistics << maxdistance << endl;

		phi1Training.clear();
		phi2Training.clear();
		distance.clear();
		statistics.close();
		samples.close();
	}
	else
	{
		phi1Training.clear();
		phi2Training.clear();
		cout << "Calculations were incorrect\n";
	}
}

//Looks for an interpretation of the region according to its phi1 and phi2 values. These requires a traning to be stored.
char comparaObjeto()
{
	const char* files[] = {"T.txt", "rectNueva.txt", "H.txt", "hNueva.txt", "triangulo.txt"};
	std::fstream infile;
	string dump;
	char result;

	double phi1_in;
	double phi2_in;
	double radioRequerido;
	
	double phi1object;
	double phi2object;
	double radioObjeto;
	for (unsigned int i = 0; i < 5; ++i)
	{
		infile.open(files[i], std::ios_base::in);
		//infile.open((files[i]).c_str());
		infile>>phi1_in>>phi2_in>>dump>>dump>>radioRequerido;

		// getline(infile, phi1)>>phi1_in;
		// getline(infile, phi2)>>phi2_in;
		// getline(infile, dump);
		// getline(infile, dump);
		// getline(infile, maxDistance)>>radioRequerido;

		cout << "Entrenamiento: " << phi1_in << "\t" << phi2_in << "\t" << radioRequerido<< endl;
		infile.close();

		for (map <int, Tabla*>::iterator it = tablaColores.begin(); it != tablaColores.end(); ++it)
		{
			phi1object = it->second->getphi1();
			phi2object = it->second->getphi2();
			radioObjeto = sqrt(pow(phi1object - phi1_in, 2) + pow(phi2object - phi2_in, 2));
			cout << "Objeto: " << phi1object<< "\t" << phi2object << "\t" << radioObjeto<< endl;
			if(radioObjeto <= radioRequerido)
			{
				putText(userInterface, format("Se ha encontrado %s", files[i]) , Point(0, 360), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
				if(reconocer)
					contadorObjetos++;
				imshow("Estadisticos", userInterface);
				 if ( (string)(files[i]) == "T.txt")
				 	result = 't';
				 else if 
				 	( (string)(files[i]) == "rectNueva.txt")
					result = 'r';
				else if (( (string)(files[i]) == "hNueva.txt")||( (string)(files[i]) == "H.txt"))
				{
					result = 'h';
				}
				else if (((string)(files[i]) == "triangulo.txt"))
					result='3';
				else
					result = 'e'; //error
				
				return result;
			}
			else
			{
				//putText(userInterface, format("File: phi1 %d phi2 %d radio %d", phi1_in, phi2_in, radioRequerido) , Point(0, 380), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255));
				//userInterface = Mat::zeros(400,600,CV_8UC3);
				imshow("Estadisticos", userInterface);
				// goto error;
				result = 'n';
			}
		}
	}
	// goto end;
	// error:
	return result;
	// end:
}

void graficaPhis()
{
	PyObject *pName, *pModule, *pFunc;
    PyObject *pValue;
	//Py_SetProgramName(argv[0]);  /* optional but recommended */
 	Py_Initialize();

 	pName = PyString_FromString("graficaPhis.py");
  	pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {
        pFunc = PyObject_GetAttrString(pModule,0);
        /* pFunc is a new reference */

        pValue = PyObject_CallObject(pFunc,0);
        
        if (pValue != NULL) {
            printf("Result of call: %ld\n", PyInt_AsLong(pValue));
            Py_DECREF(pValue);
        }
        else {
            Py_DECREF(pFunc);
            Py_DECREF(pModule);
            PyErr_Print();
            fprintf(stderr,"Call failed\n");
        }

        Py_XDECREF(pFunc);
        Py_DECREF(pModule);

    Py_Finalize();

	//for()
	}
}

//Fills the Colores matrix with the desired number of colors. These colors will be used to display the detected objects
void llenaColores(unsigned char Colores[][3], int numeroDeColores)
{
	int valor = 255 / (numeroDeColores / 6 + 1);
	for (int i=0; i < numeroDeColores / 6 + 1; i++)
	{		
		for(int caso=0; caso < 6; caso++)
		{
			if(i + caso < numeroDeColores)
			{
				if(caso == 0 && i == 0)
				{	
					//Colores in [0][] must be equal to {0,0,0} (Black)
					Colores[0][0]=0;
					Colores[0][1]=0;
					Colores[0][2]=0;
				}
				else
				{

					switch(caso)
					{
						case 0:
							Colores[6 * i + caso][0] = valor * (i + 1);
							Colores[6 * i + caso][1] = 0;
							Colores[6 * i + caso][2] = 0;
							break;
						case 1:
							Colores[6 * i + caso][0] = 0;
							Colores[6 * i + caso][1] = valor * (i + 1);
							Colores[6 * i + caso][2] = 0;
							break;
						case 2:
							Colores[6 * i + caso][0] = 0;
							Colores[6 * i + caso][1] = 0;
							Colores[6 * i + caso][2] = valor * (i + 1);
							break;
						case 3:
							Colores[6 * i + caso][0] = valor * (i + 1);
							Colores[6 * i + caso][1] = valor * (i + 1);
							Colores[6 * i + caso][2] = 0;
							break;
						case 4:
							Colores[6 * i + caso][0] = valor * (i + 1);
							Colores[6 * i + caso][1] = 0;
							Colores[6 * i + caso][2] = valor * (i + 1);
							break;
						case 5:
							Colores[6 * i + caso][0] = 0;
							Colores[6 * i + caso][1] = valor * (i + 1);
							Colores[6 * i + caso][2] = valor * (i + 1);
							break;
						//No se usa el vector de grises caso[i][j]={valor,valor,valor}
						default:
							break;
					}
				}
			}

		}

	}
}
=======
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
	Mat redHistogram = Mat(306,256,CV_8UC3);
	Mat blueHistogram = Mat(306,256,CV_8UC3);
	Mat greenHistogram = Mat(306,256,CV_8UC3);

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
			if(v != 0) //Si el mÃ¡ximo no es cero
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
	for(int y = 0; y < 306; ++y)
		for(int x = 0; x < 256; ++x)
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

	for(int x = 1; x < 256; ++x)
		for(int y = arr[x]; y>0;--y)//for(int y = 0; y < arr[x]; ++y)
		{
			redHistogram.at<Vec3b>(y+49, x)[0] = 0;
			redHistogram.at<Vec3b>(y+49, x)[1] = 0;

		}

	
		//Resaltamos el rango de colores donde se encuentre nuestro color
		//Es decir, donde se detecte el clic se filtrara la gama de color en la que se encuentra
	for(int x = redClick;x>(redClick-8);--x)		
		for(int y = arr[redClick]; y>0;--y)
		{
			redHistogram.at<Vec3b>(y+49, x)[0] = 0;
			redHistogram.at<Vec3b>(y+49, x)[1] = 0;
			redHistogram.at<Vec3b>(y+49, x)[2] = 0;
		}

	for(int x = 1; x<256;x++)
		for(int y = 0; y<50;y++)
		{
			
			redHistogram.at<Vec3b>(y, x)[0] = 0;
			redHistogram.at<Vec3b>(y, x)[1] = 0;
			redHistogram.at<Vec3b>(y, x)[2] = x;


		}


		Mat redHistogramAux = Mat(306,256,CV_8UC3);

	for(int x = 0;x<256;x++)
		for(int y = 0, i = 305; y<306; y++,i--)
		{
			redHistogramAux.at<Vec3b>(i, x)[0] = redHistogram.at<Vec3b>(y, x)[0];
			redHistogramAux.at<Vec3b>(i, x)[1] = redHistogram.at<Vec3b>(y, x)[1];
			redHistogramAux.at<Vec3b>(i, x)[2] = redHistogram.at<Vec3b>(y, x)[2];


		}

		for(int x = 0;x<256;x++)
			for(int y = 0, i = 305; y<306; y++,i--)
		{
			redHistogram.at<Vec3b>(y, x)[0] = redHistogramAux.at<Vec3b>(y, x)[0];
			redHistogram.at<Vec3b>(y, x)[1] = redHistogramAux.at<Vec3b>(y, x)[1];
			redHistogram.at<Vec3b>(y, x)[2] = redHistogramAux.at<Vec3b>(y, x)[2];

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
void generateBlueHistogram (Mat &sourceImage, Mat &blueHistogram)
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
	for(int y = 0; y < 306; ++y)
		for(int x = 0; x < 256; ++x)
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

	for(int x = 1; x < 256; ++x)
		for(int y = arr[x]; y>0;--y)//for(int y = 0; y < arr[x]; ++y)
		{
			blueHistogram.at<Vec3b>(y+49, x)[2] = 0;
			blueHistogram.at<Vec3b>(y+49, x)[1] = 0;

		}
	for(int x = blueClick;x>(blueClick-8);--x)		
		for(int y = arr[blueClick]; y>0;--y)
		{
			blueHistogram.at<Vec3b>(y+49, x)[0] = 0;
			blueHistogram.at<Vec3b>(y+49, x)[1] = 0;
			blueHistogram.at<Vec3b>(y+49, x)[2] = 0;
		}

	for(int x = 1; x<256;x++)
		for(int y = 0; y<50;y++)
		{
			
			blueHistogram.at<Vec3b>(y, x)[0] = x;
			blueHistogram.at<Vec3b>(y, x)[1] = 0;
			blueHistogram.at<Vec3b>(y, x)[2] = 0;


		}


		Mat blueHistogramAux = Mat(306,256,CV_8UC3);

	for(int x = 0;x<256;x++)
		for(int y = 0, i = 305; y<306; y++,i--)
		{
			blueHistogramAux.at<Vec3b>(i, x)[0] = blueHistogram.at<Vec3b>(y, x)[0];
			blueHistogramAux.at<Vec3b>(i, x)[1] = blueHistogram.at<Vec3b>(y, x)[1];
			blueHistogramAux.at<Vec3b>(i, x)[2] = blueHistogram.at<Vec3b>(y, x)[2];


		}

		for(int x = 0;x<256;x++)
			for(int y = 0, i = 305; y<306; y++,i--)
		{
			blueHistogram.at<Vec3b>(y, x)[0] = blueHistogramAux.at<Vec3b>(y, x)[0];
			blueHistogram.at<Vec3b>(y, x)[1] = blueHistogramAux.at<Vec3b>(y, x)[1];
			blueHistogram.at<Vec3b>(y, x)[2] = blueHistogramAux.at<Vec3b>(y, x)[2];

		}




}
void generateGreenHistogram (Mat &sourceImage, Mat &greenHistogram)
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
	for(int y = 0; y < 306; ++y)
		for(int x = 0; x < 256; ++x)
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

	for(int x = 1; x < 256; ++x)
		for(int y = arr[x]; y>0;--y)//for(int y = 0; y < arr[x]; ++y)
		{
			greenHistogram.at<Vec3b>(y+49, x)[0] = 0;
			greenHistogram.at<Vec3b>(y+49, x)[2] = 0;

		}
		for(int x = greenClick;x>(greenClick-8);--x)		
		for(int y = arr[greenClick]; y>0;--y)
		{
			greenHistogram.at<Vec3b>(y+49, x)[0] = 0;
			greenHistogram.at<Vec3b>(y+49, x)[1] = 0;
			greenHistogram.at<Vec3b>(y+49, x)[2] = 0;
		}
	for(int x = 1; x<256;x++)
		for(int y = 0; y<50;y++)
		{
			
			greenHistogram.at<Vec3b>(y, x)[0] = 0;
			greenHistogram.at<Vec3b>(y, x)[1] = x;
			greenHistogram.at<Vec3b>(y, x)[2] = 0;


		}


		Mat greenHistogramAux = Mat(306,256,CV_8UC3);

	for(int x = 0;x<256;x++)
		for(int y = 0, i = 305; y<306; y++,i--)
		{
			greenHistogramAux.at<Vec3b>(i, x)[0] = greenHistogram.at<Vec3b>(y, x)[0];
			greenHistogramAux.at<Vec3b>(i, x)[1] = greenHistogram.at<Vec3b>(y, x)[1];
			greenHistogramAux.at<Vec3b>(i, x)[2] = greenHistogram.at<Vec3b>(y, x)[2];


		}

		for(int x = 0;x<256;x++)
			for(int y = 0, i = 305; y<306; y++,i--)
		{
			greenHistogram.at<Vec3b>(y, x)[0] = greenHistogramAux.at<Vec3b>(y, x)[0];
			greenHistogram.at<Vec3b>(y, x)[1] = greenHistogramAux.at<Vec3b>(y, x)[1];
			greenHistogram.at<Vec3b>(y, x)[2] = greenHistogramAux.at<Vec3b>(y, x)[2];

		}




}
>>>>>>> 81c64c02de36817bc90249567f3d46ac953430d5
