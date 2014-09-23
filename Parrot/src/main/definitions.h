#include <iostream>
#include <list>
#include <map>
#include <time.h>
#include <cstdlib>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
//#include <Windows.h>		//Uncomment in case you are using Windows
#include "SDL/SDL.h"
#include "CHeli.h"
//This must be configured according to your computer
#include <python2.7/Python.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PI 3.14159265359
#define m(a) (floor((a) * 110))	 //Defines the scale on the image of the map. In this case 110 pixels are equivalent to one meter
//Dimensions of the image returned by the front camera of the AR Drone 1.0.
#define height 240
#define width 320
//Common condition that must be checked on Segmentation Algorithm
#define traslape ((*itSegmentos)->getLeft() <= (*nextitSegmentos)->getRight() && (*itSegmentos)->getRight() >= (*nextitSegmentos)-> getLeft())

using std::ofstream;
using std::ios;
using std::cin;
using std::list;
using std::map;
using std::cout;
using std::endl;
using std::pair;
using namespace cv;
using std::ifstream;
using std::string;
using std::getline;

//Class that defines a Segment on the processed image
class Segmento
{
private:
	int left;
	int right;
	int object;
	bool touched;

public:
	Segmento()
	{
		left = 0;
		right = 0;
		object = 0;
		touched = false;
	}
	Segmento(int izq, int der, int obj)
	{
		left = izq;
		right = der;
		object = obj;
		touched = false;
	}
	int getLeft()
	{
		return left;
	}
	void setLeft(int izq)
	{
		left = izq;
	}
	int getRight()
	{
		return right;
	}
	void setRight(int der)
	{
		right = der;
	}
	int getObject()
	{
		return object;
	}
	void setObject(int obj)
	{
		object = obj;
	}
	bool getTouched()
	{
		return touched;
	}
	void setTouched(bool t)
	{
		touched = t;
	}
};

//Class that defines a Point in the image, may be changed to default Point class in OpenCV for future versions
class Punto
{
private:
	double x;
	double y;
public:
	Punto()
	{
		x = y = 0;
	}
	Punto(double X, double Y)
	{
		x = X;
		y = Y;
	}
	double getX()
	{
		return x;
	}
	double getY()
	{
		return y;
	}
	void setX(double X)
	{
		x = X;
	}
	void setY(double Y)
	{
		y = Y;
	}
	void addX(double X)
	{
	    if(X>0)
		x += X;
	}
	void addY(double Y)
	{
	    if(Y >0)
		y += Y;
	}
};

//We define an object in terms of different moments, refer to OpenCV (http://docs.opencv.org/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html)
struct estadisticos{ 
	Punto centro;
	double m00;	// Area
	double m10; // Media en x
	double m01;	// Media en y
	double m11;	// Varianza cruzada
	double m20;	// Varianza en x
	double m02;	// Varianza en y
	double mu11;
	double mu20;
	double mu02;
	double nu11;	
	double nu20;
	double nu02;
	double phi1;
	double phi2;
	double theta;
};

//Color Table, contains the characteristics of each identified region
class Tabla
{
private:
	int Kolor;
	//int Area;
	double Perimetro;
	int openingCounter;
	Punto centro;
	double m00;	// Area
	double m01;	// Media en y
	double m10; // Media en x
	double m11;	// Varianza cruzada
	double m20;	// Varianza en x
	double m02;	// Varianza en y
	double mu02;	
	double mu20;	
	double mu11;
	double nu02;
	double nu20;
	double nu11;
	double phi1;
	double phi2;
	double theta;
public:
	Tabla(){
		Kolor = 0;
		m00 = 0;
		m01 = 0;
		m10 = 0;
		m11 = 0;
		m20 = 0;
		m02 = 0;
		mu02 = 0;	
		mu20 = 0;	
		mu11 = 0;
		nu20 = 0;
		nu02 = 0;
		nu11 = 0;
		phi1 = 0;
		phi2 = 0;
		theta = 0;
		Perimetro = 0;
		openingCounter = 0;
	}
	Tabla(int k, int a, int p){
		Kolor = k;
		m00 = a;
		m01 = 0;
		m10 = 0;
		m11 = 0;
		m20 = 0;
		m02 = 0;
		mu02 = 0;	
		mu20 = 0;	
		mu11 = 0;
		nu20 = 0;
		nu02 = 0;
		nu11 = 0;
		phi1 = 0;
		phi2 = 0;
		theta = 0;
		Perimetro = p;
		openingCounter = 0;
		centro.setX(0);
		centro.setY(0);
	}
	Tabla(int k, int a){
		Kolor = k;
		m00 = a;
		m01 = 0;
		m10 = 0;
		m11 = 0;
		m20 = 0;
		m02 = 0;
		mu02 = 0;	
		mu20 = 0;	
		mu11 = 0;
		nu20 = 0;
		nu02 = 0;
		nu11 = 0;
		phi1 = 0;
		phi2 = 0;
		theta = 0;
		Perimetro = 0;
		openingCounter = 0;
		centro.setX(0);
		centro.setY(0);
	}
	Tabla(int k){
		Kolor = k;
		m01 = 0;
		m10 = 0;
		m11 = 0;
		m20 = 0;
		m02 = 0;
		mu02 = 0;	
		mu20 = 0;	
		mu11 = 0;
		nu20 = 0;
		nu02 = 0;
		nu11 = 0;
		phi1 = 0;
		phi2 = 0;
		theta = 0;
		Perimetro = 0;
		openingCounter = 0;
		centro.setX(0);
		centro.setY(0);
	}
	int getKolor()
	{
		return Kolor;
	}
	void setKolor(int k)
	{
		Kolor = k;
	}
	double getArea()
	{
		return m00;
	}
	void setArea(double a)
	{
		m00 = a;
	}
	void addArea(double a)
	{
		m00 += a;
	}
	void addm10(double m)
	{
		m10 += m;
	}
	void addm01(double m)
	{
		m01 += m;
	}
	double getm10()
	{
		return m10;
	}
	double getm01()
	{
		return m01;
	}
	void addm20(double m)
	{
		m20 += m;
	}
	void addm02(double m)
	{
		m02 += m;
	}
	double getm11()
	{
		return m11;
	}
	double getm20()
	{
		return m20;
	}
	double getm02()
	{
		return m02;
	}
	void addm11(double m)
	{
		m11 += m;
	}
	double getmu20()
	{
		return mu20;
	}
	double getmu02()
	{
		return mu02;
	}
	double getmu11()
	{
		return mu11;
	}
	double getnu20()
	{
		return nu20;
	}
	double getnu02()
	{
		return nu02;
	}
	double getnu11()
	{
		return nu11;
	}
	double getphi1()
	{
		return phi1;
	}
	double getphi2()
	{
		return phi2;
	}
	void setmu20(double m)
	{
		mu20 = m;
	}
	void setmu02(double m)
	{
		mu02 = m;
	}
	void setmu11(double m)
	{
		mu11 = m;
	}
	void setnu20(double n)
	{
		nu20 = n;
	}
	void setnu02(double n)
	{
		nu02 = n;
	}
	void setnu11(double n)
	{
		nu11 = n;
	}
	void setphi1(double p)
	{
		 phi1 = p;
	}
	void setphi2(double p)
	{
		phi2 = p;
	}
	double getPerimetro()
	{
		return Perimetro;
	}
	void setPerimetro(int p)
	{
		Perimetro = p;
	}
	void addPerimetro(int p)
	{
		Perimetro += p;
	}
	int getCount()
	{
		return openingCounter;
	}
	void incrementCount()
	{
		++openingCounter;
	}
	void decrementCount()
	{
		--openingCounter;
	}
	Punto getCentro()
	{
		return centro;
	}
	void addCentro(double x, double y)
	{
		centro.addX(x);
		centro.addY(y);
	}
	void setCentro(Punto c)
	{
		centro = c;
	}
	double getTheta()
	{
		return theta;
	}
	void setTheta()
	{
		theta = 0.5 * atan2(2 * mu11, mu20 - mu02);
	}
	estadisticos getEstadisticos(){
		estadisticos tempEstadisticos;
		tempEstadisticos.centro = centro;
		tempEstadisticos.m00 = m00;
		tempEstadisticos.m10 = m10;
		tempEstadisticos.m01 = m01;
		tempEstadisticos.m11 = m11;
		tempEstadisticos.m20 = m20;
		tempEstadisticos.m02 = m02;
		tempEstadisticos.mu11 = mu11;
		tempEstadisticos.mu20 = mu20;
		tempEstadisticos.mu02 = mu02;
		tempEstadisticos.nu11 = nu11;
		tempEstadisticos.nu20 = nu20;
		tempEstadisticos.nu02 = nu02;
		tempEstadisticos.phi1 = phi1;
		tempEstadisticos.phi2 = phi2;
		tempEstadisticos.theta = theta;
		return tempEstadisticos;
	}
	void printTabla()
	{
		double th, temp;
		temp = getTheta() * (360 / (2 * PI));
		th = temp > 0 ? temp:360 + temp;
		cout << getKolor() << ". Kol=" << getKolor() <<  " Area=" << getArea() << " C(" << getCentro().getX() << ", " << getCentro().getY() << ", " << th << ")" << endl;
		cout << "m00=" << getArea() << " m10="  << getm10() << " m01=" << getm01() <<  " m20=" << getm20() << " m02=" << getm02()<< " m11=" << getm11()  << endl;
		cout << "mu20=" << getmu20() << " mu02=" << getmu02() << " mu11=" << getmu11() << endl;
		cout << "nu20=" << getnu20() << " nu02=" << getnu02() << " nu11=" << getnu11() << endl;
		cout << "phi1=" << getphi1() << " phi2=" << getphi2() << endl;
		cout << "e = " << getnu20() / getnu02() << endl;
		//fprintf(stdout, "%d. Kol=%d m00=%d C(%d, %d) m11=%lf m20=%lf m02=%lf\n", getKolor(), getKolor(), getArea(), getCentro().getX(), getCentro().getY(), getm11(), getm20(), getm02());
		//cout << getKolor() << ". " << getKolor() << " " << getArea() << " C(" << getCentro().getX() << ", " << getCentro().getY() << ")" << endl;
	}

};