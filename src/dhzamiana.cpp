#include <iostream>
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <math.h>
#include <stdlib.h>

using namespace std;

int main()
{
	KDL::Chain chain; // Stworzenie łańcucha kinematycznego
	
	// Deklaracje zmiennych i stałych
	double const PI = M_PI;
	double a3 = 1, teta1=PI/2, teta2=PI/2, teta3=PI/2 , d1=1;
	double zlacze[3], koncowka[3];
	double roll, pitch, yaw; // Szukane parametry

	// Dodanie segmentów - węzłów
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, 0, d1, teta1)));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, PI/2, 0, teta2)));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ),KDL::Frame::DH(a3, 0, 0, teta3)));
	
	
	for (int i = 0; i < 3; i++)
	{
			for(int j = 0; j < 3; j++)
			{
				zlacze[j] = chain.getSegment(i).getFrameToTip().p.data[j]; // Zwraca pozycję końcówki segmentu nadrzędnego
			}

		chain.getSegment(i).getFrameToTip().M.GetRPY(roll, pitch, yaw);
	
	cout << "Zlacze " << i << " xyz : " << zlacze[0] << ", " <<zlacze[1] << ", " << zlacze[2] << endl;		
	cout << "Zlacze " << i << " rpy : " << roll << ", " << pitch << ", " << yaw << endl;
	}

	return 0;
}
