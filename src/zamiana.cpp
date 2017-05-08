#include <iostream>
#include <math.h>
#include <cmath>

int main()
{
        int i;
        std::cout << "i=";
        std::cin >> i;
        std::cout << std::endl;
        float tab[i][4];
        for(int k=0;k<i;++k)
        {
                std::cout << "wartosci zmiennych dla i=" << k+1 << std::endl;
                std::cout << "a" << k+1 <<"=";
                std::cin >> tab[k][0];
                std::cout << std::endl;
                std::cout << "d" << k+1 <<"=";
                std::cin >> tab[k][1];
                std::cout << std::endl;
                std::cout << "alfa" << k+1 <<"=";
                std::cin >> tab[k][2];
                std::cout << std::endl;
                std::cout << "teta" << k+1 <<"=";
                std::cin >> tab[k][3];
                std::cout << std::endl;
        }
        for (int o=0; o<i; ++o)
        {   
		 float cost = cos(tab[o][3]);
                float sint = sin(tab[o][3]);
                float cosa = cos(tab[o][2]);
                float sina = sin(tab[o][2]);

                float yaw = atan2(cost*sina, cosa);
                float pitch = atan2(-1*sint*sina, sqrt(1 - cost*sina*cost*sina));
                float roll = atan2(sint*cosa, cost);

                std::cout << "roll: " << psi << " pitch: " << theta << " yaw: " << fi << std::endl;
        }
}

