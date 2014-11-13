#include <adap_samples_input/SavGol.hpp>


//using namespace seabotix_lbv150;
using namespace adap_samples_input;

int main(int argc, char** argv)
{




	SavGol savGol ;

	double test = 0;
	double grampoly = 0;
	double i=-1 ;
	double t=-2 ;
	double m=2 ;
	double n=2 ;
	double s=1 ;

	for (int i=-m; i<=m; i++)
	{ //for (int t=-m; t<=m; t++)
		//{
			std::cout << std::endl << "i:"<< i << " t:" << t << " m:" << m << " n:" << n << " s:" << s << std::endl;
			test = savGol.Weight(i,t,m,n,s)*70;
			std::cout << std::endl << "test: "<< test << std::endl << std::endl;

			//grampoly = savGol.GramPoly(-2,2,1,0);
			//std::cout << std::endl << "GramPoly(-2,2,1,0): "<< grampoly << std::endl << std::endl;
			//grampoly = savGol.GramPoly(0,2,1,0);
			//std::cout << std::endl << "GramPoly(0,2,1,0): "<< grampoly << std::endl << std::endl;
			//grampoly = savGol.GramPoly(-2,2,2,0);
			//std::cout << std::endl << "GramPoly(-2,2,2,0): "<< grampoly << std::endl << std::endl;
			//grampoly = savGol.GramPoly(0,2,2,0);
			//std::cout << std::endl << "GramPoly(0,2,2,0): "<< grampoly << std::endl << std::endl;
		//}

	}








	return 0;

}

