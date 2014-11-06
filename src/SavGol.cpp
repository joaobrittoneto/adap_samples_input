//General Least-Squares smoothing and differentiation by the Convoluation (Savitzky-Golay) method.
//By Peter A. Gory (1989)


#include "SavGol.hpp"

namespace adap_samples_input
{

	SavGol::SavGol()
	{
	}



	SavGol::~SavGol()
	{
	}




	// Calculates the Gram Polynomial (s=0), or its s'th derivative evaluated at i, order k, over 2m+1 points
	double SavGol::GramPoly(double i, double m, double k, double s)
	{	double gramPoly = 0 ;

		if (k > 0)
		{	double gp1 = i*GramPoly(i,m,k-1,s) + s*GramPoly(i,m,k-1,s-1);
			double gp2 = GramPoly(i,m,(k-2),s);
			double k1 = (4*k-2) / (k*(2*m-k+1));
			double k2 = ((k-1)*(2*m+k)) / (k*(2*m-k+1));

			gramPoly = k1*gp1 - k2*gp2;
		}
		else if (k==0 && s==0)
		{
			gramPoly = 1;
		}
		else
		{
			gramPoly = 0;
		}

		return gramPoly;
	}

	// Calculates the generalised factorial (a)(a-1)...(a-b+1)
	double SavGol::GenFact(int a, int b)
	{	double gf = 1;

		if (b > 0 && a > b)
		{
			for( int j= (a-b+1) ; j<=a; j++)
					{gf *= j;}
		}

		return gf;
	}

	// Calculates the weight of the i'th data point for the t'th least-Square point of the s'th derivative, over 2m+1 points, order n
	double SavGol::Weight(double i, double t, double m, double n, double s)
	{	double sum = 0;

		for(int k=0; k<=n; k++)
			{
				sum = sum + ( (2*k+1)*GenFact(2*m,k) / GenFact((2*m+k+1),(k+1)) * GramPoly(i,m,k,0)*GramPoly(t,m,k,s) );

			}


		return sum;
	}




}
