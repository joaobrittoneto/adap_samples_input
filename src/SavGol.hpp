#ifndef _SAVGOL_HPP_
#define _SAVGOL_HPP_

#include <iostream>
#include <queue>
#include <math.h>

namespace adap_samples_input
{
	class SavGol
	{
		public:

			SavGol();
			~SavGol();

			double GramPoly(double i, double m, double k, double s);
			double GenFact(int a, int b);
			double Weight(double i, double t, double m, double n, double s);





		private:

	};

} // end namespace sav_gol

#endif // _SAVGOL_HPP_
