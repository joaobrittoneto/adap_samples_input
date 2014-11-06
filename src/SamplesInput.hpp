#ifndef _SAMPLESINPUT_HPP_
#define _SAMPLESINPUT_HPP_

#include <iostream>
#include <queue>
#include <math.h>
#include "SavGol.hpp"

namespace adap_samples_input
{
	class SamplesInput
	{
		public: 

			SamplesInput();
			~SamplesInput();

			double Deriv(double sample, double last_sample, double delta_t);
			// mean value of i-n to i, with n the number of samples and i the actual sample
			//template<typename Type>
			void Queue (int size, double sample, std::queue<double> &queueOfPosition);
			double Filter_1(std::queue<double> &queueOfPosition);


			double Filter_2(std::queue<double> &queueOfPosition, double m, double n, double s);



		private:

	};

} // end namespace adap_samples_input

#endif // _SAMPLESINPUT_HPP_

