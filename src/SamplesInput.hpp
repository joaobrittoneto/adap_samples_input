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
			double Filter_1(std::queue<double> &queueOfPosition);
			double Filter_2(std::queue<double> &queueOfPosition, double m, double n, double s);
			double Filter_3(std::queue<double> &queueOfPosition, double t, double n, double s);


			template<typename Type>
			void Queue (int size, Type sample, std::queue<Type> &queueOfPosition)
			{	// number of parameters use to get the mean value.
						// Fill the queue with n samples
						if (queueOfPosition.size() < size)
						{	//add a new element in the queue
							queueOfPosition.push (sample);
						}

						if (queueOfPosition.size() > size && !queueOfPosition.empty())
						{	//add a new element in the queue while reduce its size by two till the the queue reach a smaller size
							//remove least element
							queueOfPosition.pop ();

							//insert new element
							queueOfPosition.push (sample);

							//remove least element
							queueOfPosition.pop ();
						}

						if (queueOfPosition.size() == size && !queueOfPosition.empty())
						{	//remove least element
							queueOfPosition.pop();
							//insert new element
							queueOfPosition.push (sample);
						}
					}


			template<typename Type>
			Type collect (int n_sample, std::queue<Type> &queueOfSamples)
			{
				Type temp;
				for (int i=1; i<=queueOfSamples.size(); i++)
					{
					Type element = queueOfSamples.front();
					queueOfSamples.pop ();
					if (i == n_sample)
						temp = element;
					queueOfSamples.push (element);
					}
				return temp;
			}



		private:

	};

} // end namespace adap_samples_input

#endif // _SAMPLESINPUT_HPP_

