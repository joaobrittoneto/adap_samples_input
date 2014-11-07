#ifndef _SAMPLESINPUT_HPP_
#define _SAMPLESINPUT_HPP_

#include <iostream>
#include <queue>
#include <math.h>
#include "SavGol.hpp"
#include "base/samples/RigidBodyState.hpp"
#include "base/samples/LaserScan.hpp"

namespace adap_samples_input
{
	class SamplesInput
	{
		public: 

			SamplesInput();
			~SamplesInput();

			double Deriv(double sample, double last_sample, double delta_t);
			// Filter_1: Moving average Filter
			double Filter_1(std::queue<double> &queueOfPosition);
			//Fitler_2: Filter_1 till queue.size()==size, then Savitzky-Golay filter
			double Filter_2(std::queue<double> &queueOfPosition, double size, double n, double s);
			//Fitler_3: Savitzky-Golay filter. For even values of queue.size() count the most recent sample element twice
			double Filter_3(std::queue<double> &queueOfPosition, double t, double n, double s);
			//Fitler_4: Savitzky-Golay filter. For even values of queue.size() count the most recent sample element twice. Queue of RBS (retain the time of the sample).
			base::samples::RigidBodyState Filter_4(std::queue<base::samples::RigidBodyState> &queue, double t, double n, double s);
			// Covert: LaserScan->RBS
			base::samples::RigidBodyState Convert(base::samples::LaserScan sample);
			void Remove_Outlier(std::queue<base::samples::RigidBodyState> &queueOfPosition, base::samples::RigidBodyState &sample);


			// Construct a queue
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

			// Return the n_sample of a queue
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

