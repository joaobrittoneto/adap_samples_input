#ifndef _SAMPLESINPUT_HPP_
#define _SAMPLESINPUT_HPP_

#include <iostream>
#include <queue>
#include <math.h>
#include "SavGol.hpp"
#include "base/samples/RigidBodyState.hpp"
#include "base/samples/LaserScan.hpp"
#include "base/samples/Joints.hpp"
#include "base/Eigen.hpp"

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

			void ConvertForce(base::samples::Joints &sample, base::samples::Joints &forcesTorques);
			void PWMtoDC(base::Vector6d &input, base::Vector6d &output);
			void Forces(base::Vector6d &input, base::Vector6d &output);
			void ForcesTorques(base::Vector6d &input, base::Vector6d &output);

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



		private:

	};

} // end namespace adap_samples_input

#endif // _SAMPLESINPUT_HPP_

