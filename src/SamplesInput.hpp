#ifndef _SAMPLESINPUT_HPP_
#define _SAMPLESINPUT_HPP_

/******************************************************************************/
/*  Preparing data from Avalon to apply the adaptive parameters identification
/*
/*
/* PURPOSE --- Filter the position signal, derived it to get the velocity and
/* 				compute the forces applied in the auv
/*
/*  João Da Costa Britto Neto
/*  joao.neto@dfki.de
/*  DFKI - BREMEN 2014
/*****************************************************************************/

#include <iostream>
#include <queue>
#include <math.h>
#include "SavGol.hpp"
#include "base/samples/RigidBodyState.hpp"
#include "base/samples/RigidBodyAcceleration.hpp"
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

			//////////////////////////////////////////////////////////////
			// Position-Velocity Data Processing
			///////////////////////////////////////////////////////////////
			// Fitler_SV: Savitzky-Golay filter. queue: RBS. t_th position in queue to be take in account. Converge for a polynomial n_th order. Calculate the smooth position, velocity(&actual_RBS) and acceleration(&actual_RBA)
			void Filter_SV(std::queue<base::samples::RigidBodyState> &queue, double t, double n, double step, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_RBA);
			// Covert: LaserScan->RBS
			void Convert(base::samples::LaserScan &sample, base::samples::RigidBodyState &output);
			// Remove a outlier from sample before pushing it into the queue
			void Remove_Outlier(std::queue<base::samples::RigidBodyState> &queueOfPosition, base::samples::RigidBodyState &sample);
			// Compute the velocity and the acceleration (based on ()Filter_SV and (X)Euler's method)
			void Velocity (std::queue<base::samples::RigidBodyState> &queueOfRBS, int size, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_RBA);
			// Method used in the Task, orogen component.
			void Update_Velocity(base::samples::LaserScan &sample_position, std::queue<base::samples::RigidBodyState> &queueOfRBS, int size, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_RBA);

			//////////////////////////////////////////////////////////////
			// Force Data Processing
			///////////////////////////////////////////////////////////////
			// Convert from PWM->Newton
			void ConvertForce(base::samples::Joints &sample, base::samples::Joints &forcesTorques);
			// From PWM->DC. Verify constant used
			void PWMtoDC(base::Vector6d &input, base::Vector6d &output);
			// From DC->N for each thruster. Verify constant used
			void Forces(base::Vector6d &input, base::Vector6d &output);
			// From N->N. From each thruster to net forces and torque in the AUV. Verify TCM matrix
			void ForcesTorques(base::Vector6d &input, base::Vector6d &output);
			// Method used in the Task, orogen component.
			void Update_Force(base::samples::Joints &sample, std::queue<base::samples::Joints> &queueOfForces, int size, base::samples::Joints &forces_output);

			// Compensate the position-filter delays, aligning the forces and velocities
			void Delay_Compensate(base::samples::RigidBodyState &actual_RBS, std::queue<base::samples::Joints> &queueOfForces, base::samples::Joints &forces_output);


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

