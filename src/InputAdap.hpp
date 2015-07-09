#ifndef _INPUTADAP_HPP_
#define _INPUTADAP_HPP_

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
#include "samples_dataType.h"
#include "base/samples/RigidBodyState.hpp"
#include "base/samples/RigidBodyAcceleration.hpp"
#include "base/samples/Joints.hpp"
#include "base/Eigen.hpp"


namespace adap_samples_input
{
	class InputAdap
	{
		public: 

			InputAdap(double step);
			~InputAdap();

			//////////////////////////////////////////////////////////////
			// Position-Velocity Data Processing
			///////////////////////////////////////////////////////////////
			// Fitler_SV_Vel: Savitzky-Golay filter. queue: RBS. t_th position in queue to be take in account. Converge for a polynomial n_th order. Calculate the acceleration(&actual_LInRBA) and the angular_acceleration(&actual_AngRBA) from velocities
			bool Filter_SV_Vel(std::queue<base::samples::RigidBodyState> &queue, double t, double n, double step, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA);
			// Method used in the Task, orogen component.
			// Fitler_SV_Vel: Savitzky-Golay filter. queue: RBS. t_th position in queue to be take in account. Converge for a polynomial n_th order. Calculate the acceleration(&actual_LInRBA, angular_acceleration(&actual_AngRBA) and velocities from position
			bool Filter_SV_Pos(std::queue<base::samples::RigidBodyState> &queue, double t, double n, double step, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA);
			// Method used in the Task, orogen component.
			bool calcAcceleration(base::samples::RigidBodyState &sample_RBS, std::queue<base::samples::RigidBodyState> &queueOfRBS, int size, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA);
			bool calcVelAcc(base::samples::RigidBodyState &sample_RBS, std::queue<base::samples::RigidBodyState> &queueOfRBS, int size, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA);

			// Interpolate asynchronous pose_estimation data into synchronous data. To establish the derivatives
			bool periodicSamples(std::queue<base::samples::RigidBodyState> &queue, base::samples::RigidBodyState &newSample);

			//////////////////////////////////////////////////////////////
			// Force Data Processing
			///////////////////////////////////////////////////////////////
						/////////////////////////////////////////////////////////////////////////
			// Support functions
			////////////////////////////////////////////////////////////////////////
			// Compensate the position-filter delays, aligning the forces and velocities
			bool Delay_Compensate(base::samples::RigidBodyState &actual_RBS, std::queue<base::samples::Joints> &queueOfForces, base::samples::Joints &forces_output);
			// Agglomarate the data of velocity, acceleration and force into one structure. To be used after the Delay_Compensate
			void Agglomerate(base::samples::Joints &force, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA, DynamicAUV &dynamic);
			bool Filter_Force(std::queue<base::samples::Joints> &queue, double t, double n, double step, base::samples::Joints &filteredForce);
			void Update_Force(int size, base::samples::Joints &sample, std::queue<base::samples::Joints> &queue, std::queue<base::samples::Joints> &queueFilter);

			// Construct a queue
			template<typename Type>
			void Queue (int size, Type sample, std::queue<Type> &queue)
			{	// number of parameters use to get the mean value.
				// Fill the queue with n samples
				if (queue.size() < size)
				{	//add a new element in the queue
					queue.push (sample);
				}

				else if (queue.size() > size && !queue.empty())
				{	//add a new element in the queue while reduce its size by two till the the queue reach a smaller size
					//remove least element
					queue.pop ();

					//insert new element
					queue.push (sample);

					//remove least element
					queue.pop ();
				}

				else if (queue.size() == size && !queue.empty())
				{	//remove least element
					queue.pop();
					//insert new element
					queue.push (sample);
				}
			}



		private:

			// step used to establish the derivatives
			double gstep;

	};

} // end namespace adap_samples_input

#endif // _INPUTADAP_HPP_

