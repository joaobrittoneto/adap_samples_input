#ifndef _DATAMODEL_HPP_
#define _DATAMODEL_HPP_

/******************************************************************************/
/*  Preparing data to apply the adaptive parameters identification or least-square method
/*
/*
/* PURPOSE --- Agglomerate forces, pose-estimation data and compute acceleration if required signal.
/*
/*  João Da Costa Britto Neto
/*  joao.neto@dfki.de
/*  DFKI - BREMEN 2015
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
	class DataModel
	{
		public:

			DataModel(double step, double poly,	int halfSizeQueue, double posFilter);
			~DataModel();

			//////////////////////////////////////////////////////////////
			// Velocity-Acceleration Data Processing
			///////////////////////////////////////////////////////////////
			// Fitler_SV: Savitzky-Golay filter. queue: RBS. t_th position in queue to be take in account. Converge for a polynomial n_th order. Calculate the acceleration(&actual_RBA) and the angular_acceleration(&)
			bool Filter_SV(double t, double n, double step, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA);
			// Method used in the Task, orogen component.
			bool calcAcceleration(base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA);
			void EnqueueRBS(base::samples::RigidBodyState RBS);

			/////////////////////////////////////////////////////////////////////////
			// Support functions
			////////////////////////////////////////////////////////////////////////
			void EnqueueForce(base::samples::Joints force);
			// Compensate the position-filter delays, aligning the forces and velocities
			bool Delay_Compensate(base::samples::RigidBodyState &actual_RBS, base::samples::Joints &forces_output);
			// Agglomarate the data of velocity, acceleration and force into one structure. To be used after the Delay_Compensate
			void Agglomerate(base::samples::Joints &force, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA, DynamicAUV &dynamic);

			// Construct a queue
			template<typename Type>
			void enqueueData (int size, Type sample, std::queue<Type> &queue)
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

		    std::queue<base::samples::RigidBodyState>	queueOfRBS;
			std::queue<base::samples::Joints> 			queueOfForces;

		private:

			// step used to establish the derivatives
			double gstep;
			double gpoly;
			int gsizeQueue;
			double gposFilter;



	};

} // end namespace adap_samples_input

#endif // _DATAMODEL_HPP_

