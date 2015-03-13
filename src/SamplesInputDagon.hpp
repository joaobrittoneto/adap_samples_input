#ifndef _SAMPLESINPUTDAGON_HPP_
#define _SAMPLESINPUTDAGON_HPP_

/******************************************************************************/
/*  Preparing data from Dagon to apply the adaptive parameters identification
/*	and the pseudo-inverse method
/*
/* PURPOSE --- Filter the velocity signal, derived it to get the acceleration
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
	class SamplesInputDagon
	{
		public:

			SamplesInputDagon(double step, double X_pos_Cv, double X_neg_Cv, double Y_pos_Cv, double Y_neg_Cv, Eigen::MatrixXd TCM);
			~SamplesInputDagon();

			//////////////////////////////////////////////////////////////
			// Velocity-Acceleration Data Processing
			///////////////////////////////////////////////////////////////
			// Fitler_SV: Savitzky-Golay filter. queue: RBS. t_th position in queue to be take in account. Converge for a polynomial n_th order. Calculate the smooth position, velocity(&actual_RBS) and acceleration(&actual_RBA)
			bool Filter_SV(std::queue<base::samples::RigidBodyState> &queue, double t, double n, double step, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA);
			// Method used in the Task, orogen component.
			bool Update_Velocity_Dagon(base::samples::RigidBodyState &sample_RBS, std::queue<base::samples::RigidBodyState> &queueOfRBS, int size, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA);



			//////////////////////////////////////////////////////////////
			// Force Data Processing
			///////////////////////////////////////////////////////////////
			// Convert from PWM->Newton
			void ConvertForce_Dagon(base::samples::Joints &sample, base::samples::Joints &forcesTorques);
			// From DC->N for each thruster. Verify constant used
			void Forces(base::samples::Joints &input, base::VectorXd &output);
			// From N->N. From each thruster to net forces and torque in the AUV. Verify TCM matrix
			void ForcesTorques(base::VectorXd &input, base::Vector6d &output);
			// Method used in the Task, orogen component.
			void Update_Force_Dagon(base::samples::Joints &sample, std::queue<base::samples::Joints> &queueOfForces, int size, base::samples::Joints &forces_output);


			/////////////////////////////////////////////////////////////////////////
			// Support functions
			////////////////////////////////////////////////////////////////////////
			// Compensate the position-filter delays, aligning the forces and velocities
			bool Delay_Compensate(base::samples::RigidBodyState &actual_RBS, std::queue<base::samples::Joints> &queueOfForces, base::samples::Joints &forces_output);
			// Agglomarate the data of velocity, acceleration and force into one structure. To be used after the Delay_Compensate
			void Agglomerate(base::samples::Joints &force, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA, DynamicAUV &dynamic);

			// Construct a queue
			template<typename Type>
			void Queue (int size, Type sample, std::queue<Type> &queueOfPosition)
			{	// number of parameters use to get the mean value.
				// Fill the queue with n samples
				if (queueOfPosition.size() < size)
				{	//add a new element in the queue
					queueOfPosition.push (sample);
				}

				else if (queueOfPosition.size() > size && !queueOfPosition.empty())
				{	//add a new element in the queue while reduce its size by two till the the queue reach a smaller size
					//remove least element
					queueOfPosition.pop ();

					//insert new element
					queueOfPosition.push (sample);

					//remove least element
					queueOfPosition.pop ();
				}

				else if (queueOfPosition.size() == size && !queueOfPosition.empty())
				{	//remove least element
					queueOfPosition.pop();
					//insert new element
					queueOfPosition.push (sample);
				}
			}



		private:

			// Position that will be take in account when applying the filter. May vary from -(queue.size-1)/2 to (queue.size-1)/2 where 0 is at the center of the queue.
			double gt;
			// Order of the polynomial
			double gn;
			// step used to establish the derivatives
			double gstep;

			// Coefficient for convert Voltage in Force in surge direction
			double gX_pos_Cv;
			double gX_neg_Cv;
			// Coefficient for convert Voltage in Force in sway and heave direction
			double gY_pos_Cv;
			double gY_neg_Cv;
			Eigen::MatrixXd gTCM;

	};

} // end namespace adap_samples_input

#endif // _SAMPLESINPUT_HPP_

