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
#include "samples_dataType.h"
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
			bool Filter_SV(std::queue<base::samples::RigidBodyState> &queue, double t, double n, double step, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_RBA);
			// Covert: LaserScan->RBS
			void Convert_Avalon(base::samples::LaserScan &sample, base::samples::RigidBodyState &output);
			// Remove a outlier from sample before pushing it into the queue
			void Remove_Outlier_Avalon(std::queue<base::samples::RigidBodyState> &queueOfPosition, base::samples::RigidBodyState &sample);
			// Compute the velocity and the acceleration (based on ()Filter_SV and (X)Euler's method)
			//bool Velocity (std::queue<base::samples::RigidBodyState> &queueOfRBS, int size, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_RBA, double t, double n, double step);
			// Method used in the Task, orogen component.
			bool Update_Velocity_Avalon(base::samples::LaserScan &sample_position, std::queue<base::samples::RigidBodyState> &queueOfRBS, int size, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_RBA);

			void Convert_Seabotix(base::samples::RigidBodyState &sample, base::samples::RigidBodyState &output);
			bool Update_Velocity_Seabotix(base::samples::RigidBodyState &sample_position, std::queue<base::samples::RigidBodyState> &queueOfRBS, int size, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_RBA);

			//////////////////////////////////////////////////////////////
			// Force Data Processing
			///////////////////////////////////////////////////////////////
			// Convert from PWM->Newton
			void ConvertForce_Avalon(base::samples::Joints &sample, base::samples::Joints &forcesTorques);
			// From PWM->DC. Verify constant used
			void PWMtoDC(base::VectorXd &input, base::VectorXd &output, double ThrusterVoltage);
			// From DC->N for each thruster. Verify constant used
			void Forces(base::VectorXd &input, base::VectorXd &output, double pos_Cv, double neg_Cv);
			// From N->N. From each thruster to net forces and torque in the AUV. Verify TCM matrix
			void ForcesTorques(base::VectorXd &input, base::Vector6d &output, Eigen::MatrixXd TCM);
			// Method used in the Task, orogen component.
			void Update_Force_Avalon(base::samples::Joints &sample, std::queue<base::samples::Joints> &queueOfForces, int size, base::samples::Joints &forces_output);

			void ConvertForce_Seabotix(base::samples::Joints &sample, base::samples::Joints &forcesTorques);
			void Update_Force_Seabotix(base::samples::Joints &sample, std::queue<base::samples::Joints> &queueOfForces, int size, base::samples::Joints &forces_output);

			/////////////////////////////////////////////////////////////////////////
			// Support functions
			////////////////////////////////////////////////////////////////////////
			// Compensate the position-filter delays, aligning the forces and velocities
			bool Delay_Compensate(base::samples::RigidBodyState &actual_RBS, std::queue<base::samples::Joints> &queueOfForces, base::samples::Joints &forces_output);
			// Agglomarate the data of velocity, acceleration and force into one structure. To be used after the Delay_Compensate
			void Agglomerate(base::samples::Joints &force, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_RBA, DynamicAUV &dynamic);

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

	};

} // end namespace adap_samples_input

#endif // _SAMPLESINPUT_HPP_

