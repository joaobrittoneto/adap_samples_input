#include "SamplesInput.hpp"

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


namespace adap_samples_input
{

	SamplesInput::SamplesInput()
	{
	}

	SamplesInput::~SamplesInput()
	{
	}



	// Fitler_SV: Savitzky-Golay filter. queue: RBS. t_th position in queue to be take in account. Converge for a polynomial n_th order. Calculate the smooth position, velocity(&actual_RBS) and acceleration(&actual_RBA)
	bool SamplesInput::Filter_SV(std::queue<base::samples::RigidBodyState> &queueOfSamples, double t, double n, double step, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_RBA)
	{
		base::samples::RigidBodyState element;

		element.position = Eigen::VectorXd::Zero(3);

		SavGol savgol;
		int size = queueOfSamples.size();

		if(queueOfSamples.empty() || queueOfSamples.size() <= 4 )
			{
				actual_RBS.position 	= Eigen::VectorXd::Zero(3);
				actual_RBS.velocity 	= Eigen::VectorXd::Zero(3);
				actual_RBA.acceleration = Eigen::VectorXd::Zero(3);
				return false;
			}

		else if (1 != fmod(size,2))
			{
			//std::cout << std::endl << "Queue should have a odd size "<< std::endl << std::endl;
			return false;
			}

		else if (1 == fmod(size,2) && queueOfSamples.size() > 3 )
			{
			// clean the variables, before calculate the new values
			actual_RBS.position[0] 		= 0;
			actual_RBS.velocity[0] 		= 0;
			actual_RBA.acceleration[0] 	= 0;

			for (int i=-((size-1)/2); i<=((size-1)/2); i++)
				{
					element = queueOfSamples.front();
					queueOfSamples.pop ();

					actual_RBS.position[0]  	+= (element.angular_velocity[0] * savgol.Weight(i,t,((size-1)/2),n,0));
					actual_RBS.velocity[0]  	+= (element.angular_velocity[0] * savgol.Weight(i,t,((size-1)/2),n,1));
					actual_RBA.acceleration[0]  += (element.angular_velocity[0] * savgol.Weight(i,t,((size-1)/2),n,2));

					if(i == t)
					{
						actual_RBS.time = element.time;
						actual_RBA.time = element.time;
					}
					queueOfSamples.push (element);
				}
			actual_RBS.velocity[0] /= step;
			actual_RBA.acceleration[0] /= (step*step);
			//insert the filtered position and velocity in the queue
			for (int i=-((size-1)/2); i<=((size-1)/2); i++)
				{
					element = queueOfSamples.front();
					queueOfSamples.pop ();
					if(i == t)
					{
						element.position[0] = actual_RBS.position[0];
						element.velocity[0] = actual_RBS.velocity[0];
					}
					queueOfSamples.push (element);
				}
			return true;
			}

		else
			{
			std::cout << std::endl << "Filter_SV doesn't work "<< std::endl << std::endl;
			return false;
			}
	}

	//	Convert sample from LaserScan to RBS. Put the raw position in the angular_velocity[0]. Let the position[0] to the filtered value
	void SamplesInput::Convert_Avalon(base::samples::LaserScan &sample, base::samples::RigidBodyState &output)
	{
		double angle = sample.start_angle;
		output.angular_velocity[0] = sample.ranges[0]*cos(angle)*(-1); // The wall is the reference (X0=0) and the front of the robot is the positive direction. That means the robot is in negative half-plane
		//from [mm] to [m]
		output.angular_velocity[0] /= 1000;
		output.time = sample.time;
	}

	//	Remove a outlier from sample before pushing it into the queue
	void SamplesInput::Remove_Outlier_Avalon(std::queue<base::samples::RigidBodyState> &queue, base::samples::RigidBodyState &sample)
	{
		//TODO Be careful with the outlier (initial case) and the amplitude.  Verify with sample 12645 (18:03:40.383727)
		if ( (!queue.empty()) && (queue.back().angular_velocity[0] != 0) &&
				((sample.angular_velocity[0]/queue.back().angular_velocity[0]) < 0.05 ||
		    	(sample.angular_velocity[0]/queue.back().angular_velocity[0] > 20)) )
		    	{
		    	//std::cout << std::endl << "actual_RBS.position[0]_before: "<< actual_RBS.position[0] << std::endl;
				sample.angular_velocity[0] = queue.back().angular_velocity[0];
		    	//std::cout << "actual_RBS.posititon[0]_after: "<< actual_RBS.position[0] << std::endl<< std::endl;
		    	}
		//clean the values before insert in the queue
		sample.position[0] 		= 0;
		sample.velocity[0] 		= 0;
	}

	// Compute the velocity and the acceleration (based on ()Filter_SV and (X)Euler's method)
/*	bool SamplesInput::Velocity (std::queue<base::samples::RigidBodyState> &queueOfRBS, int size, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_RBA, double t, double n, double step)
	{
		// Filter works for odd values of queue.size
		if (fmod(queueOfRBS.size(),2) == 1)
		{
			// Savitzky-Golay filter. Filter(queue, t, n, RBS, RBA). queue: RBS. t_th position in queue to be take in account. Converge for a polynomial n_th order. Calculate the smooth position, velocity and acceleration
			return Filter_SV(queueOfRBS, t, n, step, actual_RBS, actual_RBA);
		}
		else
			return false;
	}
*/

	// Method used in the Task, orogen component.
	bool SamplesInput::Update_Velocity_Avalon(base::samples::LaserScan &sample_position, std::queue<base::samples::RigidBodyState> &queueOfRBS, int size, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_RBA)
	{
		base::samples::RigidBodyState rbs;
		// Position that will be take in account when applying the filter. May vary from -(queue.size-1)/2 to (queue.size-1)/2 where 0 is at the center of the queue.
		double t = 0;
		// Order of the polynomial
		double n = 3;
		// step used to establish the derivatives
		double step = 0.065; 	// 0.065 from observed values in Avalon

		//Remove the sample at 0.100645 rad once it sample rate(~0.02s) is much less then the average(~0.066s)
		if(sample_position.start_angle <= 0.10)
		{
			Convert_Avalon(sample_position, rbs);
			Remove_Outlier_Avalon(queueOfRBS, rbs);
			Queue(size, rbs, queueOfRBS);
			return Filter_SV(queueOfRBS, t, n, step, actual_RBS, actual_RBA); //Velocity(queueOfRBS, size, actual_RBS, actual_RBA, t, n, step);
		}
		else
			return false;
	}


	void SamplesInput::Convert_Seabotix(base::samples::RigidBodyState &sample, base::samples::RigidBodyState &output)
		{
			output.angular_velocity[0] = sample.position[0];//*(-1); // The wall is the reference (X0=0) and the front of the robot is the positive direction. That means the robot is in negative half-plane
			output.time = sample.time;
		}



	// Method used in the Task, orogen component.
	bool SamplesInput::Update_Velocity_Seabotix(base::samples::RigidBodyState &sample_position, std::queue<base::samples::RigidBodyState> &queueOfRBS, int size, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_RBA)
	{
		base::samples::RigidBodyState rbs;
		// Position that will be take in account when applying the filter. May vary from -(queue.size-1)/2 to (queue.size-1)/2 where 0 is at the center of the queue.
		double t = 0;
		// Order of the polynomial
		double n = 3;
		// step used to establish the derivatives
		double step = 0.05; 	// 0.05 from observed values in Seabotix
		Convert_Seabotix(sample_position, rbs);
		Queue(size, rbs, queueOfRBS);
		return Filter_SV(queueOfRBS, t, n, step, actual_RBS, actual_RBA); //Velocity(queueOfRBS, size, actual_RBS, actual_RBA, t, n, step);

	}



	// Convert from PWM->Newton
	void SamplesInput::ConvertForce_Avalon(base::samples::Joints &sample, base::samples::Joints &forcesTorques)
	{
		forcesTorques.elements.resize(6);
		int n_thruster = sample.elements.size();
		base::VectorXd forces = Eigen::VectorXd::Zero(n_thruster);
		base::VectorXd pwm = Eigen::VectorXd::Zero(n_thruster);
		base::VectorXd DC_volt = Eigen::VectorXd::Zero(n_thruster);
		base::Vector6d forces_torques = Eigen::VectorXd::Zero(6);

		if (sample.elements.size() > 0)
			{for (int i=0; i<sample.elements.size(); i++)
				{
				pwm[i] = sample.elements[i].raw * (-1); // forces according the movement
				}
			}

		// AVALON DATA///////
		//convert the wrong direction
		pwm[3] *=-1;
		double ThrusterVoltage = 33; // Constant to be verified
		double pos_Cv = (0.0471 / 2.0);	// Constants to be verified
		double neg_Cv = (0.0547 / 2.0);
		Eigen::MatrixXd TCM;
		TCM.resize(6,6);
		double DDT0 = 0.10;	double DDT1 = 0.20;	double DDT2 = 0.30;	double DDT3 = 0.30;	double DDT4 = 0.80;	double DDT5 = 0.75;
		//Thruster Control Matrix of Avalon (Based on the work of Sankar-2010)
		// O^2 Ov1 O^3
		//     O->0				0z ->y
		//     COG				|
		//	   Ov4				vx
		//	   O->5
		TCM << 	0, 0, 1, 1, 0, 0,
				1, 0, 0, 0, 0, 1,
				0, 1, 0, 0, 1, 0,
				0, 0, 0, 0, 0, 0,
				0, DDT1, 0, 0, -DDT4, 0,
				DDT0, 0, DDT2, -DDT3, 0, 0;
		////////////////////
		PWMtoDC(pwm, DC_volt, ThrusterVoltage);
		Forces(DC_volt, forces, pos_Cv, neg_Cv);
		ForcesTorques(forces, forces_torques, TCM);

		for (int i=0; i<6; i++)
		{
			forcesTorques.elements[i].effort = forces_torques[i];
		}

		forcesTorques.time = sample.time;
	}

	// From PWM->DC. Verify constant used
	void SamplesInput::PWMtoDC(base::VectorXd &input, base::VectorXd &output, double ThrusterVoltage)
	{
		//output = Eigen::VectorXd::Zero(6);
		for(int i=0; i< input.size(); i++)
				output[i] = ThrusterVoltage * input[i];
	}

	// From DC->N for each thruster. Verify constant used
	void SamplesInput::Forces(base::VectorXd &input, base::VectorXd &output, double pos_Cv, double neg_Cv)
	{
		//output = Eigen::VectorXd::Zero(6);
		for(int i=0; i< input.size(); i++)
			{
			if( input[i] <= 0 )
				output[i] = neg_Cv * input[i] * fabs(double (input[i]));
			else
				output[i] = pos_Cv * input[i] * fabs(double (input[i]));
			}
	}

	// From N->N. From each thruster to net forces and torque in the AUV. Verify TCM matrix
	void SamplesInput::ForcesTorques(base::VectorXd &input, base::Vector6d &output, Eigen::MatrixXd TCM)
	{
		//output = Eigen::VectorXd::Zero(6);
		output = TCM * input;
	}

	// Method used in the Task, orogen component.
	void SamplesInput::Update_Force_Avalon(base::samples::Joints &sample, std::queue<base::samples::Joints> &queueOfForces, int size, base::samples::Joints &forces_output)
	{
		ConvertForce_Avalon(sample, forces_output);
		Queue(size, forces_output, queueOfForces);
	}

	// Seabotix forces
	void SamplesInput::ConvertForce_Seabotix(base::samples::Joints &sample, base::samples::Joints &forcesTorques)
	{
		forcesTorques.elements.resize(6);
		int n_thruster = sample.elements.size();
		base::VectorXd forces = Eigen::VectorXd::Zero(n_thruster);
		base::VectorXd pwm = Eigen::VectorXd::Zero(n_thruster);
		base::VectorXd DC_volt = Eigen::VectorXd::Zero(n_thruster);
		base::Vector6d forces_torques = Eigen::VectorXd::Zero(6);


		if (sample.elements.size() > 0)
			{for (int i=0; i<sample.elements.size(); i++)
				{
					pwm[i] = sample.elements[i].effort ;//* (-1); // forces according the movement
				}
			}

		// SEABOTIX DATA///////
		double ThrusterVoltage = 19; // Constant to be verified
		double pos_Cv = 0.04005;	// Constants to be verified
		double neg_Cv = 0.03457;
		Eigen::MatrixXd TCM;
		TCM.resize(6,4);
		//Thruster Control Matrix of Avalon (Based on the work of Sankar-2010)
		// O^1 Ov1 O^2
		//     O^3			<-Y	xZ
		//     COG				|
		//	   O->4				vX
		TCM << 	1, 1, 0, 0,
				1, 0, 1, 0,
				0, 1, 0, 1,
				0, 0, 0, 0,
				0.04, 0.04, 0.04, 0,
				0.05, -0.05, 0, 0.01;
		////////////////////
		PWMtoDC(pwm, DC_volt, ThrusterVoltage);
		Forces(DC_volt, forces, pos_Cv, neg_Cv);
		ForcesTorques(forces, forces_torques, TCM);

		for (int i=0; i<6; i++)
		{
			forcesTorques.elements[i].effort = forces_torques[i];
		}

		forcesTorques.time = sample.time + base::Time::fromSeconds(1.0155); // Add delay of seabotix
	}


	void SamplesInput::Update_Force_Seabotix(base::samples::Joints &sample, std::queue<base::samples::Joints> &queueOfForces, int size, base::samples::Joints &forces_output)
	{
		ConvertForce_Seabotix(sample, forces_output);
		Queue(size, forces_output, queueOfForces);
	}


	// Compensate the position-filter delays, aligning the forces and velocities
	bool SamplesInput::Delay_Compensate(base::samples::RigidBodyState &actual_RBS, std::queue<base::samples::Joints> &queueOfForces, base::samples::Joints &forces_output)
	{
		base::samples::Joints front_force;
		base::samples::Joints behind_force;
		bool stop = false;
		std::queue<base::samples::Joints> queueForces = queueOfForces;
		//interpolate the force
		while(!queueForces.empty() && !stop)
		{
			front_force = queueForces.front();
			queueForces.pop();
			if((actual_RBS.time-front_force.time).toSeconds() > 0)
			{
				behind_force = front_force;
			}
			else
			{	// Force's interpolation
				float delta = (front_force.elements[0].effort - behind_force.elements[0].effort) * ((actual_RBS.time-behind_force.time).toSeconds())
								/ ((front_force.time - behind_force.time).toSeconds());
				forces_output.elements[0].effort = delta + behind_force.elements[0].effort;
				forces_output.time = actual_RBS.time;
				stop = true;
			}
		}
		return stop;
	}

	// Agglomarate the data of velocity, acceleration and force into one structure. To be used after the Delay_Compensate
	void SamplesInput::Agglomerate(base::samples::Joints &force, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_RBA, DynamicAUV &dynamic)
	{
		dynamic.rba = actual_RBA;
		dynamic.rbs = actual_RBS;
		dynamic.joints = force;
		dynamic.time = actual_RBS.time;
	}


}
