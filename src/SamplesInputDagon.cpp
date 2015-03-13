#include "SamplesInputDagon.hpp"

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

	SamplesInputDagon::SamplesInputDagon(double step, double X_pos_Cv, double X_neg_Cv, double Y_pos_Cv, double Y_neg_Cv, Eigen::MatrixXd TCM)
	{
		gt			=	0;
		gn			=	3;
		gstep		=	step;
		gX_pos_Cv	=	X_pos_Cv;
		gX_neg_Cv	=	X_neg_Cv;
		gY_pos_Cv	=	Y_pos_Cv;
		gY_neg_Cv	=	Y_neg_Cv;
		gTCM		=	TCM;
	}

	SamplesInputDagon::~SamplesInputDagon()
	{
	}



	// Fitler_SV: Savitzky-Golay filter. queue: RBS. t_th position in queue to be take in account. Converge for a polynomial n_th order. Calculate the smooth position, velocity(&actual_RBS) and acceleration(&actual_RBA)
	bool SamplesInputDagon::Filter_SV(std::queue<base::samples::RigidBodyState> &queueOfSamples, double t, double n, double step, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA)
	{
		base::samples::RigidBodyState element;

		element.velocity = Eigen::VectorXd::Zero(3);
		element.angular_velocity = Eigen::VectorXd::Zero(3);

		SavGol savgol;
		int size = queueOfSamples.size();

		if(queueOfSamples.empty() || queueOfSamples.size() <= 4 )
			{
				actual_RBS.angular_velocity	= Eigen::VectorXd::Zero(3);
				actual_RBS.velocity			= Eigen::VectorXd::Zero(3);
				actual_LinRBA.acceleration	= Eigen::VectorXd::Zero(3);
				actual_AngRBA.acceleration	= Eigen::VectorXd::Zero(3);
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
			actual_RBS.angular_velocity	= Eigen::VectorXd::Zero(3);
			actual_RBS.velocity 		= Eigen::VectorXd::Zero(3);
			actual_LinRBA.acceleration	= Eigen::VectorXd::Zero(3);
			actual_AngRBA.acceleration	= Eigen::VectorXd::Zero(3);

			for (int i=-((size-1)/2); i<=((size-1)/2); i++)
				{
					element = queueOfSamples.front();
					queueOfSamples.pop ();

					for(int j=0; j<3; j++)
					{
						//actual_RBS.angular_velocity[j]	+= (element.angular_velocity[j] * savgol.Weight(i,t,((size-1)/2),n,0));
						//actual_RBS.velocity[j]			+= (element.velocity[j] * savgol.Weight(i,t,((size-1)/2),n,0));
						actual_LinRBA.acceleration[j]	+= (element.velocity[j] * savgol.Weight(i,t,((size-1)/2),n,1));
						actual_AngRBA.acceleration[j]	+= (element.angular_velocity[j] * savgol.Weight(i,t,((size-1)/2),n,1));
					}

					if(i == t)
					{
						actual_RBS.velocity 		=	element.velocity;
						actual_RBS.angular_velocity	=	element.angular_velocity;

						actual_RBS.time		= element.time;
						actual_LinRBA.time	= element.time;
						actual_AngRBA.time	= element.time;
					}
					queueOfSamples.push (element);
				}
			for(int j=0; j<3; j++)
			{
				actual_LinRBA.acceleration[j] /= step;
				actual_AngRBA.acceleration[j] /= step;
			}

			return true;
			}

		else
			{
			std::cout << std::endl << "Filter_SV doesn't work "<< std::endl << std::endl;
			return false;
			}
	}



	// Method used in the Task, orogen component.
	bool SamplesInputDagon::Update_Velocity_Dagon(base::samples::RigidBodyState &sample_RBS, std::queue<base::samples::RigidBodyState> &queueOfRBS, int size, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA)
	{
		base::samples::RigidBodyState rbs = sample_RBS;

		Queue(size, rbs, queueOfRBS);
		return Filter_SV(queueOfRBS, gt, gn, gstep, actual_RBS, actual_LinRBA, actual_AngRBA);
	}




	// Convert from rpm->Newton
	void SamplesInputDagon::ConvertForce_Dagon(base::samples::Joints &sample, base::samples::Joints &forcesTorques)
	{
		forcesTorques.elements.resize(6);
		forcesTorques.names.resize(6);
		int n_thruster						= sample.elements.size();
		base::VectorXd forces				= Eigen::VectorXd::Zero(n_thruster);
		base::Vector6d forces_torques		= Eigen::VectorXd::Zero(6);
		base::samples::Joints input_sample	= sample;

/*		if (sample.elements.size() > 0)
			{for (int i=0; i<sample.elements.size(); i++)
				{
				DC_volt[i] = sample.elements[i].speed; // * (-1); // forces according the movement
				}
			}
*/
		// apparntely the swail_tail, doesn't
		for (int i=0; i<n_thruster; i++)
			{
				if (input_sample.names[i]=="sway_tail")
				{
					input_sample.elements[i].speed	*= -1;
					input_sample.elements[i].raw	*= -1;
				}
			}

		Forces(input_sample, forces);
		ForcesTorques(forces, forces_torques);

		for (int i=0; i<6; i++)
		{
			forcesTorques.elements[i].effort = forces_torques[i];
		}
		forcesTorques.names[0]	= "surge";
		forcesTorques.names[1]	= "sway";
		forcesTorques.names[2]	= "heave";
		forcesTorques.names[3]	= "roll";
		forcesTorques.names[4]	= "pitch";
		forcesTorques.names[5]	= "yaw";

		forcesTorques.time		= sample.time;
	}

	// From DC->N for each thruster. Verify constant used
	void SamplesInputDagon::Forces(base::samples::Joints &input, base::VectorXd &output)
	{
		int n_thruster = input.elements.size();
		for (int i=0; i < n_thruster; i++)
		{
			if(input.names[i]=="surge_left" || input.names[i]=="surge_right")
			{
				if(input.elements[i].speed > 0)
				{
					input.elements[i].effort = gX_pos_Cv *
							(fabs(input.elements[i].speed) * input.elements[i].speed);
					output[i] = input.elements[i].effort;
				}
				else
				{
					input.elements[i].effort = gX_neg_Cv *
							(fabs(input.elements[i].speed) * input.elements[i].speed);
					output[i] = input.elements[i].effort;
				}
			}
			else
			{
				if(input.elements[i].speed > 0)
				{
					input.elements[i].effort = gY_pos_Cv *
							(fabs(input.elements[i].speed) * input.elements[i].speed);
					output[i] = input.elements[i].effort;
				}
				else
				{
					input.elements[i].effort = gY_neg_Cv *
							(fabs(input.elements[i].speed) * input.elements[i].speed);
					output[i] = input.elements[i].effort;
				}
			}
		}
	}

	// From N->N. From each thruster to net forces and torque in the AUV. Verify TCM matrix
	void SamplesInputDagon::ForcesTorques(base::VectorXd &input, base::Vector6d &output)
	{
		//output = Eigen::VectorXd::Zero(6);
		output = gTCM * input;
	}

	// Method used in the Task, orogen component.
	void SamplesInputDagon::Update_Force_Dagon(base::samples::Joints &sample, std::queue<base::samples::Joints> &queueOfForces, int size, base::samples::Joints &forces_output)
	{
		ConvertForce_Dagon(sample, forces_output);
		Queue(size, forces_output, queueOfForces);
	}



	// Compensate the velocity-filter delays, aligning the forces and velocities
	bool SamplesInputDagon::Delay_Compensate(base::samples::RigidBodyState &actual_RBS, std::queue<base::samples::Joints> &queueOfForces, base::samples::Joints &forces_output)
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
				for (int j=0; j<3; j++)
				{
					float delta = (front_force.elements[j].effort - behind_force.elements[j].effort) * ((actual_RBS.time-behind_force.time).toSeconds())
									/ ((front_force.time - behind_force.time).toSeconds());
					forces_output.elements[j].effort = delta + behind_force.elements[j].effort;
				}

				forces_output.time = actual_RBS.time;
				stop = true;
			}
		}
		return stop;
	}

	// Agglomarate the data of velocity, acceleration and force into one structure. To be used after the Delay_Compensate
	void SamplesInputDagon::Agglomerate(base::samples::Joints &force, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA, DynamicAUV &dynamic)
	{
		dynamic.rba		= actual_LinRBA;
		dynamic.ang_rba	= actual_AngRBA;
		dynamic.rbs		= actual_RBS;
		dynamic.joints	= force;
		dynamic.time	= actual_RBS.time;
	}


}
