#include "InputAdap.hpp"

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

	InputAdap::InputAdap(double step)
	{
		gstep = step;
	}

	InputAdap::~InputAdap()
	{
	}



	// Fitler_SV: Savitzky-Golay filter. queue: RBS. t_th position in queue to be take in account. Converge for a polynomial n_th order. Calculate the acceleration(&actual_RBA) and the angular_acceleration(&)
	bool InputAdap::Filter_SV(std::queue<base::samples::RigidBodyState> &queueOfSamples, double t, double n, double step, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA)
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
						actual_RBS			=	element;
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
	bool InputAdap::calcAcceleration(base::samples::RigidBodyState &sample_RBS, std::queue<base::samples::RigidBodyState> &queueOfRBS, int size, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA)
	{
		base::samples::RigidBodyState rbs = sample_RBS;

		Queue(size, rbs, queueOfRBS);
		return Filter_SV(queueOfRBS, 0, 3, gstep, actual_RBS, actual_LinRBA, actual_AngRBA);
	}



	// Interpolate asynchronous pose_estimation data into synchronous data.
	bool InputAdap::periodicSamples(std::queue<base::samples::RigidBodyState> &queue, base::samples::RigidBodyState &newSample)
	{

		base::samples::RigidBodyState front_rbs;
		base::samples::RigidBodyState behind_rbs;
		bool stop = false;
		std::queue<base::samples::RigidBodyState> queueRBS = queue;

		std::vector<double> diff_lin_vel;
		std::vector<double> diff_ang_vel;
		diff_lin_vel.resize(3);
		diff_ang_vel.resize(3);

		//interpolate the force
		while(!queueRBS.empty() && !stop)
		{
			front_rbs = queueRBS.front();
			queueRBS.pop();
			if((newSample.time -front_rbs.time).toSeconds() > -gstep)
			{
				behind_rbs = front_rbs;
			}
			else
			{	// Force's interpolation
				for(int j=0; j<3; j++)
				{
					diff_lin_vel[j] = (front_rbs.velocity[j] - behind_rbs.velocity[j]) * ((newSample.time-behind_rbs.time).toSeconds() + gstep)
									/ ((front_rbs.time - behind_rbs.time).toSeconds());
					diff_ang_vel[j] = (front_rbs.angular_velocity[j] - behind_rbs.angular_velocity[j]) * ((newSample.time-behind_rbs.time).toSeconds() + gstep)
									/ ((front_rbs.time - behind_rbs.time).toSeconds());
					newSample.velocity[j] = diff_lin_vel[j] + behind_rbs.velocity[j];
					newSample.angular_velocity[j] = diff_ang_vel[j] + behind_rbs.angular_velocity[j];
				}
				newSample.time = newSample.time + base::Time::fromSeconds(gstep);
				stop = true;
			}
		}
		return stop;
	}







	// Compensate the velocity-filter delays, aligning the forces and velocities
	bool InputAdap::Delay_Compensate(base::samples::RigidBodyState &actual_RBS, std::queue<base::samples::Joints> &queueOfForces, base::samples::Joints &forces_output)
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
				for(int j=0; j<front_force.elements.size(); j++)
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
	void InputAdap::Agglomerate(base::samples::Joints &force, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA, DynamicAUV &dynamic)
	{
		dynamic.rba		= actual_LinRBA;
		dynamic.ang_rba	= actual_AngRBA;
		dynamic.rbs		= actual_RBS;
		dynamic.joints	= force;
		dynamic.time	= actual_RBS.time;
	}


	bool InputAdap::Filter_Force(std::queue<base::samples::Joints> &queue, double t, double n, double step, base::samples::Joints &filteredForce)
	{
		base::samples::Joints element;


		SavGol savgol;
		int size = queue.size();
		int numberElements	= queue.front().elements.size();
		int numberNames		= queue.front().names.size();

		element.elements.resize(numberElements);
		element.names.resize(numberNames);

		filteredForce.elements.resize(numberElements);
		filteredForce.names.resize(numberNames);

		if(queue.empty() || queue.size() <= 4 )
			{
				return false;
			}

		else if (1 != fmod(size,2))
			{
			//std::cout << std::endl << "Queue should have a odd size "<< std::endl << std::endl;
			return false;
			}

		else if (1 == fmod(size,2) && queue.size() > 3 )
			{
			// clean the variables, before calculate the new values
			for (int i=0; i<numberElements; i++)
			{
				filteredForce.elements[i].effort = 0;
			}

			for (int i=-((size-1)/2); i<=((size-1)/2); i++)
				{
					element = queue.front();
					queue.pop ();

					for(int j=0; j<numberElements; j++)
					{
						filteredForce.elements[j].effort	+= (element.elements[j].effort	* savgol.Weight(i,t,((size-1)/2),n,0));
						filteredForce.elements[j].speed		+= (element.elements[j].speed	* savgol.Weight(i,t,((size-1)/2),n,0));
					}

					if(i == t)
					{
						filteredForce.time			=	element.time;
					}
					queue.push (element);
				}
			for(int j=0; j<numberNames; j++)
			{
				filteredForce.names[j] = element.names[j];
			}

			return true;
			}

		else
			{
			std::cout << std::endl << "Filter_Force doesn't work "<< std::endl << std::endl;
			return false;
			}
	}


	void InputAdap::Update_Force(int size, base::samples::Joints &sample, std::queue<base::samples::Joints> &queue, std::queue<base::samples::Joints> &queueFilter)
	{
		base::samples::Joints filteredForce;
		Queue(size, sample, queueFilter);
		int t = (queueFilter.size()-1)/2;
		if (Filter_Force(queueFilter, t, 3, gstep,filteredForce) && queueFilter.size()==size)
		{
			Queue(size, filteredForce, queue);
		}
	}

}
