#include "DataModel.hpp"

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

	DataModel::DataModel(double step, double poly,	int halfSizeQueue, double posFilter)
	{
		gstep = step;
		gpoly = poly;
		gsizeQueue = 2*halfSizeQueue+1;
		gposFilter = posFilter;
	}

	DataModel::~DataModel()
	{
		while(!queueOfRBS.empty())
			queueOfRBS.pop();
		while(!queueOfForces.empty())
			queueOfForces.pop();
	}



	// Fitler_SV: Savitzky-Golay filter. queue: RBS. t_th position in queue to be take in account. Converge for a polynomial n_th order. Calculate the acceleration(&actual_RBA) and the angular_acceleration(&)
	bool DataModel::Filter_SV(double t, double n, double step, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA)
	{
		base::samples::RigidBodyState element;
		std::queue<base::samples::RigidBodyState> queueRBS = queueOfRBS;

		element.velocity = Eigen::VectorXd::Zero(3);
		element.angular_velocity = Eigen::VectorXd::Zero(3);

		SavGol savgol;
		int size = queueRBS.size();

		if(queueRBS.empty() || queueRBS.size() <= 4 )
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

		else if (1 == fmod(size,2) && queueRBS.size() > 3 )
			{
			// clean the variables, before calculate the new values
			actual_RBS.angular_velocity	= Eigen::VectorXd::Zero(3);
			actual_RBS.velocity 		= Eigen::VectorXd::Zero(3);
			actual_LinRBA.acceleration	= Eigen::VectorXd::Zero(3);
			actual_AngRBA.acceleration	= Eigen::VectorXd::Zero(3);

			for (int i=-((size-1)/2); i<=((size-1)/2); i++)
				{
					element = queueRBS.front();
					queueRBS.pop ();

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
					queueRBS.push (element);
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
	bool DataModel::calcAcceleration(base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA)
	{
		return Filter_SV(gposFilter, gpoly, gstep, actual_RBS, actual_LinRBA, actual_AngRBA);
	}

	void DataModel::EnqueueRBS(base::samples::RigidBodyState RBS)
	{
		enqueueData(gsizeQueue, RBS, queueOfRBS);
	}


	void DataModel::EnqueueForce(base::samples::Joints force)
	{
		enqueueData(5*gsizeQueue, force, queueOfForces);
	}


	// Compensate the velocity-filter delays, aligning the forces and velocities
	bool DataModel::Delay_Compensate(base::samples::RigidBodyState &actual_RBS, base::samples::Joints &forces_output)
	{
		base::samples::Joints front_force;
		base::samples::Joints behind_force;
		bool stop = false;
		std::queue<base::samples::Joints> queueForces = queueOfForces;
		//interpolate the force
		front_force = queueForces.front();
		forces_output.elements.resize(front_force.elements.size());

		if((actual_RBS.time-front_force.time).toSeconds() < 0)
			return stop;

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
	void DataModel::Agglomerate(base::samples::Joints &force, base::samples::RigidBodyState &actual_RBS, base::samples::RigidBodyAcceleration &actual_LinRBA, base::samples::RigidBodyAcceleration &actual_AngRBA, DynamicAUV &dynamic)
	{
		dynamic.rba		= actual_LinRBA;
		dynamic.ang_rba	= actual_AngRBA;
		dynamic.rbs		= actual_RBS;
		dynamic.joints	= force;
		dynamic.time	= actual_RBS.time;
	}


}
