#include "SamplesInput.hpp"

namespace adap_samples_input
{

	SamplesInput::SamplesInput()
	{
	}

	SamplesInput::~SamplesInput()
	{
	}

	double SamplesInput::Deriv(double sample, double last_sample, double delta_t)
	{
		double velocity;
		velocity = (sample - last_sample) / delta_t;
		return velocity;
	}


	base::samples::RigidBodyState SamplesInput::Filter_4(std::queue<base::samples::RigidBodyState> &queueOfSamples, double t, double n, double s)
		{
			base::samples::RigidBodyState element;
			base::samples::RigidBodyState filtered_value;
			filtered_value.position = Eigen::VectorXd::Zero(3);
			filtered_value.velocity = Eigen::VectorXd::Zero(3);
			element.position = Eigen::VectorXd::Zero(3);

			SavGol savgol;
			int size = queueOfSamples.size();

			if(queueOfSamples.empty() || queueOfSamples.size() <= 4 )
				filtered_value.position = Eigen::VectorXd::Zero(3);

			// While the queue's being builded, the outpupt's time is duplicated (queue.size even and the next interaction have the same time)
			// should not avoided
			else if (1 != fmod(size,2))
				std::cout << std::endl << "Queue should have a odd size "<< std::endl << std::endl;

			else if (1 == fmod(size,2) && queueOfSamples.size() > 3 )
				{
				for (int i=-((size-1)/2); i<=((size-1)/2); i++)
					{
					element = queueOfSamples.front();
					filtered_value.position[0]  += (element.position[0] * savgol.Weight(i,t,((size-1)/2),n,s));
					filtered_value.velocity[0]  += (element.position[0] * savgol.Weight(i,t,((size-1)/2),n,1));
					queueOfSamples.pop ();
					queueOfSamples.push (element);
					if(i == t)
						filtered_value.time = element.time;
					}
				}

			else
				std::cout << std::endl << "Filter_4 doesn't work "<< std::endl << std::endl;

			return filtered_value;
		}


	base::samples::RigidBodyState SamplesInput::Convert(base::samples::LaserScan sample)
	{
		base::samples::RigidBodyState output;
		double angle = sample.start_angle;

		output.position[0] = sample.ranges[0]*cos(angle);
		//from [mm] to [m]
		output.position[0] = output.position[0]/1000;
		output.time = sample.time;

		return output;
	}

	void SamplesInput::Remove_Outlier(std::queue<base::samples::RigidBodyState> &queue, base::samples::RigidBodyState &sample)
	{
		//TODO Be careful with the outlier (initial case) and the amplitude.  Verify with sample 12645 (18:03:40.383727)
		if ( (!queue.empty()) && (queue.back().position[0] != 0) &&
				((sample.position[0]/queue.back().position[0]) < 0.05 ||
		    	(sample.position[0]/queue.back().position[0] > 20)) )
		    	{
		    	//std::cout << std::endl << "actual_sample.position[0]_before: "<< actual_sample.position[0] << std::endl;
				sample.position[0] = queue.back().position[0];
		    	//std::cout << "actual_sample.posititon[0]_after: "<< actual_sample.position[0] << std::endl<< std::endl;
		    	}
	}

	void SamplesInput::ConvertForce(base::samples::Joints &sample, base::samples::Joints &forcesTorques)
	{
		forcesTorques.elements.resize(6);
		base::Vector6d forces = Eigen::VectorXd::Zero(6);
		base::Vector6d pwm = Eigen::VectorXd::Zero(6);
		base::Vector6d DC_volt = Eigen::VectorXd::Zero(6);
		base::Vector6d forces_torques = Eigen::VectorXd::Zero(6);

		if (sample.elements.size() == 6)
			{for (int i=0; i<6; i++)
				{
				pwm[i] = sample.elements[i].raw;
				}
			}
		//convert the wrong direction
		pwm[3] *=-1;
		PWMtoDC(pwm, DC_volt);
		Forces(DC_volt, forces);
		ForcesTorques(forces, forces_torques);

		for (int i=0; i<6; i++)
		{
			forcesTorques.elements[i].effort = forces_torques[i];
		}

		forcesTorques.time = sample.time;
	}

	void SamplesInput::PWMtoDC(base::Vector6d &input, base::Vector6d &output)
	{
		output = Eigen::VectorXd::Zero(6);
		for(int i=0; i< 6; i++)
				output[i] = 33 * input[i];
	}

	void SamplesInput::Forces(base::Vector6d &input, base::Vector6d &output)
	{
		double pos_Cv = (0.0471 / 2.0);
		double neg_Cv = (0.0547 / 2.0);

		output = Eigen::VectorXd::Zero(6);
		for(int i=0; i< 6; i++)
			{
			if( input[i] <= 0 )
				output[i] = neg_Cv * input[i];
			else
				output[i] = pos_Cv * input[i];
			}
	}

	void SamplesInput::ForcesTorques(base::Vector6d &input, base::Vector6d &output)
	{
		output = Eigen::VectorXd::Zero(6);
		Eigen::MatrixXd TCM;
		TCM.resize(6,6);
		double DDT0 = 0.10;
		double DDT1 = 0.20;
		double DDT2 = 0.30;
		double DDT3 = 0.30;
		double DDT4 = 0.80;
		double DDT5 = 0.75;
		//Thruster Control Matrix
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

		output = TCM * input;
	}

}
