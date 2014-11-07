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

}
