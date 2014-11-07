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


/*	void SamplesInput::Queue(int size, double sample, std::queue<double> &queueOfPosition)
		{	// number of parameters use to get the mean value.
			// Fill the queue with n samples
			if (queueOfPosition.size() < size)
			{	//add a new element in the queue
				queueOfPosition.push (sample);
			}

			if (queueOfPosition.size() > size && !queueOfPosition.empty())
			{	//add a new element in the queue while reduce its size by two till the the queue reach a smaller size
				//remove least element
				queueOfPosition.pop ();

				//insert new element
				queueOfPosition.push (sample);

				//remove least element
				queueOfPosition.pop ();
			}

			if (queueOfPosition.size() == size && !queueOfPosition.empty())
			{	//remove least element
				queueOfPosition.pop();
				//insert new element
				queueOfPosition.push (sample);
			}
		}
*/



	double SamplesInput::Filter_1(std::queue<double> &queueOfPosition)
		{	// Moving average Filter
			double element = 0;
			double mean_value = 0;

			if (!queueOfPosition.empty())
			{
				for (int i=0; i<queueOfPosition.size(); i++)
				{
					element = queueOfPosition.front();
					mean_value += element;
					queueOfPosition.pop ();
					queueOfPosition.push (element);
				}
				mean_value /= queueOfPosition.size();
			}
			return mean_value;
		}





	double SamplesInput::Filter_2(std::queue<double> &queueOfPosition, double size, double n, double s)
		{
			double element = 0;
			double filtered_value = 0;
			SavGol savgol;

			if (queueOfPosition.size() < size)
				filtered_value = Filter_1(queueOfPosition);
			else if (1 != fmod(size,2))
			{
				std::cout << std::endl << "The Queue must have a odd size "<< std::endl << std::endl;
			}
			else if (1 == fmod(size,2))
			{
				for (int i=-((size-1)/2); i<=((size-1)/2); i++)
								{
									element = queueOfPosition.front();
									filtered_value  += (element * savgol.Weight(i,0,((size-1)/2),n,s));
									queueOfPosition.pop ();
									queueOfPosition.push (element);

								}
			}
			else
				std::cout << std::endl << "Filter_2 doesn't work "<< std::endl << std::endl;


			return filtered_value;
		}


	double SamplesInput::Filter_3(std::queue<double> &queueOfPosition, double t, double n, double s)
	{
		double element = 0;
		double filtered_value = 0;
		SavGol savgol;
		int size = queueOfPosition.size();

		if(queueOfPosition.empty() || queueOfPosition.size() == 1 )
			{//std::cout << std::endl << "IS NAN "<< std::endl << std::endl;
			return 0;
			}
		else if (queueOfPosition.size() >= 2 && queueOfPosition.size() <= 4 )
			{
			 return queueOfPosition.back();
			}

		else if (1 != fmod(size,2) && queueOfPosition.size() > 4)
			{
			double temp1, temp2;
			temp1 = queueOfPosition.back();

		/*	for (int i=1; i<=size; i++)
			{	element = queueOfPosition.front();
				queueOfPosition.pop ();
				if (i == (size-1))
					{temp2 = queueOfPosition.front();
					 //temp1 = temp1*(1+(temp1-temp2));
					}
				queueOfPosition.push(element);
			} */
			queueOfPosition.push(temp1);

			for (int i=-(((size/2))); i<=((size/2)); i++)
				{
				element = queueOfPosition.front();
				filtered_value  += (element * savgol.Weight(i,t,((size)/2),n,s));
				queueOfPosition.pop ();
				//if (i==(-((size/2)- 1)))
				//	{queueOfPosition.push (removed);
					//std::cout << std::endl << "push removed back: "<< removed << std::endl << std::endl;
				//	}
				if(i!= size/2)
						queueOfPosition.push (element);
				}
			}

		else if (1 == fmod(size,2) && queueOfPosition.size() > 3 )
			{
			for (int i=-((size-1)/2); i<=((size-1)/2); i++)
				{
				element = queueOfPosition.front();
				filtered_value  += (element * savgol.Weight(i,t,((size-1)/2),n,s));
				queueOfPosition.pop ();
				queueOfPosition.push (element);
				}
			}

		else
			std::cout << std::endl << "Filter_3 doesn't work "<< std::endl << std::endl;

		return filtered_value;
	}


	base::samples::RigidBodyState SamplesInput::Filter_4(std::queue<base::samples::RigidBodyState> &queueOfSamples, double t, double n, double s)
		{
			base::samples::RigidBodyState element;
			base::samples::RigidBodyState filtered_value;
			filtered_value.position = Eigen::VectorXd::Zero(3);
			element.position = Eigen::VectorXd::Zero(3);

			SavGol savgol;
			int size = queueOfSamples.size();

			if(queueOfSamples.empty() || queueOfSamples.size() == 1 )
				{//std::cout << std::endl << "IS NAN "<< std::endl << std::endl;
				 filtered_value.position = Eigen::VectorXd::Zero(3);
				}
			else if (queueOfSamples.size() >= 2 && queueOfSamples.size() <= 4 )
				{
				filtered_value.position = Eigen::VectorXd::Zero(3);
				}
			// While the queue's being builded, the outpupt's time is duplicated (queue.size even and the next interaction have the same time)
			// should not avoided
			else if (1 != fmod(size,2) && queueOfSamples.size() > 4)
				{
				base::samples::RigidBodyState temp;
				temp = queueOfSamples.back();

				queueOfSamples.push(temp);

				for (int i=-(((size/2))); i<=((size/2)); i++)
					{
					element = queueOfSamples.front();
					filtered_value.position[0]  += (element.position[0] * savgol.Weight(i,t,((size)/2),n,s));
					queueOfSamples.pop ();
					if(i!= size/2)
						queueOfSamples.push (element);
					if(i == t)
						filtered_value.time = element.time;
					}
				}

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
