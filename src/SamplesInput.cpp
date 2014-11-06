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

	//template<typename Type>
	void SamplesInput::Queue(int size, double sample, std::queue<double> &queueOfPosition)
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

}
