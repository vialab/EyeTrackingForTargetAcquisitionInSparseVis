#ifndef PUPIL_PROGS_AVERAGE_WINDOW_HPP
#define PUPIL_PROGS_AVERAGE_WINDOW_HPP

#include <deque>
#include <cstddef>

/* average of a fixed window of values */
class AverageWindow{
public:
	AverageWindow(const size_t WINDOW_SIZE = 30);

	/**
	 * Push value into the window, removing the oldest value from the window
	 * and sum
	 */
	void push_back(const float value);

	/**
	 * retrieve the average of the window
	 */
	float getAverage();

private:
	std::deque<float> values;
	float sum;
	const size_t WINDOW_SIZE;

	/**
	 * removes the oldest value from the window and sum
	 */
	void removeOldest();
};

#endif
