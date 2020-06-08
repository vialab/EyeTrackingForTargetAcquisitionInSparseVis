#include "averagewindow.hpp"

AverageWindow::AverageWindow(const size_t WINDOW_SIZE):
	sum(0),
	WINDOW_SIZE(WINDOW_SIZE){}

void AverageWindow::push_back(const float value){
	if(values.size() >= WINDOW_SIZE){
		removeOldest();
	}
	values.push_back(value);
	sum += value;
}

float AverageWindow::getAverage(){
	return (values.size() > 0) ? sum/values.size() : 0;
}

void AverageWindow::removeOldest(){
	if(values.size() > 0){
		const float oldest = values.front();
		values.pop_front();
		sum -= oldest;
	}
}
