#include "PupilFrameGrabber.hpp"
#include <iostream>

PupilFrameGrabber::PupilFrameGrabber(zmq::socket_t *subscriber, const int width,
	const int height):
	subscriber(subscriber),
	active(false),
	width(width),
	height(height),
	last_frame(height,width,CV_8U){};

void PupilFrameGrabber::run(){
	state_mutex.lock();
	active = true;
	state_mutex.unlock();

	zmq::message_t frame_msg;
	unsigned char *sub_buffer;
	while(active){
		subscriber->recv(&frame_msg);
		sub_buffer = new unsigned char[frame_msg.size()];
		memcpy(sub_buffer,frame_msg.data(),frame_msg.size());
		delete[] sub_buffer;

		/* see if there's more data to get, if yes then get the data */
		size_t remaining_messages = 0;
		size_t option_length = sizeof(remaining_messages);
		subscriber->getsockopt(ZMQ_RCVMORE,&remaining_messages,&option_length);
		if(remaining_messages){
			/* frame meta data */
			frame_msg.rebuild();
			subscriber->recv(&frame_msg);

			/* the frame itself */
			frame_msg.rebuild();
			subscriber->recv(&frame_msg);

			/* receive again cause frame comes after second message */
			cv::Mat frame(height,width,CV_8U);
			memcpy(frame.data,frame_msg.data(),frame_msg.size());

			/* store the frame into member field */
			storeFrame(frame);

			/* if there's more messages left in multipart message just waste them */
			while(remaining_messages){
				subscriber->getsockopt(ZMQ_RCVMORE,&remaining_messages,&option_length);
			}
		}
	}
}

void PupilFrameGrabber::stop(){
	state_mutex.lock();
	active = false;
	state_mutex.unlock();
}

cv::Mat PupilFrameGrabber::getLastFrame(){
	std::lock_guard<std::mutex> lock(state_mutex);
	return last_frame.clone();
}

void PupilFrameGrabber::storeFrame(const cv::Mat &new_frame){
	state_mutex.lock();
	new_frame.copyTo(last_frame);
	state_mutex.unlock();
}
