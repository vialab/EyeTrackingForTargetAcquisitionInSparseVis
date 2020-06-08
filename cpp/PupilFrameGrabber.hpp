/* gets gaze data from the pupil pipeline
 * performs this on seperate thread
 */
#ifndef PUPIL_FRAME_GRABBER_HPP
#define PUPIL_FRAME_GRABBER_HPP

#include <zmq.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>

class PupilFrameGrabber{
public:
	PupilFrameGrabber(zmq::socket_t *subscriber, const int width, const int height);

	/* callback function for a thread to initiate the frame grabbing */
	void run();

	/* function to stop the frame grabbing */
	void stop();

	/* gets last frame streamed from eye tracker */
	cv::Mat getLastFrame();

private:
	void storeFrame(const cv::Mat &new_frame);

	zmq::socket_t *subscriber;
	std::mutex state_mutex;
	bool active;

	/* dimensions of frame */
	const int width;
	const int height;

	/* last frame stored */
	cv::Mat last_frame;

};

#endif
