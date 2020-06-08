#ifndef PUPIL_PROGS_IMAGE_PROCESS_HPP
#define PUPIL_PROGS_IMAGE_PROCESS_HPP

#include <opencv2/opencv.hpp>

/* performs the gleam method to convert and rgb image to grayscale */
void rgb2gleam(const cv::Mat &src, cv::Mat &dst);


#endif
