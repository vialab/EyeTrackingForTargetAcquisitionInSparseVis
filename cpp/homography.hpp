#ifndef PUPIL_PROGS_HOMOGRAPHY_HPP
#define PUPIL_PROGS_HOMOGRAPHY_HPP

#include <opencv2/opencv.hpp>

enum homography_state{
	HOMOG_SUCCESS,
	HOMOG_FAIL
};

/* finds a homography between two images and creates a perspective matrix
 * so that points from one image can be mapped to points in the other
 */
homography_state retrieveHomography(cv::Mat &frame,cv::Mat &screen, cv::Mat &homography, std::vector<std::vector<cv::Point>> &edgepoints, std::vector<cv::Point2f> &reference_contour, int &camera2distance,cv::Mat *debug = NULL);

/* same as above but uses a local neighbourhood method for feature matching */
homography_state retrieveHomographyNeighbourhood(const cv::Mat &frame,const cv::Mat &screen, cv::Mat &homography, cv::Mat *debug = NULL);

#endif
