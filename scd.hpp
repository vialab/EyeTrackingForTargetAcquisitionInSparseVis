#ifndef PUPIL_PROGS_SCD_HPP
#define PUPIL_PROGS_SCD_HPP

#include <opencv2/features2d.hpp>

/**
 * Shape Context Descriptor
 * A feature descriptor based on Siddharth Agrawal's article https://algorithmicthoughts.wordpress.com/2013/06/22/computer-vision-shape-context-descriptor/
 * The feature descriptor creates a description for each feature based on the distribution
 * of other features near them. The descriptor is a 5x12 histogram of this distribution in
 * log-polar coordinates
 */
 void computeSCD(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);

#endif
