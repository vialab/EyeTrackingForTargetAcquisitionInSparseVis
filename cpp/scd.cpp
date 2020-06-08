#include "scd.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>
#if defined(_OPENMP)
#include <omp.h>
#endif

static const unsigned char RADIUS_BINS = 5;
static const unsigned char ANGLE_BINS = 12;
static const unsigned char TOTAL_BINS = RADIUS_BINS*ANGLE_BINS;
static const float RADIUS_MAX = 100;
static const float ANGLE_BIN_RADIANS = 2*M_PI/ANGLE_BINS;
static const float RADIUS_BIN_RATIO = 1.0f/exp(1);

void computeSCD(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors){
	cv::Mat keypoints_mat;
	for(size_t i = 0; i < keypoints.size();++i){
		cv::Mat row(1,2,CV_32F);
		row.at<float>(0) = keypoints[i].pt.x;
		row.at<float>(1) = keypoints[i].pt.y;
		keypoints_mat.push_back(row);
	}

	cv::FlannBasedMatcher keypointsMatcher;
	std::vector<std::vector<cv::DMatch>> matches;
	keypointsMatcher.radiusMatch(keypoints_mat,keypoints_mat,matches,RADIUS_MAX);

	cv::Mat output = cv::Mat::zeros(keypoints.size(),TOTAL_BINS,CV_32F);
	#pragma omp parallel for schedule(dynamic,5)
	for(size_t i = 0; i < matches.size(); ++i){
		const cv::Mat point = keypoints_mat.row(matches[i][0].queryIdx);
		cv::Mat descriptor_row = cv::Mat::zeros(1,TOTAL_BINS,CV_32F);
		for(size_t j = 1; j < matches[i].size(); ++j){
			const cv::Mat vector = keypoints_mat.row(matches[i][j].trainIdx) - point;

			/* compute angular bin point falls in */
			const float angle = atan2(vector.at<float>(1),vector.at<float>(0)) + M_PI;
			float current_angle = 0;
			unsigned char angle_bin = 0;
			for(unsigned char k = 0; k < ANGLE_BINS; ++k){
				if(angle < current_angle){
					angle_bin = k;
					break;
				}
				current_angle += ANGLE_BIN_RADIANS;
			}

			/* compute radius bin point falls in */
			const float radius = RADIUS_MAX - cv::norm(vector);
			float current_radius = 0;
			unsigned char radius_bin = 0;
			for(unsigned char k = 0; k < RADIUS_BINS; ++k){
				if(radius < current_radius){
					radius_bin = k;
					break;
				}
				current_radius = RADIUS_MAX*pow(RADIUS_BIN_RATIO,(RADIUS_BINS-1)-k);
			}

			/* increment the bin where this point lies */
			#pragma omp critical(outputpush)
			{
				output.at<float>(i,radius_bin*ANGLE_BINS + angle_bin) = output.at<float>(i,radius_bin*ANGLE_BINS + angle_bin) + 1;
			}
		}
	}

	//std::cout << output << std::endl;
	descriptors = output.clone();
}
