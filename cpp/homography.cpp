#include "homography.hpp"
#include <limits>
#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>
#include <sys/time.h>
#include "scd.hpp"

bool dmatchCompare(cv::DMatch first, cv::DMatch second){
	return first.distance < second.distance;
}

/* the method here is borrowed from http://docs.opencv.org/2.4/doc/tutorials/features2d/feature_homography/feature_homography.html
 * that tutorial is in 2.4 but this requires opencv 3.x
 */
homography_state retrieveHomography(cv::Mat &frame, cv::Mat &screen, cv::Mat &homography, std::vector<std::vector<cv::Point>> &edgepoints, std::vector<cv::Point2f> &reference_contour, int &camera2distance, cv::Mat *debug) {
	
	//two flag for testing, first one is for the times of this algorithm have been used, second one is the time accumulation.
	static int Homo_orb_counter = 0, Homo_referenceComparison_counter = 0,Homo_orb_time_counter = 0, Homo_referenceComparison_time_counter = 0;
	struct timeval tv1,tv2;
	gettimeofday(&tv1, NULL);

	

	// get keypoints and descriptors with ORB
	std::vector<cv::KeyPoint> keypoints_frame, keypoints_screen;
	cv::Mat descriptors_frame, descriptors_screen;

	//area of reference contour
	int area_reference_contour;
	//reference contour failure flag
	static int reference_fail_counternumber = 0, reference_success_counternumber = 0;

	//homography to filter good matches from matches 
	cv::Mat homo;

	//good matches
	std::vector<cv::DMatch> inliers;

	//use good points to select the good contour
	std::vector<cv::KeyPoint> frame_good_points;
	//goodcontour's index
	int pointNumberArray[10] = {0}, max_match_index = -1, max_number = 0, good_compare_index = -1, good_compare_area_number = 0;

	//color flag, red is good contour from orb, blue is reference contour, 1 for red, 2 for blue
	int color_flag = 0;

	if(reference_contour.size() == 0 || reference_success_counternumber >= 10){
		color_flag = 1;
		reference_success_counternumber = 0;
		//std::cout << "orb" << std::endl;
		/* create an ORB detector and keypoint vectors */	
		cv::Ptr<cv::ORB> orb = cv::ORB::create(1000, 1.2f, 8, 31, 0, 4);
		//draw mask from contours
		cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
		for (int index = 0; index < edgepoints.size(); index++) {

			cv::drawContours(mask, edgepoints, index, cv::Scalar(255, 255, 255), cv::FILLED);
		}
		//cv::imshow("mask", mask);
		//cv::waitKey(1);
		orb->detect(frame, keypoints_frame, mask);
		orb->detect(screen, keypoints_screen);

		/* extract descriptor vectors from keypoints */
		orb->compute(frame, keypoints_frame, descriptors_frame);
		orb->compute(screen, keypoints_screen, descriptors_screen);

		/* FLANN requires descriptors to be in CV_32F */
		/*
		if(descriptors_frame.type() != CV_32F)
		descriptors_frame.convertTo(descriptors_frame,CV_32F);

		if(descriptors_screen.type() != CV_32F)
		descriptors_screen.convertTo(descriptors_screen,CV_32F);
		*/
		/* match vectors with BF matcher */
		cv::BFMatcher matcher(cv::NORM_HAMMING2, true);
		std::vector<cv::DMatch> matches;
		matcher.match(descriptors_frame, descriptors_screen, matches);
		std::sort(matches.begin(), matches.end(), dmatchCompare);
		
		if (matches.size() > 10) {
			std::vector<cv::Point2f> frame_points;
			std::vector<cv::Point2f> screen_points;
			for (size_t i = 0; i < matches.size(); ++i) {
				frame_points.emplace_back(keypoints_frame[matches[i].queryIdx].pt);
				screen_points.emplace_back(keypoints_screen[matches[i].trainIdx].pt);
			}
			std::vector<unsigned char> inliersMask(screen_points.size());
			float reprojectionThreshold = 1;
			homo = cv::findHomography(screen_points, frame_points, CV_RANSAC, reprojectionThreshold,
				inliersMask);
			if (homo.empty()) {
				std::cout << "no homography" << std::endl;
				return HOMOG_FAIL;
			}
			else {
				for (size_t i = 0; i < inliersMask.size(); i++)
				{
					if (inliersMask[i]) {
						inliers.emplace_back(matches[i]);
						frame_good_points.emplace_back(keypoints_frame[matches[i].queryIdx]);
						//screen_good_points.emplace_back(keypoints_screen[matches[i].trainIdx]);
					}
				}
				//cv::putText(frame, std::to_string(matches.size())+"+"+std::to_string(inliers.size()), edgepoints[index][0], CV_FONT_HERSHEY_COMPLEX, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
				for (size_t j = 0; j < frame_good_points.size(); j++) {
					for (size_t q = 0; q < edgepoints.size(); q++) {
						//std::cout << "pointIndex" << frame_good_points[j].pt << std::endl;
						//std::cout << "contourIndex" << edgepoints[q] << std::endl;
						
						if (cv::pointPolygonTest(edgepoints[q], frame_good_points[j].pt, false) >= 0) {
							pointNumberArray[q]++;
							//std::cout << "point in the contour" << pointNumberArray[q]++ << std::endl;
						}
						if (max_number < pointNumberArray[q]) {
							max_number = pointNumberArray[q];
							max_match_index = q;
						}
						//std::cout << "max_number" << max_number << std::endl;
						//std::cout << "max_match_index" << max_match_index << std::endl;
					}
				}
				
			}
		}
		else {
			std::cout << "fail1" << std::endl;
			return HOMOG_FAIL;
		}	
	}
	else {
		int x_compare_avg, y_compare_avg, x_diff, y_diff, distance_compare, x_reference_contour, y_reference_contour;
		x_reference_contour = (reference_contour[0].x + reference_contour[1].x + reference_contour[2].x + reference_contour[3].x) / 4;
		y_reference_contour = (reference_contour[0].y + reference_contour[1].y + reference_contour[2].y + reference_contour[3].y) / 4;
		area_reference_contour = cv::contourArea(reference_contour);
		for (size_t p = 0; p < edgepoints.size(); p++) {
			x_compare_avg = (edgepoints[p][0].x + edgepoints[p][1].x + edgepoints[p][2].x + edgepoints[p][3].x) / 4;
			y_compare_avg = (edgepoints[p][0].y + edgepoints[p][1].y + edgepoints[p][2].y + edgepoints[p][3].y) / 4;
			
			//x_diff = x_reference_contour - x_compare_avg;
			//y_diff = y_reference_contour - y_compare_avg;

			//distance_compare = std::sqrt(x_diff * x_diff + y_diff * y_diff);
			//compare_map.insert(std::pair<int, int>(cv::contourArea(edgepoints[p]), distance_compare));
			if (pointPolygonTest(reference_contour, cv::Point(x_compare_avg, y_compare_avg), false) > 0 && cv::contourArea(edgepoints[p]) < area_reference_contour * 1.3 && cv::contourArea(edgepoints[p]) > area_reference_contour * 0.7) {
				max_match_index = p;
				color_flag = 2;
				reference_success_counternumber++;
				break;
			}
		}
		if (max_match_index < 0) {
			max_match_index = 255;
		}
	}
	//std::cout << "max_index"<< max_match_index << std::endl;
	//std::cout << "max_size" << max_match << std::endl;
	//std::cout << frame.size() << max_match << std::endl;
	//std::cout << "indexa" << frame_good_points.size() << std::endl;
	//std::cout << "indexb" << max_match_index << std::endl;	
	/* compute the homography using matching points */
	//std::cout <<"contourIndex"<< max_match_index << std::endl;
	if (max_match_index >= 0 && max_match_index < 255) {
		int x_avg, y_avg;
		cv::Point2f a, b, c, d;
		x_avg = (edgepoints[max_match_index][0].x + edgepoints[max_match_index][1].x + edgepoints[max_match_index][2].x + edgepoints[max_match_index][3].x) / 4;
		y_avg = (edgepoints[max_match_index][0].y + edgepoints[max_match_index][1].y + edgepoints[max_match_index][2].y + edgepoints[max_match_index][3].y) / 4;

		for (int index1 = 0; index1 < 4; index1++) {

			if (edgepoints[max_match_index][index1].x < x_avg && edgepoints[max_match_index][index1].y < y_avg)
			{
				a = edgepoints[max_match_index][index1];
			}
			else if (edgepoints[max_match_index][index1].x > x_avg && edgepoints[max_match_index][index1].y < y_avg)
			{
				d = edgepoints[max_match_index][index1];
			}
			else if (edgepoints[max_match_index][index1].x > x_avg && edgepoints[max_match_index][index1].y > y_avg)
			{
				c = edgepoints[max_match_index][index1];
			}
			else if (edgepoints[max_match_index][index1].x < x_avg && edgepoints[max_match_index][index1].y > y_avg)
			{
				b = edgepoints[max_match_index][index1];
			}
		}
		reference_contour.clear();
		reference_contour.emplace_back(a);
		reference_contour.emplace_back(b);
		reference_contour.emplace_back(c);
		reference_contour.emplace_back(d);

	}
	else if (max_match_index == 255)
	{
		if (reference_fail_counternumber > 3) {
			reference_contour.clear();
			reference_fail_counternumber = 0;
			std::cout << "fail2" << std::endl;
			return HOMOG_FAIL;
		}
		else {
			reference_fail_counternumber++;
			color_flag = 2;
		}
	}
	else 
	{
		std::cout << "fail3" << std::endl;
		return HOMOG_FAIL;
	}

		/*if (good_contours.size() > 0)
		{
			if (counternumber == 3) {
				good_contours.erase(good_contours.begin());
				good_contours.emplace_back(good_contour);
				counternumber = 0;
			}
			else if (pointPolygonTest(good_contours[0], cv::Point(x_avg, y_avg), false) > 0)
			{
				good_contours.erase(good_contours.begin());
				good_contours.emplace_back(good_contour);
			}
			else
			{
				good_contour.clear();
				good_contour.assign(good_contours[0].begin(), good_contours[0].end());
				good_contours.clear();
				counternumber++;
			}
		}
		*/

	const int W = 34;
	int F = 405;
	int dx = reference_contour[0].x - reference_contour[1].x;
	int dy = reference_contour[0].y - reference_contour[1].y;
	int p = sqrt(dx*dx + dy*dy);
	camera2distance = (F*W) / p;


	if (color_flag == 1) {
		cv::line(frame, reference_contour[0], reference_contour[1], cv::Scalar(0, 0, 255), 8);
		cv::line(frame, reference_contour[1], reference_contour[2], cv::Scalar(0, 0, 255), 8);
		cv::line(frame, reference_contour[2], reference_contour[3], cv::Scalar(0, 0, 255), 8);
		cv::line(frame, reference_contour[3], reference_contour[0], cv::Scalar(0, 0, 255), 8);
		Homo_orb_counter += 1;
		//std::cout << "orb" << std::endl;
	}
	else if(color_flag == 2) {
		cv::line(frame, reference_contour[0], reference_contour[1], cv::Scalar(255, 0, 0), 8);
		cv::line(frame, reference_contour[1], reference_contour[2], cv::Scalar(255, 0, 0), 8);
		cv::line(frame, reference_contour[2], reference_contour[3], cv::Scalar(255, 0, 0), 8);
		cv::line(frame, reference_contour[3], reference_contour[0], cv::Scalar(255, 0, 0), 8);
		//std::cout << "ReferenceComparison" << std::endl;
		Homo_referenceComparison_counter += 1;
	}



	const int height = screen.rows;
	const int width = screen.cols;
	std::vector<cv::Point2f> rect;
	rect.emplace_back(0, 0);
	rect.emplace_back(0, height - 1);
	rect.emplace_back(width - 1, height - 1);
	rect.emplace_back(width - 1, 0);

	homography = cv::getPerspectiveTransform(rect, reference_contour);
	//std::cout << "success" << std::endl;
	//cv::Mat out;
	//cv::drawKeypoints(frame, frame_good_points, out, cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
	//cv::imshow("out", out);
	//if (!homo.empty()) {
		//cv::Mat transformed;
		//cv::perspectiveTransform(rect, transformed, homo);
		//transformed.convertTo(transformed, CV_32S);
		//cv::polylines(frame, transformed, true, 255, 3, cv::LINE_AA);

		//if (debug != NULL) {
			//cv::drawKeypoints(frame, keypoints_frame_max,debug);
			/*cv::drawMatches(frame, keypoints_frame, screen, keypoints_screen, inliers, *debug,
				cv::Scalar::all(-1), cv::Scalar(255, 0, 0, 100), std::vector<char>(),
				cv::DrawMatchesFlags::DEFAULT | cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);*/
			//cv::imshow("debug", debug);
		//}
	//}
	//1000 times frame check
	/*gettimeofday(&tv2, NULL);
	if (color_flag == 1) {
		Homo_orb_time_counter += (tv2.tv_sec * 1000000 + tv2.tv_usec) - (tv1.tv_sec * 1000000 + tv1.tv_usec);
		Homo_orb_counter++;
	}
	else {
		Homo_referenceComparison_time_counter += (tv2.tv_sec * 1000000 + tv2.tv_usec) - (tv1.tv_sec * 1000000 + tv1.tv_usec);
		Homo_referenceComparison_counter++;
	}
	
	if ((Homo_referenceComparison_counter+ Homo_orb_counter) == 1000)
	{
		std::cout << "---------------------------------------------------------------" << std::endl;
		std::cout << "orbtimes" << std::endl;
		std::cout << Homo_orb_counter << std::endl;
		std::cout << "orbAveragetime" << std::endl;
		std::cout << Homo_orb_time_counter/ Homo_orb_counter << std::endl;
		std::cout << "referencecomparisontimes" << std::endl;
		std::cout << Homo_referenceComparison_counter << std::endl;
		std::cout << "referencecomparisonAveragetime" << std::endl;
		std::cout << Homo_referenceComparison_time_counter /Homo_referenceComparison_counter << std::endl;
		std::cout << "---------------------------------------------------------------" << std::endl;
	}*/
	return HOMOG_SUCCESS;

}

/* TODO: remove this when not applicable */
bool first = true;

homography_state retrieveHomographyNeighbourhood(const cv::Mat &frame, const cv::Mat &screen, cv::Mat &homography, cv::Mat *debug){
	/* create an ORB detector and keypoint vectors */
	cv::Ptr<cv::ORB> orb = cv::ORB::create(1000,1.2f,8,31,0,4);
	std::vector<cv::KeyPoint> keypoints_frame, keypoints_screen;

	/* get keypoints and descriptors with ORB */
	orb->detect(frame,keypoints_frame);
	orb->detect(screen,keypoints_screen);

	/* extract descriptor vectors from keypoints */
	cv::Mat descriptors_frame, descriptors_screen;
	computeSCD(frame,keypoints_frame,descriptors_frame);
	computeSCD(screen,keypoints_screen,descriptors_screen);

	/* TODO: remove this when not applicable */
	if(first){
		std::cout << "FRAME" << std::endl;
		std::cout << descriptors_frame << std::endl;
		std::cout << "SCREEN" << std::endl;
		std::cout << descriptors_screen << std::endl;
		first = false;
	}

	/* FLANN requires descriptors to be in CV_32F */
	/*
	if(descriptors_frame.type() != CV_32F)
		descriptors_frame.convertTo(descriptors_frame,CV_32F);

	if(descriptors_screen.type() != CV_32F)
		descriptors_screen.convertTo(descriptors_screen,CV_32F);
		*/
	/* match vectors with BF matcher */
	cv::FlannBasedMatcher matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(descriptors_frame,descriptors_screen,matches);
	std::sort(matches.begin(),matches.end(),dmatchCompare);

	/* get the top matches necessary for a homography computation */
	std::vector<cv::DMatch> good_matches;
	for(size_t i = 0; i < std::min((size_t)60,matches.size()); ++i)
			 good_matches.push_back(matches[i]);

	/* compute the homography using matching points */
	if(good_matches.size() >= 10){
		std::vector<cv::Point2f> frame_points;
		std::vector<cv::Point2f> screen_points;
		for(size_t i = 0; i < good_matches.size(); ++i){
			frame_points.push_back(keypoints_frame[good_matches[i].queryIdx].pt);
			screen_points.push_back(keypoints_screen[good_matches[i].trainIdx].pt);
		}
		homography = cv::findHomography(screen_points,frame_points,CV_RANSAC);
		if(homography.empty()){
			std::cout << "no homography" << std::endl;
			return HOMOG_FAIL;
		}

		/* rectangle on screen in the frame */
		const int height = screen.rows;
		const int width = screen.cols;
		std::vector<cv::Point2f> rect;
		rect.emplace_back(0,0);
		rect.emplace_back(0,height-1);
		rect.emplace_back(width-1,height-1);
		rect.emplace_back(width-1,0);

		cv::Mat transformed;
		cv::perspectiveTransform(rect,transformed,homography);
		transformed.convertTo(transformed,CV_32S);
		cv::polylines(frame,transformed,true,255,3,cv::LINE_AA);

		if(debug != NULL){
			//cv::drawKeypoints(frame,keypoints_frame,debug_frame);
			cv::drawMatches(frame,keypoints_frame,screen,keypoints_screen,good_matches,*debug,
				cv::Scalar::all(-1),cv::Scalar(255,0,0),std::vector<char>(),
			cv::DrawMatchesFlags::DEFAULT | cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		}

		return HOMOG_SUCCESS;
	}else{
		return HOMOG_FAIL;
	}
}
