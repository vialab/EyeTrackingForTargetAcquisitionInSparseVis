/* performs matching between video stream of the pupil eye tracker world camera
 * and continous screenshots of the target screen.
 * gaze data and world frames are collected on worker threads
 * requires C++11, socketIO-client for cpp, opencv 3.x, Xlib, zmq, msgpack
 * additional dependencies for boost comes from socketIO
 */
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <sio_client.h>

#include "PupilGazeScraper.hpp"
#include "PupilFrameGrabber.hpp"
#include "scrshot.hpp"
#include "homography.hpp"
#include "util.hpp"
#include "averagewindow.hpp"

//frame size
int frame_width;
int frame_height;
//max bar
const int blocksize_max = 20;
const int constValue_max = 20;
const int canny_max = 600;
const int max_elem = 2;
const int max_kernel_size = 21;
const int binary_threshold_max = 255;

//init bar
int dilation_elem = 0;
int dilation_size = 1;
int blocksize = 5;
int constValue = 5;
int canny = 20;
int FACTOR = 3;
int binary_threshold = 0;

cv::Mat out;
cv::Mat Processout;
cv::Mat frame;
std::vector<std::vector<cv::Point>> edgepoints;
std::vector<cv::Point2f> reference_contour;

//point to keep the gaze point while failure to find the screen
cv::Mat reference_point(1, 1, CV_64FC2);

//distance to screen 
int camera2distance = -1;
//gaze log file
//std::ofstream gaze_log_file;


void on_ProcessTrackbar(int, void*) {

	//std::cout << "preprocess" << std::endl;
	out = frame.clone();
	if (!out.empty())
	{
		
		/*int dilation_type;
		if (dilation_elem == 0) { dilation_type = cv::MORPH_RECT; }
		else if (dilation_elem == 1) { dilation_type = cv::MORPH_CROSS; }
		else if (dilation_elem == 2) { dilation_type = cv::MORPH_ELLIPSE; }*/

		//int bsize = blocksize>0? blocksize * 2 + 1:3;
		//max parameters to get best performance	
		GaussianBlur(out, Processout, cv::Size(3, 3),2, 2);
		/*if (bsize != 0 && constValue != 0) {
			cv::adaptiveThreshold(Processout, Processout,
				255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
				cv::THRESH_BINARY, bsize,
				constValue);
		}*/
		/*if (binary_threshold != 0) {
			cv::threshold(Processout, Processout, binary_threshold, 255, cv::THRESH_BINARY);
		}*/
		
		cv::Canny(Processout, Processout, canny, canny*FACTOR, FACTOR,false);
		

		cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
			cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
			cv::Point(dilation_size, dilation_size));
		cv::dilate(Processout, Processout, element);
		
		

		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;

		cv::findContours(Processout, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		std::vector<int> hull;
		std::vector<std::vector<cv::Point>> approx(contours.size());

		std::map<int, int> areamap;
		std::map <int, int>::reverse_iterator  areamap_Iter;
		int framearea = frame_width * frame_height*0.05;
		for (int index = 0; index < contours.size(); index++)
		{
			if (cv::contourArea(contours[index]) > framearea) {
				cv::convexHull(contours[index], hull);
				if (hull.size() > 3)
				{
					std::vector<cv::Point> newPoints;
					for (int index1 = 0; index1 < hull.size(); index1++) {
						newPoints.emplace_back(contours[index][hull[index1]]);
					}
					std::vector<cv::Point2f> contourHull;
					contourHull.assign(newPoints.begin(), newPoints.end());
					if (contourHull.size() == 4) {
						approx[index].assign(contourHull.begin(), contourHull.end());
						areamap.insert(std::pair<int, int>(cv::contourArea(approx[index]), index));
					}
					else {
						cv::approxPolyDP(contourHull, approx[index], cv::arcLength(contourHull, true)*0.02, true);
						if (approx[index].size() ==4) {
							areamap.insert(std::pair<int, int>(cv::contourArea(approx[index]), index));
						}
					}
				}
			}
		}

		cv::Mat linePic = cv::Mat::zeros(out.size(), CV_8UC3);
		if (areamap.size() > 0) {
			areamap_Iter = areamap.rbegin();
			for (int index3 = 0; index3 < 5; index3++) {
				//drawContours(linePic, approx, areamap_Iter->second, cv::Scalar(255, 255, 255), 5, 8);
				drawContours(frame, approx, areamap_Iter->second, cv::Scalar(255, 255, 255), 5, 8);
				edgepoints.emplace_back(approx[areamap_Iter->second]);
				areamap_Iter++;
				if (areamap_Iter == areamap.rend())break;		
			}
		}
		//cv::cvtColor(Processout, Processout, CV_GRAY2RGB);
		/*int lineType = linePic.type();
        int outType = out.type();
        std::cout << "lineType = " << lineType << std::endl;
		std::cout << "ProcessoutType = " << outType << std::endl;*/
		//cv::addWeighted(linePic, 0.5, Processout, 0.5, 1, Processout);
		//cv::imshow("screen", Processout);
		//cv::waitKey(1);
	}
}


enum DescriptorMethod{
	ORB,
	KNN
};




int main(int argc, char **argv){

	/* actual resolution of screen */
	int screen_width = 1920;
	int screen_height = 1080;
	/* resolution of screen in homography retrieval */
	const int screen_sub_width = 1280;
	const int screen_sub_height = 720;
	/* address to connect to pupil remote from */
	std::string address = "127.0.0.1";
	std::string req_port = "50020";

	int signature = 0;
	DescriptorMethod descriptor_method = ORB;

	//initialize the reference point
	reference_point.at<double>(0, 0) = 0;
	reference_point.at<double>(0, 1) = 0;

	//open the log file
	/*gaze_log_file.open("gaze_log.txt", std::ios::app);
	gaze_log_file << "\n\n\n program start\n";*/
	
	if(argc > 5){
		frame_width = atoi(argv[1]);
		frame_height = atoi(argv[2]);
		screen_width = atoi(argv[3]);
		screen_height = atoi(argv[4]);
		signature = atoi(argv[5]);
		if(argc > 6){
			if(strncmp("knn",argv[6],3) == 0){
				descriptor_method = KNN;
			}else if(strncmp("orb",argv[6],3) == 0){
				descriptor_method = ORB;
			}
			if(argc > 7){
				req_port = argv[7];
			}
		}
	}

	const size_t BUFFER_LEN = 100;
	char buffer[BUFFER_LEN] = {0};
	std::string transport("tcp://");
	
	zmq::context_t context(1);
	zmq::socket_t req(context,ZMQ_REQ);
	std::cout<< "frame_width:"<< frame_width << std::endl;
	std::cout << "frame_height:" << frame_height << std::endl;
	std::cout << "screen_width:" << screen_width << std::endl;
	std::cout << "screen_height:" << screen_height << std::endl;
	std::cout << "signature:" << signature << std::endl;
	std::cout << "descriptor_method:" << descriptor_method << std::endl;
	std::cout << "req_port:" << req_port << std::endl;
	/* connect the requester to pupil */
	req.connect((transport+address+":"+req_port).c_str());

	/* send over requester that we want a subscriber port */
	strncpy(buffer,"SUB_PORT",BUFFER_LEN);
	zmq::message_t req_msg(8);
	memcpy(req_msg.data(),buffer,8);
	req.send(req_msg);

	/* clear the message, and retrieve the subscriber port */
	req_msg.rebuild();
	req.recv(&req_msg);
	memcpy(buffer,req_msg.data(),BUFFER_LEN);

	/* extract port from all the other junk */
	int index = find_first_nonnumber(buffer,BUFFER_LEN);
	memset(buffer+index,0,BUFFER_LEN-index);
	std::string sub_port(":");
	sub_port += buffer;

	/* create subscriber for gaze data */
	zmq::socket_t gaze_sub(context,ZMQ_SUB);
	gaze_sub.connect((transport+address+sub_port).c_str());
	gaze_sub.setsockopt(ZMQ_SUBSCRIBE,"gaze",4);

	/* create subcriber for frame data */
	zmq::socket_t frame_sub(context,ZMQ_SUB);
	frame_sub.connect((transport+address+sub_port).c_str());
	frame_sub.setsockopt(ZMQ_SUBSCRIBE,"frame.world",11);

	/* create the worker threads */
	PupilGazeScraper gaze_scraper(&gaze_sub);
	std::thread gaze_thread(&PupilGazeScraper::run,&gaze_scraper);

	PupilFrameGrabber frame_grabber(&frame_sub,frame_width,frame_height);
	std::thread frame_thread(&PupilFrameGrabber::run,&frame_grabber);

	cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0);

	sio::client gaze_emitter;
	gaze_emitter.connect("http://127.0.0.1:3000");

	reference_point.at<double>(0, 0) = -1;
	reference_point.at<double>(0, 1) = -1;

	/* prepare output videos */
	/*cv::VideoWriter outputVideo;
	outputVideo.open("debug.avi", cv::VideoWriter::fourcc('M','J','P','G'),	5,
		cv::Size(frame_width+screen_sub_width,std::max(frame_height,screen_sub_height)),
		true);

	if(!outputVideo.isOpened()){
		std::cerr << "Video did not open" << std::endl;
		exit(-1);
	}*/
	/* prepare output videos */
	cv::VideoWriter outputVideo2;
	outputVideo2.open("debug2.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 5,
		cv::Size(frame_width, frame_height),
		true);

	if (!outputVideo2.isOpened()) {
		std::cerr << "Video did not open" << std::endl;
		exit(-1);
	}
	/* continously get feed from pupil for gaze data and take screenshots
	* get a homography between frame and screenshot and project the gaze coordinates
	* to the screen space
	*/
	
	cv::namedWindow("screen", cv::WINDOW_NORMAL);
	//bar name
	char cannyname[20];
	sprintf(cannyname, "Canny %d", canny_max);

	//create bar
	/*cv::createTrackbar(blocksizename, "screen", &blocksize,
		blocksize_max, on_ProcessTrackbar);

	cv::createTrackbar(constValuename, "screen", &constValue,
		constValue_max, on_ProcessTrackbar);*/

	/*cv::createTrackbar("Binary Threshold", "screen", &binary_threshold,
		binary_threshold_max, on_ProcessTrackbar);*/

	cv::createTrackbar(cannyname, "screen", &canny,
		canny_max, on_ProcessTrackbar);

	/*cv::createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "screen",
		&dilation_elem, max_elem,
		on_ProcessTrackbar);*/

	/*cv::createTrackbar("Kernel size:\n 2n +1", "screen",
		&dilation_size, max_kernel_size,
		on_ProcessTrackbar);*/
	//on_ProcessTrackbar(0, 0);
	


	int key = 0;
	cv::Mat bwscreen(frame_height,frame_width,CV_8U);
	while(key != 'q'){

		GazePoint gaze_point = gaze_scraper.getGazePoint();
		frame = frame_grabber.getLastFrame();
		clahe->apply(frame,frame);
		cv::Mat screen;
		cv::resize(printscreen(0,0,screen_width,screen_height),screen,cv::Size(frame_width,frame_height));
		cv::cvtColor(screen,bwscreen,cv::COLOR_RGBA2GRAY);
		clahe->apply(bwscreen,bwscreen);
		/* homography from screen to frame and vice-versa */
		cv::Mat homography_s2f;
		cv::Mat homography_f2s;
		cv::Mat debug;
		homography_state hs = HOMOG_FAIL;
		on_ProcessTrackbar(0, 0);
		cv::cvtColor(frame, frame, cv::COLOR_GRAY2RGB);
		//processFrame(frame);
		/*
		cv::namedWindow("medianBlur", cv::WINDOW_AUTOSIZE);
		char medianBlurName[20];
		sprintf(medianBlurName, "core value %d", g_nMedianBlurMaxValue);
		g_nMedianBlurValue = 1;

		//create track bar
		cv::createTrackbar(medianBlurName, "medianBlur", &g_nMedianBlurValue,
			g_nMedianBlurMaxValue, on_medianBlurTrackBar);
		on_medianBlurTrackBar(g_nMedianBlurValue, 0);
		*/
		
		switch (descriptor_method) {
		    case ORB:
				//std::cout << "contour size" <<edgepoints.size() << std::endl;
				//std::cout << "reference contour size" << reference_contour.size() << std::endl;
				if (edgepoints.size() > 0) {
					hs = retrieveHomography(frame, bwscreen, homography_s2f, edgepoints, reference_contour, camera2distance, &debug);
					edgepoints.clear();
					std::vector<std::vector<cv::Point>>().swap(edgepoints);
				}
				break;
			case KNN:
				hs = retrieveHomographyNeighbourhood(frame, bwscreen, homography_s2f, &debug);
				break;
			}
		
		/* using the homography, invert it to go from frame to screenspace
		 * apply the inverse homography to the gaze point, normalize the output
		 * relative to the screen dimensions, and send down socketIO for use by
		 * gaze-enabled applications
		 */
		if(hs == HOMOG_SUCCESS){
			//std::cout << "HOMOG_SUCCESS" << std::endl;
			std::cout << "success camera2distance= " << camera2distance << std::endl;
			homography_f2s = homography_s2f.inv();

			cv::Mat point(1,1,CV_64FC2);
			cv::Mat normalized_point(1,1,CV_64FC2);
			
			point.at<double>(0, 0) = gaze_point.x*frame_width;
			point.at<double>(0, 1) = gaze_point.y*frame_height;

			//cv::circle(debug, cvPoint(point.at<double>(0, 0), point.at<double>(0, 1)), 10, cvScalar(255, 0, 0), -1);
			//cv::putText(debug, "FrameSize:"+ std::to_string(frame_width)+"x"+ std::to_string(frame_height), cvPoint(0, 40), CV_FONT_HERSHEY_COMPLEX, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
			//cv::putText(debug, "ScreenSize:" + std::to_string(screen_sub_width) + "x" + std::to_string(screen_sub_height), cvPoint(0, 60), CV_FONT_HERSHEY_COMPLEX, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
			//cv::putText(debug, "gazepoint:" + std::to_string(point.at<double>(0, 0)) + "," + std::to_string(point.at<double>(0, 1)), cvPoint(0, 80), CV_FONT_HERSHEY_COMPLEX, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
			try{
				cv::perspectiveTransform(point, normalized_point, homography_f2s);
				//cv::circle(debug, cvPoint(normalized_point.at<double>(0, 0)+ frame_width, normalized_point.at<double>(0, 1)), 10, cvScalar(255, 0, 0), -1);
				//cv::putText(debug, "normalizedpoint:" + std::to_string(normalized_point.at<double>(0, 0)) + "," + std::to_string(normalized_point.at<double>(0, 1)), cvPoint(0, 100), CV_FONT_HERSHEY_COMPLEX, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
				// normalize the point and send it off down the pipeline 
				if (point.at<double>(0, 0) == 0 ) {
					normalized_point.at<double>(0, 0) = 0;				
					reference_point.at<double>(0, 0) = 0;
				}
				else {
					normalized_point.at<double>(0, 0) /= frame_width;
					reference_point.at<double>(0, 0) = normalized_point.at<double>(0, 0);
				}
				if (point.at<double>(0, 1) == 0) {
					normalized_point.at<double>(0, 1) = 0;
					reference_point.at<double>(0, 1) = 0;
				}
				else {
					normalized_point.at<double>(0, 1) /= frame_height;
					reference_point.at<double>(0, 1) = normalized_point.at<double>(0, 1);
				}
				//std::cout << "x:"<< normalized_point.at<double>(0, 0) << std::endl;
				//std::cout << "y:"<< normalized_point.at<double>(0, 1) << std::endl;
				struct timeval tv;
				char buf[64];
				gettimeofday(&tv, NULL);
				strftime(buf, sizeof(buf) - 1, "%Y-%m-%d %H:%M:%S", localtime(&tv.tv_sec));

				//std::cout << "time start" << tv.tv_sec * 1000000 + tv.tv_usec << std::endl;
				/*if (gaze_log_file.is_open()) {
					gaze_log_file << "Success" << "\n";
					gaze_log_file << buf <<"+"<< tv.tv_usec/1000<<"\n";
					gaze_log_file << "pX" << point.at<double>(0, 0) << "\n";
					gaze_log_file << "pY" << point.at<double>(0, 1) << "\n";
					gaze_log_file << "TpX" << normalized_point.at<double>(0, 0) << "\n";
					gaze_log_file << "TpY" << normalized_point.at<double>(0, 1) << "\n\n";
				}*/
				//std::cout << "Success id:"<< signature << std::endl;  
				//std::cout << "x:" << normalized_point.at<double>(0, 0) << std::endl;
				//std::cout << "y:" << normalized_point.at<double>(0, 1) << std::endl;
				sio::message::list li;
				li.push(sio::double_message::create(normalized_point.at<double>(0,0)));
				li.push(sio::double_message::create(normalized_point.at<double>(0,1)));
				li.push(sio::int_message::create(signature));
				li.push(sio::int_message::create(camera2distance));
				gaze_emitter.socket()->emit("eye pos",li.to_array_message());
			}catch (const std::exception &e){
				/* this is a non fatal error that pops up */
				std::cerr << e.what() << std::endl;
			}

			hs = HOMOG_FAIL;
			//std::cout << "HOMOG_FAIL" << std::endl;
		}
		else {

			std::cout << "fail camera2distance= " << camera2distance << std::endl;
			if (reference_point.at<double>(0, 0) >= 0 && reference_point.at<double>(0, 1) >= 0) {
				try {
					cv::Mat point(1, 1, CV_64FC2);

					point.at<double>(0, 0) = gaze_point.x*frame_width;
					point.at<double>(0, 1) = gaze_point.y*frame_height;

					struct timeval tv;
					char buf[64];
					gettimeofday(&tv, NULL);
					strftime(buf, sizeof(buf) - 1, "%Y-%m-%d %H:%M:%S", localtime(&tv.tv_sec));

					/*if (gaze_log_file.is_open()) {
						gaze_log_file << "Fail" << "\n";
						gaze_log_file << buf <<"+"<< tv.tv_usec / 1000 << "\n";
						gaze_log_file << "pX" << point.at<double>(0, 0) << "\n";
						gaze_log_file << "pY" << point.at<double>(0, 1) << "\n";
						gaze_log_file << "RpX" << reference_point.at<double>(0, 0) << "\n";
						gaze_log_file << "RpY" << reference_point.at<double>(0, 1) << "\n\n";
					}*/
					//std::cout << "Fail id:" << signature << std::endl;
					sio::message::list li;
					li.push(sio::double_message::create(reference_point.at<double>(0, 0)));
					li.push(sio::double_message::create(reference_point.at<double>(0, 1)));
					li.push(sio::int_message::create(signature));
					li.push(sio::int_message::create(camera2distance));
					gaze_emitter.socket()->emit("eye pos", li.to_array_message());
				}
				catch (const std::exception &e) {
					/* this is a non fatal error that pops up */
					std::cerr << e.what() << std::endl;
				}
			}
		}
		//cv::imshow("debug",debug);
		//outputVideo.write(debug);
		//std::cout << "frame size" << frame.size() << std::endl;
		
		//cv::imshow("Frame", frame);
		//std::cout << "frame size" << frame.size() << std::endl;
		//std::cout << "frame type" << frame.type() << std::endl;
		//outputVideo2.write(frame);
		key = cv::waitKey(1);
		//std::cout << "key:" << key << std::endl;	
		
	}
	//outputVideo.release();
	//outputVideo2.release();
	gaze_scraper.stop();
	frame_grabber.stop();

	gaze_thread.join();
	frame_thread.join();

	gaze_emitter.sync_close();

	return 0;
}

