#include "imageprocess.hpp"
#include <cmath>

typedef cv::Point3_<uint8_t> Pixel3;

/* returns gamma corrected value */
static float gamma_correct(const float value);

void rgb2gleam(const cv::Mat &src, cv::Mat &dst){
	dst.forEach<uint8_t>([src](uint8_t &pixel, const int position[]) -> void{
		if(src.type() == CV_8UC4){
			const uint8_t *srcPix = src.ptr<uint8_t>(position[0],position[1]);
			pixel = (srcPix[0]+srcPix[1]+srcPix[2])/3.0f;
		}else if(src.type() == CV_8UC3){
			const Pixel3 *srcPix = src.ptr<Pixel3>(position[0],position[1]);
			pixel = (srcPix->x+srcPix->y+srcPix->z)/3.0f;
		}
		//printf("position %d:%d\n",position[0],position[1]);
	});
}

float gamma_correct(const float value){
	return pow(value,2.2f);
}
