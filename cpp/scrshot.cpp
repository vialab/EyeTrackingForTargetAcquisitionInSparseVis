#include "scrshot.hpp"
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "imageprocess.hpp"

/* this method from http://stackoverflow.com/questions/24988164/c-fast-screenshots-in-linux-for-use-with-opencv */
cv::Mat printscreen(const int x, const int y, const int w, const int h){
	Display *display = XOpenDisplay(NULL);
	Window root = DefaultRootWindow(display);
	XMapRaised(display,root);

	XImage *img = XGetImage(display,root,x,y,w,h,AllPlanes,ZPixmap);
	const int bits_per_pixel = img->bits_per_pixel;

	cv::Mat screen (h,w,bits_per_pixel > 24 ? CV_8UC4 : CV_8UC3);
	memcpy(screen.data,img->data,h*w*(bits_per_pixel/8));

	XDestroyImage(img);
	XCloseDisplay(display);

	return screen;
}
