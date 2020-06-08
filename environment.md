# This project is built on a previous member Santiago Bonada
https://github.com/SGBon/pupil-progs/tree/master/socket/cpp
The project has to be ran under Ubuntu 
Written by Santiago Bonada
1. First we have to install the proper packages. In the commandline do:
apt install libzmq3-dev libmsgpack-dev libx11-dev libboost-system-dev libboost-random-dev libboost-date-time-dev libssl-dev cmake
2. Opencv3 is required, from when this program was written only opencv2 was available in the package manager, if opencv3 is offered then use the package manager for that (apt), otherwise do the next step
Follow the instructions here under Building OpenCV From Source: https://docs.opencv.org/3.4/d2/de6/tutorial_py_setup_in_ubuntu.html
This will install opencv libraries for use by c++, not just Python.
3.Build socket.io-client-cpp. This library is used for communication between our program and the webserver.
Follow the instructions here for With CMake: https://github.com/socketio/socket.io-client-cpp/blob/master/INSTALL.md#with-cmake
4. After building socketio, open up your file manager, navigate to the folder where the socket io repository is, copy the include/ and lib/ directory.
Under the cpp directory create a directory extern/. Within that directory create a directory SocketIO/. paste the two directories here.
5. Once this is all finished, in the terminal navigate to the cpp/ directory and do make -j which should build the program.
Pupil Capture:
this software works with pupil capture, obtainable from https://github.com/pupil-labs/pupil/releases
A couple configurations are required in pupil capture. First is to open up the plugin manager from the menu on the right and enable frame publisher and pupil remote
these should add buttons to access those plugins at the bottom right. In frame publisher set the format to 'gray image'. In pupil remote here is where you set what port data is published from. Our program defaults to 50020 for the port.
Under uvc source there is a configuration for the resolution of the world camera, this is necessary for running the program as it needs to know the resolution of frames from the world camera.
Last but not least whenever the eyetracker is moved, you have to recalibrate it in pupil capture since the calibration is based on a fixed frame of reference between the two cameras and the position of your eyes in relation to them.
Play around with pupil capture to get an idea of what it does.
To run the program you have to navigate in your terminal to where it was built and do:
./match.exe [frame_width] [frame_height] [screen_width] [screen_height] [index]
frame_width and frame_height are the values set in the UVC source for the resolution
screen_width and screen_height is the resolution of the monitor that has the content you want to interact with. The way it works now it's always the left most monitor.
index is basically an identifier for the tracker, the webserver that receives data from this program receives the index so it knows which eyetracker is interacting with it. Generally the first eye tracker will be 0, the second will be 1, etc.
there are 2 more arguments that are optional following [index], [method] and [address]
for method just use "orb", "knn" is a valid argument but it was an experimental algorithm that did not work
address is the ip address of the computer running an instance of pupil capture with pupil remote enabled.
You can also not put any arguments when running ./match.exe, this basically defaults to
./match.exe 640 480 1920 1080 0 orb 127.0.0.1
the first couple lines of the main function in match.cpp will give insight on these arguments.
The program will not do anything if there is no instance of the pupil capture program running sending data of pupil remote.
When the program is running it will grab gaze location from the eye tracker and the view from the world camera as well as screenshots of the display.
The screenshots and the world camera image are compared to find where one lies in the other and transforms the gaze position to be in the same coordinate space as the display
The computed gaze position is then sent to the webserver where it is used by the frontend.
For testing purposes, under the socket/ directory in this repository are two python programs sigtester.py and fixedtester.py
These can be used to essentially spoof the program above and send mock gaze positions to the webserver, so check them out they'll give insight in how the webserver expects data to come in.

When I mention 'webserver' in the instructions I'm referring to instances of the server that is running the frontend stuff, e.g. the demos in CGIV or the example in the gt.js repository. 
