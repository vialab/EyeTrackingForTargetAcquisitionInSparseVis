CXX=g++
INCLUDE=-Iextern/SocketIO/include/
CFLAGS=-c -Wall -Wextra -g -std=c++11 -Wno-write-strings -fopenmp $(INCLUDE)
LIBS=-lpthread -lzmq -lmsgpack -lX11 `pkg-config --libs opencv` -Lextern/SocketIO/lib/Release/ -lsioclient -lboost_system -lboost_date_time -lboost_random -lcrypto -lssl
LDFLAGS=$(LIBS) -fopenmp
SOURCES=match.cpp PupilGazeScraper.cpp util.cpp PupilFrameGrabber.cpp scrshot.cpp homography.cpp averagewindow.cpp scd.cpp imageprocess.cpp
OBJECTS=$(SOURCES:.cpp=.o)
DEPEND=.depend
EXECUTABLE=match.exe

all: $(SOURCES) $(EXECUTABLE) $(DEPEND)

$(DEPEND): $(SOURCES)
	rm -f $@
	$(CXX) -MM $^ > $@ $(CFLAGS);

include .depend

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@

%.o : %.cpp
	$(CXX) $(CFLAGS) $< -o $@

.PHONY: clean run
run: all
	./$(EXECUTABLE)
clean:
	rm $(OBJECTS) $(EXECUTABLE) $(DEPEND)
