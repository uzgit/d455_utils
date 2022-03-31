FEFE_SRC = ./wp
MEMWATCH = monitor/monitor

.PHONY: all
all: d455_opencv

#  wp-hs.log does not compile with latest ghc
# wpm-hs.log does not belong to fefes program set?
# wpt-hs.log  does not belong to fefes program set?

d455_opencv: d455_opencv.cpp
	mkdir -p build
	g++ d455_opencv.cpp -lrealsense2 -I/usr/include/opencv4 -lopencv_core -lopencv_imgcodecs -lopencv_imgproc -lopencv_highgui -o ./build/d455_opencv 
