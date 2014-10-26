CC=g++
CFLAGS=-std=c++11 -g 
LFLAGS=-lboost_iostreams -lboost_system -lboost_filesystem
DEPS=DistanceOutlier.h MDPoint.h Outlier.h OutlierDetector.h trajData.h Trajectory.h Param.h

all: main

clean:
	rm main.o Trajectory.o MDPoint.o OutlierDetector.o DistanceOutlier.o Measure.o Outlier.o main
    
main: main.o Trajectory.o MDPoint.o OutlierDetector.o DistanceOutlier.o Measure.o Outlier.o
	$(CC) $(CFLAGS) -o $@ $^ $(LFLAGS)

%.o: %.cpp $(DEPS)
	$(CC) $(CFLAGS) -c -o $@ $<
    

