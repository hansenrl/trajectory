CC=g++
CFLAGS=-std=c++11 -g
DEPS=DistanceOutlier.h MDPoint.h Outlier.h OutlierDetector.h trajData.h Trajectory.h Param.h

all: main

clean:
	rm main.o Trajectory.o MDPoint.o OutlierDetector.o DistanceOutlier.o Measure.o Outlier.o main
    
main: main.o Trajectory.o MDPoint.o OutlierDetector.o DistanceOutlier.o Measure.o Outlier.o
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.cpp $(DEPS)
	$(CC) $(CFLAGS) -c -o $@ $<
    

