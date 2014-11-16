CC = g++
CFLAGS = -std=c++11 -g 
LFLAGS = -lboost_iostreams -lboost_system -lboost_filesystem
DEPS = DistanceOutlier.h MDPoint.h Outlier.h OutlierDetector.h TrajData.h Trajectory.h Param.h

all: main

clean:
	-@rm *.o main convertData
    
main: main.o Trajectory.o MDPoint.o OutlierDetector.o DistanceOutlier.o Measure.o Outlier.o TrajData.o
	$(CC) $(CFLAGS) -o $@ $^ $(LFLAGS)

convertData: ConvertData.cpp csv_parser.cpp csv_parser.hpp
	$(CC) $(CFLAGS) -o $@ ConvertData.cpp csv_parser.cpp

%.o: %.cpp $(DEPS)
	$(CC) $(CFLAGS) -c -o $@ $<
    

