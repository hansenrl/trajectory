CC=g++
CFLAGS=-c -g -std=c++11

all: main

clean:
	rm main.o Trajectory.o MDPoint.o OutlierDetector.o main
    
main: main.o Trajectory.o MDPoint.o OutlierDetector.o
	$(CC) -g -o main main.o Trajectory.o MDPoint.o
    
main.o: main.cpp
	$(CC) $(CFLAGS) main.cpp
	
Trajectory.o: Trajectory.cpp
	$(CC) $(CFLAGS) Trajectory.cpp
	
MDPoint.o: MDPoint.cpp
	$(CC) $(CFLAGS) MDPoint.cpp
	
OutlierDetector.o: OutlierDetector.cpp
	$(CC) $(CFLAGS) OutlierDetector.cpp
