
all: main

clean:
	rm main.o Trajectory.o MDPoint.o main
    
main: main.o Trajectory.o MDPoint.o
	g++ -g -o main main.o Trajectory.o MDPoint.o
    
main.o: main.cpp
	g++ -c -g main.cpp
	
Trajectory.o: Trajectory.cpp
	g++ -c -g Trajectory.cpp
	
MDPoint.o: MDPoint.cpp
	g++ -c -g MDPoint.cpp
	