
all: main

clean:
	rm main.o main
    
main: main.o
	g++ -g -o main main.o
    
main.o: main.cpp
	g++ -c -g main.cpp