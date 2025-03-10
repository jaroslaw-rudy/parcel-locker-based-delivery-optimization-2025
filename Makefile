all:
	g++ --std=c++17 -O3 src/*.cpp -o main

debug:
	g++ --std=c++17 -g src/*.cpp -o main
