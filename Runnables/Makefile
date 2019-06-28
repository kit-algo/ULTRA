CC=g++
FLAGS=-std=c++17 -fopenmp -pipe
OPTIMIZATION=-march=native -O3
DEBUG=-rdynamic -Werror -Wpedantic -pedantic-errors -Wall -Wextra -Wparentheses -Wfatal-errors -D_GLIBCXX_DEBUG -g -fno-omit-frame-pointer
RELEASE=-ffast-math -ftree-vectorize -Wfatal-errors -DNDEBUG

all: BuildBucketCH BuildCoreCH ComputeShortcuts RunCSAQueries RunRAPTORQueries

BuildBucketCH:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o BuildBucketCH BuildBucketCH.cpp

BuildCoreCH:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o BuildCoreCH BuildCoreCH.cpp

ComputeShortcuts:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o ComputeShortcuts ComputeShortcuts.cpp

RunCSAQueries:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o RunCSAQueries RunCSAQueries.cpp
	
RunRAPTORQueries:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o RunRAPTORQueries RunRAPTORQueries.cpp