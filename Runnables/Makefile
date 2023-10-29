CC=g++ -fopenmp
FLAGS=-std=c++17 -pipe
OPTIMIZATION=-march=native -O3
DEBUG=-rdynamic -Werror -Wpedantic -pedantic-errors -Wall -Wextra -Wparentheses -Wfatal-errors -D_GLIBCXX_DEBUG -g -fno-omit-frame-pointer
RELEASE=-ffast-math -ftree-vectorize -Wfatal-errors -DNDEBUG


all: DelayExperimentsRelease NetworkRelease ULTRARelease ULTRAPHASTRelease

DelayExperimentsDebug:
	$(CC) $(INCLUDE) $(LIBRARY_PATH) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o DelayExperiments DelayExperiments.cpp $(DYN_LIBRARIES)

DelayExperimentsRelease:
	$(CC) $(INCLUDE) $(LIBRARY_PATH) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o DelayExperiments DelayExperiments.cpp $(DYN_LIBRARIES)

NetworkDebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o Network Network.cpp

NetworkRelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o Network Network.cpp

ULTRADebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o ULTRA ULTRA.cpp

ULTRARelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o ULTRA ULTRA.cpp

ULTRAPHASTDebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o ULTRAPHAST ULTRAPHAST.cpp

ULTRAPHASTRelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o ULTRAPHAST ULTRAPHAST.cpp

