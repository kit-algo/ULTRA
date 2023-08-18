CC=g++ -fopenmp
FLAGS=-std=c++17 -pipe
OPTIMIZATION=-march=native -O3
DEBUG=-rdynamic -Werror -Wpedantic -pedantic-errors -Wall -Wextra -Wparentheses -Wfatal-errors -D_GLIBCXX_DEBUG -g -fno-omit-frame-pointer
RELEASE=-ffast-math -ftree-vectorize -Wfatal-errors -DNDEBUG

INCLUDE=-I /usr/include/cairo
LIBRARY_PATH=-L /usr/lib
DYN_LIBRARIES=-lcairo

all: IsochronesRelease NetworkRelease ULTRARelease ULTRAPHASTRelease

IsochronesDebug:
	$(CC) $(INCLUDE) $(LIBRARY_PATH) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o Isochrones Isochrones.cpp $(DYN_LIBRARIES)

IsochronesRelease:
	$(CC) $(INCLUDE) $(LIBRARY_PATH) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o Isochrones Isochrones.cpp $(DYN_LIBRARIES)

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

