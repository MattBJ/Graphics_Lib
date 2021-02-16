.POSIX:
.SUFFIXES:

CC	:= g++
CFLAGS := -O0 -Wall -g `sdl2-config --libs --cflags` -ggdb3 -lSDL2_image -std=c++17 # --> for std::optional and auto stuff i think
PROGS	= main
OBJS	= bodies.o main.o

INC = ../EigenLib/eigen-3.3.9
# INC += /home/matt/
INC_PARAMS=$(foreach d, $(INC), -I$d)

.SUFFIXES: .c .o

all: $(PROGS)

main: $(OBJS)
	$(CC) $(OBJS) -o $@ $(CFLAGS)

main.o: main.cpp
	$(CC) $(INC_PARAMS) $(CFLAGS) -c $^

bodies.o: bodies.cpp
	$(CC) $(INC_PARAMS) $(CFLAGS) -c $^

clean:
	rm $(PROGS) *.o $(LIBS)
	
SOURCEDIR = src/code

SOURCES = $(wildcard $(SOURCEDIR)/*.cpp)
