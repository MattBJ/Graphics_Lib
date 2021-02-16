.POSIX:
.SUFFIXES:

# going to need box2Dlib !

CC	:= g++
# CFLAGS	:= -Wall -Werror -g -O0 # -lm
CFLAGS := -O0 -Wall -g `sdl2-config --libs --cflags` -ggdb3 -lSDL2_image -std=c++17 # --> for std::optional and auto stuff i think
#LDLIBS	:= -lbsd
PROGS	= test main
OBJS	= bodies.o main.o # PVector.o
# LIBS	= my_math_lib.a
# LIBS_DIR = /home/mbailey/Desktop/Learning/Lib_creator/my_libs
# SOURCEDIR = /home/mbailey/Desktop/Learning/Lib_creator

# using wildcard function to create a list
# using patsubst to substitute the pattern: patsubst pattern,replacement, list

INC = ../EigenLib/eigen-3.3.9
# INC += /home/matt/
INC_PARAMS=$(foreach d, $(INC), -I$d)

.SUFFIXES: .c .o

all: $(PROGS) #$(LIBS)

test: test.o bodies.o
	$(CC) -o $@ $^ $(CFLAGS)

test.o: test.cpp
	$(CC) $(INC_PARAMS) $(CFLAGS) -c $^ # test.cpp

main: $(OBJS)
	$(CC) $(OBJS) -o $@ $(CFLAGS)
#$(CC) $(OBJS) -o $(PROGS)

main.o: main.cpp
	$(CC) $(INC_PARAMS) $(CFLAGS) -c $^

PVector.o: PVector.cpp
	$(CC) $(INC_PARAMS) $(CFLAGS) -c $^

bodies.o: bodies.cpp
	$(CC) $(INC_PARAMS) $(CFLAGS) -c $^

# my_math_lib.a: matrix_math.o
# 	ar rsv my_math_lib.a matrix_math.o
#ar rsv $(LIBS_DIR)/$(LIBS) matrix_math.o
# ar rsv $(patsubst %.a,$(LIBS_DIR)/%.a,$(wildcard $(LIBS)))

# r = replace
# s = create
# v = verbose

# to test the contents: ar -t <library.a> --> outputs the contents

clean:
	rm $(PROGS) *.o $(LIBS)
	
SOURCEDIR = src/code

SOURCES = $(wildcard $(SOURCEDIR)/*.cpp)
