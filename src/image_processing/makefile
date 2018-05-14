CC=g++
#CFLAGS+=-g
CFLAGS+=`pkg-config --cflags opencv`
LDFLAGS+=`pkg-config --libs opencv`

.PHONY: all clean
program: main.o
	$(CC) -o main main.o $(LDFLAGS)

main.o: main.cpp
	$(CC) -c -std=c++11 $(CFLAGS) main.cpp

all: program