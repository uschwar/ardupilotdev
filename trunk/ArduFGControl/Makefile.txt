CC=g++
#CCFLAGS=-g -Wall -fPIC
#CCFLAGS=  -I../.. -I../../../other/install/include/opencv   
CCFLAGS += -c
#CCFLAGS +=  -I/usr/include/X11

LDFLAGS = -Wl 
#-shared  

all: objects
	$(CC) -o fgrxtx fgrxtx.o auto_float.o $(LDFLAGS)

objects: fgrxtx.cpp  
	$(CC) $(CCFLAGS) -o fgrxtx.o fgrxtx.cpp
	$(CC) $(CCFLAGS) -o auto_float.o auto_float.cpp

clean: 
	- rm -f fgrxtx.o fgrxtx.dll atan_fixed.o *~
