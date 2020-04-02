# needed environment variables
# (shold be set by the reconos toolchain)
# CROSS_COMPILE

# export CROSS_COMPILE=gcc-arm-linux-gnueabihf

CC = arm-linux-gnueabihf-gcc
AR = arm-linux-gnueabihf-ar

OBJS := timer.o

CFLAGS = -fPIC -O2 -g -Wall

all: libtimer.a

libtimer.a: $(OBJS)
	$(AR) -rcsv libtimer.a $(OBJS)

clean:
	rm -f *.o *.a
