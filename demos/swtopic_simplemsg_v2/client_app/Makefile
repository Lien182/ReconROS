CC = gcc

CFLAGS += -g -Wno-unused-variable -Wall -I ../../../../../reconos/reconos -I lib/runtime/comp/ -I /opt/ros/dashing/include/ -I lib/include 
LDFLAGS += -lpthread -L /opt/ros/dashing/lib/*.so

MKDIRP = mkdir -p

OBJS = lib/comp/ros.o lib/comp/ros_sub.o lib/comp/ros_pub.o application/main.o 

BUILD_OBJS = $(OBJS:%=_build/%)

all: $(BUILD_OBJS)
	$(CC) $(BUILD_OBJS) $(LDFLAGS) -o client_app

install: all
	cp sortdemo $(PREFIX)

clean:
	rm -rf _build client_app

_build/%.o: %.c
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -x c -c $< -o $@

_build/%.o: %.cpp
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -x c -c $< -o $@
