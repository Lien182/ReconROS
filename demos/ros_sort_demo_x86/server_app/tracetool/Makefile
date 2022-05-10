

CC = gcc
CFLAGS += -Wall -I ../../../.. -I lib/include -I /opt/ros/dashing/include/ -I ../build.msg/install/*/include/ -D RECONOS_OS_linux_x86 -D RECONOS_BOARD_zcu104_0 -DROS2_DISTRIBUTION=D -I . -fPIC 
LDFLAGS += -lpthread -L /usr/local/lib/ -llttng-ust -ldl 

MKDIRP = mkdir -p

OBJS = tracetools.o tp.o 
BUILD_OBJS = $(OBJS:%=_build/%)

all: $(BUILD_OBJS)
	$(CC) $(BUILD_OBJS) $(LDFLAGS) -shared -o libtpp.so

install: all
	cp sortdemo $(PREFIX)

clean:
	rm -rf _build sortdemo

_build/%.o: %.c
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -x c -c $< -o $@

_build/%.o: %.cpp
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

_build/%.o: %.cc
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -c $< -o $@
