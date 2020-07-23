

CC = $(CROSS_COMPILE)gcc
CFLAGS += -Wall -I ../../.. -I lib/include -I /opt/ros/dashing/include/ -D RECONOS_OS_linux -D RECONOS_BOARD_zedboard_d -I $(RECONOS)/linux/tools/timer
LDFLAGS += -lpthread -L $(RECONOS)/linux/tools/timer -ltimer -Bdynamic -L /opt/ros/dashing/lib/*.so

MKDIRP = mkdir -p

OBJS = src/application/main.o src/rt_sortdemo/hls/sortdemo.o src/rt_sortdemo/c/sortdemo.o build.sw/application/main.o build.sw/rt_sortdemo/sortdemo.o build.sw/lib/runtime/reconos_app.o build.sw/lib/runtime/reconos.o build.sw/lib/runtime/comp/mbox.o build.sw/lib/runtime/comp/rqueue.o build.sw/lib/runtime/comp/ros_sub.o build.sw/lib/runtime/comp/ros_pub.o build.sw/lib/runtime/comp/ros.o build.sw/lib/runtime/arch/arch_linux.o build.sw/lib/runtime/arch/arch_xilkernel.o info/application/main.o info/rt_sortdemo/sortdemo.o info/lib/runtime/reconos_app.o info/lib/runtime/reconos.o info/lib/runtime/comp/mbox.o info/lib/runtime/comp/rqueue.o info/lib/runtime/comp/ros_sub.o info/lib/runtime/comp/ros_pub.o info/lib/runtime/comp/ros.o info/lib/runtime/arch/arch_linux.o info/lib/runtime/arch/arch_xilkernel.o application/main.o build.hw/myReconOS.srcs/sources_1/bd/design_1/ip/design_1_processing_system7_0_0/ps7_init_gpl.o build.hw/myReconOS.srcs/sources_1/bd/design_1/ip/design_1_processing_system7_0_0/ps7_init.o rt_sortdemo/sortdemo.o lib/runtime/reconos_app.o lib/runtime/reconos.o lib/runtime/comp/mbox.o lib/runtime/comp/rqueue.o lib/runtime/comp/ros_sub.o lib/runtime/comp/ros_pub.o lib/runtime/comp/ros.o lib/runtime/arch/arch_linux.o lib/runtime/arch/arch_xilkernel.o 
BUILD_OBJS = $(OBJS:%=_build/%)

all: $(BUILD_OBJS)
	$(CC) $(BUILD_OBJS) $(LDFLAGS) -o sortdemo

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
