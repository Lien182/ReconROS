<<reconos_preproc>>

CC = gcc
CFLAGS += -Wall -I <<REPO_REL>> -I lib/include -I /opt/ros/<<ROS2_DISTRO>>/include/ <<MSGINCLUDEDIR>> -D RECONOS_OS_<<OS>> -D RECONOS_BOARD_<<BOARD>> -DROS2_DISTRIBUTION=<<ROS2_DISTRIBUTION>> <<CFLAGS>>
LDFLAGS += -lpthread <<LDFLAGS>> -Bdynamic -L /opt/ros/<<ROS2_DISTRO>>/lib/*.so -L /opt/ros/<<ROS2_DISTRO>>/lib/*/*.so -L ../build.msg/install/*/lib/*.so

MKDIRP = mkdir -p

OBJS = <<generate for OBJS>><<Source>> <<end generate>>
BUILD_OBJS = $(OBJS:%=_build/%)

all: $(BUILD_OBJS)
	$(CC) $(BUILD_OBJS) $(LDFLAGS) -o <<NAME>>

install: all
	cp <<NAME>> $(PREFIX)

clean:
	rm -rf _build <<NAME>>

_build/%.o: %.c
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -x c -c $< -o $@

_build/%.o: %.cpp
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

_build/%.o: %.cc
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -c $< -o $@
