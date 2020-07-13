<<reconos_preproc>>

CC = $(CROSS_COMPILE)gcc
CFLAGS += -Wall -I <<REPO_REL>> -I lib/include -I /opt/ros/dashing/include/ -I msg/my_reconros_services/install/my_reconros_services/include/ -D RECONOS_OS_<<OS>> -D RECONOS_BOARD_<<BOARD>> <<CFLAGS>>
LDFLAGS += -lpthread <<LDFLAGS>> -Bdynamic -L /opt/ros/dashing/lib/*.so -L msg/my_reconros_services/install/my_reconros_services/lib/*.so

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
