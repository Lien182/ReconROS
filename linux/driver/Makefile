# needed environment variables
# (shold be set by the reconos toolchain)
# ARCH, CROSS_COMPILE, KDIR

obj-m := mreconos.o
mreconos-objs := reconos.o osif_intc.o proc_control.o

module:
	$(MAKE) -C $(KDIR) M=$(PWD) CFLAGS_MODULE=-D"RECONOS_ARCH_$(RECONOS_ARCH)" modules

install: module
	cp mreconos.ko $(PREFIX)
	cp ../scripts/reconos_init.sh $(PREFIX)

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

