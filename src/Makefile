kernelver = $(shell uname -r)
obj-m := vcio2.o
ccflags-y := -I$(PWD)/include

all:
	make -C /lib/modules/${kernelver}/build M=$(PWD)

clean:
	make -C /lib/modules/${kernelver}/build M=$(PWD) clean

#install:
#	make -C /lib/modules/${kernelver}/build M=$(PWD) install
