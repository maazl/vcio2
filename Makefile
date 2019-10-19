srcdir = $(shell pwd)
module = vcio2
version = 0.2

install:
	dkms install $(srcdir)
	modprobe vcio2

remove:
	-modprobe -r vcio2
	dkms remove $(module)/$(version) --all

build:
	-dkms add $(srcdir)
	dkms build $(module)/$(version)

obj-m = src/vcio2.o
ccflags-y := -I$(PWD)/include

all:
	make -C /lib/modules/${kernelver}/build M=$(PWD)/src

clean:
	make -C /lib/modules/${kernelver}/build M=$(PWD)/src clean
	
