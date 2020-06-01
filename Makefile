srcdir = $(shell pwd)
module = vcio2
version = 0.3

install:
	dkms install $(srcdir)
	cp $(srcdir)/extra/udev/rules.d/10-vcio2.rules /etc/udev/rules.d/
	cp $(srcdir)/extra/modules-load.d/10-vcio2.conf /etc/modules-load.d/
	modprobe vcio2

remove:
	-rm /etc/modules-load.d/10-vcio2.conf
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
	
