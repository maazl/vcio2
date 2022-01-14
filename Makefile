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

kernelver = $(shell uname -r)
kvmaj = $(shell echo $(kernelver) | sed -e 's/^\([0-9][0-9]*\)\.[0-9][0-9]*\.[0-9][0-9]*.*/\1/')
kvmin = $(shell echo $(kernelver) | sed -e 's/^[0-9][0-9]*\.\([0-9][0-9]*\)\.[0-9][0-9]*.*/\1/')
kvrev = $(shell echo $(kernelver) | sed -e 's/^[0-9][0-9]*\.[0-9][0-9]*\.\([0-9][0-9]*\).*/\1/')
kernelver_ge = $(shell echo test | awk '{if($(kvmaj) < $(1)) {print 0} else { if($(kvmaj) > $(1)) {print 1} else { if($(kvmin) < $(2)) {print 0} else { if($(kvmin) > $(2)) {print 1} else { if($(kvrev) < $(3)) {print 0} else { print 1 } }}}}}')

obj-m = src/vcio2.o
ccflags-y := -I$(PWD)/include

ifeq ($(call kernelver_ge,5,10,0),1)
	ccflags-y += -DUSE_MMAP_LOCK=1
endif

all:
	make -C /lib/modules/${kernelver}/build M=$(PWD)/src

clean:
	make -C /lib/modules/${kernelver}/build M=$(PWD)/src clean
	
