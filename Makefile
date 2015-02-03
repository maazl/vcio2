srcdir = $(shell pwd)/..
module = vcio2
version = 0.1

install: build
	dkms --sourcetree $(srcdir) install $(module)/$(version)
	modprobe vcio2

remove:
	-modprobe -r vcio2
	dkms remove $(module)/$(version) --all

build:
	-dkms --sourcetree $(srcdir) add $(module)/$(version)
	dkms --sourcetree $(srcdir) build $(module)/$(version)

obj-m = src/vcio2.o

all:
	make -C /lib/modules/${kernelver}/build M=$(PWD)/src

clean:
	make -C /lib/modules/${kernelver}/build M=$(PWD)/src clean
	
