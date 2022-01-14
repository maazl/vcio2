kernelver = $(shell uname -r)

kvmaj = $(shell echo $(kernelver) | sed -e 's/^\([0-9][0-9]*\)\.[0-9][0-9]*\.[0-9][0-9]*.*/\1/')
kvmin = $(shell echo $(kernelver) | sed -e 's/^[0-9][0-9]*\.\([0-9][0-9]*\)\.[0-9][0-9]*.*/\1/')
kvrev = $(shell echo $(kernelver) | sed -e 's/^[0-9][0-9]*\.[0-9][0-9]*\.\([0-9][0-9]*\).*/\1/')

kernelver_ge = $(shell echo test | awk '{if($(kvmaj) < $(1)) {print 0} else { if($(kvmaj) > $(1)) {print 1} else { if($(kvmin) < $(2)) {print 0} else { if($(kvmin) > $(2)) {print 1} else { if($(kvrev) < $(3)) {print 0} else { print 1 } }}}}}')

obj-m := vcio2.o
ccflags-y := -I$(PWD)/../include -Wall

ifeq ($(call kernelver_ge,5,10,0),1)
	ccflags-y += -DUSE_MMAP_LOCK=1
endif

all:
	make -C /lib/modules/${kernelver}/build M=$(PWD)

clean:
	make -C /lib/modules/${kernelver}/build M=$(PWD) clean

#install:
#	make -C /lib/modules/${kernelver}/build M=$(PWD) install
