#ifndef _MACH_BCM2708_VCIO2_H
#define _MACH_BCM2708_VCIO2_H

#ifdef __KERNEL__
#include <mach/vcio.h>
#include <linux/ioctl.h>
#endif

#ifdef BCM_VCIO2_ADD
#undef BCM_VCIO_DRIVER_NAME
#define BCM_VCIO_DRIVER_NAME "bcm2708_vcio2"
#endif


#define IOCTL_MEM_TYPE 100

typedef struct
{
	union {
		struct
		{	unsigned int size;
			unsigned int alignment;
			unsigned int flags;
		} in;
		struct
		{	unsigned int handle;
		} out;
	};
} vcio_mem_allocate;
#define IOCTL_MEM_ALLOCATE _IOWR(IOCTL_MEM_TYPE, 1, vcio_mem_allocate)

#define IOCTL_MEM_RELEASE  _IOC(_IOC_READ|_IOC_WRITE, IOCTL_MEM_TYPE, 2, 0)

#define IOCTL_MEM_LOCK     _IOWR(IOCTL_MEM_TYPE, 3, unsigned)

#define IOCTL_MEM_UNLOCK   _IOC(_IOC_READ|_IOC_WRITE, IOCTL_MEM_TYPE, 4, 0)

typedef struct
{
	unsigned int uniforms;
	unsigned int code;
} vcio_exec_qpu_entry;

typedef struct
{
	struct
	{	unsigned int num_qpus;
		unsigned int control;  /**< physical address of array of vcio_exec_qpu_entry with num_qpus elements */
		unsigned int noflush;
		unsigned int timeout;
	} in;
} vcio_exec_qpu;
#define IOCTL_EXEC_QPU     _IOWR(IOCTL_MEM_TYPE, 11, vcio_exec_qpu)

/*
 * The name of the device file
 */
#undef DEVICE_FILE_NAME
#define DEVICE_FILE_NAME "vcio2"

#endif
