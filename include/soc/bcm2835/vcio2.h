#ifndef _MACH_BCM2708_VCIO2_H
#define _MACH_BCM2708_VCIO2_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define IOCTL_VCIO2_TYPE 101

/// Allocate VC memory
#define IOCTL_MEM_ALLOCATE _IOWR(IOCTL_VCIO2_TYPE, 0x01, vcio_mem_allocate)
typedef struct
{
	union {
		struct
		{	uint32_t size;       ///< Number of bytes to allocate
			uint32_t alignment;  ///< Alignment
			uint32_t flags;      ///< Allocation flags
		} in;                  ///< IOCTL parameter data
		struct
		{	uint32_t handle;     ///< QPU memory handle
		} out;                 ///< IOCTL return data
	};
} vcio_mem_allocate;       ///< IOCTL structure for IOCTL_MEM_ALLOCATE

/// Release VC memory
#define IOCTL_MEM_RELEASE  _IO(IOCTL_VCIO2_TYPE, 0x02) // < unsigned handle

/// Lock VC memory at fixed address
#define IOCTL_MEM_LOCK     _IOWR(IOCTL_VCIO2_TYPE, 0x03, unsigned) // < unsigned handle, > unsigned bus_address

/// Unlock VC memory
#define IOCTL_MEM_UNLOCK   _IO(IOCTL_VCIO2_TYPE, 0x04) // < unsigned handle

/// Enable QPU power
#define IOCTL_ENABLE_QPU   _IO(IOCTL_VCIO2_TYPE, 0x12) // < unsigned flag

/// Execute code on the QPU
#define IOCTL_EXEC_QPU     _IOW(IOCTL_VCIO2_TYPE, 0x11, vcio_exec_qpu)
typedef struct
{	uint32_t uniforms;       ///< Bus address of start of uniforms for one QPU
	uint32_t code;           ///< Bus address of start of code for one QPU
} vcio_exec_qpu_entry;     ///< Startup data for one QPU

typedef struct
{	struct
	{	uint32_t num_qpus;     ///< Number of QPUs to use
		uint32_t control;      ///< Bus address of array of vcio_exec_qpu_entry with num_qpus elements
		uint32_t noflush;      ///< 1 => do not flush L2 cache
		uint32_t timeout;      ///< Timeout in ms
	} in;
} vcio_exec_qpu;           ///< IOCTL structure for IOCTL_EXEC_QPU

#undef DEVICE_FILE_NAME
#define DEVICE_FILE_NAME "vcio2" ///< The name of the device file

#endif