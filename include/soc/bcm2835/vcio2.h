#ifndef _MACH_BCM2708_VCIO2_H
#define _MACH_BCM2708_VCIO2_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#include <stdint.h>
#endif

#define IOCTL_VCIO2_TYPE 101

/** Allocate VC memory */
#define IOCTL_MEM_ALLOCATE _IOWR(IOCTL_VCIO2_TYPE, 0x01, vcio_mem_allocate)
typedef struct
{
	union {
		struct
		{	uint32_t size;       /**< Number of bytes to allocate */
			uint32_t alignment;  /**< Alignment */
			uint32_t flags;      /**< Allocation flags */
		} in;                  /**< IOCTL parameter data */
		struct
		{	uint32_t handle;     /**< QPU memory handle */
		} out;                 /**< IOCTL return data */
	};
} vcio_mem_allocate;       /**< IOCTL structure for IOCTL_MEM_ALLOCATE */

/** Release VC memory
 * IOCTL param: unsigned handle of memory from IOCTL_MEM_ALLOCATE. */
#define IOCTL_MEM_RELEASE  _IO(IOCTL_VCIO2_TYPE, 0x02)

/** Lock VC memory at fixed address
 * IOCTL param input: uint32_t* handle of memory from IOCTL_MEM_ALLOCATE.
 * IOCTL param output: uint32_t* bus_address of the memory. */
#define IOCTL_MEM_LOCK     _IOWR(IOCTL_VCIO2_TYPE, 0x03, uint32_t)

/** Unlock VC memory
 * IOCTL param: unsigned handle of memory from IOCTL_MEM_ALLOCATE. */
#define IOCTL_MEM_UNLOCK   _IO(IOCTL_VCIO2_TYPE, 0x04)

/** Enable QPU power
 * IOCTL param: unsigned flag whether to enable or disable the QPU */
#define IOCTL_ENABLE_QPU   _IO(IOCTL_VCIO2_TYPE, 0x12)

/** Execute code on the QPU */
#define IOCTL_EXEC_QPU     _IOW(IOCTL_VCIO2_TYPE, 0x11, vcio_exec_qpu)
typedef struct
{	uint32_t uniforms;       /**< Bus address of start of uniforms for one QPU */
	uint32_t code;           /**< Bus address of start of code for one QPU */
} vcio_exec_qpu_entry;     /**< Startup data for one QPU */
typedef struct
{	struct
	{	uint32_t num_qpus;     /**< Number of QPUs to use */
		uint32_t control;      /**< Bus address of array of vcio_exec_qpu_entry with num_qpus elements */
		uint32_t noflush;      /**< 1 => do not flush L2 cache */
		uint32_t timeout;      /**< Timeout in ms */
	} in;
} vcio_exec_qpu;           /**< IOCTL structure for IOCTL_EXEC_QPU */

/** Get driver version
 * IOCTL param output: unsigned* high word: major version, low word: minor version */
#define IOCTL_GET_VCIO_VERSION _IOR(IOCTL_VCIO2_TYPE, 0xc0, uint32_t)

/** Enable performance counters
 * IOCTL param: Bit vector of counters to enable for this instance.
 * Note that no more than 16 performance counters may be enabled at the same time. */
#define IOCTL_SET_V3D_PERF_COUNT _IO(IOCTL_VCIO2_TYPE, 0xc1)
/** Performance counters supported by this driver. */
enum v3d_perf_count
{	V3D_PERF_COUNT_QPU_CYCLES_IDLE               = 1<<13,
	V3D_PERF_COUNT_QPU_CYCLES_VERTEX_SHADING     = 1<<14,
	V3D_PERF_COUNT_QPU_CYCLES_FRAGMENT_SHADING   = 1<<15,
	V3D_PERF_COUNT_QPU_CYCLES_VALID_INSTRUCTIONS = 1<<16,
	V3D_PERF_COUNT_QPU_CYCLES_STALLED_TMU        = 1<<17,
	V3D_PERF_COUNT_QPU_CYCLES_STALLED_SCOREBOARD = 1<<18,
	V3D_PERF_COUNT_QPU_CYCLES_STALLED_VARYINGS   = 1<<19,
	V3D_PERF_COUNT_QPU_INSTRUCTION_CACHE_HITS    = 1<<20,
	V3D_PERF_COUNT_QPU_INSTRUCTION_CACHE_MISSES  = 1<<21,
	V3D_PERF_COUNT_QPU_UNIFORMS_CACHE_HITS       = 1<<22,
	V3D_PERF_COUNT_QPU_UNIFORMS_CACHE_MISSES     = 1<<23,
	V3D_PERF_COUNT_TMU_TEXTURE_QUADS_PROCESSED   = 1<<24,
	V3D_PERF_COUNT_TMU_TEXTURE_CACHE_MISSES      = 1<<25,
	V3D_PERF_COUNT_VPM_CYCLES_STALLED_VDW        = 1<<26,
	V3D_PERF_COUNT_VPM_CYCLES_STALLED_VCD        = 1<<27,
	V3D_PERF_COUNT_L2C_L2_CACHE_HITS             = 1<<28,
	V3D_PERF_COUNT_L2C_L2_CACHE_MISSES           = 1<<29,
};

/** Get currently enabled performance counters of instance.
 * IOCTL param output: uint32_t
 */
#define IOCTL_GET_V3D_PERF_COUNT _IOR(IOCTL_VCIO2_TYPE, 0xc1, uint32_t)

/** Read performance counters
 * IOCTL param input: uint32_t[] where to store the counter values in ascending order.
 * No more counters than enabled for this device handle will be stored and at most 16. */
#define IOCTL_READ_V3D_PERF_COUNT _IOR(IOCTL_VCIO2_TYPE, 0xc2, uint32_t [16])

/** Reset performance counters */
#define IOCTL_RESET_V3D_PERF_COUNT _IO(IOCTL_VCIO2_TYPE, 0xc3)

#undef DEVICE_FILE_NAME
#define DEVICE_FILE_NAME "vcio2" /**< The name of the device file */

#endif
