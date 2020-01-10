/*
 *  vcio2.c
 *
 *  Copyright (C) 2015-2019 Marcel Müller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This device provides a higher level access to the Raspberry Pi
 * VideoCore processor than vcio offers.
 */

#define _GNU_SOURCE
#define DEBUG

// 1 => vcio2 driver is additional to the kernels vcio driver using dkms.
// 0 => vcio2 driver replaces the kernels vcio driver. - Currently unsupported!
#define BCM_VCIO2_ADD 1


#if defined(CONFIG_SERIAL_BCM_MBOX_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif


#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/log2.h>
#include <linux/io.h>
#include <linux/broadcom/vc_mem.h>
#include <soc/bcm2835/raspberrypi-firmware.h>
#include <soc/bcm2835/vcio2.h>


#ifdef BCM_VCIO2_ADD
#define DRIVER_NAME "vcio2"
#else
#define DRIVER_NAME "bcm2708_vcio"
#endif

#define vcio_pr_err(fmt, args...) pr_err("%s ERROR: " fmt "\n", DRIVER_NAME, ## args)
#define vcio_pr_warn(fmt, args...) pr_warn("%s WARN: " fmt "\n", DRIVER_NAME, ## args)
#define vcio_pr_info(fmt, args...) pr_info("%s: " fmt "\n", DRIVER_NAME, ## args)
#define vcio_pr_debug(fmt, args...) pr_devel("%s: " fmt "\n", DRIVER_NAME, ## args)


/// vcio device passed to vcio_probe
static struct device *vcio_pdev = NULL;
/// vcio character device id from cdev_add, non-zero if allocated.
static dev_t vcio_dev = 0;
/// vcio character device, non-zero if allocated.
static struct cdev vcio_cdev0 = { };
/// vcio character device, non-zero if allocated.
static struct cdev vcio_cdev1 = { };
/// device class from class_create, non-zero if allocated.
static struct class *vcio_class = NULL;
/// device vcio created by device_create, non-zero if allocated.
static struct device *vcio_dev0 = NULL;
/// device vcio1 created by device_create, non-zero if allocated.
static struct device *vcio_dev1 = NULL;
/// root pointer for rpi_firmware API
static struct rpi_firmware *firmware = NULL;
/// 1 if Pi is Model 1, i.e. BCM2835
static char vcio_model1 = 1;
/// V3D base address
static uint32_t* vcio_v3d_base = NULL;
/// synchronize access to the following working set.
struct mutex vcio_lock;

/// Is the QPU enabled now? Number of requesting clients.
static unsigned vcio_enabled_count = 0;

/// Mapping of V3D performance counters.
/// Each value consists of the counter index in bits [28..31] and the use count in the lower bits.
/// The array index is the counter id - 13
static unsigned vcio_perf_count_map[17] = {0};
/// Access performance counter id.
/// @param id Counter id, in range [13,29]
#define vcio_perf_count(id) vcio_perf_count_map[(id)-13]
/// Extract performance counter index, i.e. the V3D register number, from map entry.
#define VCIO_PERF_COUNT_INDEX(map) ((map) >> 28)
/// Extract performance counter usage count from map entry.
#define VCIO_PERF_COUNT_USAGE(map) ((map) & ((1<<28)-1))
/// Construct map entry with usage count 0 from counter index.
#define VCIO_PERF_COUNT_MAP(ix) ((ix) << 28)

// Mailbox functions
#include "vcio2.mailbox.c"

/** allocation entry */
typedef struct
{	uint32_t Handle;  /**< Allocation handle from firmware, automatic allocations additionally have the highest bit set. */
	uint32_t Size;    /**< Size of the memory segment in bytes */
	uint32_t Location;/**< Bus address of locked memory segment or VCIOA_LOCATION_NONE if not locked */
	unsigned long Mappings[3];/**< Virtual address in user space of memory mappings for this allocation, VCIOA_LOCATION_NONE for empty slots. */
} vcio_alloc;

#define VCIOA_LOCATION_NONE 0xffffffff

/** Find virtual address mapping slot.
 * @param vca Allocation entry to search for.
 * @param start Search for this address (in user space)
 * or pass VCIOA_LOCATION_NONE to search an empty entry. The LAST free entry will be returned in this case.
 * @return Pointer to the matching entry or NULL if no matching entry found. */
static unsigned long* vcioa_find_mapping(vcio_alloc* vca, unsigned long start)
{	unsigned long* mp = vca->Mappings + sizeof vca->Mappings / sizeof *vca->Mappings - 1;
	/*for (; *mp != start; --mp)
		if (mp == vca->Mappings)
			return NULL;
	return mp;
	... unrolled: */
	return *mp == start ? mp
		: *--mp == start ? mp
		: *--mp == start ? mp
		: NULL;
}

/** Find virtual address mapping slot by address range.
 * @param vca Allocation entry to search for.
 * @param addr Search for this start address (in user space).
 * @param size Search for a range of size bytes starting at start.
 * @return Pointer to the matching entry or NULL if no matching entry found. */
static unsigned long* vcioa_find_mapping_range(vcio_alloc* vca, unsigned long addr, unsigned long size)
{	unsigned long* mp = vca->Mappings + sizeof vca->Mappings / sizeof *vca->Mappings - 1;
	size += addr;
	/*for (; *mp > addr || *mp + vca->Size < size; --mp)
		if (mp == vca->Mappings)
			return NULL;
	return mp;
	... unrolled: */
	return *mp < addr && *mp + vca->Size > size ? mp
		: *--mp < addr && *mp + vca->Size > size ? mp
		: *--mp < addr && *mp + vca->Size > size ? mp
		: NULL;
}

/** Return first virtual user space address of allocated memory.
 * @param vca Allocation entry
 * @return Virtual address of this memory block in user space or VCIOA_LOCATION_NONE if none.
 */
static unsigned long vcioa_get_first_mapping(const vcio_alloc* vca)
{	const unsigned long* mp = vca->Mappings + sizeof vca->Mappings / sizeof *vca->Mappings - 1;
	/*for (; *mp != VCIOA_LOCATION_NONE; --mp)
		if (mp == vca->Mappings)
			return VCIOA_LOCATION_NONE;
	return *mp;
	... unrolled: */
	if (*mp == VCIOA_LOCATION_NONE && *--mp == VCIOA_LOCATION_NONE)
		--mp;
	return *mp;
}

/** Lock GPU memory at a fixed address.
 * @param vca Allocation entry to lock. The returned physical address is written to vca->Location.
 * @return Return value from LockVcMemory. */
static uint32_t vcioa_lock_mem(vcio_alloc* vca)
{	if (unlikely(vca->Location != VCIOA_LOCATION_NONE))
	{	vcio_pr_info("Tried to lock memory %x twice (at %x)", vca->Handle, vca->Location);
		return vca->Location;
	}
	return LockVcMemory(&vca->Location, vca->Handle & 0x7fffffff);
}

/** Unlock GPU memory and revoke any existing memory mappings to this block.
 * @param vca Allocation entry to unlock.
 * @param mm Memory management context of the user space process.
 * @return Return value from UnlockVcMemory. */
static uint32_t vcioa_unlock_mem(vcio_alloc* vca, struct mm_struct* mm)
{	char have_sem = 0;
	uint32_t rc;
	vcio_pr_debug("%s(%p{%x, %x[%x]}, %p)", __func__, vca, vca->Handle, vca->Location, vca->Size, mm);
	if (likely(mm))
	{	unsigned long* mp = vca->Mappings;
		unsigned long* mpe = mp + sizeof vca->Mappings / sizeof *vca->Mappings;
		do
		{	struct vm_area_struct* vma;
			vcio_pr_debug("mapping: %lx", *mp);
			if (*mp != VCIOA_LOCATION_NONE)
			{	if (!have_sem)
				{	have_sem = 1;
					down_write(&mm->mmap_sem);
				}
				vma = find_vma(mm, *mp);
				vcio_pr_debug("find_vma(%lx): %p{%lx}", *mp, vma, vma ? vma->vm_start : 0);
				if (likely(vma && vma->vm_start == *mp))
				{	zap_vma_ptes(vma, vma->vm_start, vma->vm_end - vma->vm_start);
					*mp = VCIOA_LOCATION_NONE;
				}
			}
		} while (++mp != mpe);
	}
	if (have_sem)
		up_write(&mm->mmap_sem);
	rc = UnlockVcMemory(vca->Handle & 0x7fffffff);
	if (likely(!rc))
		vca->Location = VCIOA_LOCATION_NONE;
	return rc;
}

/** dynamic array of allocation entries */
typedef struct
{	vcio_alloc* List;    /**< List of currently active allocation handles */
	unsigned    Count;   /**< Number of Entries in the list above */
	unsigned    Size;    /**< Allocated number of entries in the list above */
	struct mm_struct* MM;/**< Memory management root of the owner process, might be NULL */
} vcio_allocs;

/** Insert empty slot in vcio_allocs collection.
 * @param allocs Allocation collection.
 * @param pos Position where to insert a new entry. Must be in [0,allocs->Count].
 * @return Pointer to the just inserted entry. Must be initialized after the call.
 * Note that you must preserve the sort order by assigning an appropriate handle. */
static vcio_alloc* vcioa_insert(vcio_allocs* allocs, unsigned pos)
{
	vcio_alloc* dp;
	vcio_pr_debug("%s(%p{%p,%u,%u}, %u)", __func__, allocs, allocs->List, allocs->Count, allocs->Size, pos);

	if (allocs->Count >= allocs->Size)
	{	// reallocate
		void* ptr;
		allocs->Size = (allocs->Size << 1) + 10;
		ptr = krealloc(allocs->List, allocs->Size * sizeof *allocs->List, GFP_KERNEL);
		if (unlikely(!ptr))
			return NULL;
		allocs->List = ptr;
	}

	dp = allocs->List + pos;
	memmove(dp + 1, dp, (allocs->Count - pos) * sizeof *allocs->List);
	++allocs->Count;
	return dp;
}

/** Remove an entry from vcio_allocs collection.
 * @param allocs Allocation collection.
 * @param vca Entry to remove. */
static void vcioa_delete(vcio_allocs* allocs, vcio_alloc* vca)
{	vcio_pr_debug("%s(%p{%p,%u,%u}, %p)", __func__, allocs, allocs->List, allocs->Count, allocs->Size, vca);
	--allocs->Count;
	memmove(vca + 1, vca, (char*)(allocs->List + allocs->Count) - (char*)vca);
}

/** Locate memory handle in vcio_allocs.
 * @param allocs Allocation collection.
 * @param handle Memory handle to search for.
 * @return Location in allocs->List if found or
 * 2's complement of the location where it should be inserted if not found. */
static int vcioa_find_handle(const vcio_allocs* allocs, uint32_t handle)
{	unsigned l = 0;
	unsigned r = allocs->Count;
	vcio_pr_debug("%s(%p{%p,%u,%u}, %x)", __func__, allocs, allocs->List, allocs->Count, allocs->Size, handle);
	while (l < r)
	{	unsigned m = (l + r) >> 1;
		uint32_t h = allocs->List[m].Handle & 0x7fffffff;
		if (handle < h)
			r = m;
		else if (handle > h)
			l = m + 1;
		return m;
	}
	return ~l;
}

/** Look for an entry that contains a certain physical memory address.
 * @param allocs Allocation collection.
 * @param addr Bus address or physical address to search for. Caching bits are ignored.
 * @return Pointer to an allocation entry that contains the specified address
 * or NULL if none is found. */
static vcio_alloc* vcioa_find_addr(const vcio_allocs* allocs, uint32_t addr)
{	vcio_alloc* ap;
	vcio_alloc* ape;
	vcio_pr_debug("%s(%p{%p,%u,%u}, %x)", __func__, allocs, allocs->List, allocs->Count, allocs->Size, addr);
	addr &= VC_MEM_TO_ARM_ADDR_MASK;

	ap = allocs->List;
	ape = ap + allocs->Count;
	for (; ap != ape; ++ap)
	{	uint32_t loc = ap->Location;
		vcio_pr_debug("%s {%x,%x,%x}", __func__, ap->Handle, ap->Size, loc);
		if (loc != VCIOA_LOCATION_NONE && addr >= (loc &= VC_MEM_TO_ARM_ADDR_MASK) && addr < loc + ap->Size)
			return ap;
	}
	return NULL;
}

/** Allocate GPU memory
 * @param allocs Allocation collection.
 * @param size Number of bytes to allocate
 * @param alignment Desired alignment
 * @param flags Allocation flags
 * @return Allocation info just created or NULL in case of an error (out of memory). */
static vcio_alloc* vcioa_alloc_mem(vcio_allocs* allocs, uint32_t size, uint32_t alignment, uint32_t flags)
{	uint32_t handle;
	int pos;
	vcio_alloc* ap;
	vcio_pr_debug("%s(%p, %x, %x, %x)", __func__, allocs, size, alignment, flags);
	if (unlikely(AllocateVcMemory(&handle, size, alignment, flags)))
		return NULL;
	if (unlikely((pos = vcioa_find_handle(allocs, handle)) >= 0))
	{	vcio_pr_err("allocated handle %d twice ???", handle);
	err:
		ReleaseVcMemory(handle);
		return NULL;
	}
	if (unlikely((ap = vcioa_insert(allocs, ~pos)) == NULL))
		goto err;
	// success
	ap->Handle = handle;
	ap->Size = size;
	ap->Mappings[2] = ap->Mappings[1] = ap->Mappings[0] = ap->Location = VCIOA_LOCATION_NONE;
	vcio_pr_debug("%s: {%x, %x, }", __func__, ap->Handle, ap->Size);
	return ap;
}

/** Release VC memory and remove allocation info.
 * @param allocs
 * @param vca Allocation block to release.
 * @return Return code from ReleaseVcMemory */
static uint32_t vcioa_release_mem(vcio_allocs* allocs, vcio_alloc* vca)
{	uint32_t rc;
	if (vca->Location != VCIOA_LOCATION_NONE)
		vcioa_unlock_mem(vca, allocs->MM);
	rc = ReleaseVcMemory(vca->Handle & 0x7fffffff);
	if (likely(!rc))
		vcioa_delete(allocs, vca);
	return rc;
}

/** Query memory range
 * @param allocs
 * @param query
 * @return 0: success, EINVAL: no valid range
 */
static int vcioa_query_mem(vcio_allocs* allocs, struct vcio_mem_query* query)
{	vcio_alloc* vca;
	vcio_pr_debug("%s(%p, {%x,%x,%p,%x})", __func__, allocs, query->handle, query->bus_addr, query->virt_addr, query->size);

	if (query->handle) // have handle
	{	int pos = vcioa_find_handle(allocs, query->handle);
		if (pos < 0)
			return -EINVAL;
		vca = allocs->List + pos;

		if (query->bus_addr && vca->Location != query->bus_addr)
			return -EINVAL;
	}
	else if (query->bus_addr) // have bus address
	{	vca = vcioa_find_addr(allocs, query->bus_addr);
		if (!vca)
			return -EINVAL;
		query->handle = vca->Handle & 0x7fffffff;
	}
	else if (query->virt_addr) // have only virtual address
	{	vcio_alloc* ape = (vca = allocs->List) + allocs->Count;
		unsigned long* mp;
		do if (vca == ape)
			return -EINVAL;
		while (vca->Location == VCIOA_LOCATION_NONE || !(mp = vcioa_find_mapping_range(vca, (unsigned long)query->virt_addr, query->size)));

		query->virt_addr = (void*)*mp;
		goto done;
	}
	else
		return -EINVAL;

	if (query->virt_addr)
	{	unsigned long* mp = vcioa_find_mapping_range(vca, (unsigned long)query->virt_addr, query->size);
		if (!mp)
			return -EINVAL;
		query->virt_addr = (void*)*mp;
	} else
	{	if (query->size > vca->Size)
			return -EINVAL;
		query->virt_addr = (void*)vcioa_get_first_mapping(vca);
	}

done:
	query->bus_addr = vca->Location;
	query->size = vca->Size;
	return 0;
}

/** Discard vcio_allocs container (destructor). */
static void vcioa_destroy(vcio_allocs* allocs)
{	vcio_alloc* ap;
	vcio_alloc* ape;
	vcio_pr_debug("%s(%p{%p,%u,%u, %p})", __func__, allocs, allocs->List, allocs->Count, allocs->Size, allocs->MM);
	/* clean up memory resources */
	ap = allocs->List;
	ape = ap + allocs->Count;
	for (; ap != ape; ++ap)
	{	if (ap->Location != VCIOA_LOCATION_NONE)
			vcioa_unlock_mem(ap, allocs->MM);
		ReleaseVcMemory(ap->Handle & 0x7fffffff);
	}

	kfree(allocs->List);
	allocs->Count = allocs->Size = 0;
}

#define V3D_PCTRC (0x670>>2)               ///< Performance Counter Clear register
#define V3D_PCTRE (0x674>>2)               ///< Performance Counter Enables register
#define V3D_PCTR(n) ((0x680>>2) + (n<<1))  ///< Performance Counter Count n register
#define V3D_PCTRS(n) ((0x684>>2) + (n<<1)) ///< Performance Counter Mapping n register
#define V3D_MAX_PERF_CONUT 16

/** Private driver data for an opened device handle. */
typedef struct
{	struct mutex Lock;            /**< synchronize access to this structure. */
	vcio_allocs  Allocations;     /**< list with memory allocations of this open device instance. */
	char         Enabled;         /**< Is the QPU enabled by this device instance? */
	uint32_t     CountersEnabled; /**< bit vector of performance counters enabled by this instance */
	uint32_t     CounterValue[V3D_MAX_PERF_CONUT];/**< current values of the performance counters for this instance */
} vcio_data;

/** create vcio_data. */
static vcio_data* vcio_create(void)
{
	vcio_data* data = kzalloc(sizeof(vcio_data), GFP_KERNEL);
	mutex_init(&data->Lock);
	return data;
}

/** destroy vcio_data and release all occupied resources. */
static void vcio_destroy(vcio_data* data)
{
	mutex_destroy(&data->Lock);
	kfree(data);
}

static int vcio_set_perf_count_enable(vcio_data* data, uint32_t counters);

/** Ensure QPU enabled state.
 * @param data vcio_data structure.
 * @param value 1 => request enabled; 0: request disabled.
 * @remarks The QPU is only disabled when the /last/ open device instance disabled it. */
static int vcio_set_enabled(vcio_data* data, char value)
{	int rc = 0;
	if (data->Enabled != value)
	{	if (!value)
			vcio_set_perf_count_enable(data, 0);
		mutex_lock(&vcio_lock);
		if ( (value ? ++vcio_enabled_count == 1 : --vcio_enabled_count == 0)
			&& QpuEnable(!!value) )
		{	vcio_enabled_count += value ? -1 : 1;
			rc = -ENODEV;
		} else
			data->Enabled = value;
		vcio_pr_debug("%s: %u, %i", __func__, vcio_enabled_count, rc);
		mutex_unlock(&vcio_lock);
	}
	return rc;
}

/** Enable/disable performance counters for one instance.
 * @param data vcio_data structure.
 * @param counters Bit vector with counters to enable for this instance. 0: disable all
 * @return 0: success,
 * -EINVAL: counters contains unsupported counter
 * -EBUSY: all V3D performance counters are busy - no changes made. */
static int vcio_set_perf_count_enable(vcio_data* data, uint32_t counters)
{
	uint32_t changed = counters ^ data->CountersEnabled;
	uint32_t new = 0;
	uint32_t free;

	if (unlikely(counters & ~0x3fffe000))
		return -EINVAL;
	if (!changed)
		return 0;

	mutex_lock(&vcio_lock);

	free = vcio_v3d_base[V3D_PCTRE];
	free = (int32_t)free < 0 ? ~free & 0xffff : 0xffff;

	// check for released counters
	changed &= ~counters;
	while (changed)
	{	unsigned map;
		int id = ilog2(changed);
		uint32_t counter = 1 << id;
		changed &= ~counter;
		map = vcio_perf_count(id);
		if (VCIO_PERF_COUNT_USAGE(map) == 1)
			free |= 1 << VCIO_PERF_COUNT_INDEX(map);
	}

	// allocate new counters
	changed = counters & ~data->CountersEnabled;
	while (changed)
	{	int i;
		// get next requested counter
		int id = ilog2(changed);
		changed &= ~(1 << id);
		if (VCIO_PERF_COUNT_USAGE(vcio_perf_count(id)))
			continue; // already in use
		// find free slot
		if (unlikely(!free))
		{	mutex_unlock(&vcio_lock);
			return -EBUSY; // too many performance counters
		}
		i = ilog2(free);
		free &= ~(new |= 1 << i);
		vcio_perf_count(id) = VCIO_PERF_COUNT_MAP(i);
		data->CounterValue[i] = 0;
	}

	// execute changes
	changed = counters ^ data->CountersEnabled;
	while (changed)
	{	int id = ilog2(changed);
		uint32_t counter = 1 << id;
		changed &= ~counter;
		if (counters & counter)
			vcio_v3d_base[V3D_PCTRS(VCIO_PERF_COUNT_INDEX(++vcio_perf_count(id)))] = id;
		else
			--vcio_perf_count(id);
	}
	vcio_v3d_base[V3D_PCTRC] = new;
	free = ~free & 0xffff;
	if (free)
		free |= 0x80000000;
	vcio_v3d_base[V3D_PCTRE] = free;
	vcio_pr_debug("V3D_PCTRE %x", free);

	mutex_unlock(&vcio_lock);

	data->CountersEnabled = counters;
	return 0;
}

/** Read all performance counters enabled by this instance.
 * @param data vcio_data structure.
 * @param sign +1 or -1 to add or subtract values from accumulators. */
static void vcio_read_perf_count(vcio_data* data, int sign)
{ uint32_t counters = data->CountersEnabled;
	if (!counters)
		return;
	mutex_lock(&vcio_lock);
	while (counters)
	{	int id = ilog2(counters), i;
		counters &= ~(1 << id);
		i = VCIO_PERF_COUNT_INDEX(vcio_perf_count(id));
		data->CounterValue[i] += vcio_v3d_base[V3D_PCTR(i)] * sign;
	}
	mutex_unlock(&vcio_lock);
}

/** Fetch performance counters.
 * @param data Fetch counters from this instance.
 * @param dest Target address.
 * @return dest + number of stored counters. */
static uint32_t* vcio_fetch_perf_count(vcio_data* data, uint32_t* dest)
{	uint32_t counters = data->CountersEnabled, counter;
	int id;
	for (counter = V3D_PERF_COUNT_QPU_CYCLES_IDLE, id = ilog2(V3D_PERF_COUNT_QPU_CYCLES_IDLE);
		counter <= V3D_PERF_COUNT_L2C_L2_CACHE_MISSES; counter <<= 1, ++id)
		if (counters & counter)
			*dest++ = data->CounterValue[VCIO_PERF_COUNT_INDEX(vcio_perf_count(id))];
	return dest;
}

/** This is called whenever a process attempts to open the device file */
static int device_open(struct inode *inode, struct file *file)
{
	vcio_pr_info("opening vcio %p, %p", inode, file);

	if (MINOR(file->f_inode->i_rdev) == 1)
		file->private_data = vcio_create();

	try_module_get(THIS_MODULE);
	return 0;
}

/** This is called whenever a process attempts to close the device file */
static int device_release(struct inode *inode, struct file *file)
{
	vcio_pr_info("closing vcio %p, %p{as={%lu}}", inode, file, file->f_mapping->nrpages);

	if (file->private_data)
	{	vcio_set_perf_count_enable(file->private_data, 0);
		vcioa_destroy(&((vcio_data*)file->private_data)->Allocations);
		vcio_set_enabled(file->private_data, 0);
		vcio_destroy(file->private_data);
	}

	module_put(THIS_MODULE);
	return 0;
}

/** This function is called whenever a process tries to do an ioctl on our
 * device file. We get two extra parameters (additional to the inode and file
 * structures, which all device functions get): the number of the ioctl called
 * and the parameter given to the ioctl function.
 *
 * If the ioctl is write or read/write (meaning output is returned to the
 * calling process), the ioctl call returns the output of this function. */
static long device_ioctl(struct file *f,	/* see include/linux/fs.h */
		 unsigned int ioctl_num,	/* number and param for ioctl */
		 unsigned long ioctl_param)
{
	switch (MINOR(f->f_inode->i_rdev))
	{
#ifndef BCM_VCIO2_ADD
		case 0: // the 'old' vcio device without further checking.
			/*
			 * Switch according to the ioctl called
			 */
			switch (ioctl_num) {
			case IOCTL_MBOX_PROPERTY:
				/*
				 * Receive a pointer to a message (in user space) and set that
				 * to be the device's message.  Get the parameter given to
				 * ioctl by the process.
				 */
				mbox_copy_from_user(&size, (void *)ioctl_param, sizeof size);
				return bcm_mailbox_property((void *)ioctl_param, size);
			}
#endif
		case 1: // new vcio2 device /with/ memory checking and leak prevention.
		{	vcio_data* data = f->private_data;
			long rc = 0;
			int pos;
			mutex_lock(&data->Lock);
			switch (ioctl_num) {
				default:
					mutex_unlock(&data->Lock);
					goto fail;

				mem_fault:
					rc = -EFAULT;
					break;

				case IOCTL_GET_VCIO_VERSION:
					rc = VCIO2_VERSION_NUMBER;
					break;

				case IOCTL_MEM_ALLOCATE:
				{	struct vcio_mem_allocate p;
					vcio_alloc* ap;
					if (unlikely(copy_from_user(&p.in, (void*)ioctl_param, sizeof p.in)))
						goto mem_fault;
					if (unlikely((p.in.size & 0xe0000000) // too large
						|| (p.in.alignment & (p.in.alignment - 1)) // invalid alignment
						|| (p.in.flags & ~0xc)))
						rc = -EINVAL;
					else if (unlikely(!(ap = vcioa_alloc_mem(&data->Allocations, p.in.size, p.in.alignment, p.in.flags))))
						rc = -ENOMEM;
					else
					{	p.out.handle = ap->Handle;
						if (unlikely(copy_to_user((void*)ioctl_param, &p.out, sizeof p.out)))
							goto mem_fault;
					}
					break;
				}

				case IOCTL_MEM_RELEASE:
				{	vcio_pr_debug("IOCTL_MEM_RELEASE: %x", (unsigned)ioctl_param);
					pos = vcioa_find_handle(&data->Allocations, (uint32_t)ioctl_param);
					if (unlikely(pos < 0))
						rc = -EINVAL;
					else if (unlikely(vcioa_release_mem(&data->Allocations, data->Allocations.List + pos)))
						rc = -ENOMEM;
					break;
				}

				case IOCTL_MEM_LOCK:
				{	uint32_t param;
					if (unlikely(copy_from_user(&param, (void*)ioctl_param, sizeof param)))
						goto mem_fault;
					vcio_pr_debug("IOCTL_MEM_LOCK %x", param);
					pos = vcioa_find_handle(&data->Allocations, (uint32_t)param);
					if (unlikely(pos < 0))
						rc = -EINVAL;
					else
					{	vcio_alloc* ap = data->Allocations.List + pos;
						if (unlikely(vcioa_lock_mem(ap)))
							rc = -ENOMEM;
						else if (unlikely(copy_to_user((void*)ioctl_param, &ap->Location, sizeof param)))
							goto mem_fault;
					}
					break;
				}

				case IOCTL_MEM_UNLOCK:
				{	vcio_pr_debug("IOCTL_MEM_UNLOCK %x", (unsigned)ioctl_param);
					pos = vcioa_find_handle(&data->Allocations, ioctl_param);
					if (unlikely(pos < 0))
						rc = -EINVAL;
					else
					{	vcio_alloc* ap = data->Allocations.List + pos;
						if (unlikely(ap->Location == VCIOA_LOCATION_NONE))
						{	vcio_pr_warn("Tried to unlock memory that is not locked %x", ap->Handle);
							rc = -EPERM;
						} else if (unlikely(vcioa_unlock_mem(ap, data->Allocations.MM)))
							rc = -ENOMEM;
					}
					break;
				}

				case IOCTL_MEM_QUERY:
				{	struct vcio_mem_query q;
					if (unlikely(copy_from_user(&q, (void*)ioctl_param, sizeof q)))
						goto mem_fault;
					rc = vcioa_query_mem(&data->Allocations, &q);
					if (unlikely(!rc && copy_to_user((void*)ioctl_param, &q, sizeof q)))
						goto mem_fault;
					break;
				}

				case IOCTL_ENABLE_QPU:
				{	vcio_pr_debug("IOCTL_ENABLE_QPU %lx", ioctl_param);
					rc = vcio_set_enabled(data, !!ioctl_param);
					break;
				}

				case IOCTL_EXEC_QPU:
				{	struct vcio_exec_qpu p;
					if (unlikely(copy_from_user(&p, (void*)ioctl_param, sizeof p)))
						goto mem_fault;
					vcio_pr_debug("IOCTL_EXEC_QPU %x, %x, %x, %x", p.num_qpus, p.control, p.noflush, p.timeout);
					if (unlikely(p.num_qpus > 12 || (p.timeout & 0xff000000)))
						rc = -EINVAL;
					/* verify starting point */
					else if (unlikely(!vcioa_find_addr(&data->Allocations, p.control & VC_MEM_TO_ARM_ADDR_MASK)))
						rc = -EACCES;
					else if (likely((rc = vcio_set_enabled(data, 1)) == 0))
					{	/* TODO: verify starting points of code and uniforms too
						for (i = 0; i < p.in.num_qpus; ++i)
						{	if (!vcioa_find_addr(&data->Allocations, ))
								goto exec_fail;
						}*/
						vcio_read_perf_count(data, -1);
						if (unlikely(ExecuteQpu(p.num_qpus, p.control, !!p.noflush, p.timeout)))
							rc = -ENOEXEC;
						vcio_read_perf_count(data, 1);
					}
					break;
				}

				case IOCTL_EXEC_QPU2(1):
				case IOCTL_EXEC_QPU2(2):
				case IOCTL_EXEC_QPU2(3):
				case IOCTL_EXEC_QPU2(4):
				case IOCTL_EXEC_QPU2(5):
				case IOCTL_EXEC_QPU2(6):
				case IOCTL_EXEC_QPU2(7):
				case IOCTL_EXEC_QPU2(8):
				case IOCTL_EXEC_QPU2(9):
				case IOCTL_EXEC_QPU2(10):
				case IOCTL_EXEC_QPU2(11):
				case IOCTL_EXEC_QPU2(12):
				{	dma_addr_t bus_addr;
					struct mem
					{	u32 size;
						u32 status;
						struct
						{	struct rpi_firmware_property_tag_header hdr;
							struct
							{	u32 num_qpus;
								u32 control;
								u32 noflush;
								u32 timeout;
							} data;
						} tag;
						u32 end_tag;
						// end of message here
						struct vcio_exec_qpu_entry control_data[12];
					}* buf;

					rc = vcio_set_enabled(data, 1);
					if (unlikely(rc))
						break;

					buf = dma_alloc_coherent(vcio_dev1, PAGE_ALIGN(sizeof *buf), &bus_addr, GFP_ATOMIC);
					if (unlikely(!buf))
					{	rc = -ENOMEM;
						break;
					}
					buf->size = offsetof(struct mem, control_data);
					buf->status = RPI_FIRMWARE_STATUS_REQUEST;
					buf->tag.hdr.tag = RPI_FIRMWARE_EXECUTE_QPU;
					buf->tag.hdr.req_resp_size = buf->tag.hdr.buf_size = sizeof buf->tag.data;
					buf->tag.data.num_qpus = _IOC_SIZE(ioctl_num);
					buf->tag.data.control = bus_addr | (offsetof(struct mem, control_data) | 0x40000000);
					// vcio_exec_qpu2 is mostly binary compatible to the tail of struct mem above.
					if (unlikely(copy_from_user(&buf->tag.data.timeout, (void*)ioctl_param, buf->tag.data.num_qpus)))
					{	rc = -EFAULT;
						goto exec2_done;
					}
					buf->tag.data.num_qpus -= offsetof(struct vcio_exec_qpu2, control) >>= ilog2(sizeof(struct vcio_exec_qpu_entry));
					buf->tag.data.noflush = !!buf->end_tag; // move noflush in place
					buf->end_tag = RPI_FIRMWARE_PROPERTY_END;
					vcio_pr_debug("IOCTL_EXEC_QPU2 %x, %x, %x, %x", buf->tag.data.num_qpus, buf->tag.data.control, buf->tag.data.noflush, buf->tag.data.timeout);

					if (unlikely(buf->tag.data.timeout & 0xff000000))
					{	rc = -EINVAL;
						goto exec2_done;
					}

					vcio_read_perf_count(data, -1);
					wmb();
					rc = rpi_firmware_transaction(firmware, MBOX_CHAN_PROPERTY, bus_addr);
					rmb();
					vcio_read_perf_count(data, 1);

					if (unlikely(rc == 0 && buf->status != RPI_FIRMWARE_STATUS_SUCCESS))
						rc = -EINVAL;
				exec2_done:
					dma_free_coherent(vcio_dev1, PAGE_SIZE, buf, bus_addr);
					break;
				}

				case IOCTL_SET_V3D_PERF_COUNT:
				{	vcio_pr_debug("case IOCTL_SET_V3D_PERF_COUNT %x", (unsigned)ioctl_param);
					rc = vcio_set_perf_count_enable(data, (uint32_t)ioctl_param);
					break;
				}

				case IOCTL_GET_V3D_PERF_COUNT:
				{	if (unlikely(copy_to_user((void*)ioctl_param, &data->CountersEnabled, sizeof(uint32_t))))
						goto mem_fault;
					break;
				}

				case IOCTL_READ_V3D_PERF_COUNT:
				{	vcio_pr_debug("case IOCTL_READ_V3D_PERF_COUNT %p, %x", (void*)ioctl_param, data->CountersEnabled);
					if (unlikely(!data->CountersEnabled))
						rc = -ENODATA;
					else
					{	uint32_t counters[V3D_MAX_PERF_CONUT];
						unsigned size = (char*)vcio_fetch_perf_count(data, counters) - (char*)counters;
						if (unlikely(copy_to_user((void*)ioctl_param, counters, size)))
							goto mem_fault;
					}
					break;
				}

				case IOCTL_RESET_V3D_PERF_COUNT:
				{	memset(data->CounterValue, 0, sizeof data->CounterValue);
					break;
				}
			}
			mutex_unlock(&data->Lock);
			return rc;
		}
	}
fail:
	vcio_pr_warn("unknown ioctl: %x, minor = %d", ioctl_num, MINOR(f->f_inode->i_rdev));
	return -EINVAL;
}

/** Callback when a process unmaps GPU memory allocated by this device. */
static void vma_close(struct vm_area_struct * vma)
{
	vcio_data* data;
	uint32_t start;
	vcio_alloc* vca;
	unsigned long* mapping;
	vcio_pr_debug("%s(%p{%lx,%lx,%lx,%p})", __func__, vma, vma->vm_start, vma->vm_end, vma->vm_pgoff, vma->vm_file);
	data = vma->vm_file->private_data;
	// Accept bus address as well. Won't work on Pi 4, but this one has no VideoCore IV anyway.
	start = (vma->vm_pgoff << PAGE_SHIFT) & VC_MEM_TO_ARM_ADDR_MASK;
	// Allow RPi1 memory alias without VC4 cache.
	if (vcio_model1)
		start &= ~0x20000000;
	// undo remember mapping
	mutex_lock(&data->Lock);
	vca = vcioa_find_addr(&data->Allocations, start);
	if (unlikely(!vca))
		vcio_pr_warn("Closing unknown VMA %x", start);
	else
	{	mapping = vcioa_find_mapping(vca, vma->vm_start);
		if (unlikely(!mapping))
			vcio_pr_warn("Tried to remove unknown memory mapping %lx to address %x", vma->vm_start, start);
		else
			*mapping = VCIOA_LOCATION_NONE;
		if (vca->Handle & 0x80000000) // release automatic memory allocation
			vcioa_release_mem(&data->Allocations, vca);
	}
	mutex_unlock(&data->Lock);
}

static const struct vm_operations_struct vm_ops = {
	.close = vma_close
};

/** Implementation of mmap handler */
static int device_mmap(struct file *file, struct vm_area_struct *vma)
{
	vcio_data* const data = file->private_data;
	uint32_t size = (uint32_t)(vma->vm_end - vma->vm_start);
	int rc;
	uint32_t start;
	vcio_alloc* vca;
	unsigned long* mapping;
	vcio_pr_debug("%s(%p, %p{%lx,%lx,%lx,%p})", __func__, file, vma, vma->vm_start, vma->vm_end, vma->vm_pgoff, vma->vm_file);

	data->Allocations.MM = vma->vm_mm; // remember for later use at cleanup

	/* we only support shared mappings. Copy on write mappings are
	 rejected here. A shared mapping that is writeable must have the
	 shared flag set. */
	if (unlikely((vma->vm_flags & VM_WRITE) && !(vma->vm_flags & VM_SHARED)))
	{	vcio_pr_info("Writable mappings must be shared, rejecting");
		return -EINVAL;
	}

	if (vma->vm_pgoff)
	{	// Accept bus address as well. Won't work on Pi 4, but this one has no VideoCore IV anyway.
		vma->vm_pgoff &= VC_MEM_TO_ARM_ADDR_MASK >> PAGE_SHIFT;
		start = vma->vm_pgoff << PAGE_SHIFT;
		// Allow RPi1 memory alias without VC4 cache.
		if (vcio_model1)
			start &= ~0x20000000;

		/* Check whether the mapped memory belongs to a buffer allocated by the same file handle. */
		mutex_lock(&data->Lock);

		vca = vcioa_find_addr(&data->Allocations, start);
		if (unlikely(vca == NULL))
		{	mutex_unlock(&data->Lock);
			vcio_pr_info("Tried to map memory (%x) that is not allocated by this device.", start);
			return -EACCES;
		}
		if (unlikely(((start + size - 1) & PAGE_MASK) > ((vca->Location + vca->Size - 1) & (VC_MEM_TO_ARM_ADDR_MASK & PAGE_MASK))))
		{	vcio_pr_info("The memory region to map exceeds the allocated buffer (%x[%x] vs. %x[%x]).", start, size, vca->Location, vca->Size);
			mutex_unlock(&data->Lock);
			return -EACCES;
		}

		// remember mapping
		mapping = vcioa_find_mapping(vca, VCIOA_LOCATION_NONE);
		if (unlikely(mapping == NULL))
		{	mutex_unlock(&data->Lock);
			vcio_pr_info("Only %d mappings per allocated block (%x) supported.", sizeof vca->Mappings / sizeof *vca->Mappings, start);
			return -ENOMEM;
		}
		*mapping = vma->vm_start;
	}
	else
	{	// automatic memory allocation
		vca = vcioa_alloc_mem(&data->Allocations, size, PAGE_SIZE, 4);
		if (unlikely(!vca))
			return -ENOMEM;
		vca->Handle |= 0x80000000; // Automatic allocation tag
		if (unlikely(vcioa_lock_mem(vca)))
		{	vcioa_release_mem(&data->Allocations, vca);
			return -ENOMEM;
		}
		vca->Mappings[2] = vma->vm_start;
		start = vca->Location & VC_MEM_TO_ARM_ADDR_MASK;
		vma->vm_pgoff = start >> PAGE_SHIFT;
		if (vcio_model1)
			vma->vm_pgoff |= 0x20000000 >> PAGE_SHIFT; // uncached on PI 1 as well
	}

	mutex_unlock(&data->Lock);

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_ops = &vm_ops;
	/* Don't fork this mappings as this would create serious race conditions. */
	vma->vm_flags |= VM_DONTCOPY|VM_PFNMAP;

	rc = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size, vma->vm_page_prot);
	if (unlikely(rc))
	{	vcio_pr_warn("remap page range failed with %d", rc);
		// undo remember mapping
		mutex_lock(&data->Lock);
		vca = vcioa_find_addr(&data->Allocations, start);
		mapping = vcioa_find_mapping(vca, vma->vm_start);
		if (likely(mapping))
		{	*mapping = VCIOA_LOCATION_NONE;
			if (vca->Handle & 0x80000000) // undo automatic allocation?
				vcioa_release_mem(&data->Allocations, vca);
		}
		mutex_unlock(&data->Lock);
		return rc;
	} else if (vca->Handle & 0x80000000)
	{	// place start address into allocation
		uint32_t* mp = memremap(start, sizeof(uint32_t), MEMREMAP_WT);
		vcio_pr_debug("memremap %p", mp);
		*mp = vca->Location;
		memunmap(mp);
	}

	return 0;
}

/* Module Declarations */

/** This structure will hold the functions to be called
 * when a process does something to the device we
 * created. Since a pointer to this structure is kept in
 * the devices table, it can't be local to
 * init_module. NULL is for unimplemented functions. */
struct file_operations fops = {
	.unlocked_ioctl = device_ioctl,
	.open = device_open,
	.release = device_release,	/* a.k.a. close */
	.mmap = device_mmap,
};

static int vcio_remove(struct platform_device *pdev)
{
#ifndef BCM_VCIO2_ADD
	struct vc_mailbox *mailbox;
#endif

	mutex_destroy(&vcio_lock);

	if (likely(vcio_v3d_base))
	{	iounmap(vcio_v3d_base);
		vcio_v3d_base = NULL;
	}

	if (likely(vcio_dev0 != NULL))
	{	device_destroy(vcio_class, vcio_dev0->devt);
		vcio_dev0 = NULL;
	}
	if (likely(vcio_dev1 != NULL))
	{	device_destroy(vcio_class, vcio_dev1->devt);
		vcio_dev1 = NULL;
	}

	if (likely(vcio_class != NULL))
	{	class_destroy(vcio_class);
		vcio_class = NULL;
	}

	if (likely(vcio_cdev0.owner != NULL))
	{	cdev_del(&vcio_cdev0);
		memset(&vcio_cdev0, 0, sizeof vcio_cdev0);
	}
	if (likely(vcio_cdev1.owner != NULL))
	{	cdev_del(&vcio_cdev1);
		memset(&vcio_cdev1, 0, sizeof vcio_cdev1);
	}

	if (likely(vcio_dev))
	{	unregister_chrdev_region(vcio_dev, 2);
		vcio_dev = 0;
	}

#ifndef BCM_VCIO2_ADD
	mailbox = platform_get_drvdata(pdev);
	if (mailbox != NULL)
	{	free_irq(IRQ_ARM_MAILBOX, mailbox);

		platform_set_drvdata(pdev, NULL);
		kfree(mailbox);
	}
#endif

	return 0;
}

static int vcio_probe(struct platform_device *pdev)
{
	int ret = 0;
	vcio_pdev = &pdev->dev;

#ifndef BCM_VCIO2_ADD
	struct vc_mailbox *mailbox;
	struct resource *res;

	mailbox = kzalloc(sizeof(*mailbox), GFP_KERNEL);
	if (NULL == mailbox) {
		pr_err(DRIVER_NAME ": failed to allocate mailbox memory\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		pr_err(DRIVER_NAME ": failed to obtain memory resource\n");
		kfree(mailbox);
		return -ENODEV;
	}
	/* should be based on the registers from res really */
	mbox_init(mailbox, &pdev->dev, ARM_0_MAIL0_RD);

	platform_set_drvdata(pdev, mailbox);
	dev_mbox_register(DRIVER_NAME, &pdev->dev);

	mbox_irqaction.dev_id = mailbox;
	setup_irq(IRQ_ARM_MAILBOX, &mbox_irqaction);
	printk(KERN_INFO DRIVER_NAME ": mailbox at %p\n",
				 __io_address(ARM_0_MAIL0_RD));
#endif

	{	struct device_node *np;
		np = of_find_compatible_node(NULL, NULL, "raspberrypi,bcm2835-firmware");
		if (!of_device_is_available(np))
			return -ENODEV;

		firmware = rpi_firmware_get(np);
		if (!firmware)
		{	vcio_pr_err("rpi_firmware_get failed");
			return -ENODEV;
		}
	}

	{	// check if model 1
		uint32_t revision = GetBoardRevision();
		vcio_pr_debug("Board revision %x", revision);
		vcio_model1 = !(revision & 0x800000) || !(revision & 0xf000);
		//vcio_address_mask
	}

	{	// map I/O registers
		vcio_v3d_base = ioremap(vcio_model1 ? 0x20c00000 : 0x3fc00000, 0x1000);
		vcio_pr_debug("IO: %p", vcio_v3d_base);
		if (!vcio_v3d_base)
		{	vcio_pr_err("Failed to map V3D register");
			return -EBUSY;
		}
	}

	/* Register the character device */
	ret = alloc_chrdev_region(&vcio_dev, 0, 2, DEVICE_FILE_NAME);
	if (ret < 0) {
		vcio_pr_err("Failed registering the character device %d", ret);
		vcio_dev = 0;
		goto fail;
	}

	cdev_init(&vcio_cdev0, &fops);
	vcio_cdev0.owner = THIS_MODULE;
	ret = cdev_add(&vcio_cdev0, vcio_dev, 1);
	if (ret < 0) {
		vcio_pr_err("%s: Unable to add device (rc=%d)", __func__, ret);
		memset(&vcio_cdev0, 0, sizeof vcio_cdev0);
		goto fail;
	}
	cdev_init(&vcio_cdev1, &fops);
	vcio_cdev1.owner = THIS_MODULE;
	ret = cdev_add(&vcio_cdev1, vcio_dev+1, 1);
	if (ret < 0) {
		vcio_pr_err("%s: Unable to add device (rc=%d)", __func__, ret);
		memset(&vcio_cdev1, 0, sizeof vcio_cdev1);
		goto fail;
	}

	/* Create vcio device */
	vcio_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(vcio_class)) {
		ret = PTR_ERR(vcio_class);
		vcio_pr_err("%s: class_create failed (rc=%d)\n", __func__, ret);
		vcio_class = NULL;
		goto fail;
	}

	vcio_dev0 = device_create(vcio_class, NULL, vcio_dev, NULL, "vcio");
	if (IS_ERR(vcio_dev0)) {
		ret = PTR_ERR(vcio_dev0);
		vcio_pr_err("%s: device_create failed (rc=%d)\n", __func__, ret);
		vcio_dev0 = NULL;
		goto fail;
	}
	vcio_dev1 = device_create(vcio_class, NULL, vcio_dev+1, NULL, "vcio2");
	if (IS_ERR(vcio_dev1)) {
		ret = PTR_ERR(vcio_dev1);
		vcio_pr_err("%s: device_create failed (rc=%d)\n", __func__, ret);
		vcio_dev1 = NULL;
		goto fail;
	}

	dma_set_coherent_mask(vcio_dev1, DMA_BIT_MASK(32));

	mutex_init(&vcio_lock);
	/* succeeded! */
	return 0;

 fail:
	vcio_remove(pdev);

	return ret;
}

#ifndef BCM_VCIO2_ADD
static struct platform_driver vcio_driver = {
	.probe = vcio_probe,
	.remove = vcio_remove,

	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};
#endif

static int __init vcio_init(void)
{
	int ret = 0;

	vcio_pr_info("Enhanced Broadcom VideoCore Mailbox driver");

#ifndef BCM_VCIO2_ADD
	ret = platform_driver_register(&vcio_driver);
	if (ret != 0) {
		printk(KERN_ERR DRIVER_NAME ": failed to register on platform\n");
	}
#else
	vcio_probe(NULL);
#endif

	return ret;
}

static void __exit vcio_exit(void)
{
#ifndef BCM_VCIO2_ADD
	platform_driver_unregister(&vcio_driver);
#else
	vcio_remove(NULL);
#endif
}

#if BCM_VCIO2_ADD
module_init(vcio_init);
//MODULE_SOFTDEP("pre: vcio");
#else
arch_initcall(vcio_init);	/* Initialize early */
#endif
module_exit(vcio_exit);

MODULE_AUTHOR("Marcel Müller");
MODULE_DESCRIPTION("ARM access to VideoCore processor");
MODULE_LICENSE("GPL");
//MODULE_ALIAS("platform:bcm-mbox");
