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
//#define DEBUG

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
#include <soc/bcm2835/raspberrypi-firmware.h>
#include <soc/bcm2835/vcio2.h>


#ifdef BCM_VCIO2_ADD
#define DRIVER_NAME "vcio2"
#else
#define DRIVER_NAME "bcm2708_vcio"
#endif

#define vcio_pr_err(fmt, args...) pr_err("%s: " fmt "\n", DRIVER_NAME, ## args)
#define vcio_pr_warn(fmt, args...) pr_warn("%s: " fmt "\n", DRIVER_NAME, ## args)
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

/// synchronize access to the following working set.
struct mutex vcio_lock;
/// Is the QPU enabled now? Number of requesting clients.
static unsigned vcio_enabled_count = 0;


#define mailbox_property(tag) rpi_firmware_property_list(firmware, &tag, sizeof (tag));


static uint32_t AllocateVcMemory(uint32_t *pHandle, uint32_t size, uint32_t alignment, uint32_t flags)
{
	struct vc_tag
	{
		uint32_t m_tagId;
		uint32_t m_sendBufferSize;
		union {
			uint32_t m_sendDataSize;
			uint32_t m_recvDataSize;
		};

		struct args
		{
			union {
				uint32_t m_size;
				uint32_t m_handle;
			};
			uint32_t m_alignment;
			uint32_t m_flags;
		} m_args;
	} tag;
	int s;

	//fill in the tag for the allocation command
	tag.m_tagId = RPI_FIRMWARE_ALLOCATE_MEMORY;
	tag.m_sendDataSize = tag.m_sendBufferSize = 12;

	//fill in our args
	tag.m_args.m_size = size;
	tag.m_args.m_alignment = alignment;
	tag.m_args.m_flags = flags;

	//run the command
	s = mailbox_property(tag);

	if (s == 0 && tag.m_recvDataSize == 0x80000004)
	{
		*pHandle = tag.m_args.m_handle;
		return 0;
	}
	else
	{
		vcio_pr_warn("Failed to allocate VC memory: s=%d recv data size=%08x", s, tag.m_recvDataSize);
		return s ? s : 1;
	}
}

static uint32_t ReleaseVcMemory(uint32_t handle)
{
	struct vc_tag
	{
		uint32_t m_tagId;
		uint32_t m_sendBufferSize;
		union {
			uint32_t m_sendDataSize;
			uint32_t m_recvDataSize;
		};

		struct args
		{
			union {
				uint32_t m_handle;
				uint32_t m_error;
			};
		} m_args;
	} tag;
	int s;

	//fill in the tag for the release command
	tag.m_tagId = RPI_FIRMWARE_RELEASE_MEMORY;
	tag.m_sendDataSize = tag.m_sendBufferSize = 4;

	//pass across the handle
	tag.m_args.m_handle = handle;

	s = mailbox_property(tag);

	if (s == 0 && tag.m_recvDataSize == 0x80000004 && tag.m_args.m_error == 0)
		return 0;
	else
	{
		vcio_pr_warn("Failed to release VC memory: rc=%i recv data size=%08x error=%08x", s, tag.m_recvDataSize, tag.m_args.m_error);
		return s ? s : 1;
	}
}

static uint32_t LockVcMemory(uint32_t *pBusAddress, uint32_t handle)
{
	struct vc_tag
	{
		uint32_t m_tagId;
		uint32_t m_sendBufferSize;
		union {
			uint32_t m_sendDataSize;
			uint32_t m_recvDataSize;
		};

		struct args
		{
			union {
				uint32_t m_handle;
				uint32_t m_busAddress;
			};
		} m_args;
	} tag;
	int s;

	//fill in the tag for the lock command
	tag.m_tagId = RPI_FIRMWARE_LOCK_MEMORY;
	tag.m_sendDataSize = tag.m_sendBufferSize = 4;

	//pass across the handle
	tag.m_args.m_handle = handle;

	s = mailbox_property(tag);

	if (s == 0 && tag.m_recvDataSize == 0x80000004)
	{
		//pick out the bus address
		*pBusAddress = tag.m_args.m_busAddress;
		return 0;
	}
	else
	{
		vcio_pr_warn("Failed to lock VC memory: s=%d recv data size=%08x", s, tag.m_recvDataSize);
		return s ? s : 1;
	}
}

static uint32_t UnlockVcMemory(uint32_t handle)
{
	struct vc_tag
	{
		uint32_t m_tagId;
		uint32_t m_sendBufferSize;
		union {
			uint32_t m_sendDataSize;
			uint32_t m_recvDataSize;
		};

		struct args
		{
			union {
				uint32_t m_handle;
				uint32_t m_error;
			};
		} m_args;
	} tag;
	int s;

	//fill in the tag for the unlock command
	tag.m_tagId = RPI_FIRMWARE_UNLOCK_MEMORY;
	tag.m_sendDataSize = tag.m_sendBufferSize = 4;

	//pass across the handle
	tag.m_args.m_handle = handle;

	s = mailbox_property(tag);

	//check the error code too
	if (s == 0 && tag.m_recvDataSize == 0x80000004 && tag.m_args.m_error == 0)
		return 0;
	else
	{
		vcio_pr_warn("Failed to unlock VC memory: s=%d recv data size=%08x error%08x", s, tag.m_recvDataSize, tag.m_args.m_error);
		return s ? s : 1;
	}
}

static uint32_t QpuEnable(unsigned enable)
{
	struct vc_tag
	{
		uint32_t m_tagId;
		uint32_t m_sendBufferSize;
		union {
			uint32_t m_sendDataSize;
			uint32_t m_recvDataSize;
		};

		struct args
		{
			union {
				uint32_t m_enable;
				uint32_t m_return;
			};
		} m_args;
	} tag;
	int s;

	vcio_pr_debug("%s %d", __func__, enable);

	/* property message to VCIO channel */
	tag.m_tagId = RPI_FIRMWARE_SET_ENABLE_QPU;
	tag.m_sendDataSize = tag.m_sendBufferSize = 4;

	s = mailbox_property(tag);

	//check the error code too
	if (s == 0 && tag.m_recvDataSize == 0x80000004)
		return tag.m_args.m_return;
	else
	{
		vcio_pr_warn("Failed to execute QPU: s=%d recv data size=%08x", s, tag.m_recvDataSize);
		return s;
	}
}

static uint32_t ExecuteQpu(uint32_t num_qpus, uint32_t control, uint32_t noflush, uint32_t timeout)
{
	struct vc_tag
	{
		uint32_t m_tagId;
		uint32_t m_sendBufferSize;
		union {
			uint32_t m_sendDataSize;
			uint32_t m_recvDataSize;
		};

		struct args
		{
			union {
				uint32_t m_numQpus;
				uint32_t m_return;
			};
			uint32_t m_control;
			uint32_t m_noflush;
			uint32_t m_timeout;
		} m_args;
	} tag;
	int s;

	vcio_pr_debug("%s(%d, %x, %d, %d)", __func__, num_qpus, control, noflush, timeout);

	/* property message to VCIO channel */
	tag.m_tagId = RPI_FIRMWARE_EXECUTE_QPU;
	tag.m_sendDataSize = tag.m_sendBufferSize = 16;

	//pass across the handle
	tag.m_args.m_numQpus = num_qpus;
	tag.m_args.m_control = control;
	tag.m_args.m_noflush = noflush;
	tag.m_args.m_timeout = timeout;

	s = mailbox_property(tag);

	vcio_pr_debug("mbox: %i", s);

	//check the error code too
	if (s == 0 && tag.m_recvDataSize == 0x80000004)
		return tag.m_args.m_return;
	else
	{
		vcio_pr_warn("Failed to execute QPU: s=%d recv data size=%08x", s, tag.m_recvDataSize);
		return s;
	}
}

/** allocation entry */
typedef struct
{
	unsigned Handle;  /**< List of currently active allocation handles */
	unsigned Size;    /**< Size of the Segment in bytes */
	unsigned Location;/**< Memory segment is locked to this physical address */
} vcio_alloc;

#define VCIOA_LOCATION_NONE 0xffffffff

/** dynamic array of allocation entries */
typedef struct
{
	vcio_alloc* List; /**< List of currently active allocation handles */
	unsigned    Count;/**< Number of Entries in the list above */
	unsigned    Size; /**< Allocated number of entries in the list above */
} vcio_allocs;

/** Locate memory handle in vcio_allocs.
 * @param allocs Allocation collection.
 * @param handle Memory handle to search for.
 * @return Location in allocs->List if found or
 * 2's complement of the location where it should be inserted if not found. */
static int vcioa_locate(const vcio_allocs* allocs, unsigned handle)
{
	unsigned l = 0;
	unsigned r = allocs->Count;
	vcio_pr_debug("%s(%p{%p,%u,%u}, %x)", __func__, allocs, allocs->List, allocs->Count, allocs->Size, handle);
	while (l < r)
	{	unsigned m = (l + r) >> 1;
		unsigned h = allocs->List[m].Handle;
		if (handle < h)
			r = m;
		else if (handle > h)
			l = m + 1;
		return m;
	}
	return ~l;
}

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
		if (!ptr)
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
 * @param pos Index of the entry to remove. Must be in [0,allocs->Count].
 */
static void vcioa_delete(vcio_allocs* allocs, unsigned pos)
{
	vcio_alloc* dp;
	vcio_pr_debug("%s(%p{%p,%u,%u}, %u)", __func__, allocs, allocs->List, allocs->Count, allocs->Size, pos);
	dp = allocs->List + pos;
	--allocs->Count;
	memmove(dp + 1, dp, (allocs->Count - pos) * sizeof *allocs->List);
}

/** Look for an entry that contains a certain physical memory address.
 * @param allocs Allocation collection.
 * @param addr Address to search for.
 * @return Pointer to an allocation entry that contains the specified address
 * or NULL if none is found.
 */
static vcio_alloc* vcioa_find_addr(const vcio_allocs* allocs, unsigned addr)
{
	vcio_alloc* ap;
	vcio_alloc* ape;
	vcio_pr_debug("%s(%p{%p,%u,%u}, %x)", __func__, allocs, allocs->List, allocs->Count, allocs->Size, addr);
	ap = allocs->List;
	ape = ap + allocs->Count;
	for (; ap != ape; ++ap)
	{	vcio_pr_debug("%s {%u,%x,%x}", __func__, ap->Handle, ap->Size, ap->Location);
		if (ap->Location != VCIOA_LOCATION_NONE && addr >= ap->Location && addr < ap->Location + ap->Size)
			return ap;
	}
	return NULL;
}

/** Discard all remaining allocations in a vcio_allocs container.
 * @param allocs Allocation collection.
 */
static void vcioa_destroy(vcio_allocs* allocs)
{
	vcio_alloc* ap;
	vcio_alloc* ape;
	vcio_pr_debug("%s(%p{%p,%u,%u})", __func__, allocs, allocs->List, allocs->Count, allocs->Size);
	/* clean up memory resources */
	ap = allocs->List;
	ape = ap + allocs->Count;
	for (; ap != ape; ++ap)
	{	vcio_pr_info("cleanup GPU memory: %x @ %x[%x]", ap->Handle, ap->Location, ap->Size);
		if (ap->Location != VCIOA_LOCATION_NONE)
			UnlockVcMemory(ap->Handle);
		ReleaseVcMemory(ap->Handle);
	}

	kfree(allocs->List);
	allocs->Count = allocs->Size = 0;
}

/** Private driver data for an opened device handle. */
typedef struct
{
	struct mutex Lock;        /**< synchronize access to this structure.*/
	vcio_allocs  Allocations; /**< list with memory allocations of this open device instance. */
	char         Enabled;     /**< Is the QPU enabled by this device instance? */

} vcio_data;

/** create vcio_data. */
static vcio_data* vcio_create(void)
{
	vcio_data* data = kzalloc(sizeof(vcio_data), GFP_KERNEL);
	mutex_init(&data->Lock);
	return data;
}

/** destroy vcio_data and release all occupied resources.
 * @param data vcio_data structure.
 */
static void vcio_destroy(vcio_data* data)
{
	vcioa_destroy(&data->Allocations);
	mutex_destroy(&data->Lock);
	kfree(data);
}

/** Ensure QPU enabled state.
 * @param data vcio_data structure.
 * @param value 1 => request enabled; 0: request disabled.
 * @remarks The QPU is only disabled when the /last/ open device instance disabled it. */
static int vcio_set_enabled(vcio_data* data, char value)
{	int rc = 0;
	if (data->Enabled != value)
	{	mutex_lock(&vcio_lock);
		if ( (value ? ++vcio_enabled_count == 1 : --vcio_enabled_count == 0)
			&& QpuEnable(1) )
		{	vcio_enabled_count += value ? -1 : 1;
			rc = -ENODEV;
		} else
			data->Enabled = value;
		vcio_pr_debug("%s: %u, %i", __func__, vcio_enabled_count, rc);
		mutex_unlock(&vcio_lock);
	}
	return rc;
}


/** This is called whenever a process attempts to open the device file
 */
static int device_open(struct inode *inode, struct file *file)
{
	vcio_pr_info("opening vcio %p, %p", inode, file);

	if (MINOR(file->f_inode->i_rdev) == 1)
		file->private_data = vcio_create();

	try_module_get(THIS_MODULE);
	return 0;
}

static int device_release(struct inode *inode, struct file *file)
{
	vcio_pr_info("closing vcio %p, %p", inode, file);

	if (file->private_data)
	{	vcio_set_enabled(file->private_data, 0);
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
 * calling process), the ioctl call returns the output of this function.
 */
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
				return 0;
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

				case IOCTL_MEM_ALLOCATE:
				{
					vcio_mem_allocate p;
					vcio_alloc* ap;
					unsigned size;
					copy_from_user(&p.in, (void*)ioctl_param, sizeof p.in);
					size = p.in.size;
					vcio_pr_debug("IOCTL_MEM_ALLOCATE %x, %x, %x", size, p.in.alignment, p.in.flags);
					if (AllocateVcMemory(&p.out.handle, size, p.in.alignment, p.in.flags))
					{	p.out.handle = 0;
						rc = -ENOMEM;
					} else if ((pos = vcioa_locate(&data->Allocations, p.out.handle)) >= 0)
					{	vcio_pr_err("allocated handle %d twice ???", p.out.handle);
						ReleaseVcMemory(p.out.handle);
						p.out.handle = 0;
						rc = -EINVAL;
					} else if ((ap = vcioa_insert(&data->Allocations, ~pos)) == NULL)
					{	ReleaseVcMemory(p.out.handle);
						p.out.handle = 0;
						rc = -ENOMEM;
					} else
					{	/* success */
						ap->Handle = p.out.handle;
						ap->Size = size;
						ap->Location = VCIOA_LOCATION_NONE;
						vcio_pr_debug("IOCTL_MEM_ALLOCATE: {%d, %x, }", ap->Handle, ap->Size);
					}
					copy_to_user((void*)ioctl_param, &p.out, sizeof p.out);
					break;
				}

				case IOCTL_MEM_RELEASE:
				{
					vcio_pr_debug("IOCTL_MEM_RELEASE: %x", (unsigned)ioctl_param);
					pos = vcioa_locate(&data->Allocations, ioctl_param);
					if (pos < 0)
						rc = -EBADF;
					else
					{	vcio_alloc* ap = data->Allocations.List + pos;
						if (ap->Location != VCIOA_LOCATION_NONE)
							UnlockVcMemory(ap->Handle);
						if (ReleaseVcMemory(ioctl_param))
							rc = -ENOMEM;
						else
							vcioa_delete(&data->Allocations, pos);
					}
					break;
				}

				case IOCTL_MEM_LOCK:
				{
					unsigned param;
					copy_from_user(&param, (void*)ioctl_param, sizeof param);
					vcio_pr_debug("IOCTL_MEM_LOCK %x", param);
					pos = vcioa_locate(&data->Allocations, param);
					if (pos < 0)
						rc = -EBADF;
					else
					{	vcio_alloc* ap = data->Allocations.List + pos;
						if (ap->Location != VCIOA_LOCATION_NONE)
						{	pr_info(DRIVER_NAME ": tried to lock memory %d twice (at %x)\n", ap->Handle, ap->Location);
							param = ap->Location;
						} else if (LockVcMemory(&param, ap->Handle))
						{	param = 0;
							rc = ENOMEM;
						} else
							ap->Location = param;
					}
					vcio_pr_debug("IOCTL_MEM_LOCK: %x", param);
					copy_to_user((void*)ioctl_param, &param, sizeof param);
					break;
				}

				case IOCTL_MEM_UNLOCK:
				{
					vcio_pr_debug("IOCTL_MEM_UNLOCK %x", (unsigned)ioctl_param);
					pos = vcioa_locate(&data->Allocations, ioctl_param);
					if (pos < 0)
						rc = -EBADF;
					else
					{	vcio_alloc* ap = data->Allocations.List + pos;
						if (ap->Location == VCIOA_LOCATION_NONE)
						{	vcio_pr_warn("tried to unlock unlocked memory %d", ap->Handle);
							rc = -EPERM;
						} else if (UnlockVcMemory(ap->Handle))
							rc = -ENOMEM;
						else
							ap->Location = VCIOA_LOCATION_NONE;
					}
					break;
				}

				case IOCTL_ENABLE_QPU:
				{	vcio_pr_debug("IOCTL_ENABLE_QPU %lx", ioctl_param);
					rc = vcio_set_enabled(data, !!ioctl_param);
					break;
				}

				case IOCTL_EXEC_QPU:
				{	vcio_exec_qpu p;
					copy_from_user(&p.in, (void*)ioctl_param, sizeof p.in);
					vcio_pr_debug("IOCTL_EXEC_QPU %x, %x, %x, %x", p.in.num_qpus, p.in.control, p.in.noflush, p.in.timeout);
					/* verify starting point */
					if (!vcioa_find_addr(&data->Allocations, p.in.control))
						rc = -EACCES;
					else if ((rc = vcio_set_enabled(data, 1)) == 0)
					{	/* TODO: verify starting points of code and uniforms too
						for (i = 0; i < p.in.num_qpus; ++i)
						{	if (!vcioa_find_addr(&data->Allocations, ))
								goto exec_fail;
						}*/
						if (ExecuteQpu(p.in.num_qpus, p.in.control, p.in.noflush, p.in.timeout))
							rc = -ENOEXEC;
					}
					break;
				}
			}
			mutex_unlock(&data->Lock);
			return rc;
		}
	}
fail:
	vcio_pr_warn("unknown ioctl: %d, minor = %d", ioctl_num, MINOR(f->f_inode->i_rdev));
	return -EINVAL;
}

static const struct vm_operations_struct vm_ops = {
/*    .open =  simple_vma_open,
    .close = simple_vma_close,*/
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys
#endif
};

static int device_mmap(struct file *file, struct vm_area_struct *vma)
{
	vcio_data* data = file->private_data;
	unsigned size = vma->vm_end - vma->vm_start;
	int rc;

	/* we only support shared mappings. Copy on write mappings are
	 rejected here. A shared mapping that is writeable must have the
	 shared flag set.
	 */
	if ((vma->vm_flags & VM_WRITE) && !(vma->vm_flags & VM_SHARED))
	{
		vcio_pr_info("writeable mappings must be shared, rejecting");
		return -EINVAL;
	}

	if (data)
	{
		vcio_alloc* vca;
		unsigned start = vma->vm_pgoff << PAGE_SHIFT;

		/* Check whether the mapped memory belongs to a buffer allocated by the same file handle. */
		mutex_lock(&data->Lock);

		vca = vcioa_find_addr(&data->Allocations, start);
		if (vca == NULL)
		{
			vcio_pr_info("tried to map memory (%x) that is not allocated by this device.", start);
			return -EACCES;
		}
		if (((start + size -1) >> PAGE_SHIFT) > ((vca->Location + vca->Size -1) >> PAGE_SHIFT))
		{
			vcio_pr_info("the memory region to map exceeds the allocated buffer (%x[%x] vs. %x[%x]).",
				start, size, vca->Location, vca->Size);
			return -EACCES;
		}

		mutex_unlock(&data->Lock);
	}

	/*pr_debug(DRIVER_NAME ": device_mmap %lx, %lx, %lx, %x, %lx\n",
	 vma->vm_start, vma->vm_end, vma->vm_pgoff, vma->vm_page_prot, vma->vm_flags);*/

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_ops = &vm_ops;
	/* Don't fork this mappings as this would create serious race conditions. */
	vma->vm_flags |= VM_DONTCOPY;

	rc = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size, vma->vm_page_prot);
	if (rc)
	{
		vcio_pr_warn("remap page range failed with %d", rc);
		return rc;
	}

	return 0;
}

/* Module Declarations */

/**
 * This structure will hold the functions to be called
 * when a process does something to the device we
 * created. Since a pointer to this structure is kept in
 * the devices table, it can't be local to
 * init_module. NULL is for unimplemented functions.
 */
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

	if (vcio_dev0 != NULL)
	{	device_destroy(vcio_class, vcio_dev0->devt);
		vcio_dev0 = NULL;
	}
	if (vcio_dev1 != NULL)
	{	device_destroy(vcio_class, vcio_dev1->devt);
		vcio_dev1 = NULL;
	}

	if (vcio_class != NULL)
	{	class_destroy(vcio_class);
		vcio_class = NULL;
	}

	if (vcio_cdev0.owner != NULL)
	{	cdev_del(&vcio_cdev0);
		memset(&vcio_cdev0, 0, sizeof vcio_cdev0);
	}
	if (vcio_cdev1.owner != NULL)
	{	cdev_del(&vcio_cdev1);
		memset(&vcio_cdev1, 0, sizeof vcio_cdev1);
	}

	if (vcio_dev)
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

	struct device_node *np;
	np = of_find_compatible_node(NULL, NULL, "raspberrypi,bcm2835-firmware");
	if (!of_device_is_available(np))
		return -ENODEV;

	firmware = rpi_firmware_get(np);
	if (!firmware)
		return -ENODEV;

	//pr_info(DRIVER_NAME ": 1\n");

	/* Register the character device */
	ret = alloc_chrdev_region(&vcio_dev, 0, 2, DEVICE_FILE_NAME);
	if (ret < 0) {
		vcio_pr_err("Failed registering the character device %d", ret);
		vcio_dev = 0;
		goto fail;
	}

	//pr_info(DRIVER_NAME ": 2\n");

	cdev_init(&vcio_cdev0, &fops);
	vcio_cdev0.owner = THIS_MODULE;
	ret = cdev_add(&vcio_cdev0, vcio_dev, 1);
	if (ret < 0) {
		vcio_pr_err("%s: Unable to add device (rc=%d)", __func__, ret);
		memset(&vcio_cdev0, 0, sizeof vcio_cdev0);
		goto fail;
	}
	//pr_info(DRIVER_NAME ": 3 %d\n", ret);
	cdev_init(&vcio_cdev1, &fops);
	vcio_cdev1.owner = THIS_MODULE;
	ret = cdev_add(&vcio_cdev1, vcio_dev+1, 1);
	if (ret < 0) {
		vcio_pr_err("%s: Unable to add device (rc=%d)", __func__, ret);
		memset(&vcio_cdev1, 0, sizeof vcio_cdev1);
		goto fail;
	}
	//pr_info(DRIVER_NAME ": 3 %d\n", ret);

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

	mutex_init(&vcio_lock);
	/* succeeded! */
	return 0;

 fail:
	vcio_remove(pdev);

	return ret;
}

static struct platform_driver vcio_driver = {
	.probe = vcio_probe,
	.remove = vcio_remove,

	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

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
#else
arch_initcall(vcio_init);	/* Initialize early */
#endif
module_exit(vcio_exit);

MODULE_AUTHOR("Marcel Müller");
MODULE_DESCRIPTION("ARM access to VideoCore processor");
MODULE_LICENSE("GPL");
//MODULE_ALIAS("platform:bcm-mbox");
