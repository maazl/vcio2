/*
 *  linux/arch/arm/mach-bcm2708/vcio.c
 *
 *  Copyright (C) 2010 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This device provides a shared mechanism for writing to the mailboxes,
 * semaphores, doorbells etc. that are shared between the ARM and the
 * VideoCore processor
 */

// 1 := vcio2 driver is additional to the kernels vcio driver.
// 0 := vcio2 driver replaces the kernels vcio driver.
#define BCM_VCIO2_ADD 1


#if defined(CONFIG_SERIAL_BCM_MBOX_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/console.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <linux/io.h>

#include <mach/vcio2.h>
#include <mach/platform.h>

#include <asm/uaccess.h>


#define DRIVER_NAME BCM_VCIO_DRIVER_NAME

#ifndef BCM_VCIO2_ADD
/* ----------------------------------------------------------------------
 *	Mailbox
 * -------------------------------------------------------------------- */

/* offsets from a mail box base address */
#define MAIL_WRT	0x00	/* write - and next 4 words */
#define MAIL_RD		0x00	/* read - and next 4 words */
#define MAIL_POL	0x10	/* read without popping the fifo */
#define MAIL_SND	0x14	/* sender ID (bottom two bits) */
#define MAIL_STA	0x18	/* status */
#define MAIL_CNF	0x1C	/* configuration */

#define MBOX_MSG(chan, data28)		(((data28) & ~0xf) | ((chan) & 0xf))
#define MBOX_MSG_LSB(chan, data28) (((data28) << 4) | ((chan) & 0xf))
#define MBOX_CHAN(msg)			((msg) & 0xf)
#define MBOX_DATA28(msg)		((msg) & ~0xf)
#define MBOX_DATA28_LSB(msg)		(((uint32_t)msg) >> 4)

#define MBOX_MAGIC 0xd0d0c0de

struct vc_mailbox {
	struct device *dev;	/* parent device */
	void __iomem *status;
	void __iomem *config;
	void __iomem *read;
	void __iomem *write;
	uint32_t msg[MBOX_CHAN_COUNT];
	struct semaphore sema[MBOX_CHAN_COUNT];
	uint32_t magic;
};

static void mbox_init(struct vc_mailbox *mbox_out, struct device *dev,
		      uint32_t addr_mbox)
{
	int i;

	mbox_out->dev = dev;
	mbox_out->status = __io_address(addr_mbox + MAIL_STA);
	mbox_out->config = __io_address(addr_mbox + MAIL_CNF);
	mbox_out->read = __io_address(addr_mbox + MAIL_RD);
	/* Write to the other mailbox */
	mbox_out->write =
	    __io_address((addr_mbox ^ ARM_0_MAIL0_WRT ^ ARM_0_MAIL1_WRT) +
			 MAIL_WRT);

	for (i = 0; i < MBOX_CHAN_COUNT; i++) {
		mbox_out->msg[i] = 0;
		sema_init(&mbox_out->sema[i], 0);
	}

	/* Enable the interrupt on data reception */
	writel(ARM_MC_IHAVEDATAIRQEN, mbox_out->config);

	mbox_out->magic = MBOX_MAGIC;
}

static int mbox_write(struct vc_mailbox *mbox, unsigned chan, uint32_t data28)
{
	int rc;

	if (mbox->magic != MBOX_MAGIC)
		rc = -EINVAL;
	else {
		/* wait for the mailbox FIFO to have some space in it */
		while (0 != (readl(mbox->status) & ARM_MS_FULL))
			cpu_relax();

		writel(MBOX_MSG(chan, data28), mbox->write);
		rc = 0;
	}
	return rc;
}

static int mbox_read(struct vc_mailbox *mbox, unsigned chan, uint32_t *data28)
{
	int rc;

	if (mbox->magic != MBOX_MAGIC)
		rc = -EINVAL;
	else {
		down(&mbox->sema[chan]);
		*data28 = MBOX_DATA28(mbox->msg[chan]);
		mbox->msg[chan] = 0;
		rc = 0;
	}
	return rc;
}

static irqreturn_t mbox_irq(int irq, void *dev_id)
{
	/* wait for the mailbox FIFO to have some data in it */
	struct vc_mailbox *mbox = (struct vc_mailbox *) dev_id;
	int status = readl(mbox->status);
	int ret = IRQ_NONE;

	while (!(status & ARM_MS_EMPTY)) {
		uint32_t msg = readl(mbox->read);
		int chan = MBOX_CHAN(msg);
		if (chan < MBOX_CHAN_COUNT) {
			if (mbox->msg[chan]) {
				/* Overflow */
				printk(KERN_ERR DRIVER_NAME
				       ": mbox chan %d overflow - drop %08x\n",
				       chan, msg);
			} else {
				mbox->msg[chan] = (msg | 0xf);
				up(&mbox->sema[chan]);
			}
		} else {
			printk(KERN_ERR DRIVER_NAME
			       ": invalid channel selector (msg %08x)\n", msg);
		}
		ret = IRQ_HANDLED;
		status = readl(mbox->status);
	}
	return ret;
}

static struct irqaction mbox_irqaction = {
	.name = "ARM Mailbox IRQ",
	.flags = IRQF_DISABLED | IRQF_IRQPOLL,
	.handler = mbox_irq,
};

/* ----------------------------------------------------------------------
 *	Mailbox Methods
 * -------------------------------------------------------------------- */

static struct device *mbox_dev;	/* we assume there's only one! */

static int dev_mbox_write(struct device *dev, unsigned chan, uint32_t data28)
{
	int rc;

	struct vc_mailbox *mailbox = dev_get_drvdata(dev);
	device_lock(dev);
	rc = mbox_write(mailbox, chan, data28);
	device_unlock(dev);

	return rc;
}

static int dev_mbox_read(struct device *dev, unsigned chan, uint32_t *data28)
{
	int rc;

	struct vc_mailbox *mailbox = dev_get_drvdata(dev);
	device_lock(dev);
	rc = mbox_read(mailbox, chan, data28);
	device_unlock(dev);

	return rc;
}

extern int bcm_mailbox_write(unsigned chan, uint32_t data28)
{
	if (mbox_dev)
		return dev_mbox_write(mbox_dev, chan, data28);
	else
		return -ENODEV;
}
EXPORT_SYMBOL_GPL(bcm_mailbox_write);

extern int bcm_mailbox_read(unsigned chan, uint32_t *data28)
{
	if (mbox_dev)
		return dev_mbox_read(mbox_dev, chan, data28);
	else
		return -ENODEV;
}
EXPORT_SYMBOL_GPL(bcm_mailbox_read);

static void dev_mbox_register(const char *dev_name, struct device *dev)
{
	mbox_dev = dev;
}
#endif

static int mbox_copy_from_user(void *dst, const void *src, int size)
{
	if ( (uint32_t)src < TASK_SIZE)
	{
		return copy_from_user(dst, src, size);
	}
	else
	{
		memcpy( dst, src, size );
		return 0;
	}
}

static int mbox_copy_to_user(void *dst, const void *src, int size)
{
	if ( (uint32_t)dst < TASK_SIZE)
	{
		return copy_to_user(dst, src, size);
	}
	else
	{
		memcpy( dst, src, size );
		return 0;
	}
}

#ifndef BCM_VCIO2_ADD
static DEFINE_MUTEX(mailbox_lock);
extern int bcm_mailbox_property(void *data, int size)
{
	uint32_t success;
	dma_addr_t mem_bus;				/* the memory address accessed from videocore */
	void *mem_kern;					/* the memory address accessed from driver */
	int s = 0;

        mutex_lock(&mailbox_lock);
	/* allocate some memory for the messages communicating with GPU */
	mem_kern = dma_alloc_coherent(NULL, PAGE_ALIGN(size), &mem_bus, GFP_ATOMIC);
	if (mem_kern) {
		/* create the message */
		mbox_copy_from_user(mem_kern, data, size);

		/* send the message */
		wmb();
		s = bcm_mailbox_write(MBOX_CHAN_PROPERTY, (uint32_t)mem_bus);
		if (s == 0) {
			s = bcm_mailbox_read(MBOX_CHAN_PROPERTY, &success);
		}
		if (s == 0) {
			/* copy the response */
			rmb();
			mbox_copy_to_user(data, mem_kern, size);
		}
		dma_free_coherent(NULL, PAGE_ALIGN(size), mem_kern, mem_bus);
	} else {
		s = -ENOMEM;
	}
	if (s != 0)
		printk(KERN_ERR DRIVER_NAME ": %s failed (%d)\n", __func__, s);

        mutex_unlock(&mailbox_lock);
	return s;
}
EXPORT_SYMBOL_GPL(bcm_mailbox_property);
#endif

/* ----------------------------------------------------------------------
 *	Platform Device for Mailbox
 * -------------------------------------------------------------------- */

static unsigned int AllocateVcMemory(unsigned int *pHandle, unsigned int size, unsigned int alignment, unsigned int flags)
{
	struct vc_msg
	{
		unsigned int m_msgSize;
		unsigned int m_response;

		struct vc_tag
		{
			unsigned int m_tagId;
			unsigned int m_sendBufferSize;
			union {
				unsigned int m_sendDataSize;
				unsigned int m_recvDataSize;
			};

			struct args
			{
				union {
					unsigned int m_size;
					unsigned int m_handle;
				};
				unsigned int m_alignment;
				unsigned int m_flags;
			} m_args;
		} m_tag;

		unsigned int m_endTag;
	} msg;
	int s;

	msg.m_msgSize = sizeof(msg);
	msg.m_response = 0;
	msg.m_endTag = 0;

	//fill in the tag for the allocation command
	msg.m_tag.m_tagId = VCMSG_SET_ALLOCATE_MEM;
	msg.m_tag.m_sendBufferSize = 12;
	msg.m_tag.m_sendDataSize = 12;

	//fill in our args
	msg.m_tag.m_args.m_size = size;
	msg.m_tag.m_args.m_alignment = alignment;
	msg.m_tag.m_args.m_flags = flags;

	//run the command
	s = bcm_mailbox_property(&msg, sizeof(msg));

	if (s == 0 && msg.m_response == 0x80000000 && msg.m_tag.m_recvDataSize == 0x80000004)
	{
		*pHandle = msg.m_tag.m_args.m_handle;
		return 0;
	}
	else
	{
		printk(KERN_ERR "failed to allocate vc memory: s=%d response=%08x recv data size=%08x\n",
				s, msg.m_response, msg.m_tag.m_recvDataSize);
		return 1;
	}
}

static unsigned int ReleaseVcMemory(unsigned int handle)
{
	struct vc_msg
	{
		unsigned int m_msgSize;
		unsigned int m_response;

		struct vc_tag
		{
			unsigned int m_tagId;
			unsigned int m_sendBufferSize;
			union {
				unsigned int m_sendDataSize;
				unsigned int m_recvDataSize;
			};

			struct args
			{
				union {
					unsigned int m_handle;
					unsigned int m_error;
				};
			} m_args;
		} m_tag;

		unsigned int m_endTag;
	} msg;
	int s;

	msg.m_msgSize = sizeof(msg);
	msg.m_response = 0;
	msg.m_endTag = 0;

	//fill in the tag for the release command
	msg.m_tag.m_tagId = VCMSG_SET_RELEASE_MEM;
	msg.m_tag.m_sendBufferSize = 4;
	msg.m_tag.m_sendDataSize = 4;

	//pass across the handle
	msg.m_tag.m_args.m_handle = handle;

	s = bcm_mailbox_property(&msg, sizeof(msg));

	if (s == 0 && msg.m_response == 0x80000000 && msg.m_tag.m_recvDataSize == 0x80000004 && msg.m_tag.m_args.m_error == 0)
		return 0;
	else
	{
		printk(KERN_ERR "failed to release vc memory: s=%d response=%08x recv data size=%08x error=%08x\n",
				s, msg.m_response, msg.m_tag.m_recvDataSize, msg.m_tag.m_args.m_error);
		return 1;
	}
}

static unsigned int LockVcMemory(unsigned int *pBusAddress, unsigned int handle)
{
	struct vc_msg
	{
		unsigned int m_msgSize;
		unsigned int m_response;

		struct vc_tag
		{
			unsigned int m_tagId;
			unsigned int m_sendBufferSize;
			union {
				unsigned int m_sendDataSize;
				unsigned int m_recvDataSize;
			};

			struct args
			{
				union {
					unsigned int m_handle;
					unsigned int m_busAddress;
				};
			} m_args;
		} m_tag;

		unsigned int m_endTag;
	} msg;
	int s;

	msg.m_msgSize = sizeof(msg);
	msg.m_response = 0;
	msg.m_endTag = 0;

	//fill in the tag for the lock command
	msg.m_tag.m_tagId = VCMSG_SET_LOCK_MEM;
	msg.m_tag.m_sendBufferSize = 4;
	msg.m_tag.m_sendDataSize = 4;

	//pass across the handle
	msg.m_tag.m_args.m_handle = handle;

	s = bcm_mailbox_property(&msg, sizeof(msg));

	if (s == 0 && msg.m_response == 0x80000000 && msg.m_tag.m_recvDataSize == 0x80000004)
	{
		//pick out the bus address
		*pBusAddress = msg.m_tag.m_args.m_busAddress;
		return 0;
	}
	else
	{
		printk(KERN_ERR "failed to lock vc memory: s=%d response=%08x recv data size=%08x\n",
				s, msg.m_response, msg.m_tag.m_recvDataSize);
		return 1;
	}
}

static unsigned int UnlockVcMemory(unsigned int handle)
{
	struct vc_msg
	{
		unsigned int m_msgSize;
		unsigned int m_response;

		struct vc_tag
		{
			unsigned int m_tagId;
			unsigned int m_sendBufferSize;
			union {
				unsigned int m_sendDataSize;
				unsigned int m_recvDataSize;
			};

			struct args
			{
				union {
					unsigned int m_handle;
					unsigned int m_error;
				};
			} m_args;
		} m_tag;

		unsigned int m_endTag;
	} msg;
	int s;

	msg.m_msgSize = sizeof(msg);
	msg.m_response = 0;
	msg.m_endTag = 0;

	//fill in the tag for the unlock command
	msg.m_tag.m_tagId = VCMSG_SET_UNLOCK_MEM;
	msg.m_tag.m_sendBufferSize = 4;
	msg.m_tag.m_sendDataSize = 4;

	//pass across the handle
	msg.m_tag.m_args.m_handle = handle;

	s = bcm_mailbox_property(&msg, sizeof(msg));

	//check the error code too
	if (s == 0 && msg.m_response == 0x80000000 && msg.m_tag.m_recvDataSize == 0x80000004 && msg.m_tag.m_args.m_error == 0)
		return 0;
	else
	{
		printk(KERN_ERR "failed to unlock vc memory: s=%d response=%08x recv data size=%08x error%08x\n",
				s, msg.m_response, msg.m_tag.m_recvDataSize, msg.m_tag.m_args.m_error);
		return 1;
	}
}

static unsigned int bcm_qpu_enable(unsigned enable)
{
	struct vc_msg
	{
		unsigned int m_msgSize;
		unsigned int m_response;

		struct vc_tag
		{
			unsigned int m_tagId;
			unsigned int m_sendBufferSize;
			union {
				unsigned int m_sendDataSize;
				unsigned int m_recvDataSize;
			};

			struct args
			{
				union {
					unsigned int m_enable;
					unsigned int m_return;
				};
			} m_args;
		} m_tag;

		unsigned int m_endTag;
	} msg;
	int s;

	pr_debug(DRIVER_NAME ": qpu_enable(%d)\n", enable);

	/* property message to VCIO channel */
	/* create the message */
	msg.m_msgSize = sizeof(msg);
	msg.m_response = 0;
	msg.m_endTag = 0;

	msg.m_tag.m_tagId = VCMSG_SET_ENABLE_QPU;
	msg.m_tag.m_sendBufferSize = 4;
	msg.m_tag.m_sendDataSize = 4;

	s = bcm_mailbox_property(&msg, sizeof msg);

	//check the error code too
	if (s == 0 && msg.m_response == 0x80000000 && msg.m_tag.m_recvDataSize == 0x80000004)
		return msg.m_tag.m_args.m_return;
	else
	{
		pr_err(DRIVER_NAME ": failed to execute QPU: s=%d response=%08x recv data size=%08x\n",
				s, msg.m_response, msg.m_tag.m_recvDataSize);
		return s;
	}
}

static unsigned int bcm_execute_qpu(unsigned num_qpus, unsigned control, unsigned noflush, unsigned timeout)
{
	struct vc_msg
	{
		unsigned int m_msgSize;
		unsigned int m_response;

		struct vc_tag
		{
			unsigned int m_tagId;
			unsigned int m_sendBufferSize;
			union {
				unsigned int m_sendDataSize;
				unsigned int m_recvDataSize;
			};

			struct args
			{
				union {
					unsigned int m_numQpus;
					unsigned int m_return;
				};
				unsigned int m_control;
				unsigned int m_noflush;
				unsigned int m_timeout;
			} m_args;
		} m_tag;

		unsigned int m_endTag;
	} msg;
	int s;

	pr_info(DRIVER_NAME ": execute_qpu(%d, %x, %d, %d)\n", num_qpus, control, noflush, timeout);

	/* property message to VCIO channel */

	/* create the message */
	msg.m_msgSize = sizeof(msg);
	msg.m_response = 0;
	msg.m_endTag = 0;

	msg.m_tag.m_tagId = VCMSG_SET_EXECUTE_QPU;
	msg.m_tag.m_sendBufferSize = 16;
	msg.m_tag.m_sendDataSize = 16;

	//pass across the handle
	msg.m_tag.m_args.m_numQpus = num_qpus;
	msg.m_tag.m_args.m_control = control;
	msg.m_tag.m_args.m_noflush = noflush;
	msg.m_tag.m_args.m_timeout = timeout;

	s = bcm_mailbox_property(&msg, sizeof msg);

	//check the error code too
	if (s == 0 && msg.m_response == 0x80000000 && msg.m_tag.m_recvDataSize == 0x80000004)
		return msg.m_tag.m_args.m_return;
	else
	{
		pr_err(DRIVER_NAME ": failed to execute QPU: s=%d response=%08x recv data size=%08x\n",
				s, msg.m_response, msg.m_tag.m_recvDataSize);
		return s;
	}
}


/* seems there is no opposite of remap_pfn_range
static int vcio_munmap(struct mm_struct* mm, unsigned long start, size_t len)
{
	int ret;
	down_write(&mm->mmap_sem);
	ret = do_munmap(mm, start, len);
	up_write(&mm->mmap_sem);
	return ret;
}*/

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
 * 2's complement of the location where it should be insert if not found. */
static int vcioa_locate(const vcio_allocs* allocs, unsigned handle)
{
	unsigned l = 0;
	unsigned r = allocs->Count;
	pr_debug(DRIVER_NAME ": %s(%p{%p,%u,%u}, %x)\n", __func__, allocs, allocs->List, allocs->Count, allocs->Size, handle);
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
	pr_debug(DRIVER_NAME ": %s(%p{%p,%u,%u}, %u)\n", __func__, allocs, allocs->List, allocs->Count, allocs->Size, pos);

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
	pr_debug(DRIVER_NAME ": %s(%p{%p,%u,%u}, %u)\n", __func__, allocs, allocs->List, allocs->Count, allocs->Size, pos);
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
	pr_info(DRIVER_NAME ": %s(%p{%p,%u,%u}, %x)\n", __func__, allocs, allocs->List, allocs->Count, allocs->Size, addr);
	ap = allocs->List;
	ape = ap + allocs->Count;
	for (; ap != ape; ++ap)
	{	pr_info(DRIVER_NAME ": %s {%u,%x,%x}\n", __func__, ap->Handle, ap->Size, ap->Location);
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
	pr_debug(DRIVER_NAME ": %s(%p{%p,%u,%u})\n", __func__, allocs, allocs->List, allocs->Count, allocs->Size);
	/* clean up memory resources */
	ap = allocs->List;
	ape = ap + allocs->Count;
	for (; ap != ape; ++ap)
	{	pr_info(DRIVER_NAME ": cleanup GPU memory: %x @ %x[%x]\n", ap->Handle, ap->Location, ap->Size);
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
	struct mutex Lock;
	vcio_allocs  Allocations;

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


/** Is the device open right now? Used to prevent
 * concurrent access into the same device
 */
static atomic_t vcio_opened = ATOMIC_INIT(0);

/** This is called whenever a process attempts to open the device file
 */
static int device_open(struct inode *inode, struct file *file)
{
	pr_info(DRIVER_NAME ": opening vcio %p, %p\n", inode, file);

	if (MINOR(file->f_inode->i_rdev) == 1)
		file->private_data = vcio_create();

	if (atomic_add_return(1, &vcio_opened) == 1)
	{
		if (bcm_qpu_enable(1))
		{	atomic_dec(&vcio_opened);
			return -ENODEV;
		}
	}

	/*
	 * Initialize the message 
	 */
	try_module_get(THIS_MODULE);
	return 0;
}

static int device_release(struct inode *inode, struct file *file)
{
	pr_info(DRIVER_NAME ": closing vcio %p, %p\n", inode, file);

	if (file->private_data)
		vcio_destroy(file->private_data);

	if (atomic_dec_and_test(&vcio_opened))
	{
		bcm_qpu_enable(0);
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
	unsigned size;
	switch (MINOR(f->f_inode->i_rdev))
	{
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
					mbox_copy_from_user(&p.in, (void*)ioctl_param, sizeof p.in);
					size = p.in.size;
					pr_debug(DRIVER_NAME ": IOCTL_MEM_ALLOCATE %x, %x, %x\n", size, p.in.alignment, p.in.flags);
					if (AllocateVcMemory(&p.out.handle, size, p.in.alignment, p.in.flags))
					{	p.out.handle = 0;
						rc = -ENOMEM;
					} else if ((pos = vcioa_locate(&data->Allocations, p.out.handle)) >= 0)
					{	pr_err(DRIVER_NAME ": allocated handle %d twice ???\n", p.out.handle);
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
						pr_debug(DRIVER_NAME ": IOCTL_MEM_ALLOCATE: {%d, %x, }\n", ap->Handle, ap->Size);
					}
					mbox_copy_to_user((void*)ioctl_param, &p.out, sizeof p.out);
					break;
				}

				case IOCTL_MEM_RELEASE:
				{
					pr_debug(DRIVER_NAME ": IOCTL_MEM_RELEASE: %x\n", (unsigned)ioctl_param);
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
					mbox_copy_from_user(&param, (void*)ioctl_param, sizeof param);
					pr_debug(DRIVER_NAME ": IOCTL_MEM_LOCK %x\n", param);
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
					pr_debug(DRIVER_NAME ": IOCTL_MEM_LOCK: %x\n", param);
					mbox_copy_to_user((void*)ioctl_param, &param, sizeof param);
					break;
				}

				case IOCTL_MEM_UNLOCK:
				{
					pr_debug(DRIVER_NAME ": IOCTL_MEM_UNLOCK %x\n", (unsigned)ioctl_param);
					pos = vcioa_locate(&data->Allocations, ioctl_param);
					if (pos < 0)
						rc = -EBADF;
					else
					{	vcio_alloc* ap = data->Allocations.List + pos;
						if (ap->Location == VCIOA_LOCATION_NONE)
						{	pr_warning(DRIVER_NAME ": tried to unlock unlocked memory %d\n", ap->Handle);
							rc = -EPERM;
						} else if (UnlockVcMemory(ap->Handle))
							rc = -ENOMEM;
						else
							ap->Location = VCIOA_LOCATION_NONE;
					}
					break;
				}

				case IOCTL_EXEC_QPU:
				{	vcio_exec_qpu p;
					mbox_copy_from_user(&p.in, (void*)ioctl_param, sizeof p.in);
					pr_info(DRIVER_NAME ": IOCTL_EXEC_QPU %x, %x, %x, %x\n", p.in.num_qpus, p.in.control, p.in.noflush, p.in.timeout);
					/* verify starting point */
					if (!vcioa_find_addr(&data->Allocations, p.in.control))
					{	rc = -EACCES;
					} else
					{	/* TODO: verify starting points of code and uniforms too
						for (i = 0; i < p.in.num_qpus; ++i)
						{	if (!vcioa_find_addr(&data->Allocations, ))
								goto exec_fail;
						}*/
						if (bcm_execute_qpu(p.in.num_qpus, p.in.control, p.in.noflush, p.in.timeout))
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
	pr_err(DRIVER_NAME "unknown ioctl: %d, minor = %d\n", ioctl_num, MINOR(f->f_inode->i_rdev));
	return -EINVAL;
}

static struct vm_operations_struct vm_ops = {
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
		pr_info(DRIVER_NAME ": writeable mappings must be shared, rejecting\n");
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
			pr_info(DRIVER_NAME ": tried to map memory (%x) that is not allocated by this device.\n", start);
			return -EACCES;
		}
		if (((start + size -1) >> PAGE_SHIFT) > ((vca->Location + vca->Size -1) >> PAGE_SHIFT))
		{
			pr_info(DRIVER_NAME ": the memory region to map exceeds the allocated buffer (%x[%x] vs. %x[%x]).\n",
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
		pr_warning(DRIVER_NAME ": remap page range failed with %d\n", rc);
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

/** vcio character device id from cdev_add, non-zero if allocated. */
static dev_t vcio_dev = 0;
/** vcio character device, non-zero if allocated. */
static struct cdev vcio_cdev0 = { };
/** vcio character device, non-zero if allocated. */
static struct cdev vcio_cdev1 = { };
/** device class from class_create, non-zero if allocated. */
static struct class *vcio_class = NULL;
/** device vcio created by device_create, non-zero if allocated. */
struct device *vcio_dev0 = NULL;
/** device vcio1 created by device_create, non-zero if allocated. */
struct device *vcio_dev1 = NULL;

static int bcm_vcio_remove(struct platform_device *pdev)
{
#ifndef BCM_VCIO2_ADD
	struct vc_mailbox *mailbox;
#endif

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

static int bcm_vcio_probe(struct platform_device *pdev)
{
	int ret = 0;
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

	//pr_info(DRIVER_NAME ": 1\n");

	/* Register the character device */
	ret = alloc_chrdev_region(&vcio_dev, 0, 2, DEVICE_FILE_NAME);
	if (ret < 0) {
		pr_err(DRIVER_NAME ": Failed registering the character device %d\n", ret);
		vcio_dev = 0;
		goto fail;
	}

	//pr_info(DRIVER_NAME ": 2\n");

	cdev_init(&vcio_cdev0, &fops);
	vcio_cdev0.owner = THIS_MODULE;
	ret = cdev_add(&vcio_cdev0, vcio_dev, 1);
	if (ret < 0) {
		pr_err(DRIVER_NAME " %s: Unable to add device (rc=%d)", __func__, ret);
		memset(&vcio_cdev0, 0, sizeof vcio_cdev0);
		goto fail;
	}
	//pr_info(DRIVER_NAME ": 3 %d\n", ret);
	cdev_init(&vcio_cdev1, &fops);
	vcio_cdev1.owner = THIS_MODULE;
	ret = cdev_add(&vcio_cdev1, vcio_dev+1, 1);
	if (ret < 0) {
		pr_err(DRIVER_NAME " %s: Unable to add device (rc=%d)", __func__, ret);
		memset(&vcio_cdev1, 0, sizeof vcio_cdev1);
		goto fail;
	}
	//pr_info(DRIVER_NAME ": 3 %d\n", ret);

	/* Create vcio device */
	vcio_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(vcio_class)) {
		ret = PTR_ERR(vcio_class);
		pr_err(DRIVER_NAME " %s: class_create failed (rc=%d)\n", __func__, ret);
		vcio_class = NULL;
		goto fail;
	}

	vcio_dev0 = device_create(vcio_class, NULL, vcio_dev, NULL, "vcio");
	if (IS_ERR(vcio_dev0)) {
		ret = PTR_ERR(vcio_dev0);
		pr_err(DRIVER_NAME " %s: device_create failed (rc=%d)\n", __func__, ret);
		vcio_dev0 = NULL;
		goto fail;
	}
	vcio_dev1 = device_create(vcio_class, NULL, vcio_dev+1, NULL, "vcio2");
	if (IS_ERR(vcio_dev1)) {
		ret = PTR_ERR(vcio_dev1);
		pr_err(DRIVER_NAME " %s: device_create failed (rc=%d)\n", __func__, ret);
		vcio_dev1 = NULL;
		goto fail;
	}

	/* succeeded! */
	return 0;

 fail:
	bcm_vcio_remove(pdev);

	return ret;
}

static struct platform_driver bcm_mbox_driver = {
	.probe = bcm_vcio_probe,
	.remove = bcm_vcio_remove,

	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init bcm_mbox_init(void)
{
	int ret;

	printk(KERN_INFO "mailbox: enhanced Broadcom VideoCore Mailbox driver\n");

#ifndef BCM_VCIO2_ADD
	ret = platform_driver_register(&bcm_mbox_driver);
	if (ret != 0) {
		printk(KERN_ERR DRIVER_NAME ": failed to register on platform\n");
	}
#else
	bcm_vcio_probe(NULL);
#endif

	return ret;
}

static void __exit bcm_mbox_exit(void)
{
#ifndef BCM_VCIO2_ADD
	platform_driver_unregister(&bcm_mbox_driver);
#else
	bcm_vcio_remove(NULL);
#endif
}

#if BCM_VCIO2_ADD
module_init(bcm_mbox_init);
#else
arch_initcall(bcm_mbox_init);	/* Initialize early */
#endif
module_exit(bcm_mbox_exit);

MODULE_AUTHOR("Gray Girling, Marcel Mueller");
MODULE_DESCRIPTION("ARM I/O to VideoCore processor");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bcm-mbox");
