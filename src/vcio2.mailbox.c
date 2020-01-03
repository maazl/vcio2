/*
 *  vcio2.mailbox.c
 *
 *  Copyright (C) 2019-2020 Marcel Müller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This file contains functions to access the RPi firmware property mailbox
 */

#define mailbox_property(tag) rpi_firmware_property_list(firmware, &tag, sizeof (tag));

static uint32_t GetBoardRevision(void)
{
	struct vc_tag
	{	uint32_t tagId;
		uint32_t bufSize;
		uint32_t dataSize;

		uint32_t model;
	} tag;
	int s;

	//fill in the tag for the allocation command
	tag.tagId = RPI_FIRMWARE_GET_BOARD_REVISION;
	tag.bufSize = 4;
	tag.dataSize = 0;

	//run the command
	s = mailbox_property(tag);

	if (likely(s == 0 && tag.dataSize == 0x80000004))
		return tag.model;
	else
	{	vcio_pr_warn("Failed to get board revision: s=%d recv data size=%08x", s, tag.dataSize);
		return 0;
	}
}

static uint32_t AllocateVcMemory(uint32_t *pHandle, uint32_t size, uint32_t alignment, uint32_t flags)
{
	struct vc_tag
	{	uint32_t tagId;
		uint32_t bufSize;
		uint32_t dataSize;

		union
		{	uint32_t size;
			uint32_t handle;
		};
		uint32_t alignment;
		uint32_t flags;
	} tag;
	int s;

	//fill in the tag for the allocation command
	tag.tagId = RPI_FIRMWARE_ALLOCATE_MEMORY;
	tag.dataSize = tag.bufSize = 12;

	//fill in our args
	tag.size = size;
	tag.alignment = alignment;
	tag.flags = flags;

	//run the command
	s = mailbox_property(tag);

	if (likely(s == 0 && tag.dataSize == 0x80000004))
	{	*pHandle = tag.handle;
		return 0;
	} else
	{	vcio_pr_warn("Failed to allocate VC memory: s=%d recv data size=%08x", s, tag.dataSize);
		return s ? s : 1;
	}
}

static uint32_t ReleaseVcMemory(uint32_t handle)
{
	struct vc_tag
	{	uint32_t tagId;
		uint32_t bufSize;
		uint32_t dataSize;

		union
		{	uint32_t handle;
			uint32_t error;
		};
	} tag;
	int s;

	//fill in the tag for the release command
	tag.tagId = RPI_FIRMWARE_RELEASE_MEMORY;
	tag.dataSize = tag.bufSize = 4;

	//pass across the handle
	tag.handle = handle;

	s = mailbox_property(tag);

	if (likely(s == 0 && tag.dataSize == 0x80000004 && tag.error == 0))
		return 0;
	else
	{	vcio_pr_warn("Failed to release VC memory: rc=%i recv data size=%08x error=%08x", s, tag.dataSize, tag.error);
		return s ? s : 1;
	}
}

static uint32_t LockVcMemory(uint32_t *pBusAddress, uint32_t handle)
{
	struct vc_tag
	{	uint32_t tagId;
		uint32_t bufSize;
		uint32_t dataSize;

		union
		{	uint32_t handle;
			uint32_t busAddress;
		};
	} tag;
	int s;

	//fill in the tag for the lock command
	tag.tagId = RPI_FIRMWARE_LOCK_MEMORY;
	tag.dataSize = tag.bufSize = 4;

	//pass across the handle
	tag.handle = handle;

	s = mailbox_property(tag);

	if (likely(s == 0 && tag.dataSize == 0x80000004))
	{	*pBusAddress = tag.busAddress;
		return 0;
	} else
	{	vcio_pr_warn("Failed to lock VC memory: s=%d recv data size=%08x", s, tag.dataSize);
		return s ? s : 1;
	}
}

static uint32_t UnlockVcMemory(uint32_t handle)
{
	struct vc_tag
	{	uint32_t tagId;
		uint32_t bufSize;
		uint32_t dataSize;

		union
		{	uint32_t handle;
			uint32_t error;
		};
	} tag;
	int s;

	//fill in the tag for the unlock command
	tag.tagId = RPI_FIRMWARE_UNLOCK_MEMORY;
	tag.dataSize = tag.bufSize = 4;

	//pass across the handle
	tag.handle = handle;

	s = mailbox_property(tag);

	//check the error code too
	if (likely(s == 0 && tag.dataSize == 0x80000004 && tag.error == 0))
		return 0;
	else
	{	vcio_pr_warn("Failed to unlock VC memory: s=%d recv data size=%08x error%08x", s, tag.dataSize, tag.error);
		return s ? s : 1;
	}
}

static uint32_t QpuEnable(unsigned enable)
{
	struct vc_tag
	{	uint32_t tagId;
		uint32_t bufSize;
		uint32_t dataSize;

		union
		{	uint32_t enable;
			uint32_t ret;
		};
	} tag;
	int s;

	vcio_pr_debug("%s %d", __func__, enable);

	/* property message to VCIO channel */
	tag.tagId = RPI_FIRMWARE_SET_ENABLE_QPU;
	tag.dataSize = tag.bufSize = 4;

	s = mailbox_property(tag);

	//check the error code too
	if (likely(s == 0 && tag.dataSize == 0x80000004))
		return tag.ret;
	else
	{	vcio_pr_warn("Failed to execute QPU: s=%d recv data size=%08x", s, tag.dataSize);
		return s;
	}
}

static uint32_t ExecuteQpu(uint32_t num_qpus, uint32_t control, uint32_t noflush, uint32_t timeout)
{
	struct vc_tag
	{	uint32_t tagId;
		uint32_t bufSize;
		uint32_t dataSize;

		union
		{	uint32_t numQpus;
			uint32_t ret;
		};
		uint32_t control;
		uint32_t noflush;
		uint32_t timeout;
	} tag;
	int s;

	vcio_pr_debug("%s(%d, %x, %d, %d)", __func__, num_qpus, control, noflush, timeout);

	/* property message to VCIO channel */
	tag.tagId = RPI_FIRMWARE_EXECUTE_QPU;
	tag.dataSize = tag.bufSize = 16;

	//pass across the handle
	tag.numQpus = num_qpus;
	tag.control = control;
	tag.noflush = noflush;
	tag.timeout = timeout;

	s = mailbox_property(tag);

	//check the error code too
	if (likely(s == 0 && tag.dataSize == 0x80000004))
		return tag.ret;
	else
	{	vcio_pr_warn("Failed to execute QPU: s=%d recv data size=%08x", s, tag.dataSize);
		return s;
	}
}

#undef mailbox_property
