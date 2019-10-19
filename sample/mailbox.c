/*
Copyright (c) 2012, Broadcom Europe Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <stdint.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <soc/bcm2835/vcio2.h>

#include "mailbox.h"

#define PAGE_SIZE (4*1024)

void *mapmem(int file_desc, unsigned base, unsigned size)
{
	printf("mapmem(%d, %x, %x)\n", file_desc, base, size);
	unsigned offset = base % PAGE_SIZE;
	base = base - offset;
	void *mem = mmap(
		0,
		size,
		PROT_READ|PROT_WRITE,
		MAP_SHARED/*|MAP_FIXED*/,
		file_desc,
		base);
#ifdef DEBUG
	printf("base=0x%x, mem=%p\n", base, mem);
#endif
	if (mem == MAP_FAILED) {
		printf("mmap error: %s\n", strerror(errno));
		return NULL;
	}
	return (char *)mem + offset;
}

void unmapmem(void *addr, unsigned size)
{
	int s = munmap(addr, size);
	if (s != 0) {
		printf("munmap error %d\n", s);
		exit (-1);
	}
}

unsigned mem_alloc(int file_desc, unsigned size, unsigned align, unsigned flags)
{
	vcio_mem_allocate buf;
	int ret_val;

	buf.in.size = size;
	buf.in.alignment = align;
	buf.in.flags = flags;

	ret_val = ioctl(file_desc, IOCTL_MEM_ALLOCATE, &buf);

	if (ret_val)
	{	printf("mem_alloc ioctl failed: %d\n", ret_val);
		return 0;
	}

	return buf.out.handle;
}

unsigned mem_free(int file_desc, unsigned handle)
{
	int ret_val = ioctl(file_desc, IOCTL_MEM_RELEASE, handle);
	if (ret_val)
		printf("mem_free ioctl failed: %d\n", ret_val);
	return ret_val;
}

unsigned mem_lock(int file_desc, unsigned handle)
{
	int ret_val = ioctl(file_desc, IOCTL_MEM_LOCK, &handle);
	if (ret_val)
	{	printf("mem_lock ioctl failed: %d\n", ret_val);
		return 0;
	}
	return handle;
}

unsigned mem_unlock(int file_desc, unsigned handle)
{
	int ret_val = ioctl(file_desc, IOCTL_MEM_UNLOCK, handle);
	if (ret_val)
		printf("mem_unlock ioctl failed: %d\n", ret_val);
	return ret_val;
}

/* TODO: not implemented
unsigned execute_code(int file_desc, unsigned code, unsigned r0, unsigned r1, unsigned r2, unsigned r3, unsigned r4, unsigned r5)
{
	printf("execute_code not implemented\n");
	return -1;
}*/

unsigned execute_qpu(int file_desc, unsigned num_qpus, unsigned control, unsigned noflush, unsigned timeout)
{
	vcio_exec_qpu buf;
	int ret_val;

	buf.in.num_qpus = num_qpus;
	buf.in.control = control;
	buf.in.noflush = noflush;
	buf.in.timeout = timeout;

	ret_val = ioctl(file_desc, IOCTL_EXEC_QPU, &buf);
	if (ret_val)
		printf("execute_qpu failed: %d\n", ret_val);
	return ret_val;
}

unsigned qpu_enable(int file_desc, unsigned enable)
{
	int ret_val = ioctl(file_desc, IOCTL_ENABLE_QPU, enable);
	if (ret_val)
		printf("qpu_enable ioctl failed: %d, %s\n", ret_val, strerror(errno));
	return ret_val;
}

int mbox_open()
{
	int file_desc;

	// open a char device file used for communicating with kernel mbox driver
	file_desc = open("/dev/vcio2", O_RDWR);
	if (file_desc < 0)
		printf("Can't open device file /dev/vcio2: %s\n", strerror(errno));
	return file_desc;
}

void mbox_close(int file_desc)
{
  close(file_desc);
}
