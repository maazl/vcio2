/*
Copyright (c) 2019, Marcel Müller
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


Example application to demonstrate the usage of the vcio2 driver to execute code
on Raspberry Pi's Videocore IV GPU.
*/

#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <soc/bcm2835/vcio2.h>

#include "../shader.h"

#define VEC_COUNT 3*16
#define GPU_MEM_FLG 0xC // cached=0xC; direct=0x4


static const int input[VEC_COUNT*2] =
{	0,0, 1,1, 2,2, 3,3, 4,4, 5,5, 6,6, 7,7, 8,8, 9,9, 10,10, 11,11, 12,12, 13,13, 14,14, 15,15,
	-16,-16, -15,-15, -14,-14, -13,-13, -12,-12, -11,-11, -10,-10, -9,-9,
	-8,-8, -7,-7, -6,-6, -5,-5, -4,-4, -3,-3, -2,-2, -1,-1,
	0x3b800000,0x3b800000, 0x3c000000,0x3c000000, 0x3c800000,0x3c800000, 0x3d000000,0x3d000000,
	0x3d800000,0x3d800000, 0x3e000000,0x3e000000, 0x3e800000,0x3e800000, 0x3f000000,0x3f000000,
	0x3f800000,0x3f800000, 0x40000000,0x40000000, 0x40800000,0x40800000, 0x41000000,0x41000000,
	0x41800000,0x41800000, 0x42000000,0x42000000, 0x42800000,0x42800000, 0x43000000,0x43000000
};

static const unsigned input2[] =
{
	#include "../numbers.hex"
};

static const char op[RES_COUNT][8] =
{	"fadd", "fsub", "fmin", "fmax", "fminabs", "fmaxabs", "ftoi", "itof",
	"add", "sub", "shr", "asr", "ror", "shl", "min", "max", "and", "or", "xor", "not", "clz",
	"fmul", "mul24", "v8muld", "v8min", "v8max", "v8adds", "v8subs",
	"A0", "A9", "A10", "A11", "A25", "A26", "A27", "A28", "A29", "M0",
	"fmulP", "mul24P", "v8muldP", "v8minP", "v8addsP", "v8subsP", "add.32s"
};

static const char pack[RES_COUNT2][8] =
{	"M8abcds", "M8a", "M8b", "M8c", "M8d",
	"A16a", "A16b", "Af16a", "Af16b", "A8abcd", "A8a", "A8b", "A8c", "A8d",
	"A32s", "A16as", "A16bs", "Af16as", "Af16bs", "A8abcds", "A8as", "A8bs", "A8cs", "A8ds"
};

struct GPU
{	unsigned code[sizeof(shader) / sizeof(uint32_t)];
	unsigned input[VEC_COUNT*2];
	unsigned output[VEC_COUNT*RES_COUNT];
	unsigned input2[(sizeof input2/sizeof *input2 + 0xf) & ~0xf];
	unsigned output2[((sizeof input2/sizeof *input2 + 0xf) & ~0xf) * RES_COUNT2];
	unsigned unif[6];
	unsigned mail[2];
	int      mb;
};

static int gpu_prepare(volatile struct GPU** gpu)
{	volatile struct GPU* ptr;
	// open a char device file used for communicating with kernel mbox driver
	int mb = open("/dev/vcio2", O_RDWR);
	if (mb < 0)
	{	fprintf(stderr, "Can't open device file /dev/vcio2: %s\n", strerror(errno));
		return -1;
	}

	ptr = vcio2_malloc(mb, sizeof(struct GPU));
	if (ptr == MAP_FAILED)
	{	printf("mmap error: %s\n", strerror(errno));
		return -4;
	}

	ptr->mb = mb;
	ptr->mail[0] = (ptr->mail[1] = *(unsigned*)ptr) + offsetof(struct GPU, unif);

	*gpu = ptr;
	return 0;
}

static unsigned gpu_execute(volatile struct GPU *gpu)
{
	vcio_exec_qpu buf =
	{	1 /* 1 QPU */,
		gpu->mail[1] + offsetof(struct GPU, mail),
		1 /* no flush */,
		5000 /* timeout */
	};
	int ret_val = ioctl(gpu->mb, IOCTL_EXEC_QPU, &buf);
	if (ret_val)
		printf("execute_qpu failed: %d\n", ret_val);
	return ret_val;
}

static void gpu_release(volatile struct GPU *gpu)
{
	close(gpu->mb);
}

int main()
{
	int i,j;
	volatile struct GPU* gpu;
	int ret = gpu_prepare(&gpu);
	if (ret < 0)
		return ret;

	memcpy((void*)gpu->code, shader, sizeof gpu->code);

	gpu->unif[0] = VEC_COUNT/16;
	gpu->unif[1] = gpu->mail[1] + offsetof(struct GPU, input);
	gpu->unif[2] = gpu->mail[1] + offsetof(struct GPU, output);
	gpu->unif[3] = (sizeof input2/sizeof *input2 + 0xf) / 16;
	gpu->unif[4] = gpu->mail[1] + offsetof(struct GPU, input2);
	gpu->unif[5] = gpu->mail[1] + offsetof(struct GPU, output2);

	memcpy((void*)gpu->input, input, sizeof gpu->input);
	memset((void*)gpu->output, 0xbb, sizeof gpu->output);
	memcpy((void*)gpu->input2, input2, sizeof gpu->input2);
	memset((void*)gpu->output2, 0xbb, sizeof gpu->output2);

	printf("Exec: %x\n", gpu_execute(gpu));

	for (i = 0; i < VEC_COUNT; ++i)
	{
		int A = gpu->input[2*i];
		int B = gpu->input[2*i+1];
		printf("\n A\t%i\t0x%08x\t%g\n B\t%i\t0x%08x\t%g\n", A, A, *(float*)&A, B, B, *(float*)&B);
		volatile unsigned* rp = &gpu->output[i*RES_COUNT];
		for (j = 0; j < RES_COUNT; ++j)
			printf("%s\t%i\t0x%08x\t%g\n", op[j], rp[j], rp[j], *(float*)(rp+j));
	}

	for (i = 0; i < sizeof input2/sizeof *input2; ++i)
	{
		unsigned A = gpu->input2[i];
		printf("\n A\t%i\t0x%08x\t%g\n", A, A, *(float*)&A);
		volatile unsigned* rp = &gpu->output2[i*RES_COUNT2];
		for (j = 0; j < RES_COUNT2; ++j)
			printf("%s\t%i\t0x%08x\t%g\n", pack[j], rp[j], rp[j], *(float*)(rp+j));
	}

	gpu_release(gpu);

	return 0;
}
