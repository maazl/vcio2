#include "shader.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _MSC_VER
__declspec(align(8))
#elif defined(__GNUC__)
__attribute__((aligned(8)))
#endif
uint32_t shader[260] = {
// ::smi_start
/* [0x00000000] */ 0x11983dc0, 0xd0020827, // shl r0, elem_num, 3;
/* [0x00000008] */ 0x15827d80, 0x10020027, // mov ra0, unif # vector count
/* [0x00000010] */ 0x0c827180, 0x10020067, // add ra1, r0, unif; # base address
/* [0x00000018] */ 0x15827d80, 0x100200e7, // mov ra3, unif; # target address
/* [0x00000020] */ 0x0000007c, 0xe0021027, // mov rb0, 16*2*4-4
// :next
/* [0x00000028] */ 0x00001200, 0xe00208a7, // mov r2, vpm_setup(16, 1, v32(0,0))
/* [0x00000030] */ 0x0c9a7580, 0x10021c67, // add vw_setup, r2, elem_num;
/* [0x00000038] */ 0x8c044df6, 0xd0024078, // add ra1, ra1, 4; mov t0s, ra1
/* [0x00000040] */ 0x009e7000, 0xa00009e7, // ldtmu0
/* [0x00000048] */ 0x8c040df6, 0x10024078, // add ra1, ra1, rb0; mov t0s, ra1
/* [0x00000050] */ 0x159e7900, 0xa0020827, // mov r0, r4; ldtmu0
/* [0x00000058] */ 0x159e7900, 0x10020867, // mov r1, r4
/* [0x00000060] */ 0x019e7040, 0x10020c27, // fadd vpm, r0, r1 # Y0
/* [0x00000068] */ 0x029e7040, 0x10020c27, // fsub vpm, r0, r1
/* [0x00000070] */ 0x039e7040, 0x10020c27, // fmin vpm, r0, r1
/* [0x00000078] */ 0x049e7040, 0x10020c27, // fmax vpm, r0, r1
/* [0x00000080] */ 0x059e7040, 0x10020c27, // fminabs vpm, r0, r1 # Y4
/* [0x00000088] */ 0x069e7040, 0x10020c27, // fmaxabs vpm, r0, r1
/* [0x00000090] */ 0x079e7240, 0x10020c27, // ftoi vpm, r1
/* [0x00000098] */ 0x089e7240, 0x10020c27, // itof vpm, r1
/* [0x000000a0] */ 0x0c9e7040, 0x10020c27, // add vpm, r0, r1 # Y8
/* [0x000000a8] */ 0x0d9e7040, 0x10020c27, // sub vpm, r0, r1
/* [0x000000b0] */ 0x0e9e7040, 0x10020c27, // shr vpm, r0, r1
/* [0x000000b8] */ 0x0f9e7040, 0x10020c27, // asr vpm, r0, r1
/* [0x000000c0] */ 0x109e7040, 0x10020c27, // ror vpm, r0, r1 # Y12
/* [0x000000c8] */ 0x119e7040, 0x10020c27, // shl vpm, r0, r1
/* [0x000000d0] */ 0x129e7040, 0x10020c27, // min vpm, r0, r1
/* [0x000000d8] */ 0x139e7040, 0x10020c27, // max vpm, r0, r1
/* [0x000000e0] */ 0x149e7040, 0x10020c27, // and vpm, r0, r1 # Y16
/* [0x000000e8] */ 0x159e7040, 0x10020c27, // or vpm, r0, r1
/* [0x000000f0] */ 0x169e7040, 0x10020c27, // xor vpm, r0, r1
/* [0x000000f8] */ 0x179e7240, 0x10020c27, // not vpm, r1
/* [0x00000100] */ 0x189e7240, 0x10020c27, // clz vpm, r1 # Y20
/* [0x00000108] */ 0x209e7001, 0x100049f0, // fmul vpm, r0, r1
/* [0x00000110] */ 0x409e7001, 0x100049f0, // mul24 vpm, r0, r1
/* [0x00000118] */ 0x609e7001, 0x100049f0, // v8muld vpm, r0, r1
/* [0x00000120] */ 0x809e7001, 0x100049f0, // v8min vpm, r0, r1 # Y24
/* [0x00000128] */ 0xa09e7001, 0x100049f0, // v8max vpm, r0, r1
/* [0x00000130] */ 0xc09e7001, 0x100049f0, // v8adds vpm, r0, r1
/* [0x00000138] */ 0xe09e7001, 0x100049f0, // v8subs vpm, r0, r1
/* [0x00000140] */ 0x009e7040, 0x10020c27, // .int 0x009e7040, 0x10020c27 # Anop, Y28
/* [0x00000148] */ 0x099e7040, 0x10020c27, // .int 0x099e7040, 0x10020c27 # AddOp 9
/* [0x00000150] */ 0x0a9e7040, 0x10020c27, // .int 0x0a9e7040, 0x10020c27 # AddOp 10
/* [0x00000158] */ 0x0b9e7040, 0x10020c27, // .int 0x0b9e7040, 0x10020c27 # AddOp 11
/* [0x00000160] */ 0x199e7040, 0x10020c27, // .int 0x199e7040, 0x10020c27 # AddOp 25, Y32
/* [0x00000168] */ 0x1a9e7040, 0x10020c27, // .int 0x1a9e7040, 0x10020c27 # AddOp 26
/* [0x00000170] */ 0x1b9e7040, 0x10020c27, // .int 0x1b9e7040, 0x10020c27 # AddOp 27
/* [0x00000178] */ 0x1c9e7040, 0x10020c27, // .int 0x1c9e7040, 0x10020c27 # AddOp 28
/* [0x00000180] */ 0x1d9e7040, 0x10020c27, // .int 0x1d9e7040, 0x10020c27 # AddOp 29, Y36
/* [0x00000188] */ 0x009e7001, 0x100049f0, // .int 0x009e7001, 0x100049f0 # Mnop
/* [0x00000190] */ 0x209e7001, 0x11b049f0, // fmul vpm.8abcds, r0, r1
/* [0x00000198] */ 0x409e7001, 0x11b049f0, // mul24 vpm.8abcds, r0, r1
/* [0x000001a0] */ 0x609e7001, 0x11b049f0, // v8muld vpm.8abcds, r0, r1 # Y40
/* [0x000001a8] */ 0x809e7001, 0x11b049f0, // v8min vpm.8abcds, r0, r1
/* [0x000001b0] */ 0xc09e7001, 0x11b049f0, // v8adds vpm.8abcds, r0, r1
/* [0x000001b8] */ 0xe09e7001, 0x11b049f0, // v8subs vpm.8abcds, r0, r1
/* [0x000001c0] */ 0x0c9e7000, 0x10820427, // add ra16.32s, r0, r0
/* [0x000001c8] */ 0x009e7000, 0x100009e7, // nop
/* [0x000001d0] */ 0x15427d80, 0x10020c27, // mov vpm, ra16
/* [0x000001d8] */ 0x870e6ff6, 0xd00248a1, // mov r2, 16*4; mov r1, ra3
/* [0x000001e0] */ 0xc0000074, 0xe0021c67, // mov vw_setup, vdw_setup_1((RES_COUNT-16)*4)
/* [0x000001e8] */ 0x88104000, 0xe0021c67, // mov vw_setup, vdw_setup_0(16, 16, dma_h32(16*i,0))
/* [0x000001f0] */ 0x8c9e7289, 0x10024872, // add r1, r1, r2; mov vw_addr, r1
/* [0x000001f8] */ 0x159f2fc0, 0x100009e7, // mov -, vw_wait
/* [0x00000200] */ 0xc0000074, 0xe0021c67, // .endr
/* [0x00000208] */ 0x88104800, 0xe0021c67, // mov vw_setup, vdw_setup_1(((RES_COUNT-1)&-16)*4)
/* [0x00000210] */ 0x8c9e7289, 0x10024872, // mov vw_setup, vdw_setup_0(16, ((RES_COUNT-1)&15)+1, dma_h32((RES_COUNT-1)&-16,0))
/* [0x00000218] */ 0x159f2fc0, 0x100009e7, // mov vw_addr, r1
/* [0x00000220] */ 0xc0000080, 0xe0021c67, // mov vw_setup, vdw_setup_1(((RES_COUNT-1)&-16)*4)
/* [0x00000228] */ 0x880d5000, 0xe0021c67, // mov vw_setup, vdw_setup_0(16, ((RES_COUNT-1)&15)+1, dma_h32((RES_COUNT-1)&-16,0))
/* [0x00000230] */ 0x159e7240, 0x10021ca7, // mov vw_addr, r1
/* [0x00000238] */ 0x0d001dc0, 0xd0022027, // sub.setf ra0, ra0, 1
/* [0x00000240] */ 0xfffffdc8, 0xf03809e7, // brr.anynz -, :next
/* [0x00000248] */ 0x00000b40, 0xe00208a7, // mov r2, RES_COUNT*16*4
/* [0x00000250] */ 0x0c0e7c80, 0x100200e7, // add ra3, ra3, r2
/* [0x00000258] */ 0x159f2fc0, 0x100009e7, // mov -, vw_wait
/* [0x00000260] */ 0x11982dc0, 0xd0020827, // shl r0, elem_num, 2;
/* [0x00000268] */ 0x15827d80, 0x10020027, // mov ra0, unif # vector count
/* [0x00000270] */ 0x0c827180, 0x10020067, // add ra1, r0, unif; # base address
/* [0x00000278] */ 0x15827d80, 0x100200e7, // mov ra3, unif; # target address
/* [0x00000280] */ 0x00000040, 0xe0021027, // mov rb0, 16*4
// :next2
/* [0x00000288] */ 0x00001200, 0xe00208a7, // mov r2, vpm_setup(16, 1, v32(0,0))
/* [0x00000290] */ 0x0c9a7580, 0x10021c67, // add vw_setup, r2, elem_num;
/* [0x00000298] */ 0x8c040df6, 0x10024078, // add ra1, ra1, rb0; mov t0s, ra1
/* [0x000002a0] */ 0x009e7000, 0xa00009e7, // ldtmu0
/* [0x000002a8] */ 0x159e7900, 0x10020827, // mov r0, r4
/* [0x000002b0] */ 0x809e7000, 0x11b049f0, // nop; mov vpm.8abcds, r0
/* [0x000002b8] */ 0x809e7000, 0x114049f0, // nop; mov vpm.8as, r0
/* [0x000002c0] */ 0x809e7000, 0x115049f0, // nop; mov vpm.8bs, r0
/* [0x000002c8] */ 0x809e7000, 0x116049f0, // nop; mov vpm.8cs, r0
/* [0x000002d0] */ 0x809e7000, 0x117049f0, // nop; mov vpm.8ds, r0 # Y4
/* [0x000002d8] */ 0xbbbbbbbb, 0xe0020427, // mov ra16, 0xbbbbbbbb
/* [0x000002e0] */ 0xbbbbbbbb, 0xe0020467, // mov ra17, 0xbbbbbbbb
/* [0x000002e8] */ 0x159e7000, 0x10120427, // mov ra16.16a, r0
/* [0x000002f0] */ 0x159e7000, 0x10220467, // mov ra17.16b, r0
/* [0x000002f8] */ 0x83427036, 0x10124430, // fmin ra16.16a, r0,r0; mov vpm, ra16
/* [0x00000300] */ 0x83467036, 0x10224470, // fmin ra17.16b, r0,r0; mov vpm, ra17
/* [0x00000308] */ 0x95427036, 0x10324430, // mov ra16.8abcd, r0;  mov vpm, ra16
/* [0x00000310] */ 0x95467036, 0x10424470, // mov ra17.8a, r0;     mov vpm, ra17 # Y8
/* [0x00000318] */ 0x95427036, 0x10524430, // mov ra16.8b, r0;     mov vpm, ra16
/* [0x00000320] */ 0x95467036, 0x10624470, // mov ra17.8c, r0;     mov vpm, ra17
/* [0x00000328] */ 0x95427036, 0x10724430, // mov ra16.8d, r0;     mov vpm, ra16
/* [0x00000330] */ 0x95467036, 0x10824470, // mov ra17.32s, r0;    mov vpm, ra17 # Y12
/* [0x00000338] */ 0x95427036, 0x10924430, // mov ra16.16as, r0;   mov vpm, ra16
/* [0x00000340] */ 0x95467036, 0x10a24470, // mov ra17.16bs, r0;   mov vpm, ra17
/* [0x00000348] */ 0x83427036, 0x10924430, // fmin ra16.16as, r0,r0; mov vpm, ra16
/* [0x00000350] */ 0x83467036, 0x10a24470, // fmin ra17.16bs, r0,r0; mov vpm, ra17 # Y16
/* [0x00000358] */ 0x95427036, 0x10b24430, // mov ra16.8abcds, r0; mov vpm, ra16
/* [0x00000360] */ 0x95467036, 0x10c24470, // mov ra17.8as, r0;    mov vpm, ra17
/* [0x00000368] */ 0x95427036, 0x10d24430, // mov ra16.8bs, r0;    mov vpm, ra16
/* [0x00000370] */ 0x95467036, 0x10e24470, // mov ra17.8cs, r0;    mov vpm, ra17 # Y20
/* [0x00000378] */ 0x95427036, 0x10f24430, // mov ra16.8ds, r0;    mov vpm, ra16
/* [0x00000380] */ 0x15467d80, 0x10020c27, // mov vpm, ra17
/* [0x00000388] */ 0x15427d80, 0x10020c27, // mov vpm, ra16
/* [0x00000390] */ 0x870e6ff6, 0xd00248a1, // mov r2, 16*4; mov r1, ra3
/* [0x00000398] */ 0xc0000020, 0xe0021c67, // mov vw_setup, vdw_setup_1((RES_COUNT2-16)*4)
/* [0x000003a0] */ 0x88104000, 0xe0021c67, // mov vw_setup, vdw_setup_0(16, 16, dma_h32(16*i,0))
/* [0x000003a8] */ 0x8c9e7289, 0x10024872, // add r1, r1, r2; mov vw_addr, r1
/* [0x000003b0] */ 0x159f2fc0, 0x100009e7, // mov -, vw_wait
/* [0x000003b8] */ 0xc0000040, 0xe0021c67, // mov vw_setup, vdw_setup_1(((RES_COUNT2-1)&-16)*4)
/* [0x000003c0] */ 0x88084800, 0xe0021c67, // mov vw_setup, vdw_setup_0(16, ((RES_COUNT2-1)&15)+1, dma_h32((RES_COUNT2-1)&-16,0))
/* [0x000003c8] */ 0x159e7240, 0x10021ca7, // mov vw_addr, r1
/* [0x000003d0] */ 0x0d001dc0, 0xd0022027, // sub.setf ra0, ra0, 1
/* [0x000003d8] */ 0xfffffe90, 0xf03809e7, // brr.anynz -, :next2
/* [0x000003e0] */ 0x00000600, 0xe00208a7, // mov r2, RES_COUNT2*16*4
/* [0x000003e8] */ 0x0c0e7c80, 0x100200e7, // add ra3, ra3, r2
/* [0x000003f0] */ 0x159f2fc0, 0x100009e7, // mov -, vw_wait
// :end
/* [0x000003f8] */ 0x009e7000, 0x300009e7, // thrend
/* [0x00000400] */ 0x00000001, 0xe00209a7, // mov interrupt, 1;
/* [0x00000408] */ 0x009e7000, 0x100009e7  // nop
};
#ifdef __HIGHC__
#pragma Align_to(8, shader)
#ifdef __cplusplus
}
#endif
#endif
