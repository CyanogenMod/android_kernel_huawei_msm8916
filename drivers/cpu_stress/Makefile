#
# for CA7 cpu stress test
#
obj-y	+= slt_fp0_maxpower.o slt_fp0_maxpower_asm.o \
           slt_fp1_dhry.o slt_fp1_dhry_1.o slt_fp1_dhry_2.o \
           slt_fp2_memcpyL1.o slt_fp2_memcpyL1_asm.o \
           slt_fp3_memcpyL2.o slt_fp3_memcpyL2_asm.o \
           slt_fp4_vfp_func.o slt_fp4_vfp_func_asm.o \
           slt_fp5_adv_simd_func.o slt_fp5_adv_simd_asm.o \
           slt_fp6_scu_func.o slt_fp6_scu_asm.o \
           slt_fp7_fdct.o slt_fp7_fdct_asm.o \
           slt_fp8_imdct.o slt_fp8_imdct_asm.o

KBUILD_AFLAGS	:=$(KBUILD_AFLAGS:-msoft-float=-Wa,-mfpu=neon-vfpv4)
