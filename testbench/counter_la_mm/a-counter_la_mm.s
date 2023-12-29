	.file	"counter_la_mm.c"
	.option nopic
	.attribute arch, "rv32i2p0"
	.attribute unaligned_access, 0
	.attribute stack_align, 16
	.text
.Ltext0:
	.cfi_sections	.debug_frame
	.file 0 "/home/ubuntu/course-lab_D/testbench/counter_la_mm" "counter_la_mm.c"
	.align	2
	.globl	putchar
	.type	putchar, @function
putchar:
.LFB316:
	.file 1 "../../firmware/stub.c"
	.loc 1 19 1
	.cfi_startproc
.LVL0:
	addi	sp,sp,-16
	.cfi_def_cfa_offset 16
	sw	ra,12(sp)
	sw	s0,8(sp)
	.cfi_offset 1, -4
	.cfi_offset 8, -8
	mv	s0,a0
	.loc 1 20 2
	.loc 1 20 5 is_stmt 0
	li	a5,10
	beq	a0,a5,.L6
.LVL1:
.L2:
	.loc 1 22 13 discriminator 1
	li	a3,-268410880
	.loc 1 22 60 discriminator 1
	li	a4,1
.L3:
	.loc 1 22 60 is_stmt 1 discriminator 1
	.loc 1 22 13 is_stmt 0 discriminator 1
	lw	a5,-2044(a3)
	.loc 1 22 60 discriminator 1
	beq	a5,a4,.L3
	.loc 1 23 2 is_stmt 1
	.loc 1 23 50 is_stmt 0
	li	a5,-268410880
	sw	s0,-2048(a5)
	.loc 1 24 1
	lw	ra,12(sp)
	.cfi_remember_state
	.cfi_restore 1
	lw	s0,8(sp)
	.cfi_restore 8
	addi	sp,sp,16
	.cfi_def_cfa_offset 0
	jr	ra
.LVL2:
.L6:
	.cfi_restore_state
	.loc 1 21 3 is_stmt 1
	li	a0,13
.LVL3:
	call	putchar
.LVL4:
	j	.L2
	.cfi_endproc
.LFE316:
	.size	putchar, .-putchar
	.align	2
	.globl	print
	.type	print, @function
print:
.LFB317:
	.loc 1 27 1
	.cfi_startproc
.LVL5:
	addi	sp,sp,-16
	.cfi_def_cfa_offset 16
	sw	ra,12(sp)
	sw	s0,8(sp)
	.cfi_offset 1, -4
	.cfi_offset 8, -8
	mv	s0,a0
	.loc 1 28 2
	.loc 1 28 9
	lbu	a0,0(a0)
.LVL6:
	beq	a0,zero,.L7
.L9:
	.loc 1 29 3
	.loc 1 29 14 is_stmt 0
	addi	s0,s0,1
.LVL7:
	.loc 1 29 3
	call	putchar
.LVL8:
	.loc 1 28 9 is_stmt 1
	lbu	a0,0(s0)
	bne	a0,zero,.L9
.L7:
	.loc 1 30 1 is_stmt 0
	lw	ra,12(sp)
	.cfi_restore 1
	lw	s0,8(sp)
	.cfi_restore 8
.LVL9:
	addi	sp,sp,16
	.cfi_def_cfa_offset 0
	jr	ra
	.cfi_endproc
.LFE317:
	.size	print, .-print
	.align	2
	.globl	main
	.type	main, @function
main:
.LFB318:
	.file 2 "counter_la_mm.c"
	.loc 2 35 1 is_stmt 1
	.cfi_startproc
	addi	sp,sp,-16
	.cfi_def_cfa_offset 16
	sw	ra,12(sp)
	sw	s0,8(sp)
	.cfi_offset 1, -4
	.cfi_offset 8, -8
	.loc 2 36 2
	.loc 2 63 9
	.loc 2 63 43 is_stmt 0
	li	a5,637534208
	li	a4,8192
	addi	a3,a4,-2039
	sw	a3,160(a5)
	.loc 2 64 9 is_stmt 1
	.loc 2 64 43 is_stmt 0
	sw	a3,156(a5)
	.loc 2 65 9 is_stmt 1
	.loc 2 65 43 is_stmt 0
	sw	a3,152(a5)
	.loc 2 66 9 is_stmt 1
	.loc 2 66 43 is_stmt 0
	sw	a3,148(a5)
	.loc 2 67 9 is_stmt 1
	.loc 2 67 43 is_stmt 0
	sw	a3,144(a5)
	.loc 2 68 9 is_stmt 1
	.loc 2 68 43 is_stmt 0
	sw	a3,140(a5)
	.loc 2 69 9 is_stmt 1
	.loc 2 69 43 is_stmt 0
	sw	a3,136(a5)
	.loc 2 70 9 is_stmt 1
	.loc 2 70 43 is_stmt 0
	sw	a3,132(a5)
	.loc 2 71 9 is_stmt 1
	.loc 2 71 43 is_stmt 0
	sw	a3,128(a5)
	.loc 2 72 9 is_stmt 1
	.loc 2 72 43 is_stmt 0
	sw	a3,124(a5)
	.loc 2 73 9 is_stmt 1
	.loc 2 73 43 is_stmt 0
	sw	a3,120(a5)
	.loc 2 74 9 is_stmt 1
	.loc 2 74 43 is_stmt 0
	sw	a3,116(a5)
	.loc 2 75 9 is_stmt 1
	.loc 2 75 43 is_stmt 0
	sw	a3,112(a5)
	.loc 2 76 9 is_stmt 1
	.loc 2 76 43 is_stmt 0
	sw	a3,108(a5)
	.loc 2 77 9 is_stmt 1
	.loc 2 77 43 is_stmt 0
	sw	a3,104(a5)
	.loc 2 78 9 is_stmt 1
	.loc 2 78 43 is_stmt 0
	sw	a3,100(a5)
	.loc 2 80 9 is_stmt 1
	.loc 2 80 43 is_stmt 0
	addi	a4,a4,-2040
	sw	a4,96(a5)
	.loc 2 81 9 is_stmt 1
	.loc 2 81 43 is_stmt 0
	sw	a4,92(a5)
	.loc 2 82 9 is_stmt 1
	.loc 2 82 43 is_stmt 0
	sw	a4,88(a5)
	.loc 2 83 9 is_stmt 1
	.loc 2 83 43 is_stmt 0
	sw	a4,84(a5)
	.loc 2 84 9 is_stmt 1
	.loc 2 84 43 is_stmt 0
	sw	a4,80(a5)
	.loc 2 85 9 is_stmt 1
	.loc 2 85 43 is_stmt 0
	sw	a4,76(a5)
	.loc 2 86 9 is_stmt 1
	.loc 2 86 43 is_stmt 0
	sw	a4,72(a5)
	.loc 2 87 9 is_stmt 1
	.loc 2 87 43 is_stmt 0
	sw	a4,68(a5)
	.loc 2 88 9 is_stmt 1
	.loc 2 88 43 is_stmt 0
	sw	a4,64(a5)
	.loc 2 89 9 is_stmt 1
	.loc 2 89 43 is_stmt 0
	sw	a4,56(a5)
	.loc 2 90 9 is_stmt 1
	.loc 2 90 43 is_stmt 0
	sw	a4,52(a5)
	.loc 2 91 9 is_stmt 1
	.loc 2 91 43 is_stmt 0
	sw	a4,48(a5)
	.loc 2 92 9 is_stmt 1
	.loc 2 92 43 is_stmt 0
	sw	a4,44(a5)
	.loc 2 93 9 is_stmt 1
	.loc 2 93 43 is_stmt 0
	sw	a4,40(a5)
	.loc 2 94 9 is_stmt 1
	.loc 2 94 43 is_stmt 0
	sw	a4,36(a5)
	.loc 2 96 9 is_stmt 1
	.loc 2 96 43 is_stmt 0
	sw	a3,60(a5)
	.loc 2 100 2 is_stmt 1
	.loc 2 100 50 is_stmt 0
	li	a4,1
	li	a3,-268410880
	sw	a4,0(a3)
	.loc 2 103 2 is_stmt 1
	.loc 2 103 36 is_stmt 0
	sw	a4,0(a5)
	.loc 2 104 2 is_stmt 1
	.loc 2 104 10 is_stmt 0
	li	a3,637534208
.L13:
	.loc 2 104 43 is_stmt 1 discriminator 1
	.loc 2 104 10 is_stmt 0 discriminator 1
	lw	a5,0(a3)
	.loc 2 104 43 discriminator 1
	beq	a5,a4,.L13
	.loc 2 108 2 is_stmt 1
	.loc 2 108 114 is_stmt 0
	li	a5,-268423168
	sw	zero,12(a5)
	.loc 2 108 57
	sw	zero,28(a5)
	.loc 2 109 2 is_stmt 1
	.loc 2 109 112 is_stmt 0
	li	a4,-1
	sw	a4,8(a5)
	.loc 2 109 56
	sw	a4,24(a5)
	.loc 2 110 2 is_stmt 1
	.loc 2 110 112 is_stmt 0
	sw	zero,4(a5)
	.loc 2 110 56
	sw	zero,20(a5)
	.loc 2 111 2 is_stmt 1
	.loc 2 111 100 is_stmt 0
	sw	zero,0(a5)
	.loc 2 111 50
	sw	zero,16(a5)
	.loc 2 114 2 is_stmt 1
	.loc 2 114 36 is_stmt 0
	li	s0,637534208
	li	a4,-1421869056
	sw	a4,12(s0)
	.loc 2 117 2 is_stmt 1
	.loc 2 117 56 is_stmt 0
	sw	zero,56(a5)
	.loc 2 120 2 is_stmt 1
	.loc 2 120 112 is_stmt 0
	sw	zero,8(a5)
	.loc 2 120 56
	sw	zero,24(a5)
	.loc 2 130 2 is_stmt 1
	.loc 2 130 13 is_stmt 0
	call	matmul
.LVL10:
	.loc 2 131 2 is_stmt 1
	.loc 2 131 43 is_stmt 0
	lw	a5,0(a0)
	slli	a5,a5,16
	.loc 2 131 36
	sw	a5,12(s0)
	.loc 2 132 2 is_stmt 1
	.loc 2 132 47 is_stmt 0
	lw	a5,4(a0)
	slli	a5,a5,16
	.loc 2 132 36
	sw	a5,12(s0)
	.loc 2 133 2 is_stmt 1
	.loc 2 133 47 is_stmt 0
	lw	a5,8(a0)
	slli	a5,a5,16
	.loc 2 133 36
	sw	a5,12(s0)
	.loc 2 134 2 is_stmt 1
	.loc 2 134 47 is_stmt 0
	lw	a5,12(a0)
	slli	a5,a5,16
	.loc 2 134 36
	sw	a5,12(s0)
	.loc 2 135 2 is_stmt 1
	.loc 2 135 47 is_stmt 0
	lw	a5,16(a0)
	slli	a5,a5,16
	.loc 2 135 36
	sw	a5,12(s0)
	.loc 2 136 2 is_stmt 1
	.loc 2 136 47 is_stmt 0
	lw	a5,20(a0)
	slli	a5,a5,16
	.loc 2 136 36
	sw	a5,12(s0)
	.loc 2 137 2 is_stmt 1
	.loc 2 137 47 is_stmt 0
	lw	a5,24(a0)
	slli	a5,a5,16
	.loc 2 137 36
	sw	a5,12(s0)
	.loc 2 138 2 is_stmt 1
	.loc 2 138 47 is_stmt 0
	lw	a5,28(a0)
	slli	a5,a5,16
	.loc 2 138 36
	sw	a5,12(s0)
	.loc 2 139 2 is_stmt 1
	.loc 2 139 47 is_stmt 0
	lw	a5,32(a0)
	slli	a5,a5,16
	.loc 2 139 36
	sw	a5,12(s0)
	.loc 2 140 2 is_stmt 1
	.loc 2 140 47 is_stmt 0
	lw	a5,36(a0)
	slli	a5,a5,16
	.loc 2 140 36
	sw	a5,12(s0)
	.loc 2 141 2 is_stmt 1
	.loc 2 141 48 is_stmt 0
	lw	a5,40(a0)
	slli	a5,a5,16
	.loc 2 141 36
	sw	a5,12(s0)
	.loc 2 142 2 is_stmt 1
	.loc 2 142 48 is_stmt 0
	lw	a5,44(a0)
	slli	a5,a5,16
	.loc 2 142 36
	sw	a5,12(s0)
	.loc 2 143 2 is_stmt 1
	.loc 2 143 48 is_stmt 0
	lw	a5,48(a0)
	slli	a5,a5,16
	.loc 2 143 36
	sw	a5,12(s0)
	.loc 2 144 2 is_stmt 1
	.loc 2 144 48 is_stmt 0
	lw	a5,52(a0)
	slli	a5,a5,16
	.loc 2 144 36
	sw	a5,12(s0)
	.loc 2 145 2 is_stmt 1
	.loc 2 145 48 is_stmt 0
	lw	a5,56(a0)
	slli	a5,a5,16
	.loc 2 145 36
	sw	a5,12(s0)
	.loc 2 146 2 is_stmt 1
	.loc 2 146 48 is_stmt 0
	lw	a5,60(a0)
	slli	a5,a5,16
	.loc 2 146 36
	sw	a5,12(s0)
	.loc 2 150 2 is_stmt 1
	.loc 2 150 36 is_stmt 0
	li	a5,-1420754944
	sw	a5,12(s0)
	.loc 2 152 1
	lw	ra,12(sp)
	.cfi_restore 1
	lw	s0,8(sp)
	.cfi_restore 8
	addi	sp,sp,16
	.cfi_def_cfa_offset 0
	jr	ra
	.cfi_endproc
.LFE318:
	.size	main, .-main
.Letext0:
	.file 3 "/opt/riscv/lib/gcc/riscv32-unknown-elf/12.1.0/include/stdint-gcc.h"
	.section	.debug_info,"",@progbits
.Ldebug_info0:
	.4byte	0x131
	.2byte	0x5
	.byte	0x1
	.byte	0x4
	.4byte	.Ldebug_abbrev0
	.byte	0x5
	.4byte	.LASF14
	.byte	0x1d
	.4byte	.LASF0
	.4byte	.LASF1
	.4byte	.Ltext0
	.4byte	.Letext0-.Ltext0
	.4byte	.Ldebug_line0
	.byte	0x1
	.byte	0x1
	.byte	0x6
	.4byte	.LASF2
	.byte	0x1
	.byte	0x2
	.byte	0x5
	.4byte	.LASF3
	.byte	0x1
	.byte	0x4
	.byte	0x5
	.4byte	.LASF4
	.byte	0x1
	.byte	0x8
	.byte	0x5
	.4byte	.LASF5
	.byte	0x1
	.byte	0x1
	.byte	0x8
	.4byte	.LASF6
	.byte	0x1
	.byte	0x2
	.byte	0x7
	.4byte	.LASF7
	.byte	0x6
	.4byte	.LASF15
	.byte	0x3
	.byte	0x34
	.byte	0x1b
	.4byte	0x5c
	.byte	0x1
	.byte	0x4
	.byte	0x7
	.4byte	.LASF8
	.byte	0x1
	.byte	0x8
	.byte	0x7
	.4byte	.LASF9
	.byte	0x7
	.byte	0x4
	.byte	0x5
	.string	"int"
	.byte	0x1
	.byte	0x4
	.byte	0x7
	.4byte	.LASF10
	.byte	0x8
	.4byte	.LASF16
	.byte	0x2
	.byte	0x16
	.byte	0xd
	.4byte	0x8a
	.4byte	0x8a
	.byte	0x9
	.byte	0
	.byte	0x2
	.4byte	0x6a
	.byte	0xa
	.4byte	.LASF12
	.byte	0x2
	.byte	0x22
	.byte	0x6
	.4byte	.LFB318
	.4byte	.LFE318-.LFB318
	.byte	0x1
	.byte	0x9c
	.4byte	0xc7
	.byte	0xb
	.string	"j"
	.byte	0x2
	.byte	0x24
	.byte	0x6
	.4byte	0x6a
	.byte	0xc
	.string	"tmp"
	.byte	0x2
	.byte	0x82
	.byte	0x7
	.4byte	0x8a
	.byte	0x1
	.byte	0x5a
	.byte	0x3
	.4byte	.LVL10
	.4byte	0x78
	.byte	0
	.byte	0xd
	.4byte	.LASF17
	.byte	0x1
	.byte	0x1a
	.byte	0x6
	.4byte	.LFB317
	.4byte	.LFE317-.LFB317
	.byte	0x1
	.byte	0x9c
	.4byte	0xf4
	.byte	0x4
	.string	"p"
	.byte	0x1a
	.byte	0x18
	.4byte	0xf4
	.4byte	.LLST1
	.byte	0x3
	.4byte	.LVL8
	.4byte	0x105
	.byte	0
	.byte	0x2
	.4byte	0x100
	.byte	0x1
	.byte	0x1
	.byte	0x8
	.4byte	.LASF11
	.byte	0xe
	.4byte	0xf9
	.byte	0xf
	.4byte	.LASF13
	.byte	0x1
	.byte	0x12
	.byte	0x6
	.4byte	.LFB316
	.4byte	.LFE316-.LFB316
	.byte	0x1
	.byte	0x9c
	.byte	0x4
	.string	"c"
	.byte	0x12
	.byte	0x13
	.4byte	0xf9
	.4byte	.LLST0
	.byte	0x10
	.4byte	.LVL4
	.4byte	0x105
	.byte	0x11
	.byte	0x1
	.byte	0x5a
	.byte	0x1
	.byte	0x3d
	.byte	0
	.byte	0
	.byte	0
	.section	.debug_abbrev,"",@progbits
.Ldebug_abbrev0:
	.byte	0x1
	.byte	0x24
	.byte	0
	.byte	0xb
	.byte	0xb
	.byte	0x3e
	.byte	0xb
	.byte	0x3
	.byte	0xe
	.byte	0
	.byte	0
	.byte	0x2
	.byte	0xf
	.byte	0
	.byte	0xb
	.byte	0x21
	.byte	0x4
	.byte	0x49
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0x3
	.byte	0x48
	.byte	0
	.byte	0x7d
	.byte	0x1
	.byte	0x7f
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0x4
	.byte	0x5
	.byte	0
	.byte	0x3
	.byte	0x8
	.byte	0x3a
	.byte	0x21
	.byte	0x1
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0x2
	.byte	0x17
	.byte	0
	.byte	0
	.byte	0x5
	.byte	0x11
	.byte	0x1
	.byte	0x25
	.byte	0xe
	.byte	0x13
	.byte	0xb
	.byte	0x3
	.byte	0x1f
	.byte	0x1b
	.byte	0x1f
	.byte	0x11
	.byte	0x1
	.byte	0x12
	.byte	0x6
	.byte	0x10
	.byte	0x17
	.byte	0
	.byte	0
	.byte	0x6
	.byte	0x16
	.byte	0
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0x7
	.byte	0x24
	.byte	0
	.byte	0xb
	.byte	0xb
	.byte	0x3e
	.byte	0xb
	.byte	0x3
	.byte	0x8
	.byte	0
	.byte	0
	.byte	0x8
	.byte	0x2e
	.byte	0x1
	.byte	0x3f
	.byte	0x19
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0x3c
	.byte	0x19
	.byte	0x1
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0x9
	.byte	0x18
	.byte	0
	.byte	0
	.byte	0
	.byte	0xa
	.byte	0x2e
	.byte	0x1
	.byte	0x3f
	.byte	0x19
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x11
	.byte	0x1
	.byte	0x12
	.byte	0x6
	.byte	0x40
	.byte	0x18
	.byte	0x7a
	.byte	0x19
	.byte	0x1
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0xb
	.byte	0x34
	.byte	0
	.byte	0x3
	.byte	0x8
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0xc
	.byte	0x34
	.byte	0
	.byte	0x3
	.byte	0x8
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0x2
	.byte	0x18
	.byte	0
	.byte	0
	.byte	0xd
	.byte	0x2e
	.byte	0x1
	.byte	0x3f
	.byte	0x19
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x27
	.byte	0x19
	.byte	0x11
	.byte	0x1
	.byte	0x12
	.byte	0x6
	.byte	0x40
	.byte	0x18
	.byte	0x7a
	.byte	0x19
	.byte	0x1
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0xe
	.byte	0x26
	.byte	0
	.byte	0x49
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0xf
	.byte	0x2e
	.byte	0x1
	.byte	0x3f
	.byte	0x19
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x27
	.byte	0x19
	.byte	0x11
	.byte	0x1
	.byte	0x12
	.byte	0x6
	.byte	0x40
	.byte	0x18
	.byte	0x7a
	.byte	0x19
	.byte	0
	.byte	0
	.byte	0x10
	.byte	0x48
	.byte	0x1
	.byte	0x7d
	.byte	0x1
	.byte	0x7f
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0x11
	.byte	0x49
	.byte	0
	.byte	0x2
	.byte	0x18
	.byte	0x7e
	.byte	0x18
	.byte	0
	.byte	0
	.byte	0
	.section	.debug_loclists,"",@progbits
	.4byte	.Ldebug_loc3-.Ldebug_loc2
.Ldebug_loc2:
	.2byte	0x5
	.byte	0x4
	.byte	0
	.4byte	0
.Ldebug_loc0:
.LLST1:
	.byte	0x7
	.4byte	.LVL5
	.4byte	.LVL6
	.byte	0x1
	.byte	0x5a
	.byte	0x7
	.4byte	.LVL6
	.4byte	.LVL9
	.byte	0x1
	.byte	0x58
	.byte	0
.LLST0:
	.byte	0x7
	.4byte	.LVL0
	.4byte	.LVL1
	.byte	0x1
	.byte	0x5a
	.byte	0x7
	.4byte	.LVL1
	.4byte	.LVL2
	.byte	0x4
	.byte	0xa3
	.byte	0x1
	.byte	0x5a
	.byte	0x9f
	.byte	0x7
	.4byte	.LVL2
	.4byte	.LVL3
	.byte	0x1
	.byte	0x5a
	.byte	0x7
	.4byte	.LVL3
	.4byte	.LFE316
	.byte	0x4
	.byte	0xa3
	.byte	0x1
	.byte	0x5a
	.byte	0x9f
	.byte	0
.Ldebug_loc3:
	.section	.debug_aranges,"",@progbits
	.4byte	0x1c
	.2byte	0x2
	.4byte	.Ldebug_info0
	.byte	0x4
	.byte	0
	.2byte	0
	.2byte	0
	.4byte	.Ltext0
	.4byte	.Letext0-.Ltext0
	.4byte	0
	.4byte	0
	.section	.debug_line,"",@progbits
.Ldebug_line0:
	.section	.debug_str,"MS",@progbits,1
.LASF14:
	.string	"GNU C17 12.1.0 -mabi=ilp32 -mtune=rocket -misa-spec=2.2 -march=rv32i -g -O1 -ffreestanding"
.LASF16:
	.string	"matmul"
.LASF6:
	.string	"unsigned char"
.LASF8:
	.string	"long unsigned int"
.LASF7:
	.string	"short unsigned int"
.LASF13:
	.string	"putchar"
.LASF12:
	.string	"main"
.LASF10:
	.string	"unsigned int"
.LASF9:
	.string	"long long unsigned int"
.LASF5:
	.string	"long long int"
.LASF11:
	.string	"char"
.LASF17:
	.string	"print"
.LASF3:
	.string	"short int"
.LASF15:
	.string	"uint32_t"
.LASF4:
	.string	"long int"
.LASF2:
	.string	"signed char"
	.section	.debug_line_str,"MS",@progbits,1
.LASF0:
	.string	"counter_la_mm.c"
.LASF1:
	.string	"/home/ubuntu/course-lab_D/testbench/counter_la_mm"
	.ident	"GCC: (g1ea978e3066) 12.1.0"
