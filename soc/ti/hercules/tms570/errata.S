/*--------------------------------------------------------------------------
 Copyright (C) 2009-2016 Texas Instruments Incorporated - www.ti.com


  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the
    distribution.

    Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

-------------------------------------------------------------------------*/

.section .text
.syntax unified
.cpu cortex-r4
.arm

/*-------------------------------------------------------------------------------*/
@ Work Around for Errata CORTEX-R4#57:
@
@ Errata Description:
@            Conditional VMRS APSR_Nzcv, FPSCR May Evaluate With Incorrect Flags
@ Workaround:
@            Disable out-of-order single-precision floating point
@            multiply-accumulate instruction completion

.weak _errata_CORTEXR4_57_
.type _errata_CORTEXR4_57_, %function
_errata_CORTEXR4_57_:
	push {r0}
	mrc p15, #0, r0, c15, c0, #0 @ Read Secondary Auxiliary Control Register
	orr r0, r0, #0x10000         @ Set BIT 16 (Set DOOFMACS)
	mcr p15, #0, r0, c15, c0, #0 @ Write Secondary Auxiliary Control Register
	pop {r0}
	bx lr

/*-------------------------------------------------------------------------------*/
@ Work Around for Errata CORTEX-R4#66:
@
@ Errata Description:
@            Register Corruption During A Load-Multiple Instruction At
@            an Exception Vector
@ Workaround:
@            Disable out-of-order completion for divide instructions in
@            Auxiliary Control register

.weak _errata_CORTEXR4_66_
.type _errata_CORTEXR4_66_, %function
_errata_CORTEXR4_66_:
	push {r0}
	mrc p15, #0, r0, c1, c0, #1 @ Read Auxiliary Control register
	orr r0, r0, #0x80           @ Set BIT 7 (Disable out-of-order completion
		                        @ for divide instructions.)
	mcr p15, #0, r0, c1, c0, #1 @ Write Auxiliary Control register
	pop {r0}
	bx lr

