;***************************************************************
;
;   structured assembly macros (SAM)
;
;***************************************************************
; Copyright (C) 2010-2013  Klaus Dormann
;
; This program is free software: you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation, either version 3 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with this program.  If not, see <http://www.gnu.org/licenses/>.
;
; contact info at http://2m5.de or email K@2m5.de
;
.set  sam_version = 810
; Version: 26.12.2010
; requires a current version of AVRASM2
; see "sam_examples.asm" for usage examples
; 
;
;----------------------------------------------------------------
;
; IF-(THEN)-ELSE-END
;
;   All need a common label as the last and mostly only parameter.
;   Further if-then-else-end can be nested by asigning a different label.
;   The macros create additional labels as needed:
;     for example @0 = test creates t_test & l_test
;     test   = end of the THEN case
;     t_test = begin of the THEN case
;     l_test = end of the ELSE case
;   Condition dependent code execution:
;     true   = code after the IFxx or IFxx_far macro is executed
;     false  = code after the ELSE macro is executed, if ELSE exists
;     all    = remainder of the code is skipped up to the END macro
;
;
; IFxx(_yy) (<bit>,)<label>
;                    label  of the next matching ELSE or END macro
;             bit   = on IFBS or IFBC - bit # in SREG
;      _yy  nothing = last or only IF implying THEN
;      _far         = THEN with long code (branch would be out of range)
;      _and         = multiple conditions, immediately jumps to END/ELSE if false
;      _or          = multiple conditions, immediately jumps to THEN if true
;   xx ...          = identical to BRxx instruction used with compares or any
;                     instruction altering the SREG
;   s               = skip - used with SBIx, SBRx or CPSE
;
;
; IFxx <label> translated to the underlying IFBS or IFBC
;
; SREG.C - bit 0, carry, no carry, lower, same or higher
;
#define     ifcs        ifbs        0,
#define     ifcs_far    ifbs_far    0,
#define     ifcs_and    ifbs_and    0,
#define     ifcs_or     ifbs_or     0,
#define     ifcc        ifbc        0,
#define     ifcc_far    ifbc_far    0,
#define     ifcc_and    ifbc_and    0,
#define     ifcc_or     ifbc_or     0,
#define     iflo        ifbs        0,
#define     iflo_far    ifbs_far    0,
#define     iflo_and    ifbs_and    0,
#define     iflo_or     ifbs_or     0,
#define     ifsh        ifbc        0,
#define     ifsh_far    ifbc_far    0,
#define     ifsh_and    ifbc_and    0,
#define     ifsh_or     ifbc_or     0,
;
; SREG.Z - bit 1, equal, not equal
;
#define     ifeq        ifbs        1,
#define     ifeq_far    ifbs_far    1,
#define     ifeq_and    ifbs_and    1,
#define     ifeq_or     ifbs_or     1,
#define     ifne        ifbc        1,
#define     ifne_far    ifbc_far    1,
#define     ifne_and    ifbc_and    1,
#define     ifne_or     ifbc_or     1,
;
; SREG.N - bit 2, minus, plus
;
#define     ifmi        ifbs        2,
#define     ifmi_far    ifbs_far    2,
#define     ifmi_and    ifbs_and    2,
#define     ifmi_or     ifbs_or     2,
#define     ifpl        ifbc        2,
#define     ifpl_far    ifbc_far    2,
#define     ifpl_and    ifbc_and    2,
#define     ifpl_or     ifbc_or     2,
;
; SREG.V - bit 3, overflow, no overflow
;
#define     ifvs        ifbs        3,
#define     ifvs_far    ifbs_far    3,
#define     ifvs_and    ifbs_and    3,
#define     ifvs_or     ifbs_or     3,
#define     ifvc        ifbc        3,
#define     ifvc_far    ifbc_far    3,
#define     ifvc_and    ifbc_and    3,
#define     ifvc_or     ifbc_or     3,
;
; SREG.S - bit 4, less than (signed), greater or equal (signed)
;
#define     iflt        ifbs        4,
#define     iflt_far    ifbs_far    4,
#define     iflt_and    ifbs_and    4,
#define     iflt_or     ifbs_or     4,
#define     ifge        ifbc        4,
#define     ifge_far    ifbc_far    4,
#define     ifge_and    ifbc_and    4,
#define     ifge_or     ifbc_or     4,
;
; SREG.H - bit 5, half carry, no half carry
;
#define     ifhs        ifbs        5,
#define     ifhs_far    ifbs_far    5,
#define     ifhs_and    ifbs_and    5,
#define     ifhs_or     ifbs_or     5,
#define     ifhc        ifbc        5,
#define     ifhc_far    ifbc_far    5,
#define     ifhc_and    ifbc_and    5,
#define     ifhc_or     ifbc_or     5,
;
; SREG.T - bit 6, T flag set, T flag clear
;
#define     ifts        ifbs        6,
#define     ifts_far    ifbs_far    6,
#define     ifts_and    ifbs_and    6,
#define     ifts_or     ifbs_or     6,
#define     iftc        ifbc        6,
#define     iftc_far    ifbc_far    6,
#define     iftc_and    ifbc_and    6,
#define     iftc_or     ifbc_or     6,
;
; SREG.I - bit 7, interrupt enabled, interrupt disabled
;
#define     ifie        ifbs        7,
#define     ifie_far    ifbs_far    7,
#define     ifie_and    ifbs_and    7,
#define     ifie_or     ifbs_or     7,
#define     ifid        ifbc        7,
#define     ifid_far    ifbc_far    7,
#define     ifid_and    ifbc_and    7,
#define     ifid_or     ifbc_or     7,

;
; IFBC <bit>,<label> - if bit is clear in SREG
;
.macro      ifbc
            brbs  @0,@1
            .equ  t_@1 = PC
.endmacro
.macro      ifbc_far 
            brbc  @0,pc+2
            rjmp  @1
            .equ  t_@1 = PC
.endmacro
.macro      ifbc_and
            brbs  @0,@1
.endmacro
.macro      ifbc_or
            brbc  @0,t_@1
.endmacro
;
; IFBS <bit>,<label> - if bit is set in SREG
;
.macro      ifbs
            brbc  @0,@1
            .equ  t_@1 = PC
.endmacro
.macro      ifbs_far 
            brbs  @0,pc+2
            rjmp  @1
            .equ  t_@1 = PC
.endmacro
.macro      ifbs_and
            brbc  @0,@1
.endmacro
.macro      ifbs_or
            brbs  @0,t_@1
.endmacro
;
; IFS <label> - if skip
;
.macro      ifs
            rjmp  @0
            .equ  t_@0 = PC
.endmacro
; ifs is always far - just for completeness
.macro      ifs_far 
            rjmp  @0
            .equ  t_@0 = PC
.endmacro
.macro      ifs_and
            rjmp  @0
.endmacro
.macro      ifs_or
            rjmp  pc+2
            rjmp  t_@0
.endmacro
;
; ELSE <label>
;       label  of the matching IFxx & END macro 
;
.macro      else
            rjmp  l_@0
  .equ      @0 = PC
.endmacro
;
; END <label>
;      label  of the previous matching IFxx or ELSE macro
;
.macro      end
  .ifdef    @0
    .equ    l_@0 = PC
  .else
    .equ    @0 = PC
  .endif
.endmacro
;
;----------------------------------------------------------------
;
; DO-LOOPxx or DO-(EXIT|EXITxx)-LOOP
;
;   All need a common label as the last and mostly only parameter.
;   Further do-loops can be nested by asigning a different label.
;   The macros create an additional label as needed:
;     for example <label> = main creates x_main
;     main   = begin of loop
;     x_main = end of loop (used by EXIT)
;
;   LOOPxx can only be used at the end of the do as an immediate
;   loop, if condition xx is true similar to a loop-while statement.
;   Multiple LOOPxx can be used to loop on multiple conditions. LOOPxx
;   is suitable to construct typical FOR-NEXT loops and WHILE cases.
;
;   To be able to exit a loop at any point, you need to use the EXITxx macro.
;   This allows the construction of typical UNTIL cases. For everything else
;   including anded conditions to loop or exit and whenever LOOPxx or EXITxx
;   branches are out of range, use EXIT inside an if-block. Multiple EXITxx
;   or EXIT macros can be used and mixed in one loop. 
;
;   EXITxx or EXIT require one and only one LOOP macro (not LOOPxx) to set
;   the exit label. LOOPxx and LOOP can not be mixed.
;
;
;
; DO <label>
;     label  of the matching EXIT & LOOP or LOOPxx macros
;
.macro      do
  .equ      @0 = PC
.endmacro
;
; EXIT <label>
;       label  of the matching DO & LOOP macros
;
.macro      exit
            rjmp  x_@0
.endmacro
;
; LOOP <label>
;       label  of the matching DO & EXIT macros
;
.macro      loop
            rjmp  @0
  .equ      x_@0 = PC
.endmacro
;
;
; LOOPxx <label> translated to the underlying LOOPBS or LOOPBC
;         label  of the matching DO macro
; EXITxx <label> translated to the underlying EXITBS or EXITBC
;         label  of the matching LOOP macro
;
; SREG.C - bit 0, carry, no carry, lower, same or higher
;
#define     loopcs      loopbs     0,
#define     loopcc      loopbc     0,
#define     looplo      loopbs     0,
#define     loopsh      loopbc     0,
#define     exitcs      exitbs     0,
#define     exitcc      exitbc     0,
#define     exitlo      exitbs     0,
#define     exitsh      exitbc     0,
;
; SREG.Z - bit 1, equal, not equal
;
#define     loopeq      loopbs     1,
#define     loopne      loopbc     1,
#define     exiteq      exitbs     1,
#define     exitne      exitbc     1,
;
; SREG.N - bit 2, minus, plus
;
#define     loopmi      loopbs     2,
#define     looppl      loopbc     2,
#define     exitmi      exitbs     2,
#define     exitpl      exitbc     2,
;
; SREG.V - bit 3, overflow, no overflow
;
#define     loopvs      loopbs     3,
#define     loopvc      loopbc     3,
#define     exitvs      exitbs     3,
#define     exitvc      exitbc     3,
;
; SREG.S - bit 4, less than (signed), greater or equal (signed)
;
#define     looplt      loopbs     4,
#define     loopge      loopbc     4,
#define     exitlt      exitbs     4,
#define     exitge      exitbc     4,
;
; SREG.H - bit 5, half carry, no half carry
;
#define     loophs      loopbs     5,
#define     loophc      loopbc     5,
#define     exiths      exitbs     5,
#define     exithc      exitbc     5,
;
; SREG.T - bit 6, T flag set, T flag clear
;
#define     loopts      loopbs     6,
#define     looptc      loopbc     6,
#define     exitts      exitbs     6,
#define     exittc      exitbc     6,
;
; SREG.I - bit 7, interrupt enabled, interrupt disabled
;
#define     loopie      loopbs     7,
#define     loopid      loopbc     7,
#define     exitie      exitbs     7,
#define     exitid      exitbc     7,
;
; LOOPBC <bit>,<label> - loop if bit is clear in SREG
;
.macro      loopbc
            brbc  @0,@1
.endmacro
;
; LOOPBS <bit>,<label> - loop if bit is set in SREG
;
.macro      loopbs
            brbs  @0,@1
.endmacro
;
; EXITBC <bit>,<label> - exit if bit is clear in SREG
;
.macro      exitbc
            brbc  @0,x_@1
.endmacro
;
; EXITBS <bit>,<label> - exit if bit is set in SREG
;
.macro      exitbs
            brbs  @0,x_@1
.endmacro



