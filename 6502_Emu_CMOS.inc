;
; 6502 EMULATOR include
;
;     C M O S   c o r e   a.k.a. 65C02
;
; Copyright (C) 2013-2014  Klaus Dormann
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
;
.set  core_version = 831
#define core_string "65C02" ;identify CMOS core in version message

;*****************************************************************
;
; 6502 emulation disassembly table
;
;*****************************************************************
;addressing modes
.equ  op_s     = 0b0000000 ;single (implied) - TAX
.equ  op_a     = 0b0010000 ;absolute - LDA abs
.equ  op_x     = 0b0010001 ;absolute indexed X - LDA abs,x
.equ  op_y     = 0b0010010 ;absolute indexed Y - LDA abs,y
.equ  op_i     = 0b1010011 ;absolute indirect - JMP (abs)
.equ  op_m     = 0b0101000 ;immediate - LDA #
.equ  op_r     = 0b0011000 ;relative to PC - BEQ rel
.equ  op_z     = 0b0001000 ;zero page - LDA zp
.equ  op_zxi   = 0b1001100 ;zero page indexed X indirect LDA (zp,x)
.equ  op_zx    = 0b0001001 ;zero page indexed X - LDA zp,x
.equ  op_zy    = 0b0001010 ;zero page indexed Y - LDA zp,y
.equ  op_ziy   = 0b1001101 ;zero page indidrect indexed Y - LDA (zp),y
.equ  op_sa    = 0b0000110 ;single accumulator - ASL A
;  additional CMOS addressing modes
.equ  op_zi    = 0b1001011 ;zero page indidrect - LDA (zp)
.equ  op_xi    = 0b1010100 ;absolute indexed X indirect - JMP (abs,x)
.equ  op_bz    = 0b1101000 ;bitnum, zero page - RMB bit,zp
.equ  op_bzr   = 0b1101111 ;bitnum, zero page, relative - BBR bit,zp,rel
;                0b00_____  no prefix
;                0b01_____  # - immediate
;                0b10_____  ( - opening indirect
;                0b11_____  bitnumber, - SMB, RMB, BBS, BBR
;                0b__00___  no data - implied
;                0b__01___  1 Byte - zero page or immediate
;                0b__10___  2 Bytes - absolute
;                0b__11___  1 Byte relative - calculated absolute
;                0b____000  no suffix
;                0b____001  ,X - indexed X
;                0b____010  ,Y - indexed Y
;                0b____011  ) - closing indirect
;                0b____100  ,X) - indexed X indirect
;                0b____101  ),Y - indirect indexed Y
;                0b____110  A - implied accumulator
;                0b____111  ,1 Byte relative - calculated absolute (BBS, BBR)

dis_opcode: .db   "BRK",op_s
            .db   "ORA",op_zxi
            .db   "NOP",op_z
            .db   "NOP",op_s
            .db   "TSB",op_z
            .db   "ORA",op_z
            .db   "ASL",op_z
            .db   "RMB",op_bz
            .db   "PHP",op_s
            .db   "ORA",op_m
            .db   "ASL",op_sa
            .db   "NOP",op_s
            .db   "TSB",op_a
            .db   "ORA",op_a
            .db   "ASL",op_a
            .db   "BBR",op_bzr

            .db   "BPL",op_r
            .db   "ORA",op_ziy
            .db   "ORA",op_zi
            .db   "NOP",op_s
            .db   "TRB",op_z
            .db   "ORA",op_zx
            .db   "ASL",op_zx
            .db   "RMB",op_bz
            .db   "CLC",op_s
            .db   "ORA",op_y
            .db   "INC",op_sa
            .db   "NOP",op_s
            .db   "TRB",op_a
            .db   "ORA",op_x
            .db   "ASL",op_x
            .db   "BBR",op_bzr

            .db   "JSR",op_a
            .db   "AND",op_zxi
            .db   "NOP",op_z
            .db   "NOP",op_s
            .db   "BIT",op_z
            .db   "AND",op_z
            .db   "ROL",op_z
            .db   "RMB",op_bz
            .db   "PLP",op_s
            .db   "AND",op_m
            .db   "ROL",op_sa
            .db   "NOP",op_s
            .db   "BIT",op_a
            .db   "AND",op_a
            .db   "ROL",op_a
            .db   "BBR",op_bzr

            .db   "BMI",op_r
            .db   "AND",op_ziy
            .db   "AND",op_zi
            .db   "NOP",op_s
            .db   "BIT",op_zx
            .db   "AND",op_zx
            .db   "ROL",op_zx
            .db   "RMB",op_bz
            .db   "SEC",op_s
            .db   "AND",op_y
            .db   "DEC",op_sa
            .db   "NOP",op_s
            .db   "BIT",op_x
            .db   "AND",op_x
            .db   "ROL",op_x
            .db   "BBR",op_bzr

            .db   "RTI",op_s
            .db   "EOR",op_zxi
            .db   "NOP",op_z
            .db   "NOP",op_s
            .db   "NOP",op_z
            .db   "EOR",op_z
            .db   "LSR",op_z
            .db   "RMB",op_bz
            .db   "PHA",op_s
            .db   "EOR",op_m
            .db   "LSR",op_sa
            .db   "NOP",op_s
            .db   "JMP",op_a
            .db   "EOR",op_a
            .db   "LSR",op_a
            .db   "BBR",op_bzr

            .db   "BVC",op_r
            .db   "EOR",op_ziy
            .db   "EOR",op_zi
            .db   "NOP",op_s
            .db   "NOP",op_z
            .db   "EOR",op_zx
            .db   "LSR",op_zx
            .db   "RMB",op_bz
            .db   "CLI",op_s
            .db   "EOR",op_y
            .db   "PHY",op_s
            .db   "NOP",op_s
            .db   "NOP",op_a
            .db   "EOR",op_x
            .db   "LSR",op_x
            .db   "BBR",op_bzr

            .db   "RTS",op_s
            .db   "ADC",op_zxi
            .db   "NOP",op_z
            .db   "NOP",op_s
            .db   "STZ",op_z
            .db   "ADC",op_z
            .db   "ROR",op_z
            .db   "RMB",op_bz
            .db   "PLA",op_s
            .db   "ADC",op_m
            .db   "ROR",op_sa
            .db   "NOP",op_s
            .db   "JMP",op_i
            .db   "ADC",op_a
            .db   "ROR",op_a
            .db   "BBR",op_bzr

            .db   "BVS",op_r
            .db   "ADC",op_ziy
            .db   "ADC",op_zi
            .db   "NOP",op_s
            .db   "STZ",op_zx
            .db   "ADC",op_zx
            .db   "ROR",op_zx
            .db   "RMB",op_bz
            .db   "SEI",op_s
            .db   "ADC",op_y
            .db   "PLY",op_s
            .db   "NOP",op_s
            .db   "JMP",op_xi
            .db   "ADC",op_x
            .db   "ROR",op_x
            .db   "BBR",op_bzr

            .db   "BRA",op_r
            .db   "STA",op_zxi
            .db   "NOP",op_z
            .db   "NOP",op_s
            .db   "STY",op_z
            .db   "STA",op_z
            .db   "STX",op_z
            .db   "SMB",op_bz
            .db   "DEY",op_s
            .db   "BIT",op_m
            .db   "TXA",op_s
            .db   "NOP",op_s
            .db   "STY",op_a
            .db   "STA",op_a
            .db   "STX",op_a
            .db   "BBS",op_bzr

            .db   "BCC",op_r
            .db   "STA",op_ziy
            .db   "STA",op_zi
            .db   "NOP",op_s
            .db   "STY",op_zx
            .db   "STA",op_zx
            .db   "STX",op_zy
            .db   "SMB",op_bz
            .db   "TYA",op_s
            .db   "STA",op_y
            .db   "TXS",op_s
            .db   "NOP",op_s
            .db   "STZ",op_a
            .db   "STA",op_x
            .db   "STZ",op_x
            .db   "BBS",op_bzr

            .db   "LDY",op_m
            .db   "LDA",op_zxi
            .db   "LDX",op_m
            .db   "NOP",op_s
            .db   "LDY",op_z
            .db   "LDA",op_z
            .db   "LDX",op_z
            .db   "SMB",op_bz
            .db   "TAY",op_s
            .db   "LDA",op_m
            .db   "TAX",op_s
            .db   "NOP",op_s
            .db   "LDY",op_a
            .db   "LDA",op_a
            .db   "LDX",op_a
            .db   "BBS",op_bzr

            .db   "BCS",op_r
            .db   "LDA",op_ziy
            .db   "LDA",op_zi
            .db   "NOP",op_s
            .db   "LDY",op_zx
            .db   "LDA",op_zx
            .db   "LDX",op_zy
            .db   "SMB",op_bz
            .db   "CLV",op_s
            .db   "LDA",op_y
            .db   "TSX",op_s
            .db   "NOP",op_s
            .db   "LDY",op_x
            .db   "LDA",op_x
            .db   "LDX",op_y
            .db   "BBS",op_bzr

            .db   "CPY",op_m
            .db   "CMP",op_zxi
            .db   "NOP",op_z
            .db   "NOP",op_s
            .db   "CPY",op_z
            .db   "CMP",op_z
            .db   "DEC",op_z
            .db   "SMB",op_bz
            .db   "INY",op_s
            .db   "CMP",op_m
            .db   "DEX",op_s
            .db   "WAI",op_s
            .db   "CPY",op_a
            .db   "CMP",op_a
            .db   "DEC",op_a
            .db   "BBS",op_bzr

            .db   "BNE",op_r
            .db   "CMP",op_ziy
            .db   "CMP",op_zi
            .db   "NOP",op_s
            .db   "NOP",op_z
            .db   "CMP",op_zx
            .db   "DEC",op_zx
            .db   "SMB",op_bz
            .db   "CLD",op_s
            .db   "CMP",op_y
            .db   "PHX",op_s
            .db   "STP",op_s
            .db   "NOP",op_a
            .db   "CMP",op_x
            .db   "DEC",op_x
            .db   "BBS",op_bzr

            .db   "CPX",op_m
            .db   "SBC",op_zxi
            .db   "NOP",op_z
            .db   "NOP",op_s
            .db   "CPX",op_z
            .db   "SBC",op_z
            .db   "INC",op_z
            .db   "SMB",op_bz
            .db   "INX",op_s
            .db   "SBC",op_m
            .db   "NOP",op_s
            .db   "NOP",op_s
            .db   "CPX",op_a
            .db   "SBC",op_a
            .db   "INC",op_a
            .db   "BBS",op_bzr

            .db   "BEQ",op_r
            .db   "SBC",op_ziy
            .db   "SBC",op_zi
            .db   "NOP",op_s
            .db   "NOP",op_z
            .db   "SBC",op_zx
            .db   "INC",op_zx
            .db   "SMB",op_bz
            .db   "SED",op_s
            .db   "SBC",op_y
            .db   "PLX",op_s
            .db   "NOP",op_s
            .db   "NOP",op_a
            .db   "SBC",op_x
            .db   "INC",op_x
            .db   "BBS",op_bzr

;*****************************************************************
;
; 6 5 0 2   e m u l a t i o n   m a c r o s
;
;*****************************************************************

;
; op_decode
;
; opcode fetch and decode, prefetch low operand
.macro      op_decode
            out   abuslo,pcl        ;opcode fetch
            out   abushi,pch
                  sei                     ;allow emulation INT, NMI
                  adiw  pch:pcl,1         ;pc -> op low
                  cli
            wait_data_valid 4       ;240 minimum @ 16MHz
            in    opcode,dbusin

            out   abuslo,pcl        ;operand address low prefetch
            out   abushi,pch
            wait_data_valid 6       ;360ns minimum @ 16MHz 
                  adiw  pch:pcl,1         ;pc -> op high
                  ijmp                    ;execute opcode
.endmacro

;op decode after single, opcode already fetched
.macro      op_decode_single
            sei                     ;allow emulation INT, NMI
            out   abuslo,pcl        ;operand address low prefetch
            cli
            out   abushi,pch
            wait_data_valid 6       ;360ns minimum @ 16MHz 
                  adiw  pch:pcl,1         ;pc -> op high
                  ijmp                    ;execute opcode
.endmacro

; address generate relative
.macro      adgen_r
            rjmp  no_branch         ;condition false
            .ifndef BRA_r
               .equ BRA_r = pc
            .endif
            in    operand,dbusin
            clr   a                 ;expand sign bit
            sbrc  operand,7         ;negative?
            dec   a
            add   pcl,operand       ;set new PC
            adc   pch,a
no_branch:  op_decode
.endmacro

; address generate absolute
.macro      adgen_a
            in    oplow,dbusin     
            out   abuslo,pcl        ;operand high address fetch
            out   abushi,pch
                  adiw  pch:pcl,1         ;pc -> next instruction
                  cbr   flags,(1<<op_ind) ;operand address is direct
            wait_data_valid 3       ;180ns minimum @ 16MHz
            in    ophigh,dbusin
.endmacro

; address generate abs,X
.macro      adgen_x     ;@0 = op_label xxx_
            .equ  @0x = pc
            in    oplow,dbusin     
            out   abuslo,pcl        ;operand high address fetch
            out   abushi,pch
                  adiw  pch:pcl,1         ;pc -> next instruction
                  cbr   flags,(1<<op_ind) ;operand address is direct
                  add   oplow,regx
            wait_data_valid 4       ;240ns minimum @ 16MHz
            in    ophigh,dbusin
            adc   ophigh,zero
            rjmp  @0o               ;fetch operand
.endmacro

; address generate abs,Y
.macro      adgen_y     ;@0 = op_label xxx_
            .equ  @0y = pc
            in    oplow,dbusin     
            out   abuslo,pcl        ;operand high address fetch
            out   abushi,pch
                  adiw  pch:pcl,1         ;pc -> next instruction
                  cbr   flags,(1<<op_ind) ;operand address is direct
                  add   oplow,regy
            wait_data_valid 4       ;240ns minimum @ 16MHz
            in    ophigh,dbusin
            adc   ophigh,zero
            rjmp  @0o               ;fetch operand
.endmacro

;address generate (ZP) indirect
.macro      adgen_zi      ;@0 = op_label xxx_
            .equ  @0zi = pc
            in    oplow,dbusin
            out   abuslo,oplow      ;indirect low address fetch
            out   abushi,zero
                  inc   oplow
                  sbr   flags,(1<<op_ind)  ;operand address is indirect
;                  brne  pc+2
;                  rjmp  zp_wrap           ;exceeded end of zeropage
            wait_data_valid 2       ;120ns minimum @ 16MHz
            in    a,dbusin
            out   abuslo,oplow      ;indirect high address fetch
                  mov   oplow,a
            wait_data_valid 1       ;60ns minimum @ 16MHz
            in    ophigh,dbusin
            rjmp  @0o
.endmacro

;address generate (ZP,X) indexed indirect
.macro      adgen_zxi     ;@0 = op_label xxx_
            .equ  @0zxi = pc
            in    oplow,dbusin
            add   oplow,regx
            out   abuslo,oplow      ;indirect low address fetch
            out   abushi,zero
                  inc   oplow
                  sbr   flags,(1<<op_ind)  ;operand address is indirect
;                  brne  pc+2
;                  rjmp  zp_wrap           ;exceeded end of zeropage
            wait_data_valid 2       ;120ns minimum @ 16MHz
            in    a,dbusin
            out   abuslo,oplow      ;indirect high address fetch
                  mov   oplow,a
            wait_data_valid 1       ;60ns minimum @ 16MHz
            in    ophigh,dbusin
            rjmp  @0o
.endmacro

;address generate (ZP),Y indirect indexed
.macro      adgen_ziy     ;@0 = op_label xxx_
            .equ  @0ziy = pc
            in    oplow,dbusin
            out   abuslo,oplow      ;indirect low address fetch
            out   abushi,zero
                  inc   oplow
                  sbr   flags,(1<<op_ind)  ;operand address is indirect
;                  brne  pc+2
;                  rjmp  zp_wrap           ;exceeded end of zeropage
            wait_data_valid 2       ;120ns minimum @ 16MHz
            in    a,dbusin
            out   abuslo,oplow      ;indirect high address fetch
                  mov   oplow,a
                  add   oplow,regy
            wait_data_valid 2       ;120ns minimum @ 16MHz
            in    ophigh,dbusin
            adc   ophigh,zero
            rjmp  @0o
.endmacro

;address generate ZP,X and operand fetch 
.macro      adgen_zx     ;@0 = op_label xxx_
            .equ  @0zx = pc
            in    oplow,dbusin
            add   oplow,regx
            out   abuslo,oplow      ;operand fetch
            out   abushi,zero
            wait_data_valid 3       ;180ns minimum @ 16MHz
               clr   ophigh
               rjmp  @0m
.endmacro

;address generate ZP,Y and operand fetch 
.macro      adgen_zy     ;@0 = op_label xxx_
            .equ  @0zy = pc
            in    oplow,dbusin
            add   oplow,regy
            out   abuslo,oplow      ;operand fetch
            out   abushi,zero
            wait_data_valid 3       ;180ns minimum @ 16MHz
               clr   ophigh
               rjmp  @0m
.endmacro

;address generate ZP absolute and operand fetch 
.macro      adgen_z     ;@0 = op_label xxx_
            .equ  @0z = pc
            in    oplow,dbusin
            out   abuslo,oplow      ;operand fetch
            out   abushi,zero
            wait_data_valid 3       ;180ns minimum @ 16MHz
               clr   ophigh
               rjmp  @0m
.endmacro

; operand fetch from memory 
.macro      op_fetch
            out   abuslo,oplow      ;operand fetch
            out   abushi,ophigh
         .ifdef iomap
            wait_data_valid 3       ;180ns minimum @ 16MHz
                  cpi   ophigh,iomap      ;internal io?
                  brne  skip_IO
            rcall ioread            ;ioread returns data in operand
            rjmp  pc+2              ;skip reading from dbus   
         .else
            wait_data_valid 0       ;0ns minimum @ 16MHz
         .endif
skip_IO:
.endmacro

; operand fetch from memory for modify op (extended memory address lock)
.macro      op_fetch_modify
            out   abuslo,oplow      ;operand fetch
            out   abushi,ophigh
         .ifdef iomap
            wait_data_valid 3       ;180ns minimum @ 16MHz
                  cpi   ophigh,iomap      ;internal io?
                  brne  skip_IO
            rcall iomodify          ;ioread returns data in operand
            rjmp  pc+2              ;skip reading from dbus   
         .else
            wait_data_valid 0       ;0ns minimum @ 16MHz
         .endif
skip_IO:
.endmacro

; register fetch from memory 
.macro      reg_fetch ;@0 = register
            out   abuslo,oplow      ;register fetch
            out   abushi,ophigh
         .ifdef iomap
            wait_data_valid 3       ;180ns minimum @ 16MHz
                  cpi   ophigh,iomap      ;internal io?
                  brne  skip_IO
            rcall ioread            ;ioread returns data in operand
            mov   @0,operand  
            rjmp  pc+2              ;skip reading from dbus   
         .else
            wait_data_valid 0       ;0ns minimum @ 16MHz
         .endif
skip_IO:
.endmacro

;flag generating macro
.macro      flags_gen_nz            ;@0 = register
            out   sreg,stat         ;keep C
            tst   @0                ;generate NZ
            in    stat,sreg         ;save NZ, keep C
.endmacro

;IRQ enable/disable
.macro      IRQ_restore             ;sets IRQ according to I-flag
            .ifdef irq_dis_real     ;+7 if enabled, +5 if disabled
               sbrs  stat2,2        ;IRQ disabled?
               rjmp  IRQ_rest_ena 
                  ldi   a,IRQ_dis
                  out   ibus,a
                  out   timsk,one         ;only TOV0 (single step) stays enabled
                  ldi   a,0b11000         ;no USART interrupts, only RX & TX enable
               rjmp  IRQ_rest_exit
IRQ_rest_ena: 
                  ldi   a,IRQ_ena         ;enable IRQ
                  out   ibus,a
                  lds   a,timer_ena       ;enable timers interrupts
                  out   timsk,a
                  lds   a,usart_ena       ;enable usart interrupts
IRQ_rest_exit:
               out   ucsrb,a
            .else
               ldi   a,IRQ_ena
               sbrc  stat2,2           ;disable if I-flag set
               ldi   a,IRQ_dis
               out   ibus,a
            .endif
.endmacro

; store modified operand to memory
.macro      store_modify            ;write to previous read address
         .ifdef iomap               ;test internal I/O
            cpi   ophigh,iomap
            breq  do_io
         .endif
         .ifdef rommap              ;test write-protect
            cpi   ophigh,rommap
            brsh  skip_store
         .endif   
            out   cbus,writemem     ;WE, ~OE
            out   dbusout,operand   ;precharge write
            out   dbusddr,allon     ;output data valid
            out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
            out   dbusddr,zero      ;data hold ends after 60ns
            out   cbus,readmem      ;OE / read mode back on
skip_store:
            op_decode               ;next instruction

         .ifdef iomap
do_io:      rjmp  iowrite
         .endif
.endmacro

;address generate store ZP,X
.macro      adgen_s_zx     ;@0 = op_label xxx_
            .equ  @0zx = pc
            in    oplow,dbusin
            add   oplow,regx
            out   abuslo,oplow      ;operand fetch
            out   abushi,zero
            rjmp  @0m
.endmacro

;address generate store ZP,Y
.macro      adgen_s_zy     ;@0 = op_label xxx_
            .equ  @0zy = pc
            in    oplow,dbusin
            add   oplow,regy
            out   abuslo,oplow      ;operand fetch
            out   abushi,zero
            rjmp  @0m
.endmacro

;address generate store ZP absolute
.macro      adgen_s_z     ;@0 = op_label xxx_
            .equ  @0z = pc
            in    oplow,dbusin
            out   abuslo,oplow
            out   abushi,zero
            rjmp  @0m
.endmacro

; store register
.macro      store_reg  ;@0 = register, @1 = zp_entry STx_m
            out   abuslo,oplow 
            out   abushi,ophigh
         .ifdef iomap               ;test internal I/O
            cpi   ophigh,iomap
            breq  do_io
         .endif    
         .ifdef rommap              ;test write-protect
            cpi   ophigh,rommap
            brsh  skip_store
         .endif   
            .equ  @1 = pc
            out   cbus,writemem     ;WE, ~OE
            out   dbusout,@0        ;precharge write
            out   dbusddr,allon     ;output data valid
            out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
            out   dbusddr,zero      ;data hold ends after 60ns
            out   cbus,readmem      ;OE / read mode back on
skip_store:
            op_decode               ;next instruction

         .ifdef iomap
do_io:      mov   operand,@0
            rjmp  iowrite
         .endif
.endmacro

;fetch zp operand for BBR and set bitmask
.macro      fetch_z_BBR    ;@0 = bit number
            in    oplow,dbusin
            out   abuslo,oplow      ;ZP operand fetch
            out   abushi,zero
            wait_data_valid 3       ;180ns minimum @ 16MHz
                  ldi   a,1<<@0           ;set bitmask
                  rjmp  BBR_o
.endmacro

;fetch zp operand for BBS and set bitmask
.macro      fetch_z_BBS    ;@0 = bit number
            in    oplow,dbusin
            out   abuslo,oplow      ;ZP operand fetch
            out   abushi,zero
            wait_data_valid 3       ;180ns minimum @ 16MHz
                  ldi   a,1<<@0           ;set bitmask
                  rjmp  BBS_o
.endmacro

;fetch zp operand for RMB and set bitmask
.macro      fetch_z_RMB    ;@0 = bit number
            in    oplow,dbusin
            out   abuslo,oplow      ;ZP operand fetch
            out   abushi,zero
            wait_data_valid 3       ;180ns minimum @ 16MHz
                  ldi   a,~(1<<@0)        ;set bitmask
                  rjmp  RMB_o
.endmacro

;fetch zp operand for SMB and set bitmask
.macro      fetch_z_SMB    ;@0 = bit number
            in    oplow,dbusin
            out   abuslo,oplow      ;ZP operand fetch
            out   abushi,zero
            wait_data_valid 3       ;180ns minimum @ 16MHz
                  ldi   a,1<<@0           ;set bitmask
                  rjmp  SMB_o
.endmacro

;*****************************************************************
;
; 6 5 0 2   e m u l a t i o n
;
;*****************************************************************
;
; ***** IRQ pin low interrupt *****
;
IRQ:
         .if defined(spi_sel) || defined(i2c_sel)
            sbrs  flags,dma_rpt     ;NMI during dma transfer?
            ifs   IRQ_dma
               cbr   flags,(1<<dma_rpt)
               sts   dma_tab+1,zl      ;save current dma memory address
               sts   dma_tab+2,zh
               sts   dma_tab+5,xl      ;save dma count in progress
               sts   dma_tab+6,xh
               cbr   flags,(1<<modify) ;unlock extended memory address after modify
               out   dbusddr,zero      ;switch back to read mode
               out   cbus,readmem      ;OE to RAM
               ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
               sbiw  pch:pcl,2         ;prepare for instruction retry
               sbrs  flags,op_ind      ;3 byte instruction if direct
            end   IRQ_dma
         .endif
            sbiw  pch:pcl,1         ;adjust PC (discard prefetch)
            out   abuslo,allon      ;new PC high fetch from irq vector
            out   abushi,allon
                  ldi   zl,0xfe     ;preload vector low  
                  ldi   a,IRQ_dis
                  out   ibus,a
                  .ifdef irq_dis_real     ;+3
                     out   timsk,one         ;only TOV0 (single step) stays enabled
                     ldi   a,0b11000         ;no USART interrupts, only RX & TX enable
                     out   ucsrb,a
                  .endif
                  ;status merge while we are waiting for data
                  mov   a,stat            ;merge status in 6502 format
                  andi  a,0b11            ;------ZC
            wait_data_valid 5       ;300ns minimum @ 16MHz
            in    ophigh,dbusin
            out   abuslo,zl         ;new PC low fetch from irq vector
                  ;status merge while we are waiting for data
                  bst   stat,2            ;<----<
                  bld   a,7               ;N-------
                  or    a,stat2           ;-V1BDI--
                  sbr   stat2,0b100       ;I=1 (IRQ disabled)
                  cbr   stat2,0b1000      ;D=0 (clear decimal mode)
            wait_data_valid 5       ;300ns minimum @ 16MHz
            in    oplow,dbusin
            
            out   abuslo,spointer   ;push pch
            out   abushi,one
            dec   spointer
            out   cbus,writemem     ;WE, ~OE
            out   dbusout,pch       ;precharge write
            out   dbusddr,allon     ;output data valid
            out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz

            out   abuslo,spointer   ;push pcl
            dec   spointer
            out   cbus,writemem     ;WE
            out   dbusout,pcl       ;write
            out   cbus,clear        ;~WE / write cycle ends 120ns/16MHz

            out   abuslo,spointer   ;push status
            dec   spointer
            out   cbus,writemem     ;WE
            out   dbusout,a         ;write
            out   cbus,clear        ;~WE / write cycle ends 120ns/16MHz
            out   dbusddr,zero      ;data hold ends after 60ns
            out   cbus,readmem      ;OE / read mode back on

            movw  pcl,oplow         ;load new PC

            ldi   a,low(ramend)     ;discard AVR return address
            out   spl,a

            op_decode               ;next instruction

;
; ***** NMI pin high to low edge interrupt *****
;
NMI:
         .if defined(spi_sel) || defined(i2c_sel)
            sbrs  flags,dma_rpt     ;NMI during dma transfer?
            ifs   NMI_dma
               cbr   flags,(1<<dma_rpt)
               sts   dma_tab+1,zl      ;save current dma memory address
               sts   dma_tab+2,zh
               sts   dma_tab+5,xl      ;save dma count in progress
               sts   dma_tab+6,xh
               cbr   flags,(1<<modify) ;unlock extended memory address after modify
               out   dbusddr,zero      ;switch back to read mode
               out   cbus,readmem      ;OE to RAM
               ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
               sbiw  pch:pcl,2         ;prepare for instruction retry
               sbrs  flags,op_ind      ;3 byte instruction if direct
            end   NMI_dma
         .endif
            sbiw  pch:pcl,1         ;adjust PC (discard prefetch)
            ldi   zl,0xfb           ;vector high first
            out   abuslo,zl         ;new PC high fetch from irq vector
            out   abushi,allon
                  ldi   zl,0xfa     ;preload vector low  
                  ldi   a,IRQ_dis
                  out   ibus,a
                  .ifdef irq_dis_real     ;+3
                     out   timsk,one         ;only TOV0 (single step) stays enabled
                     ldi   a,0b11000         ;no USART interrupts, only RX & TX enable
                     out   ucsrb,a
                  .endif
                  ;status merge while we are waiting for data
                  mov   a,stat            ;merge status in 6502 format
                  andi  a,0b11            ;------ZC
            wait_data_valid 5       ;300ns minimum @ 16MHz
            in    ophigh,dbusin
            out   abuslo,zl         ;new PC low fetch from irq vector
                  ;status merge while we are waiting for data
                  bst   stat,2            ;<----<
                  bld   a,7               ;N-------
                  or    a,stat2           ;-V1BDI--
                  sbr   stat2,0b100       ;I=1 (IRQ disabled)
                  cbr   stat2,0b1000      ;D=0 (clear decimal mode)
            wait_data_valid 5       ;300ns minimum @ 16MHz
            in    oplow,dbusin
            
            out   abuslo,spointer   ;push pch
            out   abushi,one
            dec   spointer
            out   cbus,writemem     ;WE, ~OE
            out   dbusout,pch       ;precharge write
            out   dbusddr,allon     ;output data valid
            out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz

            out   abuslo,spointer   ;push pcl
            dec   spointer
            out   cbus,writemem     ;WE
            out   dbusout,pcl       ;write
            out   cbus,clear        ;~WE / write cycle ends 120ns/16MHz

            out   abuslo,spointer   ;push status
            dec   spointer
            out   cbus,writemem     ;WE
            out   dbusout,a         ;write
            out   cbus,clear        ;~WE / write cycle ends 120ns/16MHz
            out   dbusddr,zero      ;data hold ends after 60ns
            out   cbus,readmem      ;OE / read mode back on

            movw  pcl,oplow         ;load new PC

            ldi   a,low(ramend)     ;discard AVR return address
            out   spl,a

            op_decode               ;next instruction

;
; opcode template
;
; operand adressing modes:
; a    absolute - LDA abs                     
; x    absolute indexed X - LDA abs,x
; y    absolute indexed Y - LDA abs,y
; i    absolute indirect - JMP (abs)
; s    single (implied) - TAX
; m    immediate - LDA #
; r    relative to PC - BEQ rel
; z    zero page - LDA zp
; zxi  zero page indexed X indirect - LDA (zp,x)
; zx   zero page indexed X - LDA zp,x
; zy   zero page indexed Y - LDA zp,y
; ziy  zero page indirect indexed Y - LDA (zp),y
;   additional CMOS addressing modes
; zi   zero page indirect - LDA (zp)
; xi   absolute indexed X indirect - JMP (abs,x)
; zr   zero page, relative - BBRbit zp,rel
;
; adressing mode is added as suffix to the original opcode
; examples: LDA_ziy, BEQ_r, TAX_s, JMP_a

;*****************************************************************
;
; Implied adressing instructions
;     Register, Stack
;
;*****************************************************************

; opcode:         ASL A
ASL_s:      in    opcode,dbusin     ;next opcode

            lsl   rega              ;AVR emulated OP
            in    stat,sreg         ;save NZC

            op_decode_single        ;next instruction

; opcode:         CLC
CLC_s:      in    opcode,dbusin     ;next opcode

            cbr   stat,1            ;AVR emulated OP

            op_decode_single        ;next instruction

; opcode:         CLD
CLD_s:      in    opcode,dbusin     ;next opcode

            cbr   stat2,0b1000      ;AVR emulated OP

            op_decode_single        ;next instruction

; opcode:         CLI
CLI_s:      in    opcode,dbusin     ;next opcode

            cbr   stat2,0b100
            ldi   a,IRQ_ena
            out   ibus,a
            .ifdef irq_dis_real     ;+6
               lds   a,timer_ena       ;enable timers interrupts
               out   timsk,a
               lds   a,usart_ena       ;enable usart interrupts
               out   ucsrb,a
            .endif

            op_decode_single        ;next instruction

; opcode:         CLV
CLV_s:      in    opcode,dbusin     ;next opcode

            cbr   stat2,0b1000000   ;AVR emulated OP

            op_decode_single        ;next instruction

; opcode:         DEC A
DEC_s:      in    opcode,dbusin     ;next opcode

            out   sreg,stat         ;keep C
            dec   rega              ;AVR emulated OP
            in    stat,sreg         ;save NZ, keep C

            op_decode_single        ;next instruction

; opcode:         DEX
DEX_s:      in    opcode,dbusin     ;next opcode

            out   sreg,stat         ;keep C
            dec   regx              ;AVR emulated OP
            in    stat,sreg         ;save NZ, keep C

            op_decode_single        ;next instruction

; opcode:         DEX
DEY_s:      in    opcode,dbusin     ;next opcode

            out   sreg,stat         ;keep C
            dec   regy              ;AVR emulated OP
            in    stat,sreg         ;save NZ, keep C

            op_decode_single        ;next instruction

; opcode:         INC A
INC_s:      in    opcode,dbusin     ;next opcode

            out   sreg,stat         ;keep C
            inc   rega              ;AVR emulated OP
            in    stat,sreg         ;save NZ, keep C

            op_decode_single        ;next instruction

; opcode:         INX
INX_s:      in    opcode,dbusin     ;next opcode

            out   sreg,stat         ;keep C
            inc   regx              ;AVR emulated OP
            in    stat,sreg         ;save NZ, keep C

            op_decode_single        ;next instruction

; opcode:         INY
INY_s:      in    opcode,dbusin     ;next opcode

            out   sreg,stat         ;keep C
            inc   regy              ;AVR emulated OP
            in    stat,sreg         ;save NZ, keep C

            op_decode_single        ;next instruction

; opcode:         LSR A
LSR_s:      in    opcode,dbusin     ;next opcode

            lsr   rega              ;AVR emulated OP
            in    stat,sreg         ;save NZC

            op_decode_single        ;next instruction

; opcode:         NOP
; additional NOPs in the CMOS core replace invalid opcodes
NOP38_s:    wait_ns  2000,30        ;3 byte, 8 6502 cycles
            ldi   a,10              ;burn 30 cycles
NOP_delay:  dec   a
            brne  NOP_delay

NOP34_s:    wait_ns  500,4          ;3 byte, 4 6502 cycles
            adiw  pch:pcl,1
            rjmp  NOP23_s

NOP24_s:    wait_ns  500,0          ;2 byte, 4 6502 cycles
NOP23_s:    wait_ns  500,0          ;2 byte, 3 6502 cycles
NOP22_s:    wait_ns  1000,15        ;2 byte, 2 6502 cycles

            op_decode               ;next instruction

NOP_s:      wait_ns  500,3          ;1 byte, 2 6502 cycles
NOP11_s:    wait_ns  500,11         ;1 byte, 1 6502 cycle

            in    opcode,dbusin     ;next opcode
            op_decode_single        ;next instruction

; opcode:         PHA
PHA_s:      in    opcode,dbusin     ;next opcode

            out   abuslo,spointer   ;load address from SP
            out   abushi,one
            dec   spointer
            out   cbus,writemem     ;WE, ~OE
            out   dbusout,rega      ;precharge write
            out   dbusddr,allon     ;output data valid
            out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
            out   dbusddr,zero      ;data hold ends after 60ns
            out   cbus,readmem      ;OE / read mode back on

            op_decode_single        ;next instruction

; opcode:         PHX
PHX_s:      in    opcode,dbusin     ;next opcode

            out   abuslo,spointer   ;load address from SP
            out   abushi,one
            dec   spointer
            out   cbus,writemem     ;WE, ~OE
            out   dbusout,regx      ;precharge write
            out   dbusddr,allon     ;output data valid
            out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
            out   dbusddr,zero      ;data hold ends after 60ns
            out   cbus,readmem      ;OE / read mode back on

            op_decode_single        ;next instruction

; opcode:         PHY
PHY_s:      in    opcode,dbusin     ;next opcode

            out   abuslo,spointer   ;load address from SP
            out   abushi,one
            dec   spointer
            out   cbus,writemem     ;WE, ~OE
            out   dbusout,regy      ;precharge write
            out   dbusddr,allon     ;output data valid
            out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
            out   dbusddr,zero      ;data hold ends after 60ns
            out   cbus,readmem      ;OE / read mode back on

            op_decode_single        ;next instruction

; opcode:         PHP
PHP_s:      in    opcode,dbusin     ;next opcode

            mov   a,stat            ;merge status in 6502 format
            andi  a,0b11            ;------ZC
            bst   stat,2            ;<----<
            bld   a,7               ;N-------
            or    a,stat2           ;-V1BDI--
            ori   a,0b110000        ;  ^^=1 (pushed by software)

            out   abuslo,spointer   ;load address from SP
            out   abushi,one
            dec   spointer
            out   cbus,writemem     ;WE, ~OE
            out   dbusout,a         ;precharge write
            out   dbusddr,allon     ;output data valid
            out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
            out   dbusddr,zero      ;data hold ends after 60ns
            out   cbus,readmem      ;OE / read mode back on

            op_decode_single        ;next instruction

; opcode:         PLA
PLA_s:      in    opcode,dbusin     ;next opcode

            inc   spointer          
            out   abuslo,spointer   ;load address from SP
            out   abushi,one
            wait_data_valid 0       ;0ns minimum @ 16MHz
            in    rega,dbusin
            flags_gen_nz rega

            op_decode_single        ;next instruction

; opcode:         PLX
PLX_s:      in    opcode,dbusin     ;next opcode

            inc   spointer          
            out   abuslo,spointer   ;load address from SP
            out   abushi,one
            wait_data_valid 0       ;0ns minimum @ 16MHz
            in    regx,dbusin
            flags_gen_nz regx

            op_decode_single        ;next instruction

; opcode:         PLY
PLY_s:      in    opcode,dbusin     ;next opcode

            inc   spointer          
            out   abuslo,spointer   ;load address from SP
            out   abushi,one
            wait_data_valid 0       ;0ns minimum @ 16MHz
            in    regy,dbusin
            flags_gen_nz regy

            op_decode_single        ;next instruction

; opcode:         PLP
PLP_s:      in    opcode,dbusin     ;next opcode

            inc   spointer          
            out   abuslo,spointer   ;load address from SP
            out   abushi,one
            wait_data_valid 0       ;0ns minimum @ 16MHz
            in    stat2,dbusin

            IRQ_restore

            mov   stat,stat2        ;restore AVR format
            andi  stat,0b11         ;------ZC
            bst   stat2,7           ;>---->
            bld   stat,2            ;-----N--

            andi  stat2,0b1101100   ;-V1BDI-- (Break cleared)
            ori   stat2,0b0100000   ;  ^=1 (set unused)

            op_decode_single        ;next instruction

; opcode:         ROL A
ROL_s:      in    opcode,dbusin     ;next opcode

            out   sreg,stat         ;get C
            rol   rega              ;AVR emulated OP
            in    stat,sreg         ;save NZC

            op_decode_single        ;next instruction

; opcode:         ROR A
ROR_s:      in    opcode,dbusin     ;next opcode

            out   sreg,stat         ;get C
            ror   rega              ;AVR emulated OP
            in    stat,sreg         ;save NZC

            op_decode_single        ;next instruction

; opcode:         SEC
SEC_s:      in    opcode,dbusin     ;next opcode

            sbr   stat,1            ;AVR emulated OP

            op_decode_single        ;next instruction

; opcode:         SED
SED_s:      in    opcode,dbusin     ;next opcode

            sbr   stat2,0b1000      ;AVR emulated OP

            op_decode_single        ;next instruction

; opcode:         SEI
SEI_s:      in    opcode,dbusin     ;next opcode

            sbr   stat2,0b100       ;AVR emulated OP
            ldi   a,IRQ_dis
            out   ibus,a
            .ifdef irq_dis_real     ;+3
               out   timsk,one         ;only TOV0 (single step) stays enabled
               ldi   a,0b11000         ;no USART interrupts, only RX & TX enable
               out   ucsrb,a
            .endif

            op_decode_single        ;next instruction

; opcode:         STP
STP_s:      jmp   deb_stop          ;halt execution & display registers

; opcode:         TAX
TAX_s:      in    opcode,dbusin     ;next opcode

            mov   regx,rega         ;AVR emulated OP
            flags_gen_nz regx

            op_decode_single        ;next instruction

; opcode:         TAY
TAY_s:      in    opcode,dbusin     ;next opcode

            mov   regy,rega         ;AVR emulated OP
            flags_gen_nz regy

            op_decode_single        ;next instruction

; opcode:         TSX
TSX_s:      in    opcode,dbusin     ;next opcode

            mov   regx,spointer     ;AVR emulated OP
            flags_gen_nz regx

            op_decode_single        ;next instruction

; opcode:         TXA
TXA_s:      in    opcode,dbusin     ;next opcode

            mov   rega,regx         ;AVR emulated OP
            flags_gen_nz rega

            op_decode_single        ;next instruction

; opcode:         TXS
TXS_s:      in    opcode,dbusin     ;next opcode

            mov   spointer,regx     ;AVR emulated OP
                                    ;flags are not changed 

            op_decode_single        ;next instruction

; opcode:         TYA
TYA_s:      in    opcode,dbusin     ;next opcode

            mov   rega,regy         ;AVR emulated OP
            flags_gen_nz rega

            op_decode_single        ;next instruction

; opcode:         WAI
WAI_s:      in    opcode,dbusin     ;get ready for next instruction
            out   abuslo,pcl        ;operand address low prefetch
            out   abushi,pch
                  wait_data_valid 10       ;600ns minimum @ 16MHz
            adiw  pch:pcl,1

            sbrs  stat2,2           ;if interrupts disabled
            rjmp  WAI_hold

            sbis  cbusin,3          ;no IRQ pending
            ijmp                    ;execute next opcode

         .ifdef irq_dis_real
            do    WAI_loop_dis
               sbic  cbusin,3       ;wait direct IRQ               
            loop  WAI_loop_dis
            ijmp                    ;execute next opcode
         .endif

WAI_hold:   sei                     ;wait for interrupts
            sbiw  pch:pcl,2         ;discard prefetch on interrupt
            do    WAI_loop
               sbic  cbusin,3       ;or direct IRQ               
            loop  WAI_loop
            cli
            adiw  pch:pcl,2         ;use prefetch on direct IRQ

WAI_direct: ijmp                    ;execute next opcode

;*****************************************************************
;
; Program counter
;     instructions modifying the PC
;
;*****************************************************************

; opcode:         BBR
BBR0_zr:    fetch_z_BBR  0
BBR1_zr:    fetch_z_BBR  1
BBR2_zr:    fetch_z_BBR  2
BBR3_zr:    fetch_z_BBR  3
BBR4_zr:    fetch_z_BBR  4
BBR5_zr:    fetch_z_BBR  5
BBR6_zr:    fetch_z_BBR  6
BBR7_zr:    fetch_z_BBR  7

BBR_o:      in    operand,dbusin
            out   abuslo,pcl        ;relative branch address fetch
            out   abushi,pch
                  adiw  pch:pcl,1         ;pc -> next instruction
                  and   a,operand         ;test masked bit
                  brne  BBR_n             ;skip branch if 1
            wait_data_valid 4       ;240ns minimum @ 16MHz

            in    operand,dbusin
            clr   a                 ;expand sign bit
            sbrc  operand,7         ;negative?
            dec   a
            add   pcl,operand       ;set new PC
            adc   pch,a

BBR_n:      op_decode
            
; opcode:         BBS
BBS0_zr:    fetch_z_BBS  0
BBS1_zr:    fetch_z_BBS  1
BBS2_zr:    fetch_z_BBS  2
BBS3_zr:    fetch_z_BBS  3
BBS4_zr:    fetch_z_BBS  4
BBS5_zr:    fetch_z_BBS  5
BBS6_zr:    fetch_z_BBS  6
BBS7_zr:    fetch_z_BBS  7

BBS_o:      in    operand,dbusin
            out   abuslo,pcl        ;relative branch address fetch
            out   abushi,pch
                  adiw  pch:pcl,1         ;pc -> next instruction
                  and   a,operand         ;test masked bit
                  breq  BBS_n             ;skip branch if 0
            wait_data_valid 4       ;240ns minimum @ 16MHz

            in    operand,dbusin
            clr   a                 ;expand sign bit
            sbrc  operand,7         ;negative?
            dec   a
            add   pcl,operand       ;set new PC
            adc   pch,a

BBS_n:      op_decode

; opcode:         BCC
BCC_r:      sbrc  stat,0
            adgen_r                 ;branch if skip

; opcode:         BCS
BCS_r:      sbrs  stat,0
            adgen_r                 ;branch if skip

; opcode:         BEQ
BEQ_r:      sbrs  stat,1
            adgen_r                 ;branch if skip

; opcode:         BMI
BMI_r:      sbrs  stat,2
            adgen_r                 ;branch if skip

; opcode:         BNE
BNE_r:      sbrc  stat,1
            adgen_r                 ;branch if skip

; opcode:         BPL
BPL_r:      sbrc  stat,2
            adgen_r                 ;branch if skip

; opcode:         BVC
BVC_r:      sbrc  stat2,6
            adgen_r                 ;branch if skip

; opcode:         BVS
BVS_r:      sbrs  stat2,6
            adgen_r                 ;branch if skip

; opcode:         JMP
JMP_xi:     in    oplow,dbusin      ;save low pointer to pc     
            out   abuslo,pcl        ;fetch high pointer to pc
            out   abushi,pch
                  mov   pcl,oplow
                  add   pcl,regx
            wait_data_valid 2       ;120ns minimum @ 16MHz
            in    pch,dbusin        ;point to new pc
            adc   pch,zero
            out   abuslo,pcl        ;fetch pc low  
            out   abushi,pch
            wait_data_valid 4       ;240ns minimum @ 16MHz
                  adiw  pch:pcl,1
                  rjmp  JMP_a

JMP_i:      in    oplow,dbusin      ;save low pointer to pc     
            out   abuslo,pcl        ;fetch high pointer to pc
            out   abushi,pch
                  mov   pcl,oplow
            wait_data_valid 1       ;60ns minimum @ 16MHz
            in    pch,dbusin        ;point to new pc

            out   abuslo,pcl        ;fetch pc low  
            out   abushi,pch
                  adiw  pch:pcl,1
            wait_data_valid 2       ;120ns minimum @ 16MHz

JMP_a:      in    oplow,dbusin      ;save pc low   
            out   abuslo,pcl        ;fetch pc high
            out   abushi,pch
                  mov   pcl,oplow
            wait_data_valid 1       ;60ns minimum @ 16MHz
            in    pch,dbusin        ;load new pc

            op_decode               ;next instruction

; opcode:         JSR
JSR_a:      in    oplow,dbusin      ;save new PC as operand address
            out   abuslo,pcl        ;new PC high fetch
            out   abushi,pch
                                    ;pc++ skipped
            wait_data_valid 0       ;0ns minimum @ 16MHz
            in    ophigh,dbusin
            
            out   abuslo,spointer   ;push pch
            out   abushi,one
            dec   spointer
            out   cbus,writemem     ;WE, ~OE
            out   dbusout,pch       ;precharge write
            out   dbusddr,allon     ;output data valid
            out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz

            out   abuslo,spointer   ;push pcl
            dec   spointer
            out   cbus,writemem     ;WE
            out   dbusout,pcl       ;precharge write
            out   cbus,clear        ;~WE / write cycle ends 120ns/16MHz
            out   dbusddr,zero      ;data hold ends after 60ns
            out   cbus,readmem      ;OE / read mode back on

            movw  pcl,oplow         ;load new PC

            op_decode               ;next instruction

; opcode:         RTS
RTS_s:      inc   spointer          
            out   abuslo,spointer   ;pull pcl
            out   abushi,one
                  inc   spointer          
            wait_data_valid 1       ;60ns minimum @ 16MHz
            in    pcl,dbusin
            out   abuslo,spointer   ;pull pch
            wait_data_valid 0       ;0ns minimum @ 16MHz
            in    pch,dbusin
            adiw  pch:pcl,1         ;point to next instruction

RTI_skip_WAI:
            op_decode               ;next instruction

; opcode:         RTI
RTI_s:      inc   spointer          
            out   abuslo,spointer   ;pull status
            out   abushi,one
                  inc   spointer
            wait_data_valid 1       ;60ns minimum @ 16MHz
            in    stat2,dbusin
            out   abuslo,spointer   ;pull pcl
                  inc   spointer          
                  ;status restore while we are waiting for data
                  IRQ_restore
                  mov   stat,stat2        ;restore AVR format
            wait_data_valid 6       ;360ns minimum @ 16MHz
            in    pcl,dbusin
            out   abuslo,spointer   ;pull pch
                  ;status restore while we are waiting for data
                  andi  stat,0b11         ;------ZC
                  bst   stat2,7           ;>---->
                  bld   stat,2            ;-----N--
                  andi  stat2,0b1101100   ;-V1BDI-- (Break cleared)
                  ori   stat2,0b0100000   ;  ^=1 (set unused)
            wait_data_valid 5       ;300ns minimum @ 16MHz
            in    pch,dbusin

            out   abuslo,pcl        ;opcode fetch
            out   abushi,pch
                  adiw  pch:pcl,1         ;pc -> op low
            wait_data_valid 2       ;120ns minimum @ 16MHz
            in    opcode,dbusin
            cpi   opcode,0xcb       ;wait?
            breq  RTI_skip_WAI

            op_decode_single

; opcode:         BRK
BRK_s:           ;save new PC as operand address
            out   abuslo,allon      ;new PC high fetch from irq vector
            out   abushi,allon
                  ldi   zl,0xfe     ;preload vector low   
                  ldi   a,IRQ_dis
                  out   ibus,a
                  .ifdef irq_dis_real     ;+3
                     out   timsk,one         ;only TOV0 (single step) stays enabled
                     ldi   a,0b11000         ;no USART interrupts, only RX & TX enable
                     out   ucsrb,a
                  .endif
                  ;status merge while we are waiting for data
                  mov   a,stat            ;merge status in 6502 format
                  andi  a,0b11            ;------ZC
            wait_data_valid 5       ;300ns minimum @ 16MHz
            in    ophigh,dbusin
            out   abuslo,zl         ;new PC low fetch from irq vector
                  ;status merge while we are waiting for data
                  bst   stat,2            ;<----<
                  bld   a,7               ;N-------
                  or    a,stat2           ;-V1BDI--
                  sbr   a,0b110000        ;  ^^=1 (pushed by Break)
                  sbr   stat2,0b100       ;I=1 (IRQ disabled)
                  cbr   stat2,0b1000      ;D=0 (clear decimal mode)
            wait_data_valid 6       ;360ns minimum @ 16MHz
            in    oplow,dbusin
            
            out   abuslo,spointer   ;push pch
            out   abushi,one
            dec   spointer
            out   cbus,writemem     ;WE, ~OE
            out   dbusout,pch       ;precharge write
            out   dbusddr,allon     ;output data valid
            out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz

            out   abuslo,spointer   ;push pcl
            dec   spointer
            out   cbus,writemem     ;WE
            out   dbusout,pcl       ;write
            out   cbus,clear        ;~WE / write cycle ends 120ns/16MHz

            out   abuslo,spointer   ;push status
            dec   spointer
            out   cbus,writemem     ;WE
            out   dbusout,a         ;write
            out   cbus,clear        ;~WE / write cycle ends 120ns/16MHz
            out   dbusddr,zero      ;data hold ends after 60ns
            out   cbus,readmem      ;OE / read mode back on

            movw  pcl,oplow         ;load new PC

            op_decode               ;next instruction


;
; instruction decode vector table
;
            align 8           ;align to page boundary

oc_tab:     rjmp  BRK_s       ;0x00
            rjmp  ORA_zxi 
            rjmp  NOP22_s 
            rjmp  NOP11_s 
            rjmp  TSB_z 
            rjmp  ORA_z 
            rjmp  ASL_z 
            rjmp  RMB0_z 
            rjmp  PHP_s 
            rjmp  ORA_m 
            rjmp  ASL_s 
            rjmp  NOP11_s 
            rjmp  TSB_a 
            rjmp  ORA_a 
            rjmp  ASL_a 
            rjmp  BBR0_zr 
            rjmp  BPL_r       ;0x10
            rjmp  ORA_ziy 
            rjmp  ORA_zi 
            rjmp  NOP11_s 
            rjmp  TRB_z 
            rjmp  ORA_zx 
            rjmp  ASL_zx 
            rjmp  RMB1_z 
            rjmp  CLC_s 
            rjmp  ORA_y 
            rjmp  INC_s 
            rjmp  NOP11_s 
            rjmp  TRB_a 
            rjmp  ORA_x 
            rjmp  ASL_x 
            rjmp  BBR1_zr 
            rjmp  JSR_a       ;0x20
            rjmp  AND_zxi 
            rjmp  NOP22_s 
            rjmp  NOP11_s 
            rjmp  BIT_z 
            rjmp  AND_z 
            rjmp  ROL_z 
            rjmp  RMB2_z 
            rjmp  PLP_s 
            rjmp  AND_m 
            rjmp  ROL_s 
            rjmp  NOP11_s 
            rjmp  BIT_a
            rjmp  AND_a 
            rjmp  ROL_a
            rjmp  BBR2_zr
            rjmp  BMI_r       ;0x30
            rjmp  AND_ziy 
            rjmp  AND_zi
            rjmp  NOP11_s 
            rjmp  BIT_zx 
            rjmp  AND_zx 
            rjmp  ROL_zx 
            rjmp  RMB3_z 
            rjmp  SEC_s
            rjmp  AND_y
            rjmp  DEC_s
            rjmp  NOP11_s 
            rjmp  BIT_x 
            rjmp  AND_x 
            rjmp  ROL_x
            rjmp  BBR3_zr
            rjmp  RTI_s       ;0x40
            rjmp  EOR_zxi 
            rjmp  NOP22_s 
            rjmp  NOP11_s 
            rjmp  NOP23_s 
            rjmp  EOR_z 
            rjmp  LSR_z 
            rjmp  RMB4_z 
            rjmp  PHA_s 
            rjmp  EOR_m 
            rjmp  LSR_s 
            rjmp  NOP11_s 
            rjmp  JMP_a
            rjmp  EOR_a 
            rjmp  LSR_a 
            rjmp  BBR4_zr
            rjmp  BVC_r       ;0x50
            rjmp  EOR_ziy 
            rjmp  EOR_zi 
            rjmp  NOP11_s 
            rjmp  NOP24_s 
            rjmp  EOR_zx 
            rjmp  LSR_zx 
            rjmp  RMB5_z 
            rjmp  CLI_s 
            rjmp  EOR_y 
            rjmp  PHY_s 
            rjmp  NOP11_s 
            rjmp  NOP38_s 
            rjmp  EOR_x 
            rjmp  LSR_x 
            rjmp  BBR5_zr
            rjmp  RTS_s       ;0x60
            rjmp  ADC_zxi 
            rjmp  NOP22_s
            rjmp  NOP11_s 
            rjmp  STZ_z 
            rjmp  ADC_z 
            rjmp  ROR_z 
            rjmp  RMB6_z
            rjmp  PLA_s 
            rjmp  ADC_m 
            rjmp  ROR_s 
            rjmp  NOP11_s 
            rjmp  JMP_i 
            rjmp  ADC_a 
            rjmp  ROR_a 
            rjmp  BBR6_zr 
            rjmp  BVS_r       ;0x70
            rjmp  ADC_ziy 
            rjmp  ADC_zi 
            rjmp  NOP11_s 
            rjmp  STZ_zx 
            rjmp  ADC_zx 
            rjmp  ROR_zx 
            rjmp  RMB7_z
            rjmp  SEI_s 
            rjmp  ADC_y 
            rjmp  PLY_s 
            rjmp  NOP11_s 
            rjmp  JMP_xi 
            rjmp  ADC_x 
            rjmp  ROR_x 
            rjmp  BBR7_zr
            rjmp  BRA_r       ;0x80
            rjmp  STA_zxi 
            rjmp  NOP22_s 
            rjmp  NOP11_s 
            rjmp  STY_z 
            rjmp  STA_z 
            rjmp  STX_z 
            rjmp  SMB0_z
            rjmp  DEY_s
            rjmp  BIT_m2      ;flags anomaly! 
            rjmp  TXA_s 
            rjmp  NOP11_s 
            rjmp  STY_a 
            rjmp  STA_a 
            rjmp  STX_a 
            rjmp  BBS0_zr
            rjmp  BCC_r       ;0x90
            rjmp  STA_ziy 
            rjmp  STA_zi 
            rjmp  NOP11_s 
            rjmp  STY_zx 
            rjmp  STA_zx 
            rjmp  STX_zy 
            rjmp  SMB1_z 
            rjmp  TYA_s 
            rjmp  STA_y 
            rjmp  TXS_s 
            rjmp  NOP11_s 
            rjmp  STZ_a 
            rjmp  STA_x 
            rjmp  STZ_x 
            rjmp  BBS1_zr 
            rjmp  LDY_m       ;0xa0
            rjmp  LDA_zxi 
            rjmp  LDX_m 
            rjmp  NOP11_s 
            rjmp  LDY_z 
            rjmp  LDA_z 
            rjmp  LDX_z 
            rjmp  SMB2_z 
            rjmp  TAY_s 
            rjmp  LDA_m 
            rjmp  TAX_s 
            rjmp  NOP11_s 
            rjmp  LDY_a
            rjmp  LDA_a 
            rjmp  LDX_a 
            rjmp  BBS2_zr 
            rjmp  BCS_r       ;0xb0
            rjmp  LDA_ziy 
            rjmp  LDA_zi 
            rjmp  NOP11_s 
            rjmp  LDY_zx 
            rjmp  LDA_zx 
            rjmp  LDX_zy 
            rjmp  SMB3_z 
            rjmp  CLV_s 
            rjmp  LDA_y 
            rjmp  TSX_s 
            rjmp  NOP11_s 
            rjmp  LDY_x 
            rjmp  LDA_x 
            rjmp  LDX_y 
            rjmp  BBS3_zr 
            rjmp  CPY_m       ;0xc0
            rjmp  CMP_zxi 
            rjmp  NOP22_s 
            rjmp  NOP11_s 
            rjmp  CPY_z 
            rjmp  CMP_z 
            rjmp  DEC_z 
            rjmp  SMB4_z
            rjmp  INY_s 
            rjmp  CMP_m 
            rjmp  DEX_s 
            rjmp  WAI_s 
            rjmp  CPY_a 
            rjmp  CMP_a 
            rjmp  DEC_a 
            rjmp  BBS4_zr
            rjmp  BNE_r       ;0xd0
            rjmp  CMP_ziy 
            rjmp  CMP_zi 
            rjmp  NOP11_s 
            rjmp  NOP24_s 
            rjmp  CMP_zx 
            rjmp  DEC_zx 
            rjmp  SMB5_z
            rjmp  CLD_s 
            rjmp  CMP_y 
            rjmp  PHX_s 
            rjmp  STP_s 
            rjmp  NOP34_s 
            rjmp  CMP_x
            rjmp  DEC_x 
            rjmp  BBS5_zr
            rjmp  CPX_m       ;0xe0
            rjmp  SBC_zxi 
            rjmp  NOP22_s 
            rjmp  NOP11_s 
            rjmp  CPX_z 
            rjmp  SBC_z 
            rjmp  INC_z 
            rjmp  SMB6_z
            rjmp  INX_s 
            rjmp  SBC_m 
            rjmp  NOP_s 
            rjmp  NOP11_s 
            rjmp  CPX_a 
            rjmp  SBC_a 
            rjmp  INC_a 
            rjmp  BBS6_zr
            rjmp  BEQ_r       ;0xf0
            rjmp  SBC_ziy 
            rjmp  SBC_zi 
            rjmp  NOP11_s 
            rjmp  NOP24_s 
            rjmp  SBC_zx 
            rjmp  INC_zx 
            rjmp  SMB7_z
            rjmp  SED_s 
            rjmp  SBC_y 
            rjmp  PLX_s 
            rjmp  NOP11_s 
            rjmp  NOP34_s 
            rjmp  SBC_x 
            rjmp  INC_x 
            rjmp  BBS7_zr


;*****************************************************************
;
; Memory read instructions
;     Load, arithmetic & logical
;
;*****************************************************************

; opcode:         ADC
            adgen_x     ADC_        ;abs,X
            adgen_y     ADC_        ;abs,Y
            adgen_zi    ADC_        ;(ZP) indirect
            adgen_zxi   ADC_        ;(ZP,X) indexed indirect
            adgen_ziy   ADC_        ;(ZP),Y indirect indexed
            adgen_zx    ADC_        ;ZP,X
            adgen_z     ADC_        ;ZP absolute
ADC_a:      adgen_a                 ;absolute

ADC_o:      op_fetch                ;common entry after address generate
ADC_m:      in    operand,dbusin    ;immediate & operand prefetched (z, zx, zy)

            sbrc  stat2,3           ;test decimal flag
            rjmp  ADC_d
            out   sreg,stat         ;get C
            adc   rega,operand      ;AVR emulated OP
            in    stat,sreg         ;save NZC
            bst   stat,3            ;save V
            bld   stat2,6

            op_decode               ;next instruction

ADC_d:      addi  rega,0x66         ;decimal correction
            out   sreg,stat
            adc   rega,operand
            in    stat,sreg
            brcs  ADC_d2            ;MSD
            subi  rega,0x60
            out   sreg,stat
ADC_d2:     brhs  ADC_d3            ;LSD
            subi  rega,0x06
ADC_d3:     flags_gen_nz rega       ;CMOS requires NZC to be valid in BCD mode
                                    ;V is absolutely useless in BCD mode
                                    ;so V is not valid anyway
            op_decode               ;next instruction

; opcode:         AND
            adgen_x     AND_        ;abs,X
            adgen_y     AND_        ;abs,Y
            adgen_zi    AND_        ;(ZP) indirect
            adgen_zxi   AND_        ;(ZP,X) indexed indirect
            adgen_ziy   AND_        ;(ZP),Y indirect indexed
            adgen_zx    AND_        ;ZP,X
            adgen_z     AND_        ;ZP absolute
AND_a:      adgen_a                 ;absolute

AND_o:      op_fetch                ;common entry after address generate
AND_m:      in    operand,dbusin    ;immediate & operand prefetched (z, zx, zy)

            out   sreg,stat         ;keep C
            and   rega,operand      ;AVR emulated OP
            in    stat,sreg         ;save NZ, keep C

            op_decode               ;next instruction

; opcode:         BIT
            adgen_x     BIT_        ;abs,X
            adgen_zx    BIT_        ;ZP,X
            adgen_z     BIT_        ;ZP absolute
BIT_a:      adgen_a                 ;absolute

BIT_o:      op_fetch                ;common entry after address generate
BIT_m:      in    operand,dbusin

            mov   a,rega            ;keep A
            out   sreg,stat         ;keep C
            and   a,operand         ;A & M -> Z
            in    stat,sreg
            bst   operand,6         ;M6 -> V
            bld   stat2,6
            bst   operand,7         ;M7 -> N
            bld   stat,2

            op_decode               ;next instruction

BIT_m2:     in    operand,dbusin    ;true immediate op will not modify VN

            and   operand,rega      ;A & M -> Z
            in    a,sreg
            bst   a,1               ;save Z
            bld   stat,1

            op_decode               ;next instruction


; opcode:         CMP
; a compare in a 6502 works with reversed carry opposed to an AVR
; the reason: a 6502 performs an add with complemented operand as a compare
            adgen_x     CMP_        ;abs,X
            adgen_y     CMP_        ;abs,Y
            adgen_zi    CMP_        ;(ZP) indirect
            adgen_zxi   CMP_        ;(ZP,X) indexed indirect
            adgen_ziy   CMP_        ;(ZP),Y indirect indexed
            adgen_zx    CMP_        ;ZP,X
            adgen_z     CMP_        ;ZP absolute
CMP_a:      adgen_a                 ;absolute

CMP_o:      op_fetch                ;common entry after address generate
CMP_m:      in    operand,dbusin    ;immediate & operand prefetched (z, zx, zy)

            cp    rega,operand      ;AVR emulated OP
            in    stat,sreg         ;save NZC
            eor   stat,one          ;invert C

            op_decode               ;next instruction

; opcode:         CPX
; a compare in a 6502 works with reversed carry opposed to an AVR
; the reason: a 6502 performs an add with complemented operand as a compare
            adgen_z     CPX_        ;ZP absolute
CPX_a:      adgen_a                 ;absolute

CPX_o:      op_fetch                ;common entry after address generate
CPX_m:      in    operand,dbusin    ;immediate & operand prefetched (z, zx, zy)

            cp    regx,operand      ;AVR emulated OP
            in    stat,sreg         ;save NZC
            eor   stat,one          ;invert C

            op_decode               ;next instruction

; opcode:         CPY
; a compare in a 6502 works with reversed carry opposed to an AVR
; the reason: a 6502 performs an add with complemented operand as a compare
            adgen_z     CPY_        ;ZP absolute
CPY_a:      adgen_a                 ;absolute

CPY_o:      op_fetch                ;common entry after address generate
CPY_m:      in    operand,dbusin    ;immediate & operand prefetched (z, zx, zy)

            cp    regy,operand      ;AVR emulated OP
            in    stat,sreg         ;save NZC
            eor   stat,one          ;invert C

            op_decode               ;next instruction

; opcode:         EOR
            adgen_x     EOR_        ;abs,X
            adgen_y     EOR_        ;abs,Y
            adgen_zi    EOR_        ;(ZP) indirect
            adgen_zxi   EOR_        ;(ZP,X) indexed indirect
            adgen_ziy   EOR_        ;(ZP),Y indirect indexed
            adgen_zx    EOR_        ;ZP,X
            adgen_z     EOR_            ;ZP absolute
EOR_a:      adgen_a                 ;absolute

EOR_o:      op_fetch                ;common entry after address generate
EOR_m:      in    operand,dbusin    ;immediate & operand prefetched (z, zx, zy)

            out   sreg,stat         ;keep C
            eor   rega,operand      ;AVR emulated OP
            in    stat,sreg         ;save NZ, keep C

            op_decode               ;next instruction

; opcode:         LDA
            adgen_x     LDA_        ;abs,X
            adgen_y     LDA_        ;abs,Y
            adgen_zi    LDA_        ;(ZP) indirect
            adgen_zxi   LDA_        ;(ZP,X) indexed indirect
            adgen_ziy   LDA_        ;(ZP),Y indirect indexed
            adgen_zx    LDA_        ;ZP,X
            adgen_z     LDA_        ;ZP absolute
LDA_a:      adgen_a                 ;absolute

LDA_o:      reg_fetch rega          ;common entry after address generate
LDA_m:      in    rega,dbusin       ;same for immediate

            flags_gen_nz rega

            op_decode               ;next instruction

; opcode:         LDX
            adgen_y     LDX_        ;abs,Y
            adgen_zy    LDX_        ;ZP,Y
            adgen_z     LDX_        ;ZP absolute
LDX_a:      adgen_a                 ;absolute

LDX_o:      reg_fetch regx          ;common entry after address generate
LDX_m:      in    regx,dbusin       ;same for immediate

            flags_gen_nz regx

            op_decode               ;next instruction

; opcode:         LDY
            adgen_x     LDY_        ;abs,X
            adgen_zx    LDY_        ;ZP,X
            adgen_z     LDY_        ;ZP absolute
LDY_a:      adgen_a                 ;absolute

LDY_o:      reg_fetch regy          ;common entry after address generate
LDY_m:      in    regy,dbusin       ;same for immediate

            flags_gen_nz regy

            op_decode               ;next instruction

; opcode:         ORA
            adgen_x     ORA_        ;abs,X
            adgen_y     ORA_        ;abs,Y
            adgen_zi    ORA_        ;(ZP) indirect
            adgen_zxi   ORA_        ;(ZP,X) indexed indirect
            adgen_ziy   ORA_        ;(ZP),Y indirect indexed
            adgen_zx    ORA_        ;ZP,X
            adgen_z     ORA_        ;ZP absolute
ORA_a:      adgen_a                 ;absolute

ORA_o:      op_fetch                ;common entry after address generate
ORA_m:      in    operand,dbusin    ;immediate & operand prefetched (z, zx, zy)

            out   sreg,stat         ;keep C
            or    rega,operand      ;AVR emulated OP
            in    stat,sreg         ;save NZ, keep C

            op_decode               ;next instruction

; opcode:         SBC
; subtraction in a 6502 works with reversed carry, borrow = !carry
; the reason: a 6502 performs an add with complemented operand during subtract
            adgen_x     SBC_        ;abs,X
            adgen_y     SBC_        ;abs,Y
            adgen_zi    SBC_        ;(ZP) indirect
            adgen_zxi   SBC_        ;(ZP,X) indexed indirect
            adgen_ziy   SBC_        ;(ZP),Y indirect indexed
            adgen_zx    SBC_        ;ZP,X
            adgen_z     SBC_        ;ZP absolute
SBC_a:      adgen_a                 ;absolute

SBC_o:      op_fetch                ;common entry after address generate
SBC_m:      in    operand,dbusin    ;immediate & operand prefetched (z, zx, zy)

            com   operand
            out   sreg,stat         ;get C
            adc   rega,operand      ;AVR emulated OP
            in    stat,sreg         ;save NZC
            bst   stat,3            ;save V
            bld   stat2,6

            sbrs  stat2,3           ;test decimal flag
            rjmp  SBC_d             ;skip decimal mode correction

            brcs  SBC_d2            ;MSD correction
            subi  rega,0x60
            out   sreg,stat
SBC_d2:     brhs  SBC_d3            ;LSD correction
            subi  rega,0x06
SBC_d3:     flags_gen_nz rega       ;CMOS requires NZC to be valid in BCD mode
                                    ;V is absolutely useless in BCD mode
                                    ;so V is not valid anyway
SBC_d:      op_decode               ;next instruction


;*****************************************************************
;
; Modify memory instructions
;     Shift, increment & decrement, test & (re)set bits
;
;*****************************************************************

; opcode:         ASL
            adgen_x     ASL_        ;abs,X
            adgen_zx    ASL_        ;ZP,X
            adgen_z     ASL_        ;ZP absolute
ASL_a:      adgen_a                 ;absolute

ASL_o:      op_fetch_modify         ;common entry after address generate
ASL_m:      in    operand,dbusin    ;operand prefetched (z, zx, zy)

            lsl   operand           ;AVR emulated OP
            in    stat,sreg         ;save NZC

            store_modify

; opcode:         LSR
            adgen_x     LSR_        ;abs,X
            adgen_zx    LSR_        ;ZP,X
            adgen_z     LSR_        ;ZP absolute
LSR_a:      adgen_a                 ;absolute

LSR_o:      op_fetch_modify         ;common entry after address generate
LSR_m:      in    operand,dbusin    ;operand prefetched (z, zx, zy)

            lsr   operand           ;AVR emulated OP
            in    stat,sreg         ;save NZC

            store_modify

; opcode:         ROL
            adgen_x     ROL_        ;abs,X
            adgen_zx    ROL_        ;ZP,X
            adgen_z     ROL_        ;ZP absolute
ROL_a:      adgen_a                 ;absolute

ROL_o:      op_fetch_modify         ;common entry after address generate
ROL_m:      in    operand,dbusin    ;operand prefetched (z, zx, zy)

            out   sreg,stat         ;get C
            rol   operand           ;AVR emulated OP
            in    stat,sreg         ;save NZC

            store_modify

; opcode:         ROR
            adgen_x     ROR_        ;abs,X
            adgen_zx    ROR_        ;ZP,X
            adgen_z     ROR_        ;ZP absolute
ROR_a:      adgen_a                 ;absolute

ROR_o:      op_fetch_modify         ;common entry after address generate
ROR_m:      in    operand,dbusin    ;operand prefetched (z, zx, zy)

            out   sreg,stat         ;get C
            ror   operand           ;AVR emulated OP
            in    stat,sreg         ;save NZC

            store_modify

; opcode:         DEC
            adgen_x     DEC_        ;abs,X
            adgen_zx    DEC_        ;ZP,X
            adgen_z     DEC_        ;ZP absolute
DEC_a:      adgen_a                 ;absolute

DEC_o:      op_fetch_modify         ;common entry after address generate
DEC_m:      in    operand,dbusin    ;operand prefetched (z, zx, zy)

            out   sreg,stat         ;keep C
            dec   operand           ;AVR emulated OP
            in    stat,sreg         ;save NZ, keep C

            store_modify

; opcode:         INC
            adgen_x     INC_        ;abs,X
            adgen_zx    INC_        ;ZP,X
            adgen_z     INC_        ;ZP absolute
INC_a:      adgen_a                 ;absolute

INC_o:      op_fetch_modify         ;common entry after address generate
INC_m:      in    operand,dbusin    ;operand prefetched (z, zx, zy)

            out   sreg,stat         ;keep C
            inc   operand           ;AVR emulated OP
            in    stat,sreg         ;save NZ, keep C

            store_modify

; opcode:         RMB
RMB0_z:     fetch_z_RMB 0
RMB1_z:     fetch_z_RMB 1
RMB2_z:     fetch_z_RMB 2
RMB3_z:     fetch_z_RMB 3
RMB4_z:     fetch_z_RMB 4
RMB5_z:     fetch_z_RMB 5
RMB6_z:     fetch_z_RMB 6
RMB7_z:     fetch_z_RMB 7

RMB_o:      in    operand,dbusin    ;clear bit not in bitmask
            and   operand,a
                                    ;no need to check for IO/ROM
            out   cbus,writemem     ;WE, ~OE
            out   dbusout,operand   ;precharge write
            out   dbusddr,allon     ;output data valid
            out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
            out   dbusddr,zero      ;data hold ends after 60ns
            out   cbus,readmem      ;OE / read mode back on

            op_decode               ;next instruction

; opcode:         SMB
SMB0_z:     fetch_z_SMB 0
SMB1_z:     fetch_z_SMB 1
SMB2_z:     fetch_z_SMB 2
SMB3_z:     fetch_z_SMB 3
SMB4_z:     fetch_z_SMB 4
SMB5_z:     fetch_z_SMB 5
SMB6_z:     fetch_z_SMB 6
SMB7_z:     fetch_z_SMB 7

SMB_o:      in    operand,dbusin    ;set bit in bitmask
            or    operand,a
                                    ;no need to check for IO/ROM
            out   cbus,writemem     ;WE, ~OE
            out   dbusout,operand   ;precharge write
            out   dbusddr,allon     ;output data valid
            out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
            out   dbusddr,zero      ;data hold ends after 60ns
            out   cbus,readmem      ;OE / read mode back on

            op_decode               ;next instruction

; opcode:         TRB
            adgen_z     TRB_        ;ZP absolute
TRB_a:      adgen_a                 ;absolute

TRB_o:      op_fetch_modify         ;common entry after address generate
TRB_m:      in    operand,dbusin

            mov   a,rega            ;test A bits with memory
            and   a,operand         ;A & M -> Z
            in    a,sreg            ;move only Z to flags
            bst   a,1
            bld   stat,1
            
            mov   a,rega            ;clear A bits in memory
            eor   a,allon
            and   operand,a
            
            store_modify

; opcode:         TSB
            adgen_z     TSB_        ;ZP absolute
TSB_a:      adgen_a                 ;absolute

TSB_o:      op_fetch_modify         ;common entry after address generate
TSB_m:      in    operand,dbusin

            mov   a,rega            ;test A bits with memory
            and   a,operand         ;A & M -> Z
            in    a,sreg            ;move only Z to flags
            bst   a,1
            bld   stat,1
            
            mov   a,rega            ;clear A bits in memory
            or    operand,a
            
            store_modify
            

;*****************************************************************
;
; Memory write instructions
;     Store register
;
;*****************************************************************

; opcode:         STA
            adgen_x     STA_        ;abs,X
            adgen_y     STA_        ;abs,Y
            adgen_zi    STA_        ;(ZP) indirect
            adgen_zxi   STA_        ;(ZP,X) indexed indirect
            adgen_ziy   STA_        ;(ZP),Y indirect indexed
            adgen_zx    STA_        ;ZP,X
            adgen_z     STA_        ;ZP absolute
STA_a:      adgen_a                 ;absolute

STA_o:      store_reg rega,STA_m    ;common entry after address generate

; opcode:         STX
            adgen_zy    STX_        ;ZP,y
            adgen_z     STX_        ;ZP absolute
STX_a:      adgen_a                 ;absolute

STX_o:      store_reg regx,STX_m    ;common entry after address generate

; opcode:         STY
            adgen_s_zx  STY_        ;ZP,x
            adgen_s_z   STY_        ;ZP absolute
STY_a:      adgen_a                 ;absolute

STY_o:      store_reg regy,STY_m    ;common entry after address generate

; opcode:         STZ
            adgen_x     STZ_        ;abs,X
            adgen_s_zx  STZ_        ;ZP,x
            adgen_s_z   STZ_        ;ZP absolute
STZ_a:      adgen_a                 ;absolute

STZ_o:      store_reg zero,STZ_m    ;common entry after address generate

