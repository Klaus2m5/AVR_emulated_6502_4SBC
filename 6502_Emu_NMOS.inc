;
; 6502 EMULATOR include
;
;     N M O S   c o r e
;
; Copyright (C) 2013-2018  Klaus Dormann
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
.set  core_version = 840
#define core_string "N6502" ;identify NMOS core in version message

;*****************************************************************
;
; 6502 emulation disassembly table
;
;*****************************************************************
;addressing modes
.equ  op_nv    = 0b0000111 ;invalid
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
;                0b00_____  no prefix
;                0b01_____  # - immediate
;                0b10_____  ( - opening indirect
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
;                0b____111  Invalid Instruction

dis_opcode: .db   "BRK",op_s
            .db   "ORA",op_zxi
            .db   "?02",op_nv
            .db   "?03",op_nv
            .db   "?04",op_nv
            .db   "ORA",op_z
            .db   "ASL",op_z
            .db   "?07",op_nv
            .db   "PHP",op_s
            .db   "ORA",op_m
            .db   "ASL",op_sa
            .db   "?0B",op_nv
            .db   "?0C",op_nv
            .db   "ORA",op_a
            .db   "ASL",op_a
            .db   "?0F",op_nv

            .db   "BPL",op_r
            .db   "ORA",op_ziy
            .db   "?12",op_nv
            .db   "?13",op_nv
            .db   "?14",op_nv
            .db   "ORA",op_zx
            .db   "ASL",op_zx
            .db   "?17",op_nv
            .db   "CLC",op_s
            .db   "ORA",op_y
            .db   "?1A",op_nv
            .db   "?1B",op_nv
            .db   "?1C",op_nv
            .db   "ORA",op_x
            .db   "ASL",op_x
            .db   "?1F",op_nv

            .db   "JSR",op_a
            .db   "AND",op_zxi
            .db   "?22",op_nv
            .db   "?23",op_nv
            .db   "BIT",op_z
            .db   "AND",op_z
            .db   "ROL",op_z
            .db   "?27",op_nv
            .db   "PLP",op_s
            .db   "AND",op_m
            .db   "ROL",op_sa
            .db   "?2B",op_nv
            .db   "BIT",op_a
            .db   "AND",op_a
            .db   "ROL",op_a
            .db   "?2F",op_nv

            .db   "BMI",op_r
            .db   "AND",op_ziy
            .db   "?32",op_nv
            .db   "?33",op_nv
            .db   "?34",op_nv
            .db   "AND",op_zx
            .db   "ROL",op_zx
            .db   "?37",op_nv
            .db   "SEC",op_s
            .db   "AND",op_y
            .db   "?3A",op_nv
            .db   "?3B",op_nv
            .db   "?3C",op_nv
            .db   "AND",op_x
            .db   "ROL",op_x
            .db   "?3F",op_nv

            .db   "RTI",op_s
            .db   "EOR",op_zxi
            .db   "?42",op_nv
            .db   "?43",op_nv
            .db   "?44",op_nv
            .db   "EOR",op_z
            .db   "LSR",op_z
            .db   "?47",op_nv
            .db   "PHA",op_s
            .db   "EOR",op_m
            .db   "LSR",op_sa
            .db   "?4B",op_nv
            .db   "JMP",op_a
            .db   "EOR",op_a
            .db   "LSR",op_a
            .db   "?4F",op_nv

            .db   "BVC",op_r
            .db   "EOR",op_ziy
            .db   "?52",op_nv
            .db   "?53",op_nv
            .db   "?54",op_nv
            .db   "EOR",op_zx
            .db   "LSR",op_zx
            .db   "?57",op_nv
            .db   "CLI",op_s
            .db   "EOR",op_y
            .db   "?5A",op_nv
            .db   "?5B",op_nv
            .db   "?5C",op_nv
            .db   "EOR",op_x
            .db   "LSR",op_x
            .db   "?5F",op_nv

            .db   "RTS",op_s
            .db   "ADC",op_zxi
            .db   "?62",op_nv
            .db   "?63",op_nv
            .db   "?64",op_nv
            .db   "ADC",op_z
            .db   "ROR",op_z
            .db   "?67",op_nv
            .db   "PLA",op_s
            .db   "ADC",op_m
            .db   "ROR",op_sa
            .db   "?6B",op_nv
            .db   "JMP",op_i
            .db   "ADC",op_a
            .db   "ROR",op_a
            .db   "?6F",op_nv

            .db   "BVS",op_r
            .db   "ADC",op_ziy
            .db   "?72",op_nv
            .db   "?73",op_nv
            .db   "?74",op_nv
            .db   "ADC",op_zx
            .db   "ROR",op_zx
            .db   "?77",op_nv
            .db   "SEI",op_s
            .db   "ADC",op_y
            .db   "?7A",op_nv
            .db   "?7B",op_nv
            .db   "?7C",op_nv
            .db   "ADC",op_x
            .db   "ROR",op_x
            .db   "?7F",op_nv

            .db   "?80",op_nv
            .db   "STA",op_zxi
            .db   "?82",op_nv
            .db   "?83",op_nv
            .db   "STY",op_z
            .db   "STA",op_z
            .db   "STX",op_z
            .db   "?87",op_nv
            .db   "DEY",op_s
            .db   "?89",op_nv
            .db   "TXA",op_s
            .db   "?8B",op_nv
            .db   "STY",op_a
            .db   "STA",op_a
            .db   "STX",op_a
            .db   "?8F",op_nv

            .db   "BCC",op_r
            .db   "STA",op_ziy
            .db   "?92",op_nv
            .db   "?93",op_nv
            .db   "STY",op_zx
            .db   "STA",op_zx
            .db   "STX",op_zy
            .db   "?97",op_nv
            .db   "TYA",op_s
            .db   "STA",op_y
            .db   "TXS",op_s
            .db   "?9B",op_nv
            .db   "?9C",op_nv
            .db   "STA",op_x
            .db   "?9E",op_nv
            .db   "?9F",op_nv

            .db   "LDY",op_m
            .db   "LDA",op_zxi
            .db   "LDX",op_m
            .db   "?A3",op_nv
            .db   "LDY",op_z
            .db   "LDA",op_z
            .db   "LDX",op_z
            .db   "?A7",op_nv
            .db   "TAY",op_s
            .db   "LDA",op_m
            .db   "TAX",op_s
            .db   "?AB",op_nv
            .db   "LDY",op_a
            .db   "LDA",op_a
            .db   "LDX",op_a
            .db   "?AF",op_nv

            .db   "BCS",op_r
            .db   "LDA",op_ziy
            .db   "?B2",op_nv
            .db   "?B3",op_nv
            .db   "LDY",op_zx
            .db   "LDA",op_zx
            .db   "LDX",op_zy
            .db   "?B7",op_nv
            .db   "CLV",op_s
            .db   "LDA",op_y
            .db   "TSX",op_s
            .db   "?BB",op_nv
            .db   "LDY",op_x
            .db   "LDA",op_x
            .db   "LDX",op_y
            .db   "?BF",op_nv

            .db   "CPY",op_m
            .db   "CMP",op_zxi
            .db   "?C2",op_nv
            .db   "?C3",op_nv
            .db   "CPY",op_z
            .db   "CMP",op_z
            .db   "DEC",op_z
            .db   "?C7",op_nv
            .db   "INY",op_s
            .db   "CMP",op_m
            .db   "DEX",op_s
            .db   "?CB",op_nv
            .db   "CPY",op_a
            .db   "CMP",op_a
            .db   "DEC",op_a
            .db   "?CF",op_nv

            .db   "BNE",op_r
            .db   "CMP",op_ziy
            .db   "?D2",op_nv
            .db   "?D3",op_nv
            .db   "?D4",op_nv
            .db   "CMP",op_zx
            .db   "DEC",op_zx
            .db   "?D7",op_nv
            .db   "CLD",op_s
            .db   "CMP",op_y
            .db   "?DA",op_nv
            .db   "?DB",op_nv
            .db   "?DC",op_nv
            .db   "CMP",op_x
            .db   "DEC",op_x
            .db   "?DF",op_nv

            .db   "CPX",op_m
            .db   "SBC",op_zxi
            .db   "?E2",op_nv
            .db   "?E3",op_nv
            .db   "CPX",op_z
            .db   "SBC",op_z
            .db   "INC",op_z
            .db   "?E7",op_nv
            .db   "INX",op_s
            .db   "SBC",op_m
            .db   "NOP",op_s
            .db   "?EB",op_nv
            .db   "CPX",op_a
            .db   "SBC",op_a
            .db   "INC",op_a
            .db   "?EF",op_nv

            .db   "BEQ",op_r
            .db   "SBC",op_ziy
            .db   "?F2",op_nv
            .db   "?F3",op_nv
            .db   "?F4",op_nv
            .db   "SBC",op_zx
            .db   "INC",op_zx
            .db   "?F7",op_nv
            .db   "SED",op_s
            .db   "SBC",op_y
            .db   "?FA",op_nv
            .db   "?FB",op_nv
            .db   "?FC",op_nv
            .db   "SBC",op_x
            .db   "INC",op_x
            .db   "?FF",op_nv

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
            out   abushi,pch        ;opcode fetch
            out   abuslo,pcl
            sei                     ;allow emulation INT, NMI
            adiw  pch:pcl,1         ;pc -> op low
            cli
            wait_data_valid 4       ;240 minimum @ 16MHz
            in    opcode,dbusin

            out   abushi,pch        ;operand address low prefetch
            out   abuslo,pcl
            wait_data_valid 6       ;360ns minimum @ 16MHz 
                  adiw  pch:pcl,1         ;pc -> op high
                  ijmp                    ;execute opcode
.endmacro

;op decode after single, opcode already fetched
.macro      op_decode_single
            sei                     ;allow emulation INT, NMI
            out   abushi,pch        ;operand address low prefetch
            out   abuslo,pcl
            cli
            wait_data_valid 6       ;360ns minimum @ 16MHz 
                  adiw  pch:pcl,1         ;pc -> op high
                  ijmp                    ;execute opcode
.endmacro

; address generate relative
.macro      adgen_r
            rjmp  no_branch         ;condition false
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
            out   abushi,pch        ;operand high address fetch
            out   abuslo,pcl
                  adiw  pch:pcl,1         ;pc -> next instruction
                  cbr   flags,(1<<op_ind) ;operand address is direct
            wait_data_valid 3       ;180ns minimum @ 16MHz
            in    ophigh,dbusin
.endmacro

; address generate abs,X
.macro      adgen_x     ;@0 = op_label xxx_
            .equ  @0x = pc
            in    oplow,dbusin     
            out   abushi,pch        ;operand high address fetch
            out   abuslo,pcl
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
            out   abushi,pch        ;operand high address fetch
            out   abuslo,pcl
                  adiw  pch:pcl,1         ;pc -> next instruction
                  cbr   flags,(1<<op_ind) ;operand address is direct
                  add   oplow,regy
            wait_data_valid 4       ;240ns minimum @ 16MHz
            in    ophigh,dbusin
            adc   ophigh,zero
            rjmp  @0o               ;fetch operand
.endmacro

;address generate (ZP,X) indexed indirect
.macro      adgen_zxi     ;@0 = op_label xxx_
            .equ  @0zxi = pc
            in    oplow,dbusin
            add   oplow,regx
            out   abushi,zero       ;indirect low address fetch
            out   abuslo,oplow
                  inc   oplow
                  sbr   flags,(1<<op_ind)  ;operand address is indirect
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
            out   abushi,zero       ;indirect low address fetch
            out   abuslo,oplow
                  inc   oplow
                  sbr   flags,(1<<op_ind)  ;operand address is indirect
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
            out   abushi,zero       ;operand fetch
            out   abuslo,oplow
            wait_data_valid 3       ;180ns minimum @ 16MHz
               clr   ophigh
               rjmp  @0m
.endmacro

;address generate ZP,Y and operand fetch 
.macro      adgen_zy     ;@0 = op_label xxx_
            .equ  @0zy = pc
            in    oplow,dbusin
            add   oplow,regy
            out   abushi,zero       ;operand fetch
            out   abuslo,oplow
            wait_data_valid 3       ;180ns minimum @ 16MHz
               clr   ophigh
               rjmp  @0m
.endmacro

;address generate ZP absolute and operand fetch 
.macro      adgen_z     ;@0 = op_label xxx_
            .equ  @0z = pc
            in    oplow,dbusin
            out   abushi,zero       ;operand fetch
            out   abuslo,oplow
            wait_data_valid 3       ;180ns minimum @ 16MHz
               clr   ophigh
               rjmp  @0m
.endmacro

; operand fetch from memory 
.macro      op_fetch
            out   abushi,ophigh     ;operand fetch
            out   abuslo,oplow 
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
            out   abushi,ophigh     ;operand fetch
            out   abuslo,oplow 
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
            out   abushi,ophigh     ;register fetch
            out   abuslo,oplow 
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
            out   abushi,zero       ;operand fetch
            out   abuslo,oplow
            rjmp  @0m
.endmacro

;address generate store ZP,Y
.macro      adgen_s_zy     ;@0 = op_label xxx_
            .equ  @0zy = pc
            in    oplow,dbusin
            add   oplow,regy
            out   abushi,zero       ;operand fetch
            out   abuslo,oplow
            rjmp  @0m
.endmacro

;address generate store ZP absolute
.macro      adgen_s_z     ;@0 = op_label xxx_
            .equ  @0z = pc
            in    oplow,dbusin
            out   abushi,zero
            out   abuslo,oplow
            rjmp  @0m
.endmacro

; store register
.macro      store_reg  ;@0 = register, @1 = zp_entry STx_m
            out   abushi,ophigh
            out   abuslo,oplow 
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

;*****************************************************************
;
; 6 5 0 2   e m u l a t i o n
;
;*****************************************************************
;
; ***** IRQ pin low interrupt *****
;
IRQ:
            rupt_dma_mmu 1          ;handle DMA save & MMU restore
            out   abushi,allon      ;new PC high fetch from irq vector
            out   abuslo,allon
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
            wait_data_valid 4       ;240ns minimum @ 16MHz
            in    oplow,dbusin
            
            out   abushi,one        ;push pch
            out   abuslo,spointer
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
            rupt_dma_mmu 1          ;handle DMA save & MMU restore
            ldi   zl,0xfb           ;vector high first
            out   abushi,allon      ;new PC high fetch from irq vector
            out   abuslo,zl   
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
            wait_data_valid 4       ;240ns minimum @ 16MHz
            in    oplow,dbusin
            
            out   abushi,one        ;push pch
            out   abuslo,spointer
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
            ldi   oc_tabh,high(oc_tab) 

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
NOP_s:      in    opcode,dbusin     ;next opcode

            wait_ns  1000,11        ;2 6502 cycles

            op_decode_single        ;next instruction

; opcode:         PHA
PHA_s:      in    opcode,dbusin     ;next opcode

            out   abushi,one        ;load address from SP
            out   abuslo,spointer
            dec   spointer
            out   cbus,writemem     ;WE, ~OE
            out   dbusout,rega      ;precharge write
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

            out   abushi,one        ;load address from SP
            out   abuslo,spointer
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
            out   abushi,one        ;load address from SP
            out   abuslo,spointer
            wait_data_valid 0       ;0ns minimum @ 16MHz
            in    rega,dbusin
            flags_gen_nz rega

            op_decode_single        ;next instruction

; opcode:         binary PLP
PLP_s:      in    opcode,dbusin     ;next opcode

            inc   spointer          
            out   abushi,one        ;load address from SP
            out   abuslo,spointer
            wait_data_valid 0       ;0ns minimum @ 16MHz
            in    stat2,dbusin

            IRQ_restore

            mov   stat,stat2        ;restore AVR format
            andi  stat,0b11         ;------ZC
            bst   stat2,7           ;>---->
            bld   stat,2            ;-----N--

            andi  stat2,0b1101100   ;-V1BDI-- (Break cleared)
            ori   stat2,0b0100000   ;  ^=1 (set unused)
            sbrc  stat2,3           ;test decimal flag
               ldi   oc_tabh,high(oc_tabd) 

            op_decode_single        ;next instruction

; opcode:         decimal PLP
DPP_s:      in    opcode,dbusin     ;next opcode

            inc   spointer          
            out   abushi,one        ;load address from SP
            out   abuslo,spointer
            wait_data_valid 0       ;0ns minimum @ 16MHz
            in    stat2,dbusin

            IRQ_restore

            mov   stat,stat2        ;restore AVR format
            andi  stat,0b11         ;------ZC
            bst   stat2,7           ;>---->
            bld   stat,2            ;-----N--

            andi  stat2,0b1101100   ;-V1BDI-- (Break cleared)
            ori   stat2,0b0100000   ;  ^=1 (set unused)
            sbrs  stat2,3           ;test decimal flag
               ldi   oc_tabh,high(oc_tab) 

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
            ldi   oc_tabh,high(oc_tabd)

            op_decode_single        ;next instruction

; opcode:         SEI
SEI_s:      in    opcode,dbusin     ;next opcode

            sbr   stat2,0b100       ;AVR emulated OP
            ldi   a,IRQ_dis         ;only NMI stays enabled
            out   ibus,a
            .ifdef irq_dis_real     ;+3
               out   timsk,one         ;only TOV0 (single step) stays enabled
               ldi   a,0b11000         ;no USART interrupts, only RX & TX enable
               out   ucsrb,a
            .endif

            op_decode_single        ;next instruction

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

;*****************************************************************
;
; Program counter
;     instructions modifying the PC
;
;*****************************************************************

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
JMP_i:      in    oplow,dbusin      ;save low pointer to pc     
            out   abushi,pch        ;fetch high pointer to pc
            out   abuslo,pcl
                  mov   pcl,oplow
            wait_data_valid 1       ;60ns minimum @ 16MHz
            in    pch,dbusin        ;point to new pc

            out   abushi,pch        ;fetch pc low  
            out   abuslo,pcl
                  adiw  pch:pcl,1
            wait_data_valid 2       ;120ns minimum @ 16MHz

JMP_a:      in    oplow,dbusin      ;save pc low   
            out   abushi,pch        ;fetch pc high
            out   abuslo,pcl
                  mov   pcl,oplow
            wait_data_valid 1       ;60ns minimum @ 16MHz
            in    pch,dbusin        ;load new pc

            op_decode               ;next instruction

; opcode:         JSR
JSR_a:      in    oplow,dbusin      ;save new PC as operand address
            out   abushi,pch        ;new PC high fetch
            out   abuslo,pcl
                                    ;pc++ skipped
            wait_data_valid 0       ;0ns minimum @ 16MHz
            in    ophigh,dbusin
            
            out   abushi,one        ;push pch
            out   abuslo,spointer
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
            out   abushi,one        ;pull pcl
            out   abuslo,spointer
                  inc   spointer          
            wait_data_valid 1       ;60ns minimum @ 16MHz
            in    pcl,dbusin
            out   abuslo,spointer   ;pull pch
            wait_data_valid 0       ;0ns minimum @ 16MHz
            in    pch,dbusin
            adiw  pch:pcl,1         ;point to next instruction

            op_decode               ;next instruction

; opcode:         binary RTI
RTI_s:      inc   spointer          
            out   abushi,one        ;pull status
            out   abuslo,spointer
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
                  sbrc  stat2,3           ;test decimal flag
                     ldi   oc_tabh,high(oc_tabd) 
            wait_data_valid 7       ;420ns minimum @ 16MHz
            in    pch,dbusin

            op_decode               ;next instruction

; opcode:         decimal RTI
DRI_s:      inc   spointer          
            out   abushi,one        ;pull status
            out   abuslo,spointer
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
                  sbrs  stat2,3           ;test decimal flag
                     ldi   oc_tabh,high(oc_tab) 
            wait_data_valid 7       ;420ns minimum @ 16MHz
            in    pch,dbusin

            op_decode               ;next instruction

; opcode:         BRK
BRK_s:           ;save new PC as operand address
            out   abushi,allon      ;new PC high fetch from irq vector
            out   abuslo,allon
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
            wait_data_valid 5       ;300ns minimum @ 16MHz
            in    oplow,dbusin
            
            out   abushi,one        ;push pch
            out   abuslo,spointer
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

oc_tab:     rjmp  BRK_s       ;0x00 binary mode
            rjmp  ORA_zxi 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  ORA_z 
            rjmp  ASL_z 
            rjmp  illegalop 
            rjmp  PHP_s 
            rjmp  ORA_m 
            rjmp  ASL_s 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  ORA_a 
            rjmp  ASL_a 
            rjmp  illegalop 
            rjmp  BPL_r       ;0x10
            rjmp  ORA_ziy 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  ORA_zx 
            rjmp  ASL_zx 
            rjmp  illegalop 
            rjmp  CLC_s 
            rjmp  ORA_y 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  ORA_x 
            rjmp  ASL_x 
            rjmp  illegalop 
            rjmp  JSR_a       ;0x20
            rjmp  AND_zxi 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  BIT_z 
            rjmp  AND_z 
            rjmp  ROL_z 
            rjmp  illegalop 
            rjmp  PLP_s 
            rjmp  AND_m 
            rjmp  ROL_s 
            rjmp  illegalop 
            rjmp  BIT_a
            rjmp  AND_a 
            rjmp  ROL_a
            rjmp  illegalop 
            rjmp  BMI_r       ;0x30
            rjmp  AND_ziy 
            rjmp  illegalop
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  AND_zx 
            rjmp  ROL_zx 
            rjmp  illegalop 
            rjmp  SEC_s
            rjmp  AND_y
            rjmp  illegalop
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  AND_x 
            rjmp  ROL_x
            rjmp  illegalop 
            rjmp  RTI_s       ;0x40
            rjmp  EOR_zxi 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  EOR_z 
            rjmp  LSR_z 
            rjmp  illegalop 
            rjmp  PHA_s 
            rjmp  EOR_m 
            rjmp  LSR_s 
            rjmp  illegalop 
            rjmp  JMP_a
            rjmp  EOR_a 
            rjmp  LSR_a 
            rjmp  illegalop 
            rjmp  BVC_r       ;0x50
            rjmp  EOR_ziy 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  EOR_zx 
            rjmp  LSR_zx 
            rjmp  illegalop 
            rjmp  CLI_s 
            rjmp  EOR_y 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  EOR_x 
            rjmp  LSR_x 
            rjmp  illegalop 
            rjmp  RTS_s       ;0x60
            rjmp  ADC_zxi 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  ADC_z 
            rjmp  ROR_z 
            rjmp  illegalop 
            rjmp  PLA_s 
            rjmp  ADC_m 
            rjmp  ROR_s 
            rjmp  illegalop 
            rjmp  JMP_i 
            rjmp  ADC_a 
            rjmp  ROR_a 
            rjmp  illegalop 
            rjmp  BVS_r       ;0x70
            rjmp  ADC_ziy 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  ADC_zx 
            rjmp  ROR_zx 
            rjmp  illegalop 
            rjmp  SEI_s 
            rjmp  ADC_y 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  ADC_x 
            rjmp  ROR_x 
            rjmp  illegalop 
            rjmp  illegalop   ;0x80
            rjmp  STA_zxi 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  STY_z 
            rjmp  STA_z 
            rjmp  STX_z 
            rjmp  illegalop 
            rjmp  DEY_s
            rjmp  illegalop 
            rjmp  TXA_s 
            rjmp  illegalop 
            rjmp  STY_a 
            rjmp  STA_a 
            rjmp  STX_a 
            rjmp  illegalop 
            rjmp  BCC_r       ;0x90
            rjmp  STA_ziy 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  STY_zx 
            rjmp  STA_zx 
            rjmp  STX_zy 
            rjmp  illegalop 
            rjmp  TYA_s 
            rjmp  STA_y 
            rjmp  TXS_s 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  STA_x 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  LDY_m       ;0xa0
            rjmp  LDA_zxi 
            rjmp  LDX_m 
            rjmp  illegalop 
            rjmp  LDY_z 
            rjmp  LDA_z 
            rjmp  LDX_z 
            rjmp  illegalop 
            rjmp  TAY_s 
            rjmp  LDA_m 
            rjmp  TAX_s 
            rjmp  illegalop 
            rjmp  LDY_a
            rjmp  LDA_a 
            rjmp  LDX_a 
            rjmp  illegalop 
            rjmp  BCS_r       ;0xb0
            rjmp  LDA_ziy 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  LDY_zx 
            rjmp  LDA_zx 
            rjmp  LDX_zy 
            rjmp  illegalop 
            rjmp  CLV_s 
            rjmp  LDA_y 
            rjmp  TSX_s 
            rjmp  illegalop 
            rjmp  LDY_x 
            rjmp  LDA_x 
            rjmp  LDX_y 
            rjmp  illegalop 
            rjmp  CPY_m       ;0xc0
            rjmp  CMP_zxi 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  CPY_z 
            rjmp  CMP_z 
            rjmp  DEC_z 
            rjmp  illegalop 
            rjmp  INY_s 
            rjmp  CMP_m 
            rjmp  DEX_s 
            rjmp  illegalop 
            rjmp  CPY_a 
            rjmp  CMP_a 
            rjmp  DEC_a 
            rjmp  illegalop 
            rjmp  BNE_r       ;0xd0
            rjmp  CMP_ziy 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  CMP_zx 
            rjmp  DEC_zx 
            rjmp  illegalop 
            rjmp  CLD_s 
            rjmp  CMP_y 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  CMP_x
            rjmp  DEC_x 
            rjmp  illegalop 
            rjmp  CPX_m       ;0xe0
            rjmp  SBC_zxi 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  CPX_z 
            rjmp  SBC_z 
            rjmp  INC_z 
            rjmp  illegalop 
            rjmp  INX_s 
            rjmp  SBC_m 
            rjmp  NOP_s 
            rjmp  illegalop 
            rjmp  CPX_a 
            rjmp  SBC_a 
            rjmp  INC_a 
            rjmp  illegalop 
            rjmp  BEQ_r       ;0xf0
            rjmp  SBC_ziy 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  SBC_zx 
            rjmp  INC_zx 
            rjmp  illegalop 
            rjmp  SED_s 
            rjmp  SBC_y 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  SBC_x 
            rjmp  INC_x 
            rjmp  illegalop 

oc_tabd:    rjmp  BRK_s       ;0x00 decimal mode
            rjmp  ORA_zxi 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  ORA_z 
            rjmp  ASL_z 
            rjmp  illegalop 
            rjmp  PHP_s 
            rjmp  ORA_m 
            rjmp  ASL_s 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  ORA_a 
            rjmp  ASL_a 
            rjmp  illegalop 
            rjmp  BPL_r       ;0x10
            rjmp  ORA_ziy 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  ORA_zx 
            rjmp  ASL_zx 
            rjmp  illegalop 
            rjmp  CLC_s 
            rjmp  ORA_y 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  ORA_x 
            rjmp  ASL_x 
            rjmp  illegalop 
            rjmp  JSR_a       ;0x20
            rjmp  AND_zxi 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  BIT_z 
            rjmp  AND_z 
            rjmp  ROL_z 
            rjmp  illegalop 
            rjmp  DPP_s 
            rjmp  AND_m 
            rjmp  ROL_s 
            rjmp  illegalop 
            rjmp  BIT_a
            rjmp  AND_a 
            rjmp  ROL_a
            rjmp  illegalop 
            rjmp  BMI_r       ;0x30
            rjmp  AND_ziy 
            rjmp  illegalop
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  AND_zx 
            rjmp  ROL_zx 
            rjmp  illegalop 
            rjmp  SEC_s
            rjmp  AND_y
            rjmp  illegalop
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  AND_x 
            rjmp  ROL_x
            rjmp  illegalop 
            rjmp  DRI_s       ;0x40
            rjmp  EOR_zxi 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  EOR_z 
            rjmp  LSR_z 
            rjmp  illegalop 
            rjmp  PHA_s 
            rjmp  EOR_m 
            rjmp  LSR_s 
            rjmp  illegalop 
            rjmp  JMP_a
            rjmp  EOR_a 
            rjmp  LSR_a 
            rjmp  illegalop 
            rjmp  BVC_r       ;0x50
            rjmp  EOR_ziy 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  EOR_zx 
            rjmp  LSR_zx 
            rjmp  illegalop 
            rjmp  CLI_s 
            rjmp  EOR_y 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  EOR_x 
            rjmp  LSR_x 
            rjmp  illegalop 
            rjmp  RTS_s       ;0x60
            rjmp  DAD_zxi 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  DAD_z 
            rjmp  ROR_z 
            rjmp  illegalop 
            rjmp  PLA_s 
            rjmp  DAD_m 
            rjmp  ROR_s 
            rjmp  illegalop 
            rjmp  JMP_i 
            rjmp  DAD_a 
            rjmp  ROR_a 
            rjmp  illegalop 
            rjmp  BVS_r       ;0x70
            rjmp  DAD_ziy 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  DAD_zx 
            rjmp  ROR_zx 
            rjmp  illegalop 
            rjmp  SEI_s 
            rjmp  DAD_y 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  DAD_x 
            rjmp  ROR_x 
            rjmp  illegalop 
            rjmp  illegalop   ;0x80
            rjmp  STA_zxi 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  STY_z 
            rjmp  STA_z 
            rjmp  STX_z 
            rjmp  illegalop 
            rjmp  DEY_s
            rjmp  illegalop 
            rjmp  TXA_s 
            rjmp  illegalop 
            rjmp  STY_a 
            rjmp  STA_a 
            rjmp  STX_a 
            rjmp  illegalop 
            rjmp  BCC_r       ;0x90
            rjmp  STA_ziy 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  STY_zx 
            rjmp  STA_zx 
            rjmp  STX_zy 
            rjmp  illegalop 
            rjmp  TYA_s 
            rjmp  STA_y 
            rjmp  TXS_s 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  STA_x 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  LDY_m       ;0xa0
            rjmp  LDA_zxi 
            rjmp  LDX_m 
            rjmp  illegalop 
            rjmp  LDY_z 
            rjmp  LDA_z 
            rjmp  LDX_z 
            rjmp  illegalop 
            rjmp  TAY_s 
            rjmp  LDA_m 
            rjmp  TAX_s 
            rjmp  illegalop 
            rjmp  LDY_a
            rjmp  LDA_a 
            rjmp  LDX_a 
            rjmp  illegalop 
            rjmp  BCS_r       ;0xb0
            rjmp  LDA_ziy 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  LDY_zx 
            rjmp  LDA_zx 
            rjmp  LDX_zy 
            rjmp  illegalop 
            rjmp  CLV_s 
            rjmp  LDA_y 
            rjmp  TSX_s 
            rjmp  illegalop 
            rjmp  LDY_x 
            rjmp  LDA_x 
            rjmp  LDX_y 
            rjmp  illegalop 
            rjmp  CPY_m       ;0xc0
            rjmp  CMP_zxi 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  CPY_z 
            rjmp  CMP_z 
            rjmp  DEC_z 
            rjmp  illegalop 
            rjmp  INY_s 
            rjmp  CMP_m 
            rjmp  DEX_s 
            rjmp  illegalop 
            rjmp  CPY_a 
            rjmp  CMP_a 
            rjmp  DEC_a 
            rjmp  illegalop 
            rjmp  BNE_r       ;0xd0
            rjmp  CMP_ziy 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  CMP_zx 
            rjmp  DEC_zx 
            rjmp  illegalop 
            rjmp  CLD_s 
            rjmp  CMP_y 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  CMP_x
            rjmp  DEC_x 
            rjmp  illegalop 
            rjmp  CPX_m       ;0xe0
            rjmp  DSB_zxi 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  CPX_z 
            rjmp  DSB_z 
            rjmp  INC_z 
            rjmp  illegalop 
            rjmp  INX_s 
            rjmp  DSB_m 
            rjmp  NOP_s 
            rjmp  illegalop 
            rjmp  CPX_a 
            rjmp  DSB_a 
            rjmp  INC_a 
            rjmp  illegalop 
            rjmp  BEQ_r       ;0xf0
            rjmp  DSB_ziy 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  DSB_zx 
            rjmp  INC_zx 
            rjmp  illegalop 
            rjmp  SED_s 
            rjmp  DSB_y 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  illegalop 
            rjmp  DSB_x 
            rjmp  INC_x 
            rjmp  illegalop 
;*****************************************************************
;
; Exceptions
;
;*****************************************************************

; illegal opcode
illegalop:
   jmp  deb_inv_op     ;extension

; indirect addresspointer exceeded end of zeropage - NOT IMPLEMENTED
;zp_wrap:    rjmp  zp_wrap     ;diag will be added later

; stackpointer exceeded end of stackpage - NOT IMPLEMENTED!
; sp_wrap:    rjmp  sp_wrap     ;diag will be added later

; attempt to write to protected romspace - NOT implemented
;rom_wrt:    rjmp  rom_wrt     ;diag will be added later

;*****************************************************************
;
; Memory read instructions
;     Load, binary arithmetic & logical
;
;*****************************************************************

; opcode:         binary ADC
            adgen_x     ADC_        ;abs,X
            adgen_y     ADC_        ;abs,Y
            adgen_zxi   ADC_        ;(ZP,X) indexed indirect
            adgen_ziy   ADC_        ;(ZP),Y indirect indexed
            adgen_zx    ADC_        ;ZP,X
            adgen_z     ADC_        ;ZP absolute
ADC_a:      adgen_a                 ;absolute

ADC_o:      op_fetch                ;common entry after address generate
ADC_m:      in    operand,dbusin    ;immediate & operand prefetched (z, zx, zy)

;            sbrc  stat2,3           ;test decimal flag
;            rjmp  ADC_d
            out   sreg,stat         ;get C
            adc   rega,operand      ;AVR emulated OP
            in    stat,sreg         ;save NZC
            bst   stat,3            ;save V
            bld   stat2,6

ADC_d3:     op_decode               ;next instruction

;ADC_d:      addi  rega,0x66         ;decimal correction
;            out   sreg,stat
;            adc   rega,operand
;            in    stat,sreg
;            brcs  ADC_d2            ;MSD
;            subi  rega,0x60
;            out   sreg,stat
;ADC_d2:     brhs  ADC_d3            ;LSD
;            subi  rega,0x06
;            rjmp  ADC_d3

; opcode:         AND
            adgen_x     AND_        ;abs,X
            adgen_y     AND_        ;abs,Y
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

; opcode:         CMP
; a compare in a 6502 works with reversed carry opposed to an AVR
; the reason: a 6502 performs an add with complemented operand as a compare
            adgen_x     CMP_        ;abs,X
            adgen_y     CMP_        ;abs,Y
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

; opcode:         binary SBC
; subtraction in a 6502 works with reversed carry, borrow = !carry
; the reason: a 6502 performs an add with complemented operand during subtract
            adgen_x     SBC_        ;abs,X
            adgen_y     SBC_        ;abs,Y
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

;            sbrs  stat2,3           ;test decimal flag
;            rjmp  SBC_d             ;skip decimal mode correction;

;            brcs  SBC_d2            ;MSD correction
;            subi  rega,0x60
;            out   sreg,stat
;SBC_d2:     brhs  SBC_d             ;LSD correction
;            subi  rega,0x06

SBC_d:      op_decode               ;next instruction


;*****************************************************************
;
; Modify memory instructions
;     Shift, increment & decrement
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

;*****************************************************************
;
; Memory write instructions
;     Store register
;
;*****************************************************************

; opcode:         STA
            adgen_x     STA_        ;abs,X
            adgen_y     STA_        ;abs,Y
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

;*****************************************************************
;
; Memory read instructions
;     decimal arithmetic
;
;*****************************************************************

; opcode:         decimal ADC
            adgen_x     DAD_        ;abs,X
            adgen_y     DAD_        ;abs,Y
            adgen_zxi   DAD_        ;(ZP,X) indexed indirect
            adgen_ziy   DAD_        ;(ZP),Y indirect indexed
            adgen_zx    DAD_        ;ZP,X
            adgen_z     DAD_        ;ZP absolute
DAD_a:      adgen_a                 ;absolute

DAD_o:      op_fetch                ;common entry after address generate
DAD_m:      in    operand,dbusin    ;immediate & operand prefetched (z, zx, zy)

            .ifdef dec_mode_extended
               ldi   a,0x66            ;prepare to revoke decimal correction
               out   sreg,stat
               adc   rega,operand      ;perform binary add carry
               in    stat,sreg         ;save binary NZ
               brhc  DAD_d1
                  cbr   a,6               ;decimal correction valid for halfcarry
DAD_d1:        brcc  DAD_d2
                  cbr   a,0x60            ;decimal correction valid for carry
DAD_d2:        addi  rega,6            ;any low nibble BCD overflow?
               brcs  DAD_d5            ;carry reversed!
                  cbr   a,0x60            ;decimal correction valid for carry
                  sbr   stat,1            ;set carry in saved status
DAD_d5:        brhs  DAD_d3            ;halfcarry reversed!
                  sbrs  a,1               ;no previous halfcarry?
                     subi  rega,0x10         ;remove 1 halfcarry from result
                  cbr   a,6               ;decimal correction valid for halfcarry
DAD_d3:        addi  rega,0x60         ;any high nibble BCD overflow?
               brcs  DAD_d4            ;carry reversed!
                  cbr   a,0x60            ;decimal correction valid for carry
                  sbr   stat,1            ;set carry in saved status
DAD_d4:        sub   rega,a            ;revoke decimal correction if needed & set NZ
            .else
               addi  rega,0x66         ;decimal correction
               out   sreg,stat
               adc   rega,operand
               in    stat,sreg
               brcs  DAD_d2            ;MSD
               subi  rega,0x60
               out   sreg,stat
DAD_d2:        brhs  DAD_d3            ;LSD
               subi  rega,0x06
DAD_d3:
            .endif

            op_decode               ;next instruction

; opcode:         decimal SBC
; subtraction in a 6502 works with reversed carry, borrow = !carry
; the reason: a 6502 performs an add with complemented operand during subtract
            adgen_x     DSB_        ;abs,X
            adgen_y     DSB_        ;abs,Y
            adgen_zxi   DSB_        ;(ZP,X) indexed indirect
            adgen_ziy   DSB_        ;(ZP),Y indirect indexed
            adgen_zx    DSB_        ;ZP,X
            adgen_z     DSB_        ;ZP absolute
DSB_a:      adgen_a                 ;absolute

DSB_o:      op_fetch                ;common entry after address generate
DSB_m:      in    operand,dbusin    ;immediate & operand prefetched (z, zx, zy)

            com   operand
            out   sreg,stat         ;get C
            adc   rega,operand      ;AVR emulated OP
            in    stat,sreg         ;save NZC
            brcs  DSB_d2            ;MSD correction
               subi  rega,0x60   
               out   sreg,stat         ;restore halfcarry
DSB_d2:     brhs  DSB_d3            ;LSD correction
               subi  rega,0x06
               .ifdef dec_mode_extended
                  brhc  DSB_d3
                  sbrs  stat,5            ;no double borrow?
                     addi  rega,0x10         ;remove 1 borrow
               .endif

DSB_d3:     op_decode               ;next instruction
