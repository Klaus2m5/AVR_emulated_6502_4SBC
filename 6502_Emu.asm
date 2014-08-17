;
;
;   6 5 0 2   E M U L A T O R
;
; An AVR emulates a 6502 with >2MHz speed
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
; contact info at http://2m5.de or email K@2m5.de
;
#define   version "0.83a"   ;makes a printable version number
;
; version history:
;  0.8   24-jan-13   1st version distributed for testing
;  0.81  04-mar-13   added binary load/save/autoload support for applications
;                    added 6502_Emu_config.inc for all reconfigurable items.
;                    split cpu core into 6502_Emu_NMOS.inc to support change to CMOS
;                    added timer 1 access, tested and fixed all interrupts
;                    added countdown timer & interrupt vector register
;                    added optional software flow control to rx buffer
;                    improved terminal support for backspace and delete key
;  0.82  20-jul-13   added CMOS core (65C02 instructions and disassembly)
;  0.83  17-may-14   added breakpoint support to debugger
;                    added SPI support to IO module, DMA for SPI & I2C
;  0.83a 17-aug-14   changed interrupt disabled in real mode to honor NMI & single step
;                    fixed diag stop continuing until 10ms interrupt, now immediate
;                    fixed invalid opcode message broken by check for breakpoint
;                    
;
; ATMEGA16 fuse settings:
;   16 MHz crystal
;   JTAGEN unprogrammed
;   BOD enabled - 4,0V
;   Preserve EEPROM - protect non volatile program memory, else clears all saved programs
;
; related docs: http://2m5.de/6502_Emu/index.htm
;
; description of hardware:
;   porta0:7  -> address bus high : IO chip select latches
;   portb0:7  -> address bus low  : IO address & IO R/W : SPI & slave select latch
;   portc0:7  <> data bus         : I2C bus switch
;   portd0:1  RS232 monitor & ACIA emulation
;   portd2    <- ~NMI (INT0) / optional IO-select 3
;   portd3    <- ~IRQ (INT1) 
;   portd4    -> ~RD_RAM (/OE at SRAM)
;   portd5    -> ~WR_RAM (/WE at SRAM)
;                         /CE tied low at SRAM
;   portd6    -> IO-select 1
;   portd7    -> phi2 (OC2) / optional IO-select 2
;
;             pullups required for MNI, IRQ, weak pullups for WR_RAM, RD_RAM
;
;             optional weak pullups or pulldowns on IO selects and 
;             IO busses to reset external hardware during high-Z
;   
;   timer0    debugger 20ms timer: ctc interrupt
;             debugger single step: overflow interrupt
;   timer2    optional 1 MHz phase 2 output on OC2
;

; The following notes apply to the NMOS emulation core:
;
;   Emulation is documented instructions only !
;
;   Invalid 6502 instructions will cause a halt with message.
;
;   Decimal mode emulation will emulate documented behavior only:
;   a decimal result & carry-flag is valid if both operands were valid BCD
;
;   negative, overflow & zero-flags may be altered after a decimal add
;   or subtract, but have no valid meaning and may not be identical to real
;   hardware. The result of a decimal operation with non decimal operands
;   (any nibble >9) may not match the result of a hardware 6502
;
; In the CMOS emulation core all instructions are valid and if not defined
;   otherwise, will be executed as NOP instructions of various length in bytes
;   and cycles depending on the decode mechanism in the original CMOS core.
;
;   A decimal operation with valid decimal numbers will have a valid result
;   and valid NZC flags. The result and flags of decimal add or subtract with 
;   invalid BCD operands may not match a real 65C02.

 
            .NOLIST
            .include "sam.inc"
            .LIST
 

;****************************
;
; C O N F I G U R A T I O N
;
;****************************
.set config_part=1
.include "6502_Emu_config.inc"

;configuration based clock constants
.equ  cycle_time_tns    = (10000000000 / Osc_Hz) ;1/10 ns cpu clock duration
.equ  ten_ms            = ((Osc_Hz - 51200) / 102400) ;10ms OCR0 value rounded

;configuration based baudrate constants and error checking
.equ UBRR_value = ((Osc_Hz + Baud * 8) / (Baud * 16) - 1)  ; rounded
.equ Baud_error = ((1000 * Osc_Hz / (16 * (UBRR_value + 1)) / Baud) - 1000) ;0/00 deviation
.if ((Baud_error > 10) || (Baud_error < -10)) ; +/-10 0/00 acceptable
  .error "Unsafe baudrate - at current cpu clock the deviation is > 1%!"
.endif

;configuration based serial EEPROM constants
.ifdef eep_adr
   .if eep_adr < 8
      .ifdef spi_sel
         .ifdef spi_vat
            .equ spi_eep_adr = spi_idle^(1<<eep_adr)
         .else
            .equ spi_eep_adr = eep_adr
         .endif
         .equ eep_vld = 1
      .endif
   .else
      .ifdef i2c_sel
         .equ i2c_eep_adr = eep_adr
         .equ eep_vld = 1
      .endif
   .endif
.endif

; reserved registers
;
; (I): used during interrupt, (C): interrupt to program communication
; (E): Register of the emulated CPU
; (D)  used by the debug monitor - must be preserved except Z (Z is saved
;      before main is called)
;
; 
; 0x00 non immediate
.DEF  c           =r0         ; GPR3
.DEF  d           =r1         ; GPR4
.DEF  e           =r2         ; GPR5
.DEF  f           =r3         ; GPR6
; 0x04
.DEF  k           =r4         ; (I) GPR2 during interrupt
.DEF  allon       =r5         ; const 0xff
.DEF  one         =r6         ; const 1
.DEF  zero        =r7         ; const 0
; 0x08
.DEF  regx        =r8         ; (E) index X
.DEF  regy        =r9         ; (E) index Y
.DEF  operand     =r10        ; memory operand
.DEF  spointer    =r11        ; (E) stack pointer
; 0x0c
.DEF  clear       =r12        ; precharged cbus clear
.DEF  readmem     =r13        ; precharged cbus read memory
.DEF  writemem    =r14        ; precharged cbus write memory
.DEF  s           =r15        ; (I) SREG save during interrupt
; 0x10 immediate
.DEF  a           =r16        ; immediate GPR
.DEF  b           =r17        ; immediate GPR2
.DEF  i           =r18        ; (I) immediate GPR during interrupt
.DEF  flags       =r19        ; (D) flags 
.equ  deb_on  = 0             ;     debugger running if set, else stopped
                              ;     acia emulation disconnected if set
.equ  emu_run = 1             ;     emulator running if set, else stopped
.equ  deb_act = 2             ;     deb instance started if set - locks T0 cmp start
.equ  rvs_vid = 3             ;     reverse video if set, else normal video
.equ  op_ind  = 4             ;     operand indirect if set, operand direct if cleared
                              ;     needed for instruction retry in IO emulation (register busy) 
.equ  modify  = 5             ;     extended memory address locked until rewrite
.equ  string  = 6             ;     allow lower case input for string
.equ  dma_rpt = 7             ;     DMA interrrupt modifier
; 0x14
.DEF  unused_r20  =r20        ; unused
.DEF  rega        =r21        ; (E) accumulator
.DEF  stat2       =r22        ; (E) processor flags 6502 only 0V1BDI00
.DEF  stat        =r23        ; (E) processor flags AVR compatible (ITHSV)NZC
; 0x18 immediate word
.DEF  pcl         =r24        ; (E) program counter low
.DEF  pch         =r25        ; (E) program counter high
;     x                       ; word GPR / (I) buffer index (rx/tx)
; 0x1c
;     y                       ; operand address, (D) buffer index
#define     oplow       yl
#define     ophigh      yh
#define     opointer    y
;     z                       ; instruction decode
#define     opcode      zl
#define     oc_tabh     zh    ; restore when used as index
#define     oc_vector   z

;
; internal EEPROM usage
;
.ESEG
.ifdef   eep_adr
   ;non volatile program storage control structure for external I2C EEPROM
   .org  1
   eep_string: .byte 50          ;string to send to program on autoload
                                 ;0xff marks end
   .org  0xff
   eep_auto:   .byte 1           ;autoload program number, 0xff = none
   eep_pat:    .byte 256         ;program allocation table, 0xff = slot free
                                 ;0x00 - 0xfe = program number in slot
.endif

.macro      align             ;align to (1<<@0)
alignfromhere:
      .if (alignfromhere & ((1<<@0)-1))   ;if not already aligned
         .org  (alignfromhere & (0xffff<<@0)) + (1<<@0)
      .endif
.endmacro
;
; SRAM usage
;
.DSEG
;(D) command line buffer
cmd_buf:    .byte 48          ;48 character command line buffer
cmd_end:    .byte 1           ;+1 for end of string marker
cmd_inx:    .byte 1           ;input index to cmd_buf
cmd_esc_timer:                ;10ms increments to discard esc sequence
            .byte 1           ;bit 7 = 0:true escape, 1:discarded esc seq
cmd_reg_timer:                ;timer to refresh register display
            .byte 1
lmem_display:
            .byte 2           ;last memory block displayed
lmem_write: .byte 2           ;last memory address written +1
lmem_disas: .byte 2           ;last memory address disassembled +1
sp_save:    .byte 2           ;saved sp for mainloop to allow subs to exit
adr_limit:  .byte 2           ;maximum address +1 accepted in deb_adr
rstflag:    .byte 1           ;reset flags from mcucsr
prog_num:   .byte 1           ;program number for EEP save/load, 0xff if closed
brkpt_lo:   .byte 10          ;breakpoint address low
brkpt_hi:   .byte 10          ;breakpoint address high
brkpt_op:   .byte 10          ;breakpoint original opcode
;I/O emulation
;global
irq_mask:   .byte 1           ;interrupt mask, bit set if enabled, default 0b00000001
                              ;bit 7=tx empty, 6=rx full, 0=10ms tick,
irq_flag:   .byte 1           ;interrupt flags, bit set if pending
                              ;bit 7=tx empty, 6=rx full, 0=10ms tick
;timer 1
timer_ena:  .byte 1           ;t0 & t1 interrupt enable bits
t1_adr:     .byte 1           ;index to t1 register
t1_ctrl:    .byte 1           ;readback value of TCCR1
;tick (10ms) countdown timer
tcdn_count: .byte 1           ;countdown to flag/interrupt
tcdn_top:   .byte 1           ;reload value at bottom
;diagnostic
selftest:   .byte 1           ;selftest register, force bits 2=reset, 1=NMI, 0=IRQ
;I2C
i2c_statreg:
            .byte 1           ;status, bit 7=interrupt, 6=ack, 5=stuck, 4=set EEP 
                              ;3=read ack pending, 2=speed 400kHz, 1=start, 0=stop  
;LCD
lcd_flags:  .byte 1           ;0xff=usable, 0=busy timeout

;SPI
spi_cmd:    .byte 1           ;save for slave select hold
spi_rdat:   .byte 1           ;data in SPI shifter after last write

;DMA
dma_last_cmd:
            .byte 1           ;command in progress | last successful command + 0x10
                              ;| 0xff= can't continue
dma_tab_index:
            .byte 1           ;index to block transfer table, 0xff if trashed
;dma block transfer table:
;        SPI            I2C            EEPROM
;  byte  slave address  -don't care-   program number
;  word  start address  start address  start address
;  word  byte count     byte count     end address
;  word  temp count     temp count     -don't care-  
dma_tab:    .byte 7

;RS232 Buffers
usart_ena:  .byte 1           ;usart interrupt enable bits
tx_inx:     .byte 1           ;input index to tx fifo
tx_fill:    .byte 1           ;fill level in tx fifo (0 = empty)
rx_inx:     .byte 1           ;input index to rx fifo
rx_fill:    .byte 1           ;fill level in rx fifo (0 = empty)
flow_cmd:   .byte 1           ;XON/XOFF - bit 7: 1=done, 0=pending
            align 8           ;buffers on page boundary
tx_buf:     .byte 256         ;transmit fifo
rx_buf:     .byte 256         ;receive fifo


; end of SRAM

.CSEG
;
; Definition of Hardware
;
; Address Bus
.EQU  abuslo      = portb
.EQU  aloddr      = ddrb
.EQU  abushi      = porta
.EQU  ahiddr      = ddra
; Data Bus
.EQU  dbusin      = pinc
.EQU  dbusout     = portc
.EQU  dbusddr     = ddrc
; Control Bus
.EQU  cbusin      = pind
.EQU  cbus        = portd
.EQU  cbusddr     = ddrd
; External Interrupts
.EQU  ibus        = gicr
.ifdef   nmi_ena
   .EQU  IRQ_ena     = 0b11000000   ;enable IRQ
   .EQU  IRQ_dis     = 0b01000000   ;disable IRQ
.else
   .EQU  IRQ_ena     = 0b10000000   ;enable IRQ
   .EQU  IRQ_dis     = 0b00000000   ;disable IRQ
.endif
.EQU  NMI_dis     = 0   ;disable both during monitor
; USART
.EQU  usart_txi_dis  = 0b10011000   ;RXCIE, RXEN, TXEN
.EQU  usart_txi_ena  = 0b10111000   ;RXCIE, UDRIE, RXEN, TXEN

.macro      addi
            subi  @0,-@1            ;subtract the negative of an immediate value
.endmacro
.macro      addiw ;(RdL:RdH, k)
            subi  @0l,low(-@1)      ;word subtract the negative of an immediate value  
            sbci  @0l,high(-@1)
.endmacro 

;
; wait_ns  waittime in ns , cyles already used 
;
;     cycles already used will be subtracted from the delay
;     the waittime resolution is 1 cycle (delay from exact to +1 cycle)
;     the maximum delay at 20MHz (50ns/clock) is 750ns
;
.macro      wait_ns
      .set cycles = ((@0 * 10 + cycle_time_tns - 1) / cycle_time_tns - @1)
      .if (cycles > 15)
        .error "MACRO wait_ns - too many cycles to burn"
      .else
        .if (cycles > 0)
          .if   (cycles & 8)
            rjmp  pc+1
            rjmp  pc+1
            rjmp  pc+1
            rjmp  pc+1
          .endif
          .if   (cycles & 4)
            rjmp  pc+1
            rjmp  pc+1
          .endif
          .if   (cycles & 2)
            rjmp  pc+1
          .endif
          .if   (cycles & 1)
            nop
          .endif
        .endif
      .endif
.endmacro
; wait_data_valid  cycles already used
#define     wait_data_valid   wait_ns data_valid_ns,

; print string - debugger
.macro   PrintStr ;@0 = message pointer
   ldi   zl,low(@0*2)
   ldi   zh,high(@0*2)
   rcall prtstr
.endmacro
.macro   PrintStr_far ;@0 = message pointer
   ldi   zl,low(@0*2)
   ldi   zh,high(@0*2)
   call  prtstr
.endmacro


;*****************************************************************
;
; reset and interrupt vectors
;
;*****************************************************************

            jmp   reset       ; Reset handler
            rjmp  NMI         ; INT0 handler
            nop
            rjmp  IRQ         ; INT1 handler
            nop
            jmp   illegalint  ; Timer2 compare match
            jmp   illegalint  ; Timer2 overflow
         .ifdef iomap
            jmp   t1_icr      ; Timer1 capture event
            jmp   t1_ocra     ; Timer1 compare match A 
            jmp   t1_ocrb     ; Timer1 compare match B
            jmp   t1_ovi      ; Timer1 overflow handler
         .else
            jmp   illegalint  ; Timer1 capture event
            jmp   illegalint  ; Timer1 compare match A 
            jmp   illegalint  ; Timer1 compare match B
            jmp   illegalint  ; Timer1 overflow handler
         .endif
            jmp   t0_ovi      ; Timer0 overflow handler - debugger single step
            jmp   illegalint  ; SPI - Serial transfer complete
            rjmp  rx_int      ; USART - RX complete
            nop
            rjmp  tx_udre     ; USART - data register empty
            nop
            jmp   illegalint  ; USART - TX complete
            jmp   illegalint  ; ADC conversion complete
            jmp   illegalint  ; EEPROM ready
            jmp   illegalint  ; Analog comparator
            jmp   illegalint  ; TWI serial interface
            jmp   illegalint  ; INT2 handler
            jmp   t0_cmi      ; Timer0 compare match - debugger 10ms timer
            jmp   illegalint  ; SPM ready

;*****************************************************************
;
; RS232 buffer
;
;*****************************************************************
;
; RX complete interrupt - rx fifo input
;
rx_int:
   in    s,sreg
   push  xl
   push  xh
   in    k,ucsra
   in    i,udr
   sbrs  k,fe              ;no stop bit = possible break
   ifs_and  rx_break
   cpi   i,0               ;break should have all bits 0
   ifeq     rx_break
      sbr   flags,(1<<deb_on) ;turn debugging on
      cbr   flags,(1<<emu_run) ;stop emulation
      lds   i,irq_flag        ;clear ACIA TDRE & RDRF
      cbr   i,0b11000000
      sts   irq_flag,i
      lds   k,irq_mask        ;other interrupts pending & enabled?
      and   i,k
      ifeq  rx_break_irq
         cbi   cbusddr,3         ;clear any pending acia interrupts
      end   rx_break_irq
      sts   rx_fill,one       ;clear fifo
      ldi   i,10              ;start with a linefeed 
      ldi   xh,high(rx_buf)   ;load buffer index
      lds   xl,rx_inx
      st    x+,i              ;store data
      sts   rx_inx,xl         ;store pointers
      ldi   i,27              ;force escape
   end      rx_break
   lds   k,rx_fill         ;check buffer full
   inc   k
   ifne  rx_no_overrun
      ldi   xh,high(rx_buf)   ;load buffer index
      lds   xl,rx_inx
      st    x+,i              ;store data
      sts   rx_inx,xl         ;store pointers
      sts   rx_fill,k
   end   rx_no_overrun
   .ifdef flowlo
      mov   i,k
      cpi   i,flowhi          ;buffer upper watermark?
      ifsh  rx_flow
         lds   i,flow_cmd
         cpi   i,0x93         ;xoff done?
         ifne  rx_send_xoff
            ldi   i,0x13         ;post xoff pending
            sts   flow_cmd,i
             ldi   i,usart_txi_ena  ;notify transmitter
            out   ucsrb,i
            .ifdef irq_dis_real
               sts   usart_ena,i
            .endif
         end   rx_send_xoff
      end   rx_flow
   .endif
   .ifdef iomap                  ;acia emulation
      sbrc  flags,deb_on         ;acia connected?
      ifs   rx_acia_on
         lds   i,irq_mask        ;check acia RDRF IRQ enabled
         andi  i,0b01000000
         ifne  rx_int_ena
            sbi   cbusddr,3      ;set IRQ
         end   rx_int_ena
         lds   i,irq_flag
         sbr   i,0b01000000   ;set RDRF 
         sts   irq_flag,i        ;store status
      end   rx_acia_on       
   .endif
   pop   xh
   pop   xl
   out   sreg,s
   reti

;
; RS232 transmit register empty interrupt, tx fifo output
;
tx_udre:
   in    s,sreg
   push  xl
   push  xh
   lds   k,tx_fill         ;no more bytes available?
   .ifdef flowlo
      lds   i,flow_cmd        ;do flowcontrol
      tst   i                 ;already sent?
      ifpl  tx_flow
         out   udr,i             ;send xon/xoff
         ori   i,0x80            ;mark sent
         sts   flow_cmd,i
         tst   k
      else  tx_flow
         ldi   xh,high(tx_buf)   ;send data
         lds   xl,tx_inx         ;calc output index
         sub   xl,k
         ld    i,x               ;load from fifo head
         out   udr,i
         dec   k                 ;update pointer
      end   tx_flow
   .else
      ldi   xh,high(tx_buf)
      lds   xl,tx_inx         ;calc output index
      sub   xl,k
      ld    i,x               ;load from fifo head
      out   udr,i
      dec   k                 ;update pointer
   .endif
   ifeq  tx_fifo_empty
      ldi   i,usart_txi_dis   ;tx buffer empty - stop rupt
      out   ucsrb,i
      .ifdef irq_dis_real
         sts   usart_ena,i
      .endif
   end   tx_fifo_empty
   sts   tx_fill,k
   .ifdef iomap                  ;acia emulation
      sbrc  flags,deb_on         ;acia connected?
      ifs   tx_acia_on
         lds   i,irq_mask        ;check TDRE IRQ enabled
         andi  i,0b10000000
         ifne  tx_int_ena
            sbi   cbusddr,3      ;set IRQ
         end   tx_int_ena
         lds   i,irq_flag
         sbr   i,0b10000000   ;set TDRE
         sts   irq_flag,i     ;store status
      end   tx_acia_on       
   .endif
   pop   xh
   pop   xl
   out   sreg,s
   reti

; Messages part 1 (optimizing empty space before opcode table for CMOS)
; 76 words available if rx/tx maximum size and DMA (16 more for NMOS)
;                              word count, X = don't use in part 1 ---> ;##
;.ifdef cmos_core
;stp_instr:     .db   "STP - Emulator halted",0                          ; X
;.else
;inv_instr:     .db   "Illegal Opcode ",0                                ; X
;.endif
;emu_msg:       .db   13,10,13,10,core_string," Emulator V",version,0    ; X
;built_msg:     .db   " built ",__DATE__," ",__TIME__,0                  ;14
load_wait:     .db   13,10,"Loading, <ESC> to abort",13,10,0            ;14
load_abort:    .db   " Load aborted",0                                  ; 7
;err_chksum:    .db   " Checksum failed",0,0                             ; 9
;rs_vect_empty: .db   13,10,"Check reset vector",0,0                     ;11
reset_msg:     .db   "  Reset",0                                        ; 4
bpt_clrd_msg:  .db   10,13,"All breakpoints cleared",0                  ;13
bpt_info:      .db   10,13,"Breakpoints  (slot#:address)",13,10,0,0     ;17
bpt_info_none: .db   10,13,"No breakpoints active",0                    ;12
bpt_slot_full: .db   10,13,"No more breakpoint slots available",0,0     ;19
.ifndef irq_dis_real       ;+53 words available (6 less for NMOS)
load_ok:       .db   13,"Load OK",0,0                                   ; 5
err_nonhex:    .db   " Non-Hex data in record",0                        ;12
err_func:      .db   " Invalid function or count in record",0,0         ;19
illegal_int:   .db   13,10,"AVR Illegal Interrupt",0                    ;12
back_line:     .db   13,27,91,"K",27,91,"1A",0,0                        ; 5
.endif

;**************************************************
;
; 6502 emulation core
;
;**************************************************
      .ifdef cmos_core
         .include "6502_Emu_CMOS.inc"
      .else
         .include "6502_Emu_NMOS.inc"
      .endif

;**************************************************
;
; I/O page (iomap) address decode - emulated I/O
;
;**************************************************
      .ifdef iomap
         .include "6502_Emu_IO.inc"
      .endif

;**************************************************
;
; Monitor / Debugger Interrupts
;
;**************************************************

;
; an illegal interrupt has occured
;
illegalint:
   PrintStr illegal_int
   do    i_stop     
   loop  i_stop      ; reset required

;
; Timer 0 compare match A - 10ms CTC interrupt
;
t0_cmi:
   in    s,sreg
   lds   i,cmd_esc_timer   ;service esc sequence timer
   mov   k,i
   andi  i,0x7f
   ifne  t0_esc_expired
      dec   k
      sts   cmd_esc_timer,k
   end   t0_esc_expired
   lds   i,cmd_reg_timer   ;service register refresh timer
   tst   i
   ifne  t0_reg_expired
      dec   i
      sts   cmd_reg_timer,i
   end   t0_reg_expired

   lds   i,irq_flag        ;check 10 ms timer
   sbr   i,1               ;set tick flag
   lds   k,tcdn_count      ;update tcdn count
   dec   k                 ;expired?
   ifeq  t0_tcdn_expired
      lds   k,tcdn_top        ;reset tcdn count
      sbr   i,2               ;set tcdn flag
   end   t0_tcdn_expired
   sts   tcdn_count,k      ;save count
   sts   irq_flag,i        ;save flags
   lds   k,irq_mask        ;tick or tcdn IRQ enabled?
   and   i,k
   ifne  t0_tcdn_irq
      sbi   cbusddr,3         ;set IRQ
   end   t0_tcdn_irq

   sbrc  flags,deb_act     ;no debugger instance active
   ifs_and  debug_start
   sbrs  flags,deb_on      ;debug mode
   ifs      debug_start
      .if defined(spi_sel) || defined(i2c_sel)
         sbrs  flags,dma_rpt     ;interupt during dma transfer?
         ifs   t0_dma
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
         end   t0_dma
      .endif
      sbiw  pch:pcl,1      ;adjust PC to next opcode
      sbr   flags,(1<<deb_act)  ;prevent double_activation
      ldi   a,NMI_dis      ;no emulator interrupts during monitor
      out   ibus,a
      pop   a              ;discard return address from stack
      pop   a
      sei
      rjmp  debugger
   end      debug_start
   out   sreg,s
   reti
;
; Timer 0 overflow interrupt - single step
;
t0_ovi:
   ldi   a,NMI_dis         ;no emulator interrupts during monitor
   out   ibus,a
   sbiw  pch:pcl,1         ;adjust PC to next opcode
   pop   a                 ;discard return address from stack
   pop   a
   sei
   rjmp  end_command

.ifdef cmos_core
; STP instruction handling
deb_stop:
   ldi   flags,(1<<deb_on)|(1<<deb_act)
   lds   a,irq_flag        ;disconnect acia, clear TDRE & RDRF
   cbr   a,0b11000000
   sts   irq_flag,zero
   lds   c,irq_mask        ;no other internal IRQ pending & enabled?
   and   a,c
   ifeq  inv_op_irq_clear
      cbi   cbusddr,3         ;clear irq
   end  inv_op_irq_clear
   ldi   a,NMI_dis         ;no emulator interrupts during monitor
   out   ibus,a
   .ifdef irq_dis_real
      ldi   a,0b11            ;enable t0 interrupts (tick, single step)
      out   timsk,a
      lds   a,usart_ena       ;enable usart interrupts
      out   ucsrb,a
   .endif
   sbiw  pch:pcl,2         ;position to current instruction
   out   tifr,one          ;cancel single step interrupt (tov0)
   sei
   rcall brkpt_chk_pc
   iftc  deb_stop_unexp    ;skip if breakpoint
      rcall show_regs
      PrintStr stp_instr      ;"STP - Emulator halted"
      adiw  pch:pcl,1         ;position to next instruction
   end   deb_stop_unexp
   rjmp  end_command
.else
; illegal opcode handling 
deb_inv_op:
   ldi   flags,(1<<deb_on)|(1<<deb_act)
   lds   a,irq_flag        ;disconnect acia, clear TDRE & RDRF
   cbr   a,0b11000000
   sts   irq_flag,zero
   lds   c,irq_mask        ;no other internal IRQ pending & enabled?
   and   a,c
   ifeq  inv_op_irq_clear
      cbi   cbusddr,3         ;clear irq
   end  inv_op_irq_clear
   ldi   a,NMI_dis      ;no emulator interrupts during monitor
   out   ibus,a
   .ifdef irq_dis_real
      ldi   a,0b11            ;enable t0 interrupts (tick, single step)
      out   timsk,a
      lds   a,usart_ena       ;enable usart interrupts
      out   ucsrb,a
   .endif
   sbiw  pch:pcl,2         ;position to current instruction
   out   tifr,one          ;cancel single step interrupt (tov0)
   sei
   cpi   opcode,0xdb       ;potential breakpoint
   ifne_or  deb_stop_unexp
   rcall brkpt_chk_pc
   iftc     deb_stop_unexp ;skip if breakpoint
      mov   d,opcode          ;temporary save opcode
      rcall show_regs
      PrintStr inv_instr      ;"Illegal Opcode "
      mov   a,d
      rcall PrintHex
      adiw  pch:pcl,1         ;position to next instruction
   end      deb_stop_unexp
   rjmp  end_command
.endif
   
;**************************************************
;
; Reset
;
;**************************************************

; reset all IO extension registers
rs_IO_ext:
   .set io_sel_direct = 0
   .set rs_ddr_done = 0
;set ddr for 1st IO register configured
.macro   rs_set_ddr   
   .if   rs_ddr_done == 0
      out   ahiddr,allon      ;activate io select output
      .ifdef  nmi_ena
         ldi   a,0b11110000      ;Memory R/W signals + IO-select 1-2
      .else
         ldi   a,0b11110100      ;Memory R/W signals + IO-select 1-3
      .endif
      out   cbusddr,a         ;activate io select strobe
      .set rs_ddr_done = 1
   .endif
.endmacro
;reset external IO select registers
.macro   rs_set_iosel      ;@0 = ios#_default, @1 = cbus_pin#
   .ifdef @0
      .if   io_sel_direct != @1
         ldi   b,@0
         out   abushi,b
         sbi   cbus,@1
         rs_set_ddr
         cbi   cbus,@1
      .endif
   .endif
.endmacro
;reset spi slave select register (& its IO select register)
   .ifdef spi_sel
      ldi   a,spi_idle        ;reset slave select pins
      out   spi_out,a         ;precharge slave sel (abuslo)
      set_io_select spi_sel   ;precharge io select register
      ena_io_select
      ldi   a,~(1<<spi_miso)  ;set all output exept miso
      out   spi_ddr,a         ;activate spi output
      rs_set_ddr
      nop2                    ;wait 2 cycles 
      dis_spi_select          ;end SPI register strobe
   .endif
   out   cbus,clear        ;clear IO select
   out   aloddr,allon      ;set abuslo output (spi_ddr)
   out   spcr,zero         ;clear SPI hardware
   out   spsr,zero
;reset IO select register if configured and not already done
   rs_set_iosel ios1_default,6
   rs_set_iosel ios2_default,7
   rs_set_iosel ios3_default,2
   rs_set_ddr              ;if none of the above
;hw reset to IO ICs if configured
   .ifdef iomap
      .ifdef   io_reset_pin
         set_io_reset   io_reset_pin
         out   abushi,b
         out   cbus,clear
      .endif
   .endif
   ret
;
; *********** HARDWARE RESET VECTOR ****************
;
reset:
   ldi   a,high(ramend)    ;set Stack-Pointer
   out   sph,a
   ldi   a,low(ramend)
   out   spl,a

;setup of constant regs
   clr   allon
   dec   allon    ;0xff
   clr   one
   inc   one      ;1
   clr   zero     ;0
   ldi   a,0b00110000      ;cbus inactive const
   mov   clear,a
   ldi   a,0b00100000      ;cbus memory /OE const
   mov   readmem,a
   ldi   a,0b00010000      ;cbus memory /WE const
   mov   writemem,a
   ldi   oc_tabh,high(oc_tab) ;vector to opcode translation

;setup processor bus
   rcall rs_IO_ext         ;reset IO extension registers
   ldi   a,0b100           ;globaly disable pullups
   out   sfior,a
   ldi   a,0b0010          ;IRQ=low level, NMI=falling edge
   out   mcucr,a
   .ifdef phi2_ena         ;phase 2 output on OC2 (1 MHz)
      out   tcnt2,zero        ;timer 2 init
      ldi   a,0x7             ;500µs @ 16MHz
      out   ocr2,a
      ldi   a,0b00011001      ;timer 2 ctc, toggle OC2, clock / 1
      out   tccr2,a
   .endif
   
   sts   rx_fill,zero      ;initialize fifos
   sts   tx_fill,zero
   ldi   a,0x11            ;send xon if enabled
   sts   flow_cmd,a        ;
   ldi   a,low(cmd_buf)    ;initialize command buffer
   sts   cmd_inx,a
   ldi   a,0x80            ;initialize esc timer
   sts   cmd_esc_timer,a
   sts   prog_num,allon    ;mark eep as closed
   ldi   a,low(UBRR_value) ;set Baudrate
   out   ubrrl,a
   ldi   a,high(UBRR_value)
   out   ubrrh,a
   ldi   a,usart_txi_dis   ;rx,tx, rxcie enable
   out   ucsrb,a
   .ifdef irq_dis_real
      sts   usart_ena,a
   .endif
   out   tcnt0,zero        ;timer 0 init
   ldi   a,ten_ms
   out   ocr0,a
   ldi   a,0b00001101      ;timer 0 ctc, clock / 1024
   out   tccr0,a
   out   tcnt0,zero
   ldi   a,0b11            ;t0 compare match interrupt enable
   out   tifr,a
   out   timsk,a
   ldi   flags,(1<<deb_on)|(1<<deb_act)
   sei
   rcall crlf

;clearing the emulated machine if power on or brownout
   in    a,mcucsr             ;borf or porf
   andi  a,0b101
   ifne  clear_mem
      clr   zl                ;clear memory
      clr   zh
      out   cbus,clear        ;prepare to write memory
      out   dbusddr,allon
      do     memory_clear
         out   abuslo,zl         ;next address
         out   abushi,zh
         out   cbus,writemem     ;WE, ~OE
         out   dbusout,zero      ;write data
         out   cbus,clear        ;~WE / write cycle ends 120ns/16MHz
         adiw  z,1
      loopne memory_clear
      out   dbusddr,zero      ;prepare to read
      out   cbus,readmem      ;OE / read mode back on
      ldi   yl,low(brkpt_lo)  ;clear breakpoints
      ldi   yh,high(brkpt_lo)
      do       brkpt_reset
         st    y+,allon
         cpi   yl,low(brkpt_op)
      loopne   brkpt_reset
   else  clear_mem
      rcall crlf
      rcall show_regs
      PrintStr reset_msg      ;"  Reset"
   end   clear_mem

   rcall soft_reset

   PrintStr emu_msg        ;13,10,"6502 emulator  Vx.xxx"
   PrintStr built_msg      ;" built ",__DATE__

; load & run on power on
; if a program is selected for autoload
   .ifdef eep_vld
      in    a,mcucsr          ;borf or porf
      andi  a,0b101
      ifne  rspo_autoload
         out   eearh,zero     ;read eep autoload byte
         out   eearl,allon
         out   eecr,one
         in    a,eedr
         out   eearl,zero     ;protect
         cpi   a,0xff         ;test autoload program set
         ifne  reset_autoload
            sts   prog_num,a
            rcall eep_load_prog
            tst   a           ;load O.K.
            ifeq  eep_load_ok
               rcall load_reset_vector
               tst   pch         ;valid PC?
               ifne  pc_load_ok
                  in    d,sreg            ;atomic modify receive buffer
                  cli
                  ldi   yh,high(rx_buf)   ;setup rx buffer address
                  ldi   yl,0
                  mov   c,one             ;string address
                  do       eep_load_string
                     out   eearl,c           ;read internal EEPROM
                     out   eecr,one
                     in    a,eedr
                     cpi   a,0xff            ;string end?
                  exiteq   eep_load_string
                     cpi   a,92              ;backslash?
                     ifeq  eep_load_special
                        inc   c
                        out   eearl,c           ;read internal EEPROM
                        out   eecr,one
                        in    a,eedr
                        cpi   a,0xff            ;string end?
                  exiteq   eep_load_string
                        cpi   a,'r'             ;carriage return?
                        ifeq  eep_auto_cr
                           ldi   a,13
                        end   eep_auto_cr
                        cpi   a,'q'             ;double quote?
                        ifeq  eep_auto_quote
                           ldi   a,34
                        end   eep_auto_quote
                     end   eep_load_special
                     st    y+,a              ;store in rx buffer
                     inc   c                 ;next EEPROM address
                  loop     eep_load_string
                  sts   rx_inx,yl         ;save buffer pointers
                  sts   rx_fill,yl
                  out   sreg,d            ;restore interrupt enable
                  out   eearl,zero        ;protect internal EEPROM
                  rcall crlf
                  cli
                  ldi   flags,(1<<emu_run)   ;run program
                  in    a,mcucsr          ;HW reset complete
                  andi  a,0xe0            ;mask out reset flags
                  out   mcucsr,a          ;clear in mcucsr
                  rjmp  end_exit
               else  pc_load_ok
                  PrintStr rs_vect_empty     ;13,10,"Check reset vector"
               end   pc_load_ok
            end   eep_load_ok
         end   reset_autoload
      end   rspo_autoload
   .endif
   in    a,mcucsr          ;HW reset complete
   andi  a,0xe0            ;mask out reset flags
   out   mcucsr,a          ;clear in mcucsr
;*********************************************************
;
; Monitor / Debugger main loop
;
;     bypasses buffers/timers when interrupts are disabled
;     updates/uses timer 0 to discard escape sequences
;     updates/uses timer 0 to refresh register display
;     samples input, translates to upper case
;     allows backspace & escape (erase last/all input)
;     parses & executes commandline on carriage return
;
;*********************************************************
   ldi   a,10      ;linefeed
   rcall PrintChr
   rcall show_regs
   rcall show_prompt

debugger:      ;entry from t0 cmp match
   do    debug_main
      do    deb_wait4input
; disabled update of esc & showprompt timer
         ifid_and  deb_timer_update ;disabled timer update 
         in    yh,tifr
         sbrs  yh,ocf0
         ifs       deb_timer_update
            ldi   yh,0b10           ;clear t1 oc flag
            out   tifr,yh
            lds   yl,cmd_esc_timer  ;test escape timer expired
            andi  yl,0x7f
            ifne  deb_esc_timer_update
               lds   yl,cmd_esc_timer
               dec   yl
               sts   cmd_esc_timer,yl
            end   deb_esc_timer_update
            lds   yl,cmd_reg_timer  ;test register timer expired
            tst   yl
            ifne  deb_reg_timer_update
               dec   yl
               sts   cmd_reg_timer,yl
            end   deb_reg_timer_update
         end       deb_timer_update
; new line triggered by true <ESC> based on timer
         lds   yl,cmd_esc_timer ;test escape timer expired
         tst   yl
         ifeq  deb_true_esc
            ldi   yl,0x80           ;flag line cleared
            sts   cmd_esc_timer,yl
            cbr   flags,(1<<string) ;enable case translation
            rcall show_regs         ;redraw registers & prompt
            rcall show_prompt
            rcall erase2eol
            ldi   yh,low(cmd_buf)   ;ready for next command
            sts   cmd_inx,yh
         end   deb_true_esc
; show registers in prompt based on refresh timer
         lds   yh,cmd_reg_timer
         tst   yh                ;show register timer expired &
         ifeq_and deb_show_regs
         cpi   yl,0x80           ;no active escape sequence
         ifeq     deb_show_regs
            sbrs  flags,emu_run
            ifs   deb_regs_run
               rcall escape            ;save position - esc7
               ldi   a,'7'
               rcall PrintChr
               rcall show_regs         ;show registers at beginning of line
               rcall show_prompt
               rcall escape            ;restore position - esc7
               ldi   a,'8'
               rcall PrintChr
            else  deb_regs_run
               sts   cmd_reg_timer,allon  ;no changes expected soon
            end   deb_regs_run
         end      deb_show_regs
; read RS232
         in    d,sreg      ;preserve sreg interrupt enable
         cli
         lds   c,rx_fill   ;check buffer
         tst   c
         ifne  deb_read_buf
            lds   yl,rx_inx      ;prepare address to read rx fifo
            ldi   yh,high(rx_buf)
            sub   yl,c
            ld    a,y            ;read
            dec   c              ;update pointer
            sts   rx_fill,c
            .ifdef flowlo
               mov   i,c
               cpi   i,flowlo          ;buffer lower watermark?
               iflo  deb_flow
                  lds   i,flow_cmd
                  cpi   i,0x91         ;xon done?
                  ifne  deb_send_xon
                     ldi   i,0x11         ;post xon pending
                     sts   flow_cmd,i
                     ldi   i,usart_txi_ena ;notify transmitter
                     out   ucsrb,i
                     .ifdef irq_dis_real
                        sts   usart_ena,i
                     .endif
                 end  deb_send_xon
               end   deb_flow
            .endif
            out   sreg,d
            exit  deb_wait4input
         else  deb_read_buf
            out   sreg,d
            ifid_and  deb_read_direct
            sbis  ucsra,rxc      
            ifs       deb_read_direct
               in    a,udr        ;read direct if disabled & buffer empty
               exit  deb_wait4input
            else      deb_read_direct
               sbrc  flags,emu_run     ;allow emulator to run
               exit  debug_main
            end       deb_read_direct
         end   deb_read_buf
      loop  deb_wait4input
; process command buffer input
      lds   yl,cmd_esc_timer
      andi  yl,0x07f             ;discard esc sequence chars
      ifne  deb_esc_seq
         in    d,sreg
         cli
         lds   yl,cmd_esc_timer
         sbr   yl,0b10000000     ;flag esc sequence
         sts   cmd_esc_timer,yl
         out   sreg,d
      else  deb_esc_seq
         ldi   yh,high(cmd_buf)  ;load command buffer index high
         lds   yl,cmd_inx
         cpi   a,0x7f            ;delete?
         ifeq  deb_del2bs
            ldi   a,8               ;replace with backspace
         end   deb_del2bs
         cpi   a,8               ;backspace?
         ifeq  deb_bs
            cpi   yl,low(cmd_buf)   ;prevent underrun
            ifne   deb_bs_ok
               rcall PrintChr          ;perform erase
               rcall space
               ldi   a,8
               rcall PrintChr
               dec   yl
               ld    a,y               ;deleted character
               cpi   a,34              ;is quote?
               ifeq  deb_bs_quote
                  ldi   a,(1<<string)     ;toggle case translation
                  eor   flags,a
               end   deb_bs_quote
            end   deb_bs_ok
            ldi   a,0            ;mark non-printable
         end    deb_bs
         cpi   a,' '             ;test printable character &
         ifsh_and deb_echo
         cpi   yl,cmd_end        ;prevent overrun
         iflo     deb_echo
            cpi   a,'a'          ;test for lower case
            ifsh_and deb_lower_case
            cpi   a,'z'+1
            iflo_and deb_lower_case
            sbrc  flags,string   ;conversion enabled?
            ifs      deb_lower_case
               subi  a,0x20          ;convert to upper case
            end      deb_lower_case
            rcall PrintChr
            st    y+,a
         end      deb_echo
         sts   cmd_inx,yl
; <">
         cpi   a,34              ;test for string input
         ifeq  deb_string_mark
            ldi   b,(1<<string)  ;switch case translation
            eor   flags,b
         end   deb_string_mark
; <ESC>
         cpi   a,27              ;test for escape
         ifeq  deb_esc
            ifid  deb_esc_disabled
               ldi   yh,0b10        ;reset t0 compare match
               out   tifr,yh        ;if disabled 
            end   deb_esc_disabled
            ldi   yh,6              ;further chars within 50-60ms     
            sts   cmd_esc_timer,yh  ;discarded as esc sequence
         end   deb_esc
; <TAB>            
         cpi   a,9               ;test for tab (step 1 instruction)
         ifeq_and deb_step
         sbrc  flags,emu_run     ;must be stopped
         ifs      deb_step
            rcall show_regs      ;show command to be executed
            movw  y,pch:pcl
            rcall disasm
            .ifndef cmos_core
               cpi   b,7            ;invalid opcode?
               ifeq  deb_step_inv
                  rjmp  end_command
               end   deb_step_inv
            .endif
            cli                  ;force t0 overflow at end of instruction
            out   tcnt0,allon    ;overflow next count
            ldi   a,0b00001001   ;timer 0 ctc, clock / 1
            out   tccr0,a
            ldi   a,0b00001101   ;timer 0 ctc, clock / 1024
            out   tccr0,a
            rjmp  emu_start      ;executes also breakpointed opcodes
         end      deb_step
; <CR>
         cpi   a,13              ;test for carriage return
         ifeq_far deb_cr
            cpi   yl,low(cmd_buf)   ;test empty commandline
            ifeq  deb_empty
               ld    a,y               ;test for previous M or D
               cpi   a,'M'
               ifeq_or  deb_shortcut
               cpi   a,'D'
               ifeq     deb_shortcut
                  PrintStr back_line      ;seamless output
                  adiw  y,1               ;keep M or D
                  ldi   a,'+'
                  st    y+,a
               end      deb_shortcut
            end   deb_empty
            st    y,zero            ;mark end of commandline
            cbr   flags,(1<<string) ;enable case translation
;**************************************************
;
; parsing & executing commandline
;
;************************************************** 
            in    a,spl             ;save sp - allows sub to exit
            sts   sp_save,a
            in    a,sph
            sts   sp_save+1,a
            ldi   yl,low(cmd_buf)   ;ready to parse command
            sts   cmd_inx,yl        ;reset next commandline
            ld    a,y+
;
; A = alter register   Syntax:Arbb<CR>|APwwww<CR>
;     r    = register A X Y P S F
;            accu indexX indexY PC SP flags(proc.status)
;     bb   = hex byte of data
;     wwww = hex word of data, leading zeroes can be omitted
;
            cpi   a,'A'
            ifeq  alter_reg
               ld    a,y+      ;get register
;alter accumulator
               cpi   a,'A'     ;accumulator
               ifeq  alter_rega
                  rcall get_byte
                  mov   rega,a
                  rjmp  end_command
               end   alter_rega
;alter X index register
               cpi   a,'X'     ;index register X
               ifeq  alter_regx
                  rcall get_byte
                  mov   regx,a
                  rjmp  end_command
               end   alter_regx
;alter Y index register
               cpi   a,'Y'     ;index register Y
               ifeq  alter_regy
                  rcall get_byte
                  mov   regy,a
                  rjmp  end_command
               end   alter_regy
;alter stack pointer
               cpi   a,'S'     ;stack pointer
               ifeq  alter_spointer
                  rcall get_byte
                  mov   spointer,a
                  rjmp  end_command
               end   alter_spointer
;alter flags (prcocessor status)
               cpi   a,'F'     ;flags = processor status
               ifeq  alter_status
                  rcall get_byte
                  mov   stat2,a
                  mov   stat,stat2        ;restore AVR format
                  andi  stat,0b11         ;------ZC
                  bst   stat2,7           ;>---->
                  bld   stat,2            ;-----N--
                  ori   stat2,0b0100000   ;  1      (always 1)
                  andi  stat2,0b1101100   ;-V1BDI-- (Break cleared)
                  rjmp  end_command
               end   alter_status
;alter program counter
               cpi   a,'P'     ;program counter
               ifeq  alter_pc
                  rcall get_adr
                  movw  pch:pcl,z
                  rjmp  end_command
               end   alter_pc
               rjmp  invalid_command
            end   alter_reg
;
; B = breakpoint utility
;     BI<cr>      = information - list active breakpoints by slot number
;     BSaaaa<cr>  = set breakpoint at address aaaa, leading zeros can be omitted
;     BC#<cr>     = clear breakpoint slot #, #=0-9, #=A for all
;
            cpi   a,'B'
            ifeq_far br_util
               ld    a,y+              
               cpi   a,'I'
               ifeq  br_info
                  ld    a,y+           ;check validity (no parms)
                  cpse  a,zero
                  rjmp  invalid_command
                  rcall brkpt_info
                  rjmp  end_command
               end   br_info
               cpi   a,'S'
               ifeq  br_set
                  rcall get_adr
                  ldi   yl,low(brkpt_lo)  ;address already set?
                  ldi   yh,high(brkpt_lo)
                  do       br_set_already
                     ld    a,y+
                     cp    a,zl
                     ifeq_and br_already_set
                     ldd   a,y+9
                     cp    a,zh
                     ifeq     br_already_set
                        rcall brkpt_info
                        rjmp  end_command
                     end      br_already_set
                     cpi   yl,low(brkpt_hi)
                  loopne   br_set_already
                  ldi   yl,low(brkpt_lo)  ;free slot?
                  do       br_set_free
                     ld    a,y+
                     cp    a,allon
                     ifeq_and br_set_slot
                     ldd   a,y+9
                     cp    a,allon
                     ifeq     br_set_slot
                        sbiw  y,1
                        st    y,zl              ;set new breakpoint address
                        std   y+10,zh
                        out   abuslo,zl         ;fetch original opcode
                        out   abushi,zh
                        wait_data_valid 0       ;0ns minimum @ 16MHz
                        in    a,dbusin
                        std   y+20,a            ;set original opcode
                        ldi   a,0xdb            ;replace with STP opcode
                        out   cbus,writemem     ;WE, ~OE
                        out   dbusout,a         ;precharge write
                        out   dbusddr,allon     ;dbus = output
                        out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
                        out   dbusddr,zero      ;dbus = input
                        out   cbus,readmem      ;OE / read mode back on
                        rcall brkpt_info
                        rjmp  end_command
                     end      br_set_slot
                     cpi   yl,low(brkpt_hi)
                  loopne   br_set_free
                  PrintStr bpt_slot_full  ;10,13,"No more breakpoint slots available"
                  rcall brkpt_info
                  rjmp  end_command
               end   br_set
               cpi   a,'C'
               ifeq  br_clr         ;get slot# or all
                  do     br_clr_parse
                     ld    a,y+
                     tst   a              ;premature end of input
                     breq  br_inv_cmd
                     cpi   a,32           ;skip blanks
                  loopeq br_clr_parse
                  ld    c,y+           ;check validity (1 parameter)
                  cpse  c,zero
                  rjmp  invalid_command
                  sbiw  y,1
                  cpi   a,'A'
                  ifeq  br_clr_all
                     rcall brkpt_clr_all
                     tst   c
                     ifeq  br_clr_none_msg
                        PrintStr bpt_info_none  ;10,13,"No breakpoints active"
                     end   br_clr_none_msg
                     rjmp  end_command
                  end   br_clr_all
                  subi  a,'0'           ;0-9?
                  cpi   a,10
                  iflo  br_clr_slot
                     ldi   yl,low(brkpt_lo)  ;select breakpoint
                     add   yl,a
                     ldi   yh,high(brkpt_lo)
                     adc   yh,zero
                     rcall brkpt_clr_one
                     rcall brkpt_info
                     rjmp  end_command
                  end   br_clr_slot
               end   br_clr
br_inv_cmd:    rjmp  invalid_command
            end      br_util


;
; D = disassemble memory   Syntax: D(aaaa|+)<CR>
;     aaaa = address, leading zeros can be omitted
;     + = next, disassembles 20 instructions
;
            cpi   a,'D'
            ifeq  show_disasm
               ld    a,y+              
               cpi   a,'+'
               ifeq  dis_next
                  ld    a,y+           ;no parameter after +
                  cpse  a,zero
                  rjmp  invalid_command
                  lds   yl,lmem_disas     ;load previous address
                  lds   yh,lmem_disas+1
               else   dis_next
                  sbiw  y,1               ;allow immediate hex
                  rcall get_adr           ;get Address
                  movw  y,z
               end   dis_next
               ldi   a,10              ;count
               do     dis_loop
                  push  a
                  rcall crlf
                  mov   a,yh              ;show address
                  rcall PrintHex
                  mov   a,yl
                  rcall PrintHex
                  rcall space
                  rcall disasm
                  pop   a
                  dec   a
               loopne dis_loop            
               sts   lmem_disas,yl     ;load previous address
               sts   lmem_disas+1,yh
               rjmp  end_command
            end   show_disasm
            .ifdef   eep_vld
;
; E = I2C EEPROM non volatile program storage utility
;     EI<cr>   = information on programs stored and free slots
;     ESpp<cr> = save program from RS232 input to EEPROM
;     ELpp<cr> = load program from EEPROM to RAM
;     EDpp<cr> = delete program in EEPROM
;     EApp"C\rLOAD $xx\rRUN\r"<cr>
;              = autoload program, send string on power on
;                either parameter may be omitted
;                FF = no autoload, "" (empty string) = send nothing
;       pp     = program number 00 - FE
;
               cpi   a,'E'
               ifeq_far eep_util
                  ld    a,y+
;EEPROM information
                  cpi   a,'I'
                  ifeq  eep_info
                     ld    a,y+           ;check validity (no parms)
                     cpse  a,zero
                     rjmp  invalid_command
                     rcall eep_auto_info
                     rcall eep_cmd_info
                     rjmp  end_command
                  end   eep_info
;EEPROM save
                  cpi   a,'S'
                  ifeq_far eep_save
                     rcall get_byte          ;get prog#
                     cpi   a,0xff
                     ifeq  eep_save_inv_cmd
                        rjmp  invalid_command
                     end   eep_save_inv_cmd
                     sts   prog_num,a
                     rcall eep_open
                     ldi   yl,0              ;initial slot allocation
                     ldi   yh,0
                     out   eearh,one         ;upper bytes in internal EEPROM
                     PrintStr save_wait      ;13,10,"Saving, <ESC> to abort",13,10
                     do    eep_save_record
                        do     eep_wait_colon    ;wait for start of record
                           rcall read_serial_esc
                           cpi   a,':'
                        loopne eep_wait_colon
                        clr   xh             ;checksum
                        rcall read_byte_esc  ;read & save count
                        mov   xl,a
                        rcall eep_write
                        ldi   a,13
                        rcall PrintChr
                        rcall read_byte_esc  ;read  address high
                        tst   xl             ;count 0?
                        ifne  eep_save_adr   ;show & save address for count >0
                           rcall eep_prsv       ;address high, read next
                           rcall eep_prsv       ;address low, read function
                           cpi   a,0            ;load function?
                           ifeq  eep_save_data
                              do     eep_save_data_loop
                                 rcall read_byte_esc     ;read & save data
                                 rcall eep_write
                                 dec   xl
                              loopne eep_save_data_loop
                              rcall read_byte_esc     ;test checksum
                              tst   xh
                              ifne  eep_save_error_checksum
                                 PrintStr err_chksum     ;" Checksum failed"
                                 rjmp  eep_write_err
                              end   eep_save_error_checksum
                           else  eep_save_data
                              PrintStr err_func       ;" Invalid function or count in record"
                              rjmp  eep_write_err
                           end   eep_save_data
                        else  eep_save_adr      ;show & save PC
                           rcall eep_prsv          ;PCH, read next
                           rcall eep_prsv          ;PCL, read function
                           cpi   a,1               ;end of record function?
                           ifeq  eep_save_complete
                              PrintStr save_ok        ;13,"Save OK"
                              rjmp  eep_write_close
                           else  eep_save_complete
                              PrintStr err_func       ;" Invalid function or count in record"
                              rjmp  eep_write_err
                           end   eep_save_complete
                        end  eep_save_adr
                     loop  eep_save_record
eep_prsv:            ; Output to RS232 & EEP
                     push  a
                     rcall PrintHex       ;show address
                     pop   a
                     rcall eep_write      ;save address
                     rjmp  read_byte_esc  ;get next byte
                  end      eep_save
;EEPROM load
                  cpi   a,'L'
                  ifeq  eep_load
                     rcall get_byte
                     cpi   a,0xff
                     breq  eep_inv_cmd
                     sts   prog_num,a
                     rcall brkpt_clr_all  ;clear all breakpoints
                     rcall eep_load_prog
                     rjmp  end_command
eep_inv_cmd:         rjmp  invalid_command
                  end   eep_load
;EEPROM delete
                  cpi   a,'D'
                  ifeq  eep_del
                     rcall get_byte
                     cpi   a,0xff
                     breq  eep_inv_cmd
                     clr   c              ;loopcount 256 slots
                     clr   b              ;slotcount
                     out   eearh,one
                     do       eep_del_loop
                        out   eearl,c        ;find prog#
                        out   eecr,one
                        in    d,eedr
                        cp    d,a
                        ifeq  eep_del_slot
                           out   eedr,allon     ;mark free
                           in    d,sreg         ;atomic write sequence
                           cli
                           sbi   eecr,eemwe
                           sbi   eecr,eewe
                           out   sreg,d
                           inc   b              ;count slots
                           do    eep_del_wait   ;write complete?
                              sbic  eecr,eewe
                           loop  eep_del_wait
                        end   eep_del_slot
                        inc   c              ;all slots done?
                     loopne   eep_del_loop
                     out   eearl,zero        ;set to unused address
                     out   eearh,zero
                     tst   b              ;count?
                     ifne  eep_del_free
                        rcall crlf    
                        mov   a,b
                        rcall PrintHex          ;slotcount
                        PrintStr eep_info3      ;" slots free"
                        ldi   a,'d'
                        rcall PrintChr
                     else  eep_del_free
                        mov   b,a
                        PrintStr eep_prog       ;10,13,"Program "
                        mov   a,b
                        rcall PrintHex          ;prog#
                        PrintStr eep_notfound   ;" not found"
                     end   eep_del_free
                     rjmp  end_command
                  end   eep_del
;EEPROM autoload
                  cpi   a,'A'
                  ifeq  eep_autoload
                     out   eearh,zero     ;write eep lower page
                     do       eep_auto_parse
                        ld    a,y+
                        tst   a              ;end of input
                        breq  eep_inv_cmd    ;needs at least 1 parameter
                        cpi   a,' '          ;skip blanks
                     loopeq   eep_auto_parse
                     cpi   a,34           ;begin of string
                     ifne  eep_auto_number
                        rcall get_wbyte
                        out   eearl,allon
                        rcall eep_auto_write
                        do       eep_auto_parse2
                           ld    a,y+
                           cpi   a,' '          ;skip blanks
                        loopeq   eep_auto_parse2
                     end   eep_auto_number
                     tst   a              ;end of input
                     ifne  eep_auto_next
                        cpi   a,34
                        brne  eep_auto_invalid  ;only valid input is string
                        ldi   b,eep_string
                        do       eep_auto_string
                           ld    a,y+
                           tst   a              ;end of command?
                           ifeq_or  eep_string_end
                           cpi   a,34           ;closing quotes
                           ifeq     eep_string_end
                              ldi   a,0xff      ;end of string marker
                           end      eep_string_end
                           out   eearl,b
                           rcall eep_auto_write
                           inc   b              ;next EEPROM address
                           cpi   a,0xff         ;end marker?
                        loopne   eep_auto_string
                     end   eep_auto_next
                     rcall eep_auto_info
                     out   eearl,zero     ;protect eep from unintentional writes
                     rjmp  end_command
eep_auto_invalid:    rjmp  invalid_command
                  end   eep_autoload
               end      eep_util
            .endif
;        
; M = show memory   Syntax: M(aaaa|+|-|)<CR>
;     aaaa = address, leading zeros can be omitted
;     + = next block, - = previous block, <CR> = same block 
;     displays 0x100 Bytes at full line address
;
            cpi   a,'M'   
            ifeq  show_mem
               out   cbus,readmem      ;prepare to read
               lds   zl,lmem_display   ;load previous address
               lds   zh,lmem_display+1
               ld    a,y+              
               tst   a                 ;no address - show same
               breq  skip_get_adr
               cpi   a,'+'
               ifeq  show_next
                  inc   zh
                  ld    a,y+           ;no parameter after +
                  tst   a
                  breq  skip_get_adr
                  rjmp  invalid_command
               end   show_next
               cpi   a,'-'
               ifeq  show_previous
                  dec   zh
                  ld    a,y+           ;no parameter after -
                  tst   a
                  breq  skip_get_adr
                  rjmp  invalid_command
               end   show_previous
               sbiw  y,1            ;allow immediate hex
               rcall get_adr        ;get Address
skip_get_adr:
               andi  zl,0xe0        ;mask full lines only
               mov   d,zl
               sts   lmem_display,zl
               sts   lmem_display+1,zh
               do    show_mem_line
                  rcall crlf
                  mov   a,zh           ;show address
                  rcall PrintHex
                  mov   a,zl
                  rcall PrintHex
                  rcall colon            
                  do     show_mem_data      
                     out   abuslo,zl      ;memory fetch
                     out   abushi,zh
                     clt                  ;clear reverse video
                     wait_data_valid 1    ;60ns minimum @ 16MHz
                     in    a,dbusin
                     cpi   a,0xdb         ;is potential breakpoint?
                     ifeq  show_mem_brk
                        rcall brkpt_chk
                     end   show_mem_brk
                     rcall RevHex
                     adiw  z,1
                     ldi   a,3            ;seperate words with spaces
                     and   a,zl
                  loopne show_mem_data
                     rcall space
                     ldi   a,31           ;new line after 32 bytes              
                     and   a,zl
                  loopne show_mem_data
                  cp    zl,d           ;0x100 bytes displayed?
               loopne show_mem_line
               rjmp  end_command
            end   show_mem
;
; W = Write memory   Syntax: W(aaaa|+) bb..bb<CR>
;     aaaa = address, leading zeros can be omitted
;     + = write next byte(s)
;     bb = bytes of data, may be separated with spaces
;
            cpi   a,'W'
            ifeq  write_mem
               ld    a,y+              
               cpi   a,'+'
               ifeq  write_next
                  lds   zl,lmem_write     ;load previous next address
                  lds   zh,lmem_write+1
               else  write_next
                  sbiw  y,1               ;allow immediate hex
                  rcall get_wadr          ;get write address
                  sts   lmem_display,zl   ;save for next display
                  sts   lmem_display+1,zh
               end   write_next
               mov   d,yl
               do    write_mem_dryrun  ;syntax check before write
                  ld    a,y+
                  tst   a                 ;end of write
                  exiteq write_mem_dryrun
               cpi   a,32              ;skip space
               loopeq write_mem_dryrun
                  rcall get_wbyte         ;get data
               loop  write_mem_dryrun
               mov   yl,d
               out   cbus,clear        ;readmode off
               out   dbusddr,allon     ;dbus = output
               do    write_mem_realrun  ;real write
                  ld    a,y+
                  tst   a                 ;end of write
                  exiteq write_mem_realrun
               cpi   a,32              ;skip space
               loopeq write_mem_realrun
                  rcall get_wbyte         ;get data
                  rcall brkpt_chk_write   ;check write to breakpoint
                  out   abuslo,zl 
                  out   abushi,zh
                  out   cbus,writemem     ;WE, ~OE
                  out   dbusout,a         ;dbus write
                  adiw  z,1
                  out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
               loop  write_mem_realrun      
               out   dbusddr,zero      ;dbus = input
               out   cbus,readmem      ;OE / read mode back on
               sts   lmem_write,zl
               sts   lmem_write+1,zh
               rjmp  end_command
            end   write_mem
;
; L = Load Intel Hex Record   Syntax: L<CR>
;     loads records to memory, <ESC> to abort
;
            cpi   a,'L'
            ifeq_far load_mem
               ld    a,y+           ;check validity (no parms)
               cpse  a,zero
               rjmp  invalid_command
               cbr   flags,(1<<emu_run) ;force emulator halted  
               rcall brkpt_clr_all  ;clear all breakpoints
               PrintStr load_wait
               do    load_mem_record
                  do     wait_colon    ;wait for start of record
                     rcall read_serial_esc
                     cpi   a,':'
                  loopne wait_colon
                  clr   xh             ;checksum
                  rcall read_byte_esc  ;read count
                  mov   xl,a
                  ldi   a,13
                  rcall PrintChr
                  rcall read_byte_esc  ;read address - big endian
                  mov   zh,a
                  rcall PrintHex
                  rcall read_byte_esc
                  mov   zl,a
                  rcall PrintHex
                  rcall read_byte_esc  ;read function
                  cpi   a,0            ;load function
                  ifeq_and load_mem_data
                  tst   xl             ;count > 0
                  ifne     load_mem_data
                     out   cbus,clear        ;readmode off
                     out   dbusddr,allon     ;dbus = output
                     do     load_data
                        rcall read_byte_esc     ;read data
                        out   abuslo,zl 
                        out   abushi,zh
                        out   cbus,writemem     ;WE, ~OE
                        out   dbusout,a         ;dbus write
                        adiw  z,1
                        out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
                        dec   xl
                     loopne load_data
                     out   dbusddr,zero      ;dbus = input
                     out   cbus,readmem      ;OE / read mode back on
                     rcall read_byte_esc     ;read checksum
                     tst   xh
                     ifne  load_error_checksum
                        PrintStr err_chksum     ;" Checksum failed"
                        rjmp  discard_serial_stream
                     end   load_error_checksum
                  else     load_mem_data
                     cpi   a,1               ;end of record function
                     ifeq  load_complete
                        tst   zh                ;address is valid PC?
                        ifne  load_pc
                           movw  pcl:pch,z         ;set PC
                        end   load_pc
                        PrintStr load_ok        ;13,"Load OK"
                        rjmp  discard_serial_stream   ;OK message - wait for eol
                     else  load_complete
                        PrintStr err_func       ;" Invalid function or count in record"
                        rjmp  discard_serial_stream
                     end   load_complete
                  end      load_mem_data
               loop  load_mem_record
               rjmp  end_command
            end      load_mem
;
; H = Halt Emulator execution   Syntax: H<CR>
;
            cpi   a,'H'
            ifeq  halt_emu
               ld    a,y+           ;check validity (no parms)
               cpse  a,zero
               rjmp  invalid_command
               cbr   flags,(1<<emu_run)   
               rjmp  end_command
            end   halt_emu
;
; G = Go/Start Emulator execution   Syntax: G<CR>
;
            cpi   a,'G'
            ifeq  start_emu
               ld    a,y+           ;check validity (no parms)
               cpse  a,zero
               rjmp  invalid_command
               ldi   yl,low(cmd_buf)
               sts   cmd_inx,yl
               ldi   a,10        ;linefeed
               rcall PrintChr
               rcall show_regs
               sbr   flags,(1<<emu_run)   
               rcall show_prompt
               cli                        ;debug instance exiting
               cbr   flags,(1<<deb_act)   ;allow t0 to call again
emu_start:
               out   abuslo,pcl        ;opcode fetch - no interrupts
               out   abushi,pch
                     ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
                     IRQ_restore             ;enable NMI, IRQ if allowed
               wait_data_valid 5       ;300ns minimum @ 16MHz
               in    opcode,dbusin
               cpi   opcode,0xdb       ;potential breakpoint?
               ifeq  emu_start_brk
                  rcall brkpt_chk_pc      ;get original opcode
               end   emu_start_brk
               adiw  pch:pcl,1         ;pc -> op low
               out   abuslo,pcl        ;operand address low prefetch
               out   abushi,pch
               wait_data_valid 5       ;300ns minimum @ 16MHz 
                     adiw  pch:pcl,1         ;pc -> op high
                     ijmp                    ;execute opcode
            end   start_emu
;
; X = Exit Debugger / start Emulator execution   Syntax: X<CR>
;
            cpi   a,'X'
            ifeq  exit_deb
               ld    a,y+           ;check validity (no parms)
               cpse  a,zero
               rjmp  invalid_command
               rcall crlf   
               cli
               ldi   flags,(1<<emu_run)   ;clear all other flags
               rjmp  emu_start
            end   exit_deb
;
; R = Reset emulation registers & set PC to reset vector   Syntax: R<CR>
;
            cpi   a,'R'
            ifeq  deb_reset
               ld    a,y+           ;check validity (no parms)
               cpse  a,zero
               rjmp  invalid_command
               rcall soft_reset
               rjmp  end_command
            end   deb_reset

;**************************************************
;
; end parsing & executing commandline
;
;************************************************** 
 
; Invalid command

invalid_command:
            sbiw  y,1               ;caused by?
            ld    a,y
            cpse  a,zero            ;end of line?
            adiw  y,1               ;last character not eol
            ldi   a,'?'             ;mark invalid commannd
            st    y,a
            ldi   yl,low(cmd_buf)
            rcall show_regs
            rcall show_prompt
            do    deb_invalid
               ld    a,y+
               rcall PrintChr
               cpi   a,34              ;test for string input
               ifeq  inv_string_mark
                  ldi   b,(1<<string)  ;switch case translation
                  eor   flags,b
               end   inv_string_mark
               cpi   a,'?'
            loopne   deb_invalid
            sts   cmd_inx,yl
            rcall erase2eol         ;esc[K - clear rest of line
         
            rjmp  debug_main        ;loop with invalid command
            
; Valid command            
            
end_command:                  ;new prompt
            ldi   yl,low(cmd_buf)
            sts   cmd_inx,yl
            ldi   a,10        ;linefeed
            rcall PrintChr
            rcall show_regs
            rcall show_prompt
         end      deb_cr
      end   deb_esc_seq
   loop  debug_main
   cli                        ;debug instance exiting
   cbr   flags,(1<<deb_act)   ;allow t0 to call again
end_exit:
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   IRQ_restore                ;allow NMI, IRQ only if allowed
   op_decode                  ;start processing
 
;*****************************************************************
;
; debugger/monitor output to terminal
;
;*****************************************************************

; Hex Output to RS232, reverse video if T flag
RevHex:     push  a
            swap  a
            rcall revx_dgt
            pop   a
revx_dgt:   andi  a,0x0f
            addi  a,'0'           ;ASCII "0-9"
            cpi   a,'9'+1         ;ASCII "A-F"
            brlo  RevChr
            addi  a,7

RevChr:     brtc  PrintChr          ;print character reverse video if t set
            sbrc  flags,rvs_vid
            ifs   rvs_video_off
               sbr   flags,(1<<rvs_vid)
               push  a
               rcall esc91             ;ESC[7m - reverse video character on
               ldi   a,'7'
               rcall prtc
               ldi   a,'m'
               rcall prtc
               pop   a                 ;print saved character
            end   rvs_video_off
            rjmp  prtc

erase2eol:  rcall esc91             ;ESC[K - clear rest of line
            ldi   a,'K'
            rjmp  prtc

esc91:      rcall escape            ;ESC[
            ldi   a,91
            rjmp  prtc

escape:     ldi   a,27              ;ESC
            rjmp  prtc

space:      ldi   a,0x20            ;print space
            rjmp  PrintChr

colon:      ldi   a,0x3a
            rjmp  PrintChr

crlf:       ldi   a,10              ;print carriage return, linefeed
            rcall PrintChr
            ldi   a,13
            rjmp  PrintChr

; Print String (called by PrintStr Macro)
prtstr:
   do    print_string
      lpm   a,z+
      tst   a
      exiteq print_string
      rcall PrintChr
   loop  print_string
   ret

; Hex Output to RS232
PrintHex:   push  a
            swap  a
            rcall prtx_dgt
            pop   a
prtx_dgt:   andi  a,0x0f
            addi  a,'0'           ;ASCII "0-9"
            cpi   a,'9'+1         ;ASCII "A-F"
            brlo  PrintChr
            addi  a,7

; Character output to RS232
PrintChr:
   sbrs  flags,rvs_vid
   ifs   reverse_video_off
      push  a
      cbr   flags,(1<<rvs_vid)
      rcall esc91             ;ESC[m - character video attributes off
      ldi   a,'m'
      rcall prtc
      pop   a
   end   reverse_video_off
prtc:                   ;bypass of rvs video mode check
      push  yl
      push  yh
   ifie  prt_fifo       ;fifo only when enabled
      push  c
      lds   yl,tx_inx      ;load buffer pointer
      ldi   yh,high(tx_buf)
      do     prt_fifo_full    ;wait for space in fifo
         lds   c,tx_fill
         inc   c              ;check for overflow
      loopeq prt_fifo_full
      cli
      lds   c,tx_fill
      ldi   i,usart_txi_ena   ;notify transmitter
      out   ucsrb,i
      .ifdef irq_dis_real
         sts   usart_ena,i
      .endif
      st    y+,a           ;store in fifo
      inc   c              ;update pointers
      sts   tx_inx,yl
      sts   tx_fill,c
      sei
      pop   c
   else  prt_fifo 
      lds   k,tx_fill         ;empty buffer before printing direct?
      do       prt_fifo_disabled
         tst   k
      exiteq   prt_fifo_disabled
         ldi   yh,high(tx_buf)
         lds   yl,tx_inx         ;calc output index
         sub   yl,k
         ld    i,y               ;load from fifo head
         do    prt_fifo_wait_udre
            sbis  ucsra,udre
         loop  prt_fifo_wait_udre
         out   udr,i
         dec   k                 ;update pointer
      loop     prt_fifo_disabled
      sts   tx_fill,k         ;buffer now empty
      ldi   i,usart_txi_dis   ;stop rupt
      out   ucsrb,i
      .ifdef irq_dis_real
         sts   usart_ena,i
      .endif
      do    prt_wait_udre     ;direct output
         sbis  ucsra,udre
      loop  prt_wait_udre
      out   udr,a
   end   prt_fifo
   pop   yh
   pop   yl
   ret

;*****************************************************************
;
; Soft Reset
;     clears pending interrupts, emulated registers & status
;     loads reset vector to PC
;
;*****************************************************************

soft_reset:
   sts   irq_mask,zero     ;disable all internal IRQ
   sts   irq_flag,zero
   cbi   cbusddr,3
   sbrc  flags,deb_on      ;acia connected?
   ifs   sr_chk_tdre
      ldi   a,tx_fill      ;tx buffer full?
      inc   a
      ifne  sr_set_tdre
         ldi   a,0b10000000   ;set tx buffer empty
         sts   irq_flag,a
      end   sr_set_tdre
   end   sr_chk_tdre
   .ifdef   iomap
      call  io_reset             ;reset IO
      ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   .endif
   sts   selftest,zero     ;clear diag forced interrupts
   .ifdef  nmi_ena
      ldi   a,0b11110000      ;Memory R/W signals + IO-select 1-2
   .else
      ldi   a,0b11110100      ;Memory R/W signals + IO-select 1-3
   .endif
   out   cbusddr,a
   ldi   a,0xc0            ;clear pending int0, int1 (NMI, IRQ)
   out   gifr,a
   sts   dma_last_cmd,allon   ;dma setup required
   clr   rega              ;clear registers
   clr   regx
   clr   regy
   mov   spointer,allon
   clr   stat
   ldi   stat2,0b00100100  ;B=0, D=0, I=1
load_reset_vector:
   ldi   oplow,low(0xfffc) ;reset vector
   ldi   ophigh,high(0xfffc)
   out   abuslo,oplow      ;indirect address low prefetch
   out   abushi,ophigh
   wait_data_valid 1       ;60ns minimum @ 16MHz 
         adiw  opointer,1  ;pc -> adr high
   in    pcl,dbusin        ;save low pointer to pc     
   out   abuslo,oplow      ;fetch high pointer to pc
   wait_data_valid 0       ;0ns minimum @ 16MHz
   in    pch,dbusin        ;point to new pc
   ret
;*****************************************************************
;
; show 6502 registers
;
;     shows register, sets refresh to 500ms
;     output: 1ss pppp aa Xxx Yyy ffffffff r>
;     ss = stack pointer, pppp = program counter, aa = accumulator
;     xx = Index X, yy = Index Y, ffffffff = bitwise processor status
;     r  = runmode                           (1 = reverse video)
;
;*****************************************************************

show_regs:
   ldi   a,50            ;refresh timer 500ms (50 * 10)
   sts   cmd_reg_timer,a
   ldi   a,13            ;return
   rcall PrintChr
   ldi   a,'1'           ;stackpointer
   rcall PrintChr
   mov   a,spointer
   rcall PrintHex
   rcall space
   sbrs  flags,emu_run
   ifs   show_reg_run   ;show breakpoint if halted
      clt
   else  show_reg_run
      rcall brkpt_chk_pc
   end   show_reg_run
   mov   a,pch           ;program counter
   rcall RevHex
   mov   a,pcl
   rcall RevHex
   rcall space
   mov   a,rega          ;accumulator
   rcall PrintHex
   rcall space
   ldi   a,'x'           ;index register X
   rcall PrintChr
   mov   a,regx
   rcall PrintHex
   rcall space
   ldi   a,'y'           ;index register y
   rcall PrintChr
   mov   a,regy
   rcall PrintHex
   rcall space   
   ldi   a,'N'           ;processor status
   bst   stat,2
   rcall RevChr
   ldi   a,'V'
   bst   stat2,6
   rcall RevChr
   ldi   a,'-'           ;don´t care
   rcall PrintChr
   ldi   a,'B'           ;only on stack
   rcall PrintChr
   ldi   a,'D'
   bst   stat2,3
   rcall RevChr
   ldi   a,'I'
   bst   stat2,2
   rcall RevChr
   ldi   a,'Z'
   bst   stat,1
   rcall RevChr
   ldi   a,'C'
   bst   stat,0
   rcall RevChr
   rjmp  space

show_prompt:
   sbrs  flags,emu_run
   ifs   show_runmode
      ldi   a,'G'          ;go / running
   else  show_runmode
      ldi   a,'H'          ;halted / stopped
   end   show_runmode
   rcall PrintChr
   ldi   a,'>'
   rjmp  PrintChr

;*****************************************************************
;
; disassemble instruction
;
;     expects addresss in y
;     returns b = 7 if invalid instruction
;     uses a c d z
;
;*****************************************************************

disasm:
   out   abuslo,yl         ;memory fetch
   out   abushi,yh
   clr   d                 ;load index to opcode table
   clt                     ;clear reverse video
   movw  z,y
   wait_data_valid 3       ;180ns minimum @ 16MHz
   in    a,dbusin
   cpi   a,0xdb            ;is potential breakpoint?
   ifeq  show_dis_brk
      rcall brkpt_chk
      movw  y,z
   end   show_dis_brk
   mov   c,a               ;opcode
   adiw  y,1               ;next byte
   lsl   c                 ;*4 / each entry 4 bytes
   rol   d
   lsl   c
   rol   d
   ldi   zl,low(dis_opcode<<1)
   ldi   zh,high(dis_opcode<<1)
   add   zl,c
   adc   zh,d
   lpm   a,z+              ;show mnemonics
   rcall RevChr
   lpm   a,z+
   rcall RevChr
   lpm   a,z+
   rcall RevChr
   rcall space
   lpm   b,z               ;show addressing
   andi  b,0b1100000       ;mask prefix
   cpi   b,0b0100000       
   ifeq  dis_immediate
      ldi   a,'#'
      rcall PrintChr
   end   dis_immediate
   cpi   b,0b1000000
   ifeq  dis_pre_indirect
      ldi   a,'('
      rcall PrintChr
   end   dis_pre_indirect
   cpi   b,0b1100000
   ifeq  dis_bitnum
      in    a,dbusin          ;opcode -> bitnum
      swap  a
      andi  a,7
      addi  a,'0'
      rcall PrintChr
      ldi   a,','
      rcall PrintChr
   end   dis_bitnum
   lpm   b,z
   andi  b,0b0011000       ;mask op addressing
   cpi   b,0b0001000             
   ifeq  dis_data_one      ;zero page or immediate
      out   abuslo,yl         ;memory fetch
      out   abushi,yh
      adiw  y,1
      wait_data_valid 1       ;60ns minimum @ 16MHz
      in    a,dbusin
      rcall PrintHex
   end   dis_data_one
   cpi   b,0b0010000             
   ifeq  dis_data_abs      ;2 Byte absolute (reverse order)
      out   abuslo,yl         ;memory fetch
      out   abushi,yh
      adiw  y,1
      wait_data_valid 1       ;60ns minimum @ 16MHz
      in    c,dbusin
      out   abuslo,yl         ;memory fetch
      out   abushi,yh
      adiw  y,1
      wait_data_valid 1       ;60ns minimum @ 16MHz
      in    a,dbusin
      rcall PrintHex
      mov   a,c
      rcall PrintHex
   end   dis_data_abs
   cpi   b,0b0011000             
   ifeq  dis_data_rel      ;1 Byte relative (calculated absolute)
      rcall dis_rel
   end   dis_data_rel
   lpm   b,z
   andi  b,0b0000111       ;mask suffix
   cpi   b,0b001           ;,X
   ifeq  dis_x
      ldi   a,','
      rcall PrintChr
      ldi   a,'X'
      rcall PrintChr
   end   dis_x
   cpi   b,0b010           ;,Y
   ifeq  dis_y
      ldi   a,','
      rcall PrintChr
      ldi   a,'Y'
      rcall PrintChr
   end   dis_y
   cpi   b,0b011           ;)
   ifeq  dis_i
      ldi   a,')'
      rcall PrintChr
   end   dis_i
   cpi   b,0b100           ;,X)
   ifeq  dis_xi
      ldi   a,','
      rcall PrintChr
      ldi   a,'X'
      rcall PrintChr
      ldi   a,')'
      rcall PrintChr
   end   dis_xi
   cpi   b,0b101           ;),Y
   ifeq  dis_iy
      ldi   a,')'
      rcall PrintChr
      ldi   a,','
      rcall PrintChr
      ldi   a,'Y'
      rcall PrintChr
   end   dis_iy
   cpi   b,0b110           ;A
   ifeq  dis_a
      ldi   a,'A'
      rcall PrintChr
   end   dis_a
   cpi   b,0b111           ;invalid instruction or BBR/BBS
   ifeq  dis_nv
      .ifdef cmos_core
         ldi   a,','
         rcall PrintChr
         rcall dis_rel
      .else
         PrintStr inv_instr
      .endif
   end   dis_nv
   ret

;fetch relative address and display as absolute
dis_rel:
   out   abuslo,yl         ;memory fetch
   out   abushi,yh
   adiw  y,1
   clr   a
   wait_data_valid 2       ;120ns minimum @ 16MHz
   in    c,dbusin
   tst   c
   ifmi  dis_rel_sign
      dec   a                 ;extend sign to high
   end   dis_rel_sign
   add   c,yl              ;calculate & show absolute
   adc   a,yh
   rcall PrintHex
   mov   a,c
   rjmp  PrintHex

;*****************************************************************
;
; get hex parameters
;
;*****************************************************************

;ASCII to hex nibble conversion
get_hex:
   subi  a,'0'           ;hex 0-9
   cpi   a,10
   ifsh  hex_a_f
      cpi   a,17         ;non hex >0 <A
      brlo  inv_cmd
      subi  a,'A'-'0'-10 ;hex A-F
      cpi   a,16         ;non hex >F
      brsh  inv_cmd
   end   hex_a_f
   ret
;
; get 1 hex byte from commandline
;     expects 2 Hex digits, eol
;     returns hex byte in a
;
get_byte:
   do     byte_parse
      ld    a,y+
      tst   a            ;premature end of input
      breq  inv_cmd
      cpi   a,32         ;skip blanks
   loopeq byte_parse
   rcall get_hex        ;high nibble
   mov   c,a
   swap  c
   ld    a,y+
   tst   a
   breq  inv_cmd
   rcall get_hex        ;low nibble
   or    a,c
   ld    d,y+           ;end of line?
   tst   d
   brne  inv_cmd
   ret               
;
; get nth hex byte from commandline
;     expects 2 Hex digits
;     returns hex byte in a
;
get_wbyte:
   rcall get_hex        ;high nibble
   mov   c,a
   swap  c
   ld    a,y+
   tst   a
   breq  inv_cmd
   rcall get_hex        ;low nibble
   or    a,c
   ret               

inv_cmd:
   lds   a,sp_save      ;restore sp to main loop
   out   spl,a
   lds   a,sp_save+1
   out   sph,a
   rjmp  invalid_command   ;immediate end of command

;
; get hex address from commandline, single parameter
;     expects y pointing to 1-4 hex digits, end of line
;     uses a b
;     returns address in z
;
get_adr:
   do     adr_parse
      ld    a,y+
      tst   a               ;premature end of input
      breq  inv_cmd
      cpi   a,32            ;skip blanks
   loopeq adr_parse
   rcall get_hex        ;1st nibble
   clr   zh
   mov   zl,a
   do        adr_nibble
      ld    a,y+            ;next nibble
      tst   a
      exiteq adr_nibble
      rcall get_hex
      ldi   b,4
      do    adr_shift_nibble
         lsl   zl
         rol   zh
         brcs  inv_cmd        ;exceeds 16bit
         dec   b
      loopne adr_shift_nibble
      or    zl,a
   loop      adr_nibble
   ret               
;
; get hex address from commandline, multiple parameter
;     expects 1-4 hex digits, space
;     returns address in z
;
get_wadr:
   do     wadr_parse
      ld    a,y+
      tst   a                 ;premature end of input
      breq  inv_cmd
      cpi   a,32              ;skip blanks
   loopeq wadr_parse
   rcall get_hex           ;1st nibble
   clr   zh
   mov   zl,a
   do        wadr_nibble
      ld    a,y+              ;next nibble
      cpi   a,32
      exiteq wadr_nibble
      rcall get_hex
      ldi   b,4
      do    wadr_shift_nibble
         lsl   zl
         rol   zh
         brcs  inv_cmd           ;exceeds 16bit
         dec   b
      loopne wadr_shift_nibble
      or    zl,a
   loop      wadr_nibble
   ret               

;*****************************************************************
;
; breakpoint utilities
;
;*****************************************************************
;
; breakpoint info - list active breakpoints
;     uses a c y z
;
brkpt_info:
   ldi   yl,low(brkpt_lo)  ;any breakpoints active?
   ldi   yh,high(brkpt_lo)
   clr   c
   do       br_find_actv
      ldd   zh,y+10           ;brkpt_hi set?
      cpse  zh,allon
      inc   c
      ld    zl,y+             ;brkpt_lo set?
      cpse  zl,allon
      inc   c
      cpi   yl,low(brkpt_hi)
   loopne   br_find_actv
   cpse  c,zero
   ifs   br_info_none
      PrintStr bpt_info_none  ;10,13,"No breakpoints active"
   else  br_info_none
      PrintStr bpt_info       ;10,13,"Breakpoints  (slot#:address)",13,10
      ldi   yl,low(brkpt_lo)  ;find breakpoints
      do       br_info_all
         ld    zl,y              ;brkpt_lo set?
         ldd   zh,y+10           ;brkpt_hi set?
         cp    zl,allon
         ifne_or  br_info_slot
         cp    zh,allon
         ifne     br_info_slot
            rcall space
            mov   a,yl              ;slot#
            subi  a,low(brkpt_lo) - '0'
            rcall PrintChr
            rcall colon
            mov   a,zh              ;address
            rcall PrintHex
            mov   a,zl
            rcall PrintHex
         end      br_info_slot
         adiw  y,1               ;next slot
         cpi   yl,low(brkpt_hi)
      loopne   br_info_all
   end   br_info_none
   ret
;
; clear all breakpoints
;     uses a y z
;     returns c = count of slots cleared
;
brkpt_clr_all:
   ldi   yl,low(brkpt_lo)  ;find breakpoints
   ldi   yh,high(brkpt_lo)
   clr   c
   do       br_clr_all_slots
      rcall brkpt_clr_one
      adiw  y,1               ;next slot
      cpi   yl,low(brkpt_hi)
   loopne   br_clr_all_slots
   tst   c
   ifne  br_clr_all_msg
      PrintStr bpt_clrd_msg   ;10,13,"All breakpoints cleared"
   end   br_clr_all_msg
   ret
;
; clear 1 breakpoint slot
;     uses a y z
;     returns c+1 if slot was active / got cleared
;
brkpt_clr_one:
   ld    zl,y              ;brkpt_lo set?
   ldd   zh,y+10           ;brkpt_hi set?
   cp    zl,allon
   ifne_or  br_clr_one_slot
   cp    zh,allon
   ifne     br_clr_one_slot
      ;write original opcode back to its RAM location
      ldd   a,y+20            ;brkpt_op
      out   abuslo,zl 
      out   abushi,zh
      out   cbus,writemem     ;WE, ~OE
      out   dbusout,a         ;dbus write
      out   dbusddr,allon     ;dbus = output
      out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
      out   dbusddr,zero      ;dbus = input
      out   cbus,readmem      ;OE / read mode back on
      st    y,allon           ;reset brkpt_lo
      std   y+10,allon        ;reset brkpt_hi
      inc   c
   end      br_clr_one_slot
   ret
;
; check STP is breakpoint
;     expects current address in z
;     on match replace a with original opcode
;      "   "   set T flag to signal reverse video
;     uses c y
;
brkpt_chk:
   ldi   yl,low(brkpt_lo)  ;match breakpoint address
   ldi   yh,high(brkpt_lo)
   do       br_chk_adr
      ld    c,y+              ;brkpt_lo match?
      cp    c,zl
      ifeq_and br_slot_match
      ldd   c,y+9             ;brkpt_hi match?
      cp    c,zh
      ifeq     br_slot_match
         cp    zl,allon
         ifne_or  br_slot_valid
         cp    zh,allon
         ifne     br_slot_valid
            ldd   a,y+19            ;show original opcode
            set                     ;signal breakpoint = reverse video
         end      br_slot_valid
         ret
      end      br_slot_match
      cpi   yl,low(brkpt_hi)
   loopne   br_chk_adr
   ret
;
; check PC is breakpoint
;     expects current address in pcl:pch
;     on match replace opcode with original opcode
;     T flag = breakpoint true/false
;     uses c
;
brkpt_chk_pc:
   push  yl
   push  yh
   ldi   yl,low(brkpt_lo)  ;match breakpoint address
   ldi   yh,high(brkpt_lo)
   clt
   do       br_pc_adr
      ld    c,y+              ;brkpt_lo match?
      cp    c,pcl
      ifeq_and br_pc_match
      ldd   c,y+9             ;brkpt_hi match?
      cp    c,pch
      ifeq     br_pc_match
         cp    pcl,allon
         ifne_or  br_pc_valid
         cp    pch,allon
         ifne     br_pc_valid
            ldd   opcode,y+19       ;show original opcode
            set                     ;signal breakpoint = reverse video
         end      br_pc_valid
         rjmp  br_chkp_exit
      end      br_pc_match
      cpi   yl,low(brkpt_hi)
   loopne   br_pc_adr
br_chkp_exit:
   pop   yh
   pop   yl
   ret
;
; check write memory to breakpoint address
;     expects current address in z
;     on match saves a as original opcode
;      "   "   replaces a with STP opcode
;     uses c
;
brkpt_chk_write:
   push  yl
   push  yh
   ldi   yl,low(brkpt_lo)  ;match breakpoint address
   ldi   yh,high(brkpt_lo)
   do       br_chkw_adr
      ld    c,y+              ;brkpt_lo match?
      cp    c,zl
      ifeq_and br_slotw_match
      ldd   c,y+9             ;brkpt_hi match?
      cp    c,zh
      ifeq     br_slotw_match
         cp    zl,allon
         ifne_or  br_slotw_valid
         cp    zh,allon
         ifne     br_slotw_valid
            std   y+19,a            ;save original opcode
            ldi   a,0xdb            ;write STP opcode to memory
         end      br_slotw_valid
         rjmp  br_chkw_exit
      end      br_slotw_match
      cpi   yl,low(brkpt_hi)
   loopne   br_chkw_adr
br_chkw_exit:
   pop   yh
   pop   yl
   ret
;
;*****************************************************************
;
; load and EEPROM save
;
;*****************************************************************
;
; RS232 read from RX fifo or direct and exit on <ESC>
;     uses d, (i with cli)
;     updates checksum in xh
;     returns data in a
;
read_serial_esc:
   push  yl
   push  yh
;   push  c
   do    wait_ser_data
      in    d,sreg      ;preserve sreg interrupt enable
      cli
      lds   i,rx_fill   ;check buffer
      tst   i
      ifne  esc_read_buf
         lds   yl,rx_inx      ;prepare address to read rx fifo
         ldi   yh,high(rx_buf)
         sub   yl,i
         ld    a,y            ;read
         dec   i              ;update pointer
         sts   rx_fill,i
         .ifdef flowlo
;            mov   i,c
            cpi   i,flowlo          ;buffer lower watermark?
            iflo  esc_flow
               lds   i,flow_cmd
               cpi   i,0x91         ;xon done?
               ifne  esc_send_xon
                  ldi   i,0x11         ;post xon pending
                  sts   flow_cmd,i
                  ldi   i,usart_txi_ena ;notify transmitter
                  out   ucsrb,i
                  .ifdef irq_dis_real
                     sts   usart_ena,i
                  .endif
               end  esc_send_xon
            end   esc_flow
         .endif
         out   sreg,d
         exit  wait_ser_data
      else  esc_read_buf
         out   sreg,d
         ifid_and  esc_read_direct
         sbis  ucsra,rxc      
         ifs       esc_read_direct
            in    a,udr        ;read direct if disabled & buffer empty
            exit  wait_ser_data
         end       esc_read_direct
      end   esc_read_buf
   loop  wait_ser_data
;   pop   c
   pop   yh
   pop   yl
   cpi   a,27
   ifeq  read_abort
      .ifdef eep_vld
         lds   a,prog_num     ;I2C close required?
         cpi   a,0xff
         ifeq  esc_read_load
            PrintStr load_abort  ;" Load aborted"
            rjmp  skip_main
         else  esc_read_load
            PrintStr save_abort  ;" Save aborted"
eep_close_err:             ;close SPI/I2C after abort or error
            jmp  eep_write_err
         end   esc_read_load
      .else
         PrintStr load_abort  ;" Load aborted"
         rjmp  skip_main
      .endif
   end  read_abort
ret

;load hex nibble
load_hex:
   rcall read_serial_esc
   subi  a,'0'           ;hex 0-9
   cpi   a,10
   ifsh  load_a_f
      cpi   a,17         ;non hex >0 <A
      brlo  inv_hex
      subi  a,'A'-'0'-10 ;hex A-F
      cpi   a,16         ;non hex >F
      brsh  inv_hex
   end   load_a_f
   ret
;non hex message
inv_hex:
   PrintStr err_nonhex  ;" Non-Hex data in record"
.ifdef eep_vld
   lds   a,prog_num     ;SPI/I2C close required?
   cpi   a,0xff
   brne  eep_close_err
.endif
;
; RS232 discard from RX fifo or direct with escape timer
;     uses a, d, (i with cli)
;
; wait for eof (skip remaining records)
discard_serial_stream:
   ldi   yl,6        ;continuous data within 50ms will be discarded
   sts   cmd_esc_timer,yl
   do    discard_serial
      in    d,sreg      ;preserve sreg interrupt enable
      cli
      lds   i,rx_fill   ;check buffer
      tst   i
      ifne  disc_read_buf
         lds   yl,rx_inx      ;prepare address to read rx fifo
         ldi   yh,high(rx_buf)
         sub   yl,c
         ld    a,y            ;read
         dec   i              ;update pointer
         sts   rx_fill,i
         .ifdef flowlo
;            mov   i,c
            cpi   i,flowlo          ;buffer lower watermark?
            iflo  disc_flow
               lds   i,flow_cmd
               cpi   i,0x91         ;xon done?
               ifne  disc_send_xon
                  ldi   i,0x11         ;post xon pending
                  sts   flow_cmd,i
                  ldi   i,usart_txi_ena ;notify transmitter
                  out   ucsrb,i
                  .ifdef irq_dis_real
                     sts   usart_ena,i
                  .endif
               end  disc_send_xon
            end   disc_flow
         .endif
         out   sreg,d
         ldi   yl,6           ;further characters
         sts   cmd_esc_timer,yl
         cpi   a,13           ;EOL
         ifeq  disc_read_keepalive
            ldi   a,0
            rcall PrintChr
         end   disc_read_keepalive
      else  disc_read_buf
         out   sreg,d
         ifid_and  disc_read_direct
         sbis  ucsra,rxc      
         ifs       disc_read_direct
            in    a,udr       ;read direct if disabled & buffer empty
            ldi   yl,6        ;further characters
            sts   cmd_esc_timer,yl
            cpi   a,13
            ifeq  disc_read_keepalive2
               ldi   a,0
               rcall PrintChr
            end   disc_read_keepalive2
         end       disc_read_direct
      end   disc_read_buf
      ifid_and disc_timer_update ;disabled timer update 
      in    yh,tifr
      sbrs  yh,ocf0
      ifs      disc_timer_update
         ldi   yh,0b10           ;clear t1 oc flag
         out   tifr,yh
         lds   yl,cmd_esc_timer  ;test escape timer expired
         andi  yl,0x7f
         ifne  disc_esc_timer_update
            lds   yl,cmd_esc_timer
            dec   yl
            sts   cmd_esc_timer,yl
         end   disc_esc_timer_update
      end      disc_timer_update
      lds   yl,cmd_esc_timer
      tst   yl
   exiteq   discard_serial
   loop     discard_serial
   ldi   yl,0x80           ;re-init
   sts   cmd_esc_timer,yl
; return to mainloop
skip_main:
   .ifdef eep_vld
      call eep_close
   .else
      out   dbusddr,zero      ;dbus = input
      out   cbus,readmem      ;OE / read mode back on
   .endif
   lds   a,sp_save         ;restore sp to main loop
   out   spl,a
   lds   a,sp_save+1
   out   sph,a
   sbrc  flags,deb_act     ;on error
      rjmp  end_command       ;debugger or
   rcall crlf 
   rjmp  end_exit          ;next instruction
;
; Read Hex Byte from RX fifo or direct and exit on <ESC>
read_byte_esc:
   ;push  c
   rcall load_hex       ;high nibble
   swap  a        
   mov   c,a
   rcall load_hex       ;low nibble
   or    a,c
   ;pop   c
   add   xh,a           ;update checksum
   ret

.ifdef eep_vld
; EEPROM info - show directory
eep_cmd_info:
   PrintStr eep_info2   ;" - "
   clr   c              ;count free slots
   clr   a
   out   eearh,one
   do       eep_get_free
      out   eearl,c
      out   eecr,one
      in    b,eedr
      cp    b,allon
      ifeq  eep_inc_free
         inc   a           ;+1 for every free block
         ifeq  eep_100_free
            ldi   a,'1'       ;0x100 free slots
            rcall PrintChr
            clr   a
         end   eep_100_free
      end   eep_inc_free
      inc   c   
   loopne   eep_get_free
   rcall PrintHex       ;2 digits free slots
   PrintStr eep_info3   ;" slots free"
   ldi   zl,1           ;progs/line, next index
   clr   b              ;lowest prog# not listed
   do    eep_list
      ldi   a,0xff         ;prog#, max to init
      do    eep_pslots
         out   eearl,c        ;scan all slots
         out   eecr,one
         in    d,eedr
         cp    d,b            ;not already listed?
         ifsh  eep_next
            cp    d,a            ;count same, new if lower
            ifeq  eep_inc_same
               inc   yl             ;count slots
            end   eep_inc_same
            iflo  eep_find_low
               ldi   yl,1           ;1st slot
               mov   a,d            ;new lowest prog# to list
            end   eep_find_low
         end   eep_next
         inc   c                 ;loop for 256 slots
      loopne   eep_pslots
      cpi   a,0xff            ;no more progs to list
    exiteq  eep_list
      mov   b,a
      dec   zl                ;max progs per line?
      ifeq  eep_list_index
         PrintStr eep_info4   ;"prog#/slots  "
         ldi   zl,10          ;next 10 progs on this line
         mov   a,b            ;restore a
      end   eep_list_index
      inc   b                 ;next lowest prog#
      rcall PrintHex          ;prog#
      ldi   a,'/'
      rcall PrintChr
      mov   a,yl
      rcall PrintHex          ;slotcount
      rcall space
   loop     eep_list
   out   eearh,zero     ;protect eep from unintentional writes
   out   eearl,zero
   ret
   
eep_auto_info:
   PrintStr eep_info1   ;crlf,"EEPROM autoload "
   out   eearh,zero     ;read eep autoload byte
   out   eearl,allon
   out   eecr,one
   in    a,eedr
   cpi   a,0xff         ;test autoload program set
   ifeq  eep_no_auto
      PrintStr eep_none    ;"none"
   else  eep_no_auto
      rcall PrintHex       ;program number
      mov   c,one             ;beginning of string
      out   eearl,c
      out   eecr,one
      in    b,eedr
      cpi   b,0xff         ;string present?
      ifne  eep_auto_info_string
         rcall space
         ldi   a,34           ;quote
         rcall PrintChr
         mov   a,b
         do       eep_auto_info_str2
            rcall PrintChr
            inc   c
            out   eearl,c
            out   eecr,one
            in    a,eedr
            cpi   a,0xff         ;string end?
         loopne   eep_auto_info_str2
         ldi   a,34
         rcall PrintChr
      end   eep_auto_info_string
   end   eep_no_auto
   ret

; internal EEPROM write byte
eep_auto_write:
   out   eedr,a
   in    c,sreg         ;atomic write sequence
   cli
   sbi   eecr,eemwe
   sbi   eecr,eewe
   out   sreg,c
   do    eep_auto_wait  ;write complete?
      sbic  eecr,eewe
   loop  eep_auto_wait
   ret
.endif

; Messages part 2 (part 1 moved before opcode table to reduce empty space)
;                              word count, X = don't use in part 1 ---> ;##
.ifdef cmos_core
stp_instr:     .db   "STP - Emulator halted",0                          ; X
.else
inv_instr:     .db   "Illegal Opcode ",0                                ; X
.endif
emu_msg:       .db   13,10,13,10,core_string," Emulator V",version,0    ; X
built_msg:     .db   " built ",__DATE__," ",__TIME__,0                  ;14
;load_wait:     .db   13,10,"Loading, <ESC> to abort",13,10,0            ;14
;load_abort:    .db   " Load aborted",0                                  ; 7
err_chksum:    .db   " Checksum failed",0,0                             ; 9
rs_vect_empty: .db   13,10,"Check reset vector",0,0                     ;11
;reset_msg:     .db   "  Reset",0                                        ; 4
;bpt_clrd_msg:  .db   10,13,"All breakpoints cleared",0                  ;13
;bpt_info:      .db   10,13,"Breakpoints  (slot#:address)",13,10,0,0     ;17
;bpt_info_none: .db   10,13,"No breakpoints active",0                    ;12
;bpt_slot_full: .db   10,13,"No more breakpoint slots available",0,0     ;19
.ifdef irq_dis_real    ;more messages not in part 1
load_ok:       .db   13,"Load OK",0,0                                   ; 5
err_nonhex:    .db   " Non-Hex data in record",0                        ;12
err_func:      .db   " Invalid function or count in record",0,0         ;19
illegal_int:   .db   13,10,"AVR Illegal Interrupt",0                    ;12
back_line:     .db   13,27,91,"K",27,91,"1A",0,0                        ; 5
.endif


;verify minimum versions of includes
;required versions
.if   io_version < 831
   .error "6502_Emu_IO.inc is below the required minimum version!"
.endif
.ifdef   cmos_core
   .if   core_version < 831
      .error "6502_Emu_CMOS.inc is below the required minimum version!"
   .endif
.else
   .if   core_version < 831
      .error "6502_Emu_NMOS.inc is below the required minimum version!"
   .endif
.endif
.if   config_version < 830
   .error "6502_Emu_config.inc is below the required minimum version!"
.endif
.if   sam_version < 810
   .error "sam.inc is below the required minimum version!"
.endif
;above base versions
.if   io_version > 831
   .error "6502_Emu_IO.inc is above the base version!"
.endif
.ifdef   cmos_core
   .if   core_version > 831
      .error "6502_Emu_CMOS.inc is above the base version!"
   .endif
.else
   .if   core_version > 831
      .error "6502_Emu_NMOS.inc is above the base version!"
   .endif
.endif
.if   config_version > 831
   .error "6502_Emu_config.inc is above the base version!"
.endif
.if   sam_version > 831
   .error "sam.inc is above the base version!"
.endif
;recommended versions
.if   io_version < 831
   .warning "6502_Emu_IO.inc is below the recommended minimum version!"
.endif
.ifdef   cmos_core
   .if   core_version < 831
      .warning "6502_Emu_CMOS.inc is below the recommended minimum version!"
   .endif
.else
   .if   core_version < 831
      .warning "6502_Emu_NMOS.inc is below the recommended minimum version!"
   .endif
.endif
.if   config_version < 830
   .warning "6502_Emu_config.inc is below the recommended minimum version!"
.endif
.if   sam_version < 810
   .warning "sam.inc is below the recommended minimum version!"
.endif
