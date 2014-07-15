;
; 6502 EMULATOR include
;     I/O page (iomap) address decode & emulated I/O
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
.set  io_version = 830
;*****************************************************************
;
; IO macros
;
;*****************************************************************

;I/O address least significant nibble address decode 
;
;@0 = table of 16 addresses for decoded nibble (0x0 - 0xf)
.macro   io_adr_dec
   .if (@0 & 0xff) > 0xf0
      .error "Address decode table @0 crosses a page boundary!"
   .endif
   ldi   zh,high(@0)       ;register address decode
   mov   zl,oplow          ;lower byte
   andi  zl,0xf            ;lower 4 bits
   addi  zl,low(@0)        ;add offset
   ijmp                    ;address specific code
.endmacro

;macro set IO select pin
;
;@0 = io_strobe pin direct (1-3) or extended (10-17, 20-27, 30-37)
.macro   set_io_select
   out   cbus,clear        ;turn off -OE to RAM
   .set io_select = @0
   .if   io_select > 3     ;generate strobe
      .set  io_select = io_select -10
      .if   io_select < 8 
         ldi   b,ios1_default    ;precharge IO-select register  
         .set  io_strobe_pol = !((ios1_default >> io_select) & 1)
         .set  io_sel_direct = 6 ;bit to enable extension register
      .else
         .set  io_select = io_select -10
         .if   io_select < 8 
            ldi   b,ios2_default    ;precharge IO-select register
            .set  io_strobe_pol = !((ios2_default >> io_select) & 1)
            .set  io_sel_direct = 7 ;bit to enable extension register
         .else
            .set  io_select = io_select -10
            ldi   b,ios3_default    ;precharge IO-select register
            .set  io_strobe_pol = !((ios3_default >> io_select) & 1)
            .set  io_sel_direct = 2 ;bit to enable extension register
         .endif
      .endif
      out   abushi,b      
      sbi   cbus,io_sel_direct   ;make register transparent
   .else
      .set io_sel_direct = 0
      .if (io_select == 3)       ;translate to pin
         .set io_select = 2
      .else
         .set io_select = io_select + 5
      .endif
   .endif
.endmacro

;macro set IO selectpin and keep reset pin low
;
;@0 io_strobe pin direct (1-3) or extended (10-17, 20-27, 30-37)
.macro   set_io_reset
   out   cbus,clear        ;turn off -OE to RAM
   .set io_select = @0
   .if   io_select > 3     ;generate strobe
      .set  io_select = io_select -10
      .if   io_select < 8 
         ldi   b,ios1_default    ;precharge IO-select register  
         .set  io_strobe_pol = !((ios1_default >> io_select) & 1)
         .set  io_sel_direct = 6 ;bit to enable extension register
      .else
         .set  io_select = io_select -10
         .if   io_select < 8 
            ldi   b,ios2_default    ;precharge IO-select register
            .set  io_strobe_pol = !((ios2_default >> io_select) & 1)
            .set  io_sel_direct = 7 ;bit to enable extension register
         .else
            .set  io_select = io_select -10
            ldi   b,ios3_default    ;precharge IO-select register
            .set  io_strobe_pol = !((ios3_default >> io_select) & 1)
            .set  io_sel_direct = 2 ;bit to enable extension register
         .endif
      .endif
      .ifdef io_reset_pin
         .if   (@0 & 0x30) == (io_reset_pin & 0x30)
            cbr   b,(1 << (io_reset_pin & 7))
         .endif
      .endif
      out   abushi,b      
      sbi   cbus,io_sel_direct   ;make register transparent
   .else
      .set io_sel_direct = 0
      .if (io_select == 3)       ;translate to pin
         .set io_select = 2
      .else
         .set io_select = io_select + 5
      .endif
   .endif
.endmacro

;macro prepare IO select pin
;use with make_io_select, precharges yl & yh with strobe
;
;@0 = io_strobe pin direct (1-3) or extended (10-17, 20-27, 30-37)
.macro   prep_io_select
   .set io_select = @0
   .if   io_select > 3     ;generate strobe
      .set  io_select = io_select -10
      .if   io_select < 8 
         ldi   b,ios1_default    ;precharge IO-select register  
         ldi   yl,ios1_default^(1<<io_select)
         .set  io_strobe_pol = !((ios1_default >> io_select) & 1)
         .set  io_sel_direct = 6 ;bit to enable extension register
      .else
         .set  io_select = io_select -10
         .if   io_select < 8 
            ldi   b,ios2_default    ;precharge IO-select register
            ldi   yl,ios2_default^(1<<io_select)
            .set  io_strobe_pol = !((ios2_default >> io_select) & 1)
            .set  io_sel_direct = 7 ;bit to enable extension register
         .else
            .set  io_select = io_select -10
            ldi   b,ios3_default    ;precharge IO-select register
            ldi   yl,ios3_default^(1<<io_select)
            .set  io_strobe_pol = !((ios3_default >> io_select) & 1)
            .set  io_sel_direct = 2 ;bit to enable extension register
         .endif
      .endif
      ldi   yh,0x30|(1<<io_sel_direct)
   .else
      .set io_sel_direct = 0
      .if (io_select == 3)       ;translate to pin
         .set io_select = 2
      .else
         .set io_select = io_select + 5
      .endif
      ldi   yh,0x30|(1<<io_select)
   .endif
.endmacro

;macro make IO select pin
;
.macro   make_io_select
   .if (io_sel_direct > 0)
      out   abushi,yl      
   .endif
   out   cbus,yh
.endmacro

;macro enable IO select pin
;
.macro   ena_io_select
   .if (io_sel_direct > 0)
      .if io_strobe_pol
         sbi   abushi,io_select     ;strobe high
      .else
         cbi   abushi,io_select     ;strobe low
      .endif
   .else
      sbi   cbus,io_select       ;strobe high
   .endif
.endmacro

;macro disable IO select pin
;
.macro   dis_io_select
   .if (io_sel_direct > 0)
      out   abushi,b             ;end strobe
   .else
      out   cbus,clear           ;end strobe
   .endif
.endmacro

;macro disable SPI select pin
;
.macro   dis_spi_select
   .if (io_sel_direct > 0)
      out   abushi,b             ;end strobe if io extension
   .endif
;must be followed by
;  out cbus,{readmem|clear}
.endmacro

;macro SPI wait for spdr complete
;
.macro   wait_spi
wait_spdr:
   sbis  spsr,spif
   rjmp  wait_spdr
.endmacro

;macro out @0 to SPI and wait for spdr complete
;
.macro   out_spi
   out   spdr,@0
wait_spdr:
   sbis  spsr,spif
   rjmp  wait_spdr
.endmacro

;macro in @0 from SPI and wait for data ready
;
.macro   in_spi
   out   spdr,allon
wait_spdr:
   sbis  spsr,spif
   rjmp  wait_spdr
   in    @0,spdr
.endmacro

;macro parallel read
;
;@0 minimum read strobe in ns, max 800ns 
;  maximum time to data valid is @0 -1/2 clock, -32ns
.macro   par_rd
   sbi   abuslo,7          ;generate R/-W if needed
   ena_io_select
   wait_ns   @0,1
   dis_io_select
   in    operand,dbusin       ;data latched during previous cycle
   out   cbus,readmem         ;IO-select off & read mode back on
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret
.endmacro

;macro parallel write
;
;@0 minimum write strobe duration in ns, max 800ns
.macro   par_wrt
   cbi   abuslo,7          ;generate R/-W if needed
   out   dbusout,operand   ;precharge data to be written
   out   dbusddr,allon     ;data write mode
   ena_io_select
   wait_ns   @0,1
   dis_io_select
   out   dbusddr,zero      ;data hold ends
.endmacro

;parallel pre read macro - data to a, no return
;
;@0 minimum read strobe in ns, max 800ns 
;  maximum time to data valid is @0 -1/2 clock, -32ns
.macro   par_pre_rd  
   sbi   abuslo,7          ;generate R/-W
   ena_io_select
   wait_ns   @0,1
   dis_io_select
   in    a,dbusin       ;data latched during previous cycle
.endmacro

;phase 2 synchronisation macro
;  next instruction after macro is at tcnt=7, OC2=1
;
;           you are here | at end of macro
;           _ _ _ _ _ _ _V_                 _ _
; Phi2  _ _|               |_ _ _ _ _ _ _ _|
;
; tcnt  6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1
;
; 500ns/phase, 62.5ns/count

.macro   phi2_sync
phi2_syncloop:
      in    zl,tcnt2     
      sbic  pind,7            ;wait for phi2 low
   rjmp  phi2_syncloop
   clr   zh
   subi  zl,low(-phi2_synctable) ;calculate offset cycles
   sbci  zh,high(-phi2_synctable)             
   ijmp

;  .if (PC & 0xff) > 0xf8   ;keep table on same page - avoid adc zh,zero
;      .org  (PC + 0x100) & 0xff00
;   .endif
phi2_synctable:            ;delay -1 cycle for each count in tcnt
   nop                     ;0/1 - tcnt=0/OC2=1                  
   nop                     ;1/1
   nop                     ;2/1
   nop                     ;3/1
   nop                     ;4/1
   nop                     ;5/1
   nop                     ;6/1
;  next                     7/1 - output to next cycle, input from previous cycle
.endmacro

.macro   nop2              ;2 cycle nop
   rjmp  pc+1
.endmacro

;macro phase 2 synchronized read
;
;
;              data latched 1/2 clock ahead of IN +
;      address valid +                            |
;     after sync +   |                            |
;                |   |    _ _ _ _ _ _ _ _ _ _ _ _ | _ _ 
; chip select _ _|_ _|_ _|                        |    |_ _
;         _ _ _ _V_  |              _ _ _ _ _ _ _ V        
; Phi2             |_V_ _ _ _ _ _ _|               |_ _ _ _
;
; tcnt    3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3
;
.macro   phi2_rd
   phi2_sync
   sbi   abuslo,7          ;7/1 generate R/-W - tcnt=7/OC2=1
   ena_io_select           ;1/0 CS strobe >300ns before phi2
   nop2                    ;3/0
   nop2                    ;5/0
   nop2                    ;7/0
   nop2                    ;1/1
   nop2                    ;3/1
   nop2                    ;5/1
   nop                     ;7/1
   in    operand,dbusin    ;0/0 input latched at 7/1 +30ns
   dis_io_select           ;1/0 end strobe 125ns after phi2 te
   out   cbus,readmem      ;IO-select off & read mode back on
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret
.endmacro

;macro phase 2 synchronized write
;
;                    output data valid +
;      address valid +                 |
;     after sync +   |                 |
;                |   |    _ _ _ _ _ _ _|_ _ _ _ _ _ _ 
; chip select _ _|_ _|_ _|             |             |_ _ _
;         _ _ _ _V_  |              _ _V_ _ _ _ _ _        
; Phi2             |_V_ _ _ _ _ _ _|               |_ _ _ _
;
; tcnt    3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3
;
.macro   phi2_wrt
   phi2_sync
   cbi   abuslo,7          ;7/1 generate R/-W - tcnt=7/OC2=1
   ena_io_select           ;1/0 CS strobe high >300ns before phi2
   nop2                    ;3/0
   nop2                    ;5/0
   nop                     ;7/0
   out   dbusout,operand   ;0/1 precharge data to be written
   out   dbusddr,allon     ;1/1 write data valid 125 ns after phi2 le
   nop2                    ;2/1
   nop2                    ;4/1
   nop2                    ;6/1
   dis_io_select           ;0/0 end strobe 60ns after phi2 te
   out   dbusddr,zero      ;data hold ends
   rjmp  iw_exit           ;next instruction
.endmacro

;****************************************************************
;
; Begin IO address decode after load, store or modify instruction
;
;****************************************************************

;address decode for higher nibble of internally mapped address
iomodify:
   sbr   flags,(1<<modify) ;lock extended memory address until rewrite
ioread:
   ldi   zh,high(ior_tab)
   mov   zl,oplow          ;lower byte 
   andi  zl,0xf0           ;upper 4 bits
   swap  zl                ;to low position
   addi  zl,low(ior_tab)   ;add offset
   ijmp                    ;address specific code

;address decode for higher nibble of internally mapped address
iowrite:
   ldi   zh,high(iow_tab)
   mov   zl,oplow          ;lower address byte
   andi  zl,0xf0           ;upper 4 bits
   swap  zl                ;to low position
   addi  zl,low(iow_tab)   ;add offset
   ijmp                    ;address specific code

;**************************************************************
;
; IO RESET
;
;**************************************************************
io_reset:   
   cli
   in    a,timsk        ;no timer interrupts for reset delays
   cbr   a,0b11
   out   timsk,a
   ldi   a,0b00000101   ;timer 0 ctc off, clock / 1024
   out   tccr0,a

   call  rs_IO_ext      ;reset IO extension registers

;*** custom IO resets go here ***
   rcall t1_rs          ;reset timer 1
   rcall irq_rs         ;reset interrupts & flags
   rcall lcd_rs         ;reset character LCD
   rcall spi_rs         ;reset SPI bus if present
   rcall i2c_rs         ;reset I2C bus if present
   ; spi reset with IO expansion select reset (as early as possible)
   io_modules_reset     ;defined by a macro in 6502_Emu_config.inc

;end of reset sequence - restore t0, end HW reset strobe
   out   tcnt0,zero        ;timer 0 init
   ldi   a,ten_ms
   out   ocr0,a
   ldi   a,0b00001101      ;timer 0 ctc, clock / 1024
   out   tccr0,a
   out   tcnt0,zero

   .ifdef   io_reset_pin
      ldi   a,10              ;wait 100ms
      do       reset_delay
         in    c,tifr
         sbrs  c,ocf0
         ifs   reset_10ms_expired
            ldi   b,(1 << ocf0)
            out   tifr,b            ;clear flag
            dec   a                 ;next 10ms
      exiteq   reset_delay
         end   reset_10ms_expired
      loop     reset_delay
      set_io_select  io_reset_pin
      out   cbus,clear
   .endif   

   out   tcnt0,zero        ;restart t0
   ldi   a,0b11            ;t0 compare match interrupt enable
   out   tifr,a
   in    a,timsk
   sbr   a,0b11
   out   timsk,a
   out   cbus,readmem      ;OE / read mode back on
   sei
   ret

.ifdef lcd_sel
;*****************************************************************
;
;character LCD IO
;
;  LCD address pin 7 (0=write, 1=read), pin 0 (0=cmd/busy, 1=data)
;  automatic busy scan for address 0 & 1 if lcd responded during reset
;  raw access at 2 & 3

lcd_rd:                 ;referenced from ior_tab = address
   set_io_select lcd_sel
   lds   a,lcd_flags    ;lcd responded during reset?
   tst   a
   ifne_and lcd_rd_scan
   sbrc  oplow,1        ;address 0 or 1 / not 2 or 3?
   ifs      lcd_rd_scan ;scan busy flag
      cbi   abuslo,0       ;status register
;      do    lcd_rd_busy
      par_pre_rd 230
      sbrc  a,7            ;test status busy
      rjmp  ir_retry       ;on busy
;      loop  lcd_rd_busy
      out   abuslo,oplow   ;back to original address
      ldi   a,32           ;delay 96 cycles = 6탎
      do     lcd_rd_adr    ;for older hd44780 requiring 
         dec   a           ;extra time after busy
      loopne lcd_rd_adr    ;to increment RAM address
   end      lcd_rd_scan
   par_rd 230           ;230ns enable strobe

lcd_wrt:                ;referenced from iow_tab = address
   set_io_select lcd_sel
   lds   a,lcd_flags    ;lcd responded during reset?
   tst   a
   ifne_and lcd_wrt_scan
   sbrc  oplow,1        ;address 0 or 1 / not 2 or 3?
   ifs      lcd_wrt_scan   ;scan busy flag
      cbi   abuslo,0       ;status register
;      do    lcd_wrt_busy
         par_pre_rd 230
         sbrc  a,7            ;test status busy
         rjmp  iw_retry       ;on busy
;      loop  lcd_wrt_busy
      out   abuslo,oplow   ;back to original address
      ldi   a,32           ;delay 96 cycles = 6탎
      do     lcd_wrt_adr   ; for older hd44780 requiring 
         dec   a           ; extra time after busy
      loopne lcd_wrt_adr ; to increment RAM address
   end      lcd_wrt_scan

   par_wrt 230          ;230ns enable strobe
   rjmp  iw_exit        ;next instruction

lcd_rs:
; character LCD - check presence
   ldi   a,0x30         ;reset LCD 8-bit mode
   mov   operand,a
   out   abuslo,zero    ;-w & -rs (write command)
   sts   lcd_flags,allon ;set lcd usable
   set_io_reset lcd_sel     ;select lcd strobe
   ldi   a,0x15         ;15ms power up delay
   rcall lcd_rs_force   ;write 8-bit mode
   ldi   a,0xbf         ;4.1ms delay
   rcall lcd_rs_force   ;repeat write 8-bit mode
   ldi   a,0xfd         ;100탎 delay
   rcall lcd_rs_force   ;repeat write 8-bit mode
                        ;continue init checking busy
   ldi   zl,low(lcd_init<<1) 
   ldi   zh,high(lcd_init<<1)
   ldi   oplow,0        ;command register
   do       lcd_rs_init 
      lpm   a,z+
      tst   a           ;end of string?
   exiteq   lcd_rs_init
      cpi   a,0xff      ;switch to data register?
      ifeq  lcd_rs_text
         ldi   oplow,1  
      else  lcd_rs_text
         rcall lcd_rs_wrt
      end   lcd_rs_text
   loop     lcd_rs_init
   ldi   oplow,0        ;verify text on LCD
   ldi   a,0x80         ;cursor position 0
   rcall lcd_rs_wrt
   ldi   zl,low((lcd_init<<1)+5) 
   ldi   zh,high((lcd_init<<1)+5)
   ldi   oplow,1
   do       lcd_rs_readback
      rcall lcd_rs_rd
      lpm   a,z+
      tst   a
   exiteq   lcd_rs_readback
      cp    a,operand
      ifne  lcd_rs_rd_fail
         sts   lcd_flags,zero
      end   lcd_rs_rd_fail
   loop     lcd_rs_readback
   lds   a,lcd_flags    ;message, if LCD failed
   tst   a
   ifeq  lcd_rs_fail
      PrintStr_far lcd_fail
   end   lcd_rs_fail
   ret

lcd_init:        ;8 bit bus, 2 display lines
                 ;    ;display on, cursor off
                 ;    ;   ;clear & home position
                 ;    ;   ; ;position increment, no shift
                 ;    ;   ; ; ;switch to data (internal)
            .db  0x38,0xc,1,6,0xff,core_string," Emu V",version,0
lcd_fail:   .db   13,10,"LCD reset failed",0,0
            
; subroutines for lcd reset - keep within io_select for lcd
lcd_rs_force:        ;delayed forced write to lcd 
   out   tcnt0,a
   do       rs_lcd_delay
      in    a,tcnt0
      tst   a
   loopne   rs_lcd_delay
   rjmp  lcd_rs_fwrt

lcd_rs_wrt:          ;write to lcd with timeout on busy
   out   tcnt0,zero
   mov   operand,a
   cbi   abuslo,0    ;set status register
   do    lcd_rs_busy
      in    a,tcnt0        ;busy timeout? 
      cpi   a,0x9c         ;>10ms?
      ifeq  lcd_rs_timeout
         sts   lcd_flags,zero
         ret
      end   lcd_rs_timeout
      par_pre_rd 230
      sbrc  a,7            ;test status busy
   loop  lcd_rs_busy
   ldi   a,32           ;delay 96 cycles = 6탎
   do     lcd_rs_adr    ; for older hd44780 requiring 
      dec   a           ; extra time after busy
   loopne lcd_rs_adr    ; to increment RAM address
lcd_rs_fwrt:
   out   abuslo,oplow   ;set register
   par_wrt 230          ;write with 230ns strobe
   ret

lcd_rs_rd:           ;read from lcd with timeout on busy
   out   tcnt0,zero
   cbi   abuslo,0    ;set status register
   do    lcd_rs_rd_busy
      in    a,tcnt0        ;busy timeout? 
      cpi   a,0x9c         ;>10ms?
      ifeq  lcd_rs_rd_timeout
         sts   lcd_flags,zero
         ret
      end   lcd_rs_rd_timeout
      par_pre_rd 230
      sbrc  a,7            ;test status busy
   loop  lcd_rs_rd_busy
   ldi   a,32           ;delay 96 cycles = 6탎
   do     lcd_rs_rd_adr ; for older hd44780 requiring 
      dec   a           ; extra time after busy
   loopne lcd_rs_rd_adr ; to increment RAM address
   out   abuslo,oplow   ;set register
   par_pre_rd  230      ;read with 230ns strobe
   mov   operand,a
   ret
.endif

;*****************************************************************
;
; emulated 6551 ACIA - data register only
;
; ACIA is not available, when the debugger is using the RS232 interface
; use exit command (X) to enable ACIA, use break to return to the debugger
;
;data register, TX on write, RX on read both with 256 byte FIFO buffer
acia_rx:
   lds   a,irq_flag
   cbr   a,0b01000000   ;clear rdrf
   lds   c,irq_mask     ;other interrupts pending & enabled?
   and   c,a
   ifeq  rx_irq_clear
      .ifdef ena_diag
         lds   c,selftest
         sbrs  c,0            ;forced IRQ by selftest
      .endif
      cbi   cbusddr,3      ;clear IRQ
   end   rx_irq_clear
   clr   operand        ;default response
   sbrc  flags,deb_on   ;acia disconnected?
   ifs   acia_rx_ena
      lds   c,rx_fill      ;precharge rx level
      .ifdef irq_dis_real
         sbrs  stat2,2        ;interrupts disabled?
         ifs   acia_rx_bypass
            sbis  ucsra,udre     ;UDR available?
            ifs_and  acia_rx_tx
            lds   d,tx_fill      ;empty buffer?
            tst   d
            ifne     acia_rx_tx
               ldi   xh,high(tx_buf)
               lds   xl,tx_inx      ;calc output index
               sub   xl,d
               ld    b,x            ;load from fifo head
               out   udr,b
               dec   d              ;update pointer
               ifeq  acia_rx_tx_empty
                  cbi   ucsrb,udrie    ;stop rupt
               end   acia_rx_tx_empty
               sts   tx_fill,d
            end      acia_rx_tx
            lds   b,irq_mask     ;check TDRE IRQ enabled
            andi  b,0b10000000
            ifne  acia_rx_tx_int_ena
               sbi   cbusddr,3   ;set IRQ
            end   acia_rx_tx_int_ena
            lds   b,irq_flag
            sbr   b,0b10000000   ;set TDRE
            sts   irq_flag,b     ;store status
            tst   c              ;buffer empty?
            ifeq  acia_rx_direct
               sbic  ucsra,rxc      ;data waiting in UDR?  
                  in    operand,udr    ;read direct if disabled & buffer empty
            end   acia_rx_direct
         end   acia_rx_bypass
      .endif
      tst   c     ;buffer empty?
      ifne  acia_rx_empty
         lds   xl,rx_inx      ;prepare address to read rx fifo
         ldi   xh,high(rx_buf)
         sub   xl,c
         ld    operand,x      ;read
         dec   c     ;update pointer
         sts   rx_fill,c
         ifne  acia_rx_more
            lds   b,irq_mask     ;check acia irq enabled
            andi  b,0b01000000   ;RDRF IRQ enabled ?
            ifne  acia_rx_int
               sbi   cbusddr,3      ;set IRQ
            end   acia_rx_int
            sbr   a,0b01000000   ;set rdrf - more rx data
         end   acia_rx_more
         .ifdef flowlo
            mov   i,c
            cpi   i,flowlo          ;buffer lower watermark?
            iflo  acia_flow
               lds   i,flow_cmd
               cpi   i,0x91         ;xon done?
               ifne  acia_send_xon
                  ldi   i,0x11         ;post xon pending
                  sts   flow_cmd,i
                  sbi   ucsrb,udrie    ;notify transmitter
               end  acia_send_xon
            end   acia_flow
         .endif
      end      acia_rx_empty
      sts   irq_flag,a
   end   acia_rx_ena
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret

acia_tx:
   lds   c,tx_fill      ;precharge level
   .ifdef irq_dis_real
      sbrs  stat2,2        ;interrupts disabled?
      ifs_and  acia_tx_bypass
      sbis  ucsra,udre     ;UDR empty?
      ifs      acia_tx_bypass
         cbr   flags,(1<<deb_on) ;force connect
         tst   c              ;data in buffer
         ifne  acia_tx_filled
            ldi   xh,high(tx_buf)
            lds   xl,tx_inx      ;calc output index
            sub   xl,c
            ld    d,x            ;load from fifo head
            out   udr,d
            dec   c
         else  acia_tx_filled
            out   udr,operand    ;direct output
            rjmp  iw_exit
         end   acia_tx_filled
      end   acia_tx_bypass
   .endif
   sbrc  flags,deb_on   ;acia connected?
iw_retry_ext:
      rjmp  iw_retry       ;disconnected - loop until buffer is available
   inc   c              ;space in buffer ?
   breq  iw_retry_ext   ;full - loop until buffer is available
   lds   a,irq_flag
   cbr   a,0b10000000   ;clear tdre
   lds   d,irq_mask     ;other interrupts pending & enabled?
   and   d,a
   ifeq  tx_irq_clear
      .ifdef ena_diag
         lds   d,selftest
         sbrs  d,0            ;forced IRQ by selftest
      .endif
      cbi   cbusddr,3      ;clear IRQ
   end   tx_irq_clear

   sbi   ucsrb,udrie    ;enable udre interrupt
   lds   xl,tx_inx      ;load pointer
   ldi   xh,high(tx_buf)
   st    x+,operand     ;send data to fifo
   sts   tx_inx,xl      ;save index
   sts   tx_fill,c
   inc   c     ;more tx fifo available?
   ifne  acia_tx_more
      lds   b,irq_mask     ;TDRE IRQ enabled
      andi  b,0b10000000
      ifne  acia_tx_int
         sbi   cbusddr,3      ;set IRQ
      end   acia_tx_int
      sbr   a,0b10000000   ;set tdre - tx buffer available
   end  acia_tx_more
   sts   irq_flag,a
   rjmp  iw_exit

;**************************************************************
;
; Timer 1 registers
;
;  no output or input pins available, internal operations only
;  see ATMega datasheet for register and bit functionality
;
; write address register, then read or write data register
;  output high byte first, automatic decrement to low byte
;  input low byte first, automatic increment to high byte
;  
;  0 = ICR1L      1 = ICR1H      2 = OCR1BL     3 = OCR1BH
;  4 = OCR1AL     5 = OCR1AH     6 = TCNT1L     7 = TCNT1H
;  8 = combined TCCR1 (merged TCCR1A, TCCR1B, SFIOR)
;
;TCCR1 bit assignment
;write [WGM13|WGM12|WGM11|WGM10|PSR10|CS12|CS11|CS10]
;read  [WGM13|WGM12|WGM11|WGM10|  0  |CS12|CS11|CS10]

t1r_adr:
   lds   operand,t1_adr       ;respond with current address
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret

t1w_adr:
   sts   t1_adr,operand
   rjmp  iw_exit

t1r_data:
   lds   xl,t1_adr
   cpi   xl,8            ;TCCR1?
   iflo  t1r_data_word
      addi  xl,0x46        ;add memory offset to t1 registers
      ldi   xh,0
      ld    operand,x
      lds   xl,t1_adr
      eor   xl,one         ;switch between low & high byte
      sts   t1_adr,xl
   else  t1r_data_word
      ifeq  t1r_data_ctl
         lds   operand,t1_ctrl
      else  t1r_data_ctl
         clr   operand
      end   t1r_data_ctl
   end   t1r_data_word
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret

t1w_data:
   lds   xl,t1_adr
   cpi   xl,8            ;TCCR1?
   iflo  t1w_data_word
      addi  xl,0x46        ;add memory offset to t1 registers
      ldi   xh,0
      st    x,operand
      lds   xl,t1_adr
      eor   xl,one         ;switch between high & low byte
      sts   t1_adr,xl
   else  t1w_data_word
      ifeq  t1w_data_ctl
         mov   a,operand      ;test prescaler bits valid
         andi  a,7            ;mask clock select bits
         mov   b,operand
         andi  b,0xf7         ;strip psr10 - is write only
         sts   t1_ctrl,b      ;store for readback
         andi  b,0x30         ;mask wgm11:10
         swap  b              ;move into position
         out   TCCR1A,b
         mov   b,operand      ;mask wgm13:12
         andi  b,0xc0
         swap  b              ;move into position
         lsl   b
         or    a,b            ;merge with clock select
         out   TCCR1B,a
         in    a,SFIOR        ;move psr10 to sfior
         bst   operand,3
         bld   a,0
         out   SFIOR,a
      end   t1w_data_ctl
   end   t1w_data_word
   rjmp  iw_exit

t1_rs:
   out   TCCR1A,zero    ;stop timer
   out   TCCR1B,zero
   sts   t1_adr,zero    ;clear regs
   sts   t1_ctrl,zero
   ret

;timer 1 interrupts
t1_ovi:
   lds   i,irq_flag     ;set t1 overflow
   sbr   i,(1<<2)
   sts   irq_flag,i
   sbi   cbusddr,3      ;set IRQ
   reti
t1_ocra:
   lds   i,irq_flag     ;set t1 output compare flag A
   sbr   i,(1<<4)
   sts   irq_flag,i
   sbi   cbusddr,3      ;set IRQ
   reti
t1_ocrb:
   lds   i,irq_flag     ;set t1 output compare flag B
   sbr   i,(1<<3)
   sts   irq_flag,i
   sbi   cbusddr,3      ;set IRQ
   reti
t1_icr:
   lds   i,irq_flag     ;set t1 input compare flag
   sbr   i,(1<<5)
   sts   irq_flag,i
   sbi   cbusddr,3      ;set IRQ
   reti

;tick countdown timer
;write # of ticks (10ms) to flag/interrupt
;read remaining ticks to next flag/interrupt
;reloads last countdown at bottom - periodic countdown
tcdn_r:
   lds   operand,tcdn_count   ;respond with remaining ticks
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret

tcdn_w:
   sts   tcdn_count,operand   ;store current count
   sts   tcdn_top,operand     ;store periodic count
   rjmp  iw_exit

;interrupt mask register
;allows avr internal io-flags to cause an IRQ on the 6502 cpu
;write:[TDRE|RDRF|ICF1|OCF1A|OCF1B|TOV1|tcdn|tick]
; read:[TDRE|RDRF|ICF1|OCF1A|OCF1B|TOV1|tcdn|tick]
;tick = periodic 10ms interrupt (t0 ctc)
;tcdn = tick countdown interrupt
;T1 related masks - see ATMega datasheet
;RDRF: set if receiver fifo has data - cleared if buffer empty
;TDRE: set if transmitter fifo not full - cleared if buffer full
ir_mask:
   lds   operand,irq_mask     ;respond with current mask
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret

iw_mask:
   mov   a,operand            ;set t1 mask
   andi  a,0x3c
   in    b,timsk
   andi  b,0xc3
   or    a,b
   out   timsk,a
   mov   a,operand            ;save mask
   sts   irq_mask,a
;   in    b,tifr               ;check pending flags enabled
;   andi  b,0x3c
   lds   d,irq_flag
;   or    d,b                  ;merge t1 & saved           
   and   d,a
   tst   d
   ifeq  mask_irq_clear       ;all masked flags serviced?
      cbi   cbusddr,3            ;clear IRQ 
   else  mask_irq_clear
      sbi   cbusddr,3            ;set IRQ pending
   end   mask_irq_clear
   rjmp  iw_exit


;interrupt flag register
;avr internal io-flags
;write:[ -  | -  |ICF1|OCF1A|OCF1B|TOV1|tcdn|tick]
; read:[TDRE|RDRF|ICF1|OCF1A|OCF1B|TOV1|tcdn|tick]
;tick: periodic 10ms interrupt (t0 ctc) - write 1 to clear
;tcdn: tick countdown reached 0 - write 1 to clear
;T1 related masks - see ATMega datasheet - write 1 to clear
;RDRF: set if ACIA receiver fifo has data - cleared if empty
;TDRE: set if ACIA transmitter fifo not full - cleared if full
ir_flag:
   lds   operand,irq_flag     ;respond with pending flags
   in    a,tifr               ;get t1 flags
   andi  a,0x3c
   or    operand,a            ;merge to saved flags
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret

iw_flag:
   mov   a,operand            ;clear t1 flags
   andi  a,0x3c
   out   tifr,a
   in    b,tifr               ;keep pending flags
   andi  b,0x3c
   mov   a,operand            ;clear saved flags
   andi  a,0b00111111         ;ACIA can not be cleared
   eor   a,allon              ;reverse mask
   lds   d,irq_flag
   or    d,b                  ;merge t1 & saved
   and   d,a                  ;mask over flags
   sts   irq_flag,d
   lds   c,irq_mask           ;check remaining enabled
   and   d,c
   ifeq  flag_irq_clear       ;all flags serviced?
      cbi   cbusddr,3            ;clear IRQ 
   end   flag_irq_clear
   rjmp  iw_exit

;interrupt vector register
;avr internal io interrupt vectors, read only
;reading the vector clears the corresponding flag except
;external IRQ must be cleared at the source
;RDRF & TDRE are cleared by writing or reading the ACIA data reg
; 0 = external IRQ - low on IRQ input pin
; 2 = tick  - periodic 10ms interrupt (t0 ctc)
; 4 = tcdn  - tick countdown reached 0
; 6 = TOV1  - timer 1 overflow
; 8 = OCF1B - timer 1 output compare register b match
;10 = OCF1A - timer 1 output compare register a match
;12 = ICF1  - timer 1 input capture register match
;14 = RDRF  - ACIA receive data register full
;16 = TDRE  - ACIA transmit data register empty
ir_vector:
   clr   operand              ;initialize vector
   lds   c,irq_flag           ;read pending flags
   lds   d,irq_mask           ;mask with enabled interrupts
   mov   b,c                  ;make 6 enabled
   and   b,d
   ifne  ir_vect_internal
      set                        ;precharge to set bit
      mov   a,b
      andi  a,0xf0               ;interrupt source 2^2?
      ifne  ir_vect_b2
         mov   b,a                  ;isolate bit group
         bld   operand,2            ;add 2^2
      end   ir_vect_b2
      mov   a,b
      andi  a,0xcc               ;interrupt source 2^1?
      ifne  ir_vect_b1
         mov   b,a                  ;isolate bit group
         bld   operand,1            ;add 2^1
      end   ir_vect_b1
      mov   a,b
      andi  a,0xaa               ;interrupt source 2^0?
      ifne  ir_vect_b0
         mov   b,a                  ;isolate bit group
         bld   operand,0            ;add 2^0
      end   ir_vect_b0
      inc   operand              ;make vector 2 4 6 ... 16
      lsl   operand
      eor   b,allon              ;clear flag
      sbr   b,0b11000000         ;can't clear RDRF & TDRE
      and   c,b
      sts   irq_flag,c
      and   d,c                  ;all enabled flags done?
      ifeq  ir_vect_clr_int
         cbi   cbusddr,3            ;clear IRQ 
      end   ir_vect_clr_int
   end   ir_vect_internal
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret

;diag selftest register
;allows setting of external control lines if enabled
;enter debugger, query debugger running, query DMA available
;write:[DEB| - | - | - | - |RES|NMI|IRQ]
;        + start debugger, halt emulation
; read:[DEB| 1 | 0 | 0 | 0 | 0 |NMI|IRQ]
;        |   + IO version >= 0.83 (has DMA)
;        + debugger currently running, ACIA disconnected
ir_diag:
   lds   operand,selftest  ;respond with saved selftest flags
   ldi   a,0b1000000       ;mark current IO version
   sbrc  flags,deb_on      ;mark debugger is running
      sbr   a,0b10000000
   or    operand,a
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret

iw_diag:
   sbrs  operand,7         ;test enter diag
   ifs   iw_diag_enter
      sbr   flags,(1<<deb_on) ;turn debugging on
      cbr   flags,(1<<emu_run) ;stop emulation
      lds   a,irq_flag        ;clear ACIA TDRE & RDRF
      cbr   a,0b11000000
      sts   irq_flag,a
      lds   c,irq_mask        ;other interrupts pending & enabled?
      and   a,c
      ifeq  iw_diag_irq
         cbi   cbusddr,3         ;clear any pending acia interrupts
      end   iw_diag_irq
      ldi   yh,high(rx_buf)   ;load buffer index
      ldi   yl,0              ;purge buffer
      ldi   a,10              ;start with a linefeed 
      st    y+,a              ;store data
      ldi   a,27              ;force escape
      st    y+,a              ;store data
      sts   rx_inx,yl         ;store pointers
      sts   rx_fill,yl
   end   iw_diag_enter
.ifdef ena_diag
   sbrc  operand,2         ;test force reset
   rjmp  diag_reset        ;currently emulated
   sbrc  operand,1         ;test force NMI
   sbi   cbusddr,2         ;set low (open drain)
   sbrs  operand,1         ;test clear NMI
   cbi   cbusddr,2         ;release high (open drain)
   sbrc  operand,0         ;test force IRQ
   sbi   cbusddr,3         ;set low (open drain)
   sbrs  operand,0         ;test clear IRQ
   cbi   cbusddr,3         ;set low (open drain)
   mov   a,operand         ;mask last 2 bits
   andi  a,3
   sts   selftest,a        ;save for read
   rjmp  iw_exit

diag_reset:
   call soft_reset
.endif
   rjmp  iw_exit

;reset of interrupt masks and flags
irq_rs:
   in    a,timsk           ;clear t1 mask
   cbr   a,0x3c
   out   timsk,a
   ldi   a,0x3c            ;clear t1 flags
   out   tifr,a
   cbi   cbusddr,3         ;clear IRQ
   cbi   cbusddr,2         ;clear NMI
   sts   selftest,zero     ;clear saved flags
   sts   irq_mask,zero
   ldi   a,0x80            ;TDRE remains active
   sts   irq_flag,a
   ret

;*****************************************************************
;
; common IO exits (unused addresses and retry on busy)
;
;*****************************************************************

;retrying a read I/O instruction until the data becomes available
ir_retry:
   pop   a        ;discard return address
   pop   a

;retrying a write I/O instruction until the register (buffer) becomes available
iw_retry:
   sbiw  pch:pcl,2         ;prepare for instruction retry
   sbrs  flags,op_ind      ;3 byte instruction if direct
   sbiw  pch:pcl,1

;no mapping for write or normal exit
iw_exit:
   cbr   flags,(1<<modify) ;unlock extended memory address after modify
   out   cbus,readmem      ;OE / read mode back on
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   op_decode

;no mapping for read
ir_exit:
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
rs_exit:
   ret

.ifndef lcd_sel   
   .equ lcd_rd  = ir_exit
   .equ lcd_wrt = iw_exit
   .equ lcd_rs  = rs_exit
.endif

;**************************************************************
;
; I/O address decode tables and customizable IO modules
; part 2 of configuration
;
;**************************************************************
;
.set config_part = 2
.include "6502_Emu_config.inc"

;read address decode of internally mapped registers
iordreg:
   io_adr_dec  ior_int_tab
    
;write address decode of internally mapped registers
iowtreg:
   io_adr_dec  iow_int_tab

.ifdef spi_sel
;********************************************************************
;
;     SPI
;
;********************************************************************
;fixed definitions for the SPI interface - port share with abuslo
.EQU  spi_in      =pinb       ; SPI input
.EQU  spi_ddr     =ddrb       ; SPI DDR,
.EQU  spi_out     =portb      ; SPI output
.EQU  spi_sck     =7          ; spi clock output
.EQU  spi_miso    =6          ; master input
.EQU  spi_mosi    =5          ; master output
;     spi_ss      =0-4        ; spi slave select

;SPI command - write SPCR, SPSR, slave select hold
;w    [  x  |SSHLD|DORD |SPI2X|CPOL |CPHA |SPR1 |SPR0 ]
;                          >-----------------------v
;SPSR [  0  |  0  |  0  |  0  |  0  |  0  |  0  |SPI2X]
;
;SPCR [  0  |  0  |DORD |  1  |CPOL |CPHA |SPR1 |SPR0 ]
; --> [SPIE | SPE |DORD |MSTR |CPOL |CPHA |SPR1 |SPR0 ]
spw_cmd:
   sbrc  operand,6         ;test clearing sshld
   ifs_and  spw_cmd_sshld
   lds   a,spi_cmd
   sbrs  a,6               ;test sshld not already clear 
   ifs      spw_cmd_sshld 
      ldi   a,spi_idle        ;turn off ss
      out   abuslo,a
      sbi   spcr,spe          ;enable SPI
      set_io_select spi_sel
      ena_io_select
      dis_spi_select          ;latch spi interface
      cbi   spcr,spe          ;stop spi
      out   cbus,readmem      ;clear IO ext select & set read mode
   end   spw_cmd_sshld 
   sts   spi_cmd,operand   ;save slave select hold   
   mov   a,operand         ;charge with forced bits
   sbr   a,(1<<mstr)       ;always master
   cbr   a,(1<<spie|1<<spe) ;no interrupt enable & no spi enable
   out   spcr,a
   out   spsr,zero
   sbrc  operand,4         ;test SPI2X
      out   spsr,one          ;SPI2X is set
   rjmp  iw_exit

;SPI data - r/w SPDR
;
spw_data:
   .ifdef   spi_vat        ;do slave address translation
      mov   zl,oplow
      andi  zl,7              ;A0:2 only
      addi  zl,low(spi_vss<<1)
      ldi   zh,high(spi_vss<<1)
      lpm   c,z
      out   spi_out,c
   .endif
   sbi   spcr,spe          ;enable SPI
   set_io_select spi_sel
   ena_io_select
   out   spdr,operand      ;write data
   ldi   a,spi_idle        ;prepare to turn off ss
   lds   c,spi_cmd         ;get slave select hold
   wait_spi
   in    d,spdr            ;read incoming data
   sts   spi_rdat,d
   sbrs  c,6               ;slave select hold?
      out   abuslo,a          ;end slave select
   dis_spi_select          ;latch spi interface
   cbi   spcr,spe          ;stop spi
   out   cbus,readmem      ;clear IO ext select & set read mode
   rjmp  iw_exit
spr_data:
   .ifdef   spi_vat        ;do slave address translation
      mov   zl,oplow
      andi  zl,7              ;A0:2 only
      addi  zl,low(spi_vss<<1)
      ldi   zh,high(spi_vss<<1)
      lpm   c,z
      out   spi_out,c
   .endif
   sbi   spcr,spe          ;enable SPI
   set_io_select spi_sel
   ena_io_select
   out   spdr,allon        ;dummy write
   ldi   a,spi_idle        ;prepare to turn off ss
   lds   c,spi_cmd         ;get slave select hold
   wait_spi
   in    operand,spdr      ;read incoming data
   sbrs  c,6               ;slave select hold?
      out   abuslo,a          ;end slave select
   dis_spi_select          ;latch spi interface
   cbi   spcr,spe          ;stop spi
   out   cbus,readmem      ;clear IO ext select & set read mode
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret

;read data received during last write of the SPI
spr_dat_lst:
   lds   operand,spi_rdat
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret

;SPI reset
spi_rs:
   ldi   a,spi_idle        ;no slave selected
   out   spi_out,a
   ldi   a,0b01010011      ;enable, master, dead slow
   out   spsr,zero         ;clear double speed
   out   spcr,a
   set_io_reset   spi_sel
   ena_io_select
   out_spi zero            ;8 clocks to nobody
   in_spi a                ;check MISO pulled high
   cpi   a,0xff
   ifne  spi_rs_fail
      PrintStr_far spi_fail
   end   spi_rs_fail
   dis_spi_select          ;latch spi interface
   out   cbus,readmem      ;clear IO ext select & set read mode
   sts   spi_cmd,zero      ;set default
   ldi   a,0x10            ;master & default
   out   spcr,a
   ret
spi_fail:   .db   13,10,"SPI bus stuck",0

;DMA SPI read block
dma_spi_rb:
   lds   a,spi_cmd         ;verify slave select hold
   sbrs  a,6
   ifs   dma_spi_rssh
      lds   a,dma_last_cmd    ;verify last command and parameters
      lds   b,dma_tab_index
      cpi   a,3               ;current SPI read block (retry)?
      ifeq  spi_drb_retry
         lds   xl,dma_tab+5            ;load in progress block count
         lds   xh,dma_tab+6
         rjmp  dma_spi_rb1
      end   spi_drb_retry
      cpi   a,0x13            ;previous SPI read block?
      ifeq_or  spi_drb
      cpi   a,0               ;previous set block address table?
      ifeq_and spi_drb
      cpi   b,5               ;block address table valid?
      ifeq     spi_drb
         lds   xl,dma_tab+3      ;load block count
         lds   xh,dma_tab+4
         rjmp  dma_spi_rb1
      end      spi_drb
   end   dma_spi_rssh
   sts   dma_last_cmd,allon ;mark invalid sequence of command
   rjmp  iw_exit
dma_spi_rb1:
   sts   dma_last_cmd,operand ;mark command in progress
   .ifdef   spi_vat        ;do slave address translation
      lds   zl,dma_tab
      andi  zl,7              ;A0:2 only
      addi  zl,low(spi_vss<<1)
      ldi   zh,high(spi_vss<<1)
      lpm   c,z
   .else
      lds   c,dma_tab         ;load slave select
   .endif
   lds   zl,dma_tab+1         ;load memory address
   lds   zh,dma_tab+2
   in    a,spcr            ;load SPCR values
   mov   d,a               ;disabled
   sbr   a,(1<<spe)
   mov   e,a               ;enabled
   out   cbus,clear        ;readmode off
   prep_io_select spi_sel
   out   dbusddr,allon     ;prepare to write mem
   in    a,timsk           ;block single step on retry
   cbr   a,1<<toie0
   out   timsk,a
   sbr   flags,(1<<dma_rpt) ;set dma interrupt modifier
   do       spi_drb_loop
      .ifdef irq_dis_real
         sbrs  stat2,2           ;all interrupts disable
      .endif
      sei
      ;read SPI
      out   spi_out,c         ;slave select
      cli
      out   spcr,e            ;enable SPI
      make_io_select
      in_spi a                ;read incoming data
      dis_spi_select          ;latch spi interface
      out   spcr,d            ;disable spi
      out   cbus,clear
      ;write memory
      out   abuslo,zl         ;set memory address
      out   abushi,zh
      out   cbus,writemem     ;WE, ~OE
      out   dbusout,a         ;dbus write
      adiw  z,1
      out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
      sbiw  x,1
   loopne   spi_drb_loop
   out   dbusddr,zero      ;end writing memory
   sts   dma_tab+1,zl      ;save next memory address
   sts   dma_tab+2,zh
   ldi   a,0x13            ;last successful command was read
   sts   dma_last_cmd,a
   in    a,timsk           ;undo block single step
   sbr   a,1<<toie0
   out   timsk,a
   cbr   flags,(1<<dma_rpt) ;clear dma interrupt modifier
   rjmp  iw_exit
;DMA SPI write block
dma_spi_wb:
   lds   a,spi_cmd         ;verify slave select hold
   sbrs  a,6
   ifs   dma_spi_wssh
      lds   a,dma_last_cmd    ;verify last command and parameters
      lds   b,dma_tab_index
      cpi   a,2               ;current SPI read block (retry)?
      ifeq  spi_dwb_retry
         lds   xl,dma_tab+5      ;load in progress block count
         lds   xh,dma_tab+6
         rjmp  dma_spi_wb1
      end   spi_dwb_retry
      cpi   a,0x12            ;previous SPI read block?
      ifeq_or  spi_dwb
      cpi   a,0               ;previous set block address table?
      ifeq_and spi_dwb
      cpi   b,5               ;block address table valid?
      ifeq     spi_dwb
         lds   xl,dma_tab+3      ;load block count
         lds   xh,dma_tab+4
         rjmp  dma_spi_wb1
      end      spi_dwb
   end   dma_spi_wssh
   sts   dma_last_cmd,allon ;mark invalid sequence of command
   rjmp  iw_exit
dma_spi_wb1:
   sts   dma_last_cmd,operand ;mark command in progress
   .ifdef   spi_vat        ;do slave address translation
      lds   zl,dma_tab
      andi  zl,7              ;A0:2 only
      addi  zl,low(spi_vss<<1)
      ldi   zh,high(spi_vss<<1)
      lpm   c,z
   .else
      lds   c,dma_tab         ;load slave select
   .endif
   lds   zl,dma_tab+1      ;load memory address
   lds   zh,dma_tab+2
   in    a,spcr            ;load SPCR values
   mov   d,a               ;disabled
   sbr   a,(1<<spe)
   mov   e,a               ;enabled
   prep_io_select spi_sel
   in    a,timsk           ;block single step on retry
   cbr   a,1<<toie0
   out   timsk,a
   sbr   flags,(1<<dma_rpt) ;set dma interrupt modifier
   do       spi_dwb_loop
      ;read memory
      out   abuslo,zl
      out   abushi,zh
      .ifdef irq_dis_real
         sbrs  stat2,2           ;all interrupts disable
      .endif
            sei
            nop
            cli
            adiw  z,1
      wait_data_valid 4
      in    a,dbusin
      ;write SPI
      out   spi_out,c         ;slave select
      out   spcr,e            ;enable SPI
      make_io_select
      out_spi a               ;write data to spi
      dis_spi_select          ;latch spi interface
      out   spcr,d            ;stop spi
      out   cbus,readmem
      sbiw  x,1
   loopne   spi_dwb_loop
   sts   dma_tab+1,zl      ;save next memory address
   sts   dma_tab+2,zh
   ldi   a,0x12            ;last successful command was read
   sts   dma_last_cmd,a
   in    a,timsk           ;undo block single step
   sbr   a,1<<toie0
   out   timsk,a
   cbr   flags,(1<<dma_rpt) ;clear dma interrupt modifier
   rjmp  iw_exit
.else
   .equ  spw_cmd     = iw_exit
   .equ  spw_data    = iw_exit
   .equ  dma_spi_wb  = dma_cmd_inv
   .equ  spr_data    = ir_exit
   .equ  spr_dat_lst = ir_exit
   .equ  dma_spi_rb  = dma_cmd_inv
   .equ  spi_rs      = rs_exit
.endif

;********************************************************************
;
;     DMA for SPI, I2C and EEPROM or SD card load & save
;
;********************************************************************
dmw_cmd:
   mov   a,operand
   cpi   a,dma_ct_end-dma_cmd_tab
   iflo  dmw_cmd_valid
      ldi   zl,low(dma_cmd_tab)
      add   zl,a
      ldi   zh,high(dma_cmd_tab)
      adc   zh,zero
      ijmp
   end   dmw_cmd_valid
dma_cmd_inv:
   sts   dma_last_cmd,allon   ;set invalid command
   rjmp  iw_exit
dma_cmd_tab:                  ;table of valid DMA commands
   rjmp  dma_set_tab   
   rjmp  dma_set_tab
   rjmp  dma_spi_wb
   rjmp  dma_spi_rb
   rjmp  dma_i2c_wb
   rjmp  dma_i2c_rb
   rjmp  dma_eep_save
   rjmp  dma_eep_load
dma_ct_end:

;dma set block transfer table:
;        SPI (cmd=0)    I2C (cmd=1)    EEPROM (cmd=0)
;  byte  slave address  don't care     program number
;  word  start address  start address  start address
;  word  byte count     byte count     end address

dma_set_tab:
   sts   dma_tab_index,operand ;1 = skip 1st byte
   sts   dma_last_cmd,operand       
   rjmp  iw_exit

;dma read block transfer status
dmr_stat:
   lds   operand,dma_last_cmd ;dummy response
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret

;dma write block transfer table
dmw_data:
   lds   a,dma_last_cmd    ;set tab?
   cpi   a,2   
   iflo_and dmw_data_valid
   lds   a,dma_tab_index
   cpi   a,5               ;tab not already filled?
   iflo     dmw_data_valid
      ldi   zl,low(dma_tab)
      ldi   zh,high(dma_tab)
      add   zl,a
      adc   zh,zero
      st    z,operand
      inc   a
      sts   dma_tab_index,a
      rjmp  iw_exit
   end      dmw_data_valid
   sts   dma_last_cmd,allon   ;invalid sequence
   sts   dma_tab_index,allon
   rjmp  iw_exit

;dma read block transfer table
;  used on load from EEPROM to fetch end address
dmr_data:
   lds   a,dma_tab_index
   cpi   a,5               ;end of tab?
   ifsh     dmr_tab_end
      mov   a,zero            ;start over
   end   dmr_tab_end
   ldi   zl,low(dma_tab)
   ldi   zh,high(dma_tab)
   add   zl,a
   adc   zh,zero
   ld    operand,z
   inc   a
   sts   dma_tab_index,a
   sts   dma_last_cmd,allon   ;can't continue
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret

.ifdef i2c_sel
;********************************************************************
;
;     TWI / I2C
;
;********************************************************************
;
;  TWI / I2C Hardware        VCC
;                             +
;                             |
;   repeat for each signal    |
;                            .-.
;                            | |
;             1/4 74HC4066   | |2k7
;                   .---.    '-'
;                   |   |     |
;  o----------------|/ -|-----o--------------------o
;   PortC 0:3   I/O |   | I/O    SCL/SDA/-INT/-RESET
;                   '---'     |
;                     |CTL
;  >------------------+       |
;   TWI activate
;                             o
;        optional Interrupt   | -INT only
;                             |
;                             |
;  <---------------------->|--+
;   -IRQ or -NMI in    Schottky
;
;define your I2C interface
;i2c_sel defined in main file
.EQU  i2c_in      =dbusin     ; I2C input
.EQU  i2c_ddr     =dbusddr    ; I2C DDR
.EQU  i2c_out     =dbusout    ; I2C output
.EQU  i2c_scl     =0          ; clock i/o
.EQU  i2c_sda     =1          ; data i/o
.EQU  i2c_int     =2          ; -interrupt input (optional to detect i2c int)
.EQU  i2c_res     =3          ; -reset output (optional to reset i2c slaves)
; # of clocks for 400kHz SCL phase (1/2 clock) @ Osc_Hz rounded up
.EQU  i2c_clk     =((Osc_Hz / 2 - 1) / 400000) + 1
#define i2c_data a
;used registers a (data), b (timing), zl (status), e (io select default)  

;
; open i2c interface
;
i2c_open:
   set_io_select i2c_sel
i2c_open_rs:
   out   dbusout,allon     ;float all outputs high
   out   dbusddr,allon     ;(discharge low cap)
   mov   e,b
   out   dbusddr,zero
   out   dbusout,zero      ;open collector drive via ddr
   ena_io_select
   ret

;
; read i2c data
;
;     send ack before reading except on 1st read
i2c_data_rd:   
   lds   zl,i2c_statreg    ;load parameters
   sbrc  zl,0              ;test interface stopped
   ifs   i2c_read_ok
      rcall i2c_open
      bst   zl,2              ;set speed
      clc                     ;ack
      sbrc  zl,3              ;read ack required?
         rcall i2c_putack
      rcall i2c_read
      mov   operand,i2c_data
      sbr   zl,0b1000         ;set read ack required
      sts   i2c_statreg,zl    ;save status
      .if (io_sel_direct > 0)
         out   abushi,e          ;close i2c interface
      .endif
      out   cbus,readmem      ;clear IO select & set read mode
   else  i2c_read_ok
      mov   operand,zero
   end   i2c_read_ok
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret
;
; write i2c data if not stopped or previous read
; write i2c address if start condition
;
;     if nak response (slave not ready/present) send stop
i2c_data_wrt:
   lds   zl,i2c_statreg    ;load parameters
   bst   zl,2              ;set speed
   sbrs  zl,1              ;start pending?
   ifs   i2c_adr
      mov   a,operand
      rcall i2c_open
      sec                     ;nak
      sbrc  zl,3              ;read ack outstanding?
         rcall i2c_putack
      mov   i2c_data,operand
      .ifdef i2c_eep_adr
         cpi   i2c_data,eep_adr  ;protect nv program EEPROM
         ifeq  i2c_eep_protect
            sec
         else  i2c_eep_protect
            rcall i2c_start
         end   i2c_eep_protect
      .else
         rcall i2c_start
      .endif
      andi  zl,0b100          ;keep speed, clear the rest
      ifcc  i2c_start_ack
         sbr   zl,0b1000000      ;set ack
      else  i2c_start_ack
         cbr   zl,0b1000000      ;clear ack
         sbr   zl,0b1            ;set stop
         rcall i2c_stop
      end   i2c_start_ack
      sts   i2c_statreg,zl
      .if (io_sel_direct > 0)
         out   abushi,e          ;close i2c interface
      .endif
      out   cbus,readmem      ;clear IO select & set read mode
      rjmp  iw_exit
   end   i2c_adr
   mov   a,zl
   andi  a,0b1001          ;not stopped and not read?
   ifeq  i2_wrt_ok
      rcall i2c_open
      mov   i2c_data,operand
      rcall i2c_write
      andi  zl,0b100          ;keep speed, clear the rest
      ifcc  i2c_write_ack
         sbr   zl,0b1000000      ;set ack
      else  i2c_write_ack
         cbr   zl,0b1000000      ;clear ack
         sbr   zl,0b1            ;set stop
         rcall i2c_stop
      end   i2c_write_ack
      sts   i2c_statreg,zl
      .if (io_sel_direct > 0)
         out   abushi,e          ;close i2c interface
      .endif
      out   cbus,readmem      ;clear IO select & set read mode
      rjmp  iw_exit
   end   i2_wrt_ok
   rjmp  iw_exit

;
;  write i2c command
;
;0=send stop, nak & stop if pending read
;1=set start(next data write = address), speed 100kHz
;2=set start(next data write = address), speed 400kHz
;3=send reset (9 naks), optional hw reset
;  allow sufficient time after resetting hardware 
;
i2c_cmd:
   lds   zl,i2c_statreg
   mov   a,operand
   cpi   a,3
   iflo  i2c_cmd_012
      cpi   a,1
      iflo  i2c_cmd_stop      ;command code 0 - send stop
         rcall i2c_send_stop
         rjmp  i2c_cmd_exit
      end   i2c_cmd_stop 
      ifeq  i2c_start_100     ;command code 1 - set start 100kHz 
         cbr   zl,0b100       ;speed 100 kHz
         sbrc  zl,5           ;bus not stuck?        
            rcall i2c_reset   ;stuck bus requires a reset
         sbrs  zl,5           ;bus still stuck?
            sbr   zl,0b10     ;OK to start
         rjmp  i2c_cmd_exit
      end   i2c_start_100     ;command code 2 - set start 400kHz
      cbr   zl,0b100          ;speed 100 kHz for reset
      sbrc  zl,5              ;bus not stuck?        
         rcall i2c_reset      ;stuck bus requires a reset
      sbrs  zl,5              ;bus still stuck?
         sbr   zl,0b110       ;OK to start with 400kHz
      rjmp  i2c_cmd_exit
   end   i2c_cmd_012
   ifeq  i2c_cmd_reset        ;command code 3 - reset i2c bus
      rcall i2c_reset
   end   i2c_cmd_reset
i2c_cmd_exit:
   sts   i2c_statreg,zl
   rjmp  iw_exit

;stop i2c
i2c_send_stop:
   sbrc  zl,0              ;not already stopped?
   ifs   i2c_not_stop
      rcall i2c_open
      bst   zl,2              ;set speed
      sec                     ;nak
      sbrc  zl,3              ;read ack outstanding?
         rcall i2c_putack
      andi  zl,0b100          ;keep speed, clear the rest
      sbr   zl,0b1            ;set stop
      rcall i2c_stop
      .if (io_sel_direct > 0)
         out   abushi,e          ;close i2c interface
      .endif
      out   cbus,readmem      ;clear IO select & set read mode
   end   i2c_not_stop
   ret

;
; read i2c status
;
;  [ IRQ | ACK |Stuck|  0  |Read |Speed|Start|Stop ]
;IRQ:    1=I2C Interrupt pending
;ACK:    1=slave acknowledge - device present/ready
;Stuck:  1=SCL or SDA stuck - interface needs reset
;  0
;Read:   1=Send read acknowledge pending
;Speed:  1=400kHz, 0=100kHz
;Start:  1=next write = address
;Stop:   1=interface stopped
i2c_stat:
   lds   zl,i2c_statreg    ;load status
   rcall i2c_open
   sbis  i2c_in,i2c_int    ;skip if no interrupt
      sbr   zl,0b10000000     ;merge interrupt to status
   mov   operand,zl
   .if (io_sel_direct > 0)
      out   abushi,e          ;close i2c interface
   .endif
   out   cbus,readmem      ;clear IO select & set read mode
   ldi   oc_tabh,high(oc_tab) ;restore zh as opcode table
   ret
   
;
; I2C reset - mark unusable if stuck
;
i2c_rs:
   set_io_reset   i2c_sel
   rcall i2c_open_rs
   rcall i2c_reset_rs
   sts   i2c_statreg,zl
   sbrs  zl,5           ;message, if I2C stuck
   ifs   i2c_rs_fail
      PrintStr_far i2c_fail
   end   i2c_rs_fail
   ret
i2c_fail:   .db   13,10,"I2C bus stuck",0

;DMA I2C read block
dma_i2c_rb:
   lds   a,dma_last_cmd
   lds   b,dma_tab_index
   cpi   a,5               ;current i2c read block (retry)?
   ifeq  i2c_drb_retry
      lds   xl,dma_tab+5            ;load in progress block count
      lds   xh,dma_tab+6
      rjmp  dma_i2c_rb1
   end   i2c_drb_retry
   cpi   a,0x15            ;previous i2c read block?
   ifeq_or  i2c_drb
   cpi   a,1               ;previous set block address table?
   ifeq_and i2c_drb
   cpi   b,5               ;block address table valid?
   ifeq     i2c_drb
      lds   xl,dma_tab+3      ;load block count
      lds   xh,dma_tab+4
      rjmp  dma_i2c_rb1
   end      i2c_drb
dma_i2c_rbfail:
   sts   dma_last_cmd,allon ;mark invalid sequence of command
   rjmp  iw_exit
dma_i2c_rb1:
   sts   dma_last_cmd,operand ;mark command in progress
   lds   zl,dma_tab+1      ;load memory address
   lds   zh,dma_tab+2

   lds   yl,i2c_statreg    ;load current I2C status
   sbrc  yl,0              ;test interface stopped
      rjmp  dma_i2c_rbfail
   out   cbus,clear        ;readmode off
   rcall i2c_open
   in    c,cbus            ;save I2C register enable
   in    d,abushi          ;save I2C enable
   bst   yl,2              ;set speed
   clc                     ;ack
   sbrc  yl,3              ;read ack required?
      rcall i2c_putack
   sbr   yl,0b1000         ;set read ack required
   sts   i2c_statreg,yl    ;save status
   in    a,timsk           ;block single step on retry
   cbr   a,1<<toie0
   out   timsk,a
   sbr   flags,(1<<dma_rpt) ;set dma interrupt modifier
   do       i2c_drb_loop
      ;continue reading I2C
      rcall i2c_read
      .if (io_sel_direct > 0)
         out   abushi,e          ;close i2c interface
      .endif
      out   cbus,clear        ;clear IO select
      ;write memory
      out   dbusddr,allon     ;prepare to write mem
      out   abuslo,zl         ;set memory address
      out   abushi,zh
      out   cbus,writemem     ;WE, ~OE
      out   dbusout,a         ;dbus write
      adiw  z,1
      out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
      sbiw  x,1
   exiteq   i2c_drb_loop
      ;read i2c
      out   dbusout,allon     ;float all outputs high
      .ifdef irq_dis_real
         sbrs  stat2,2           ;all interrupts disable
      .endif
      sei
      out   dbusddr,zero
      cli
      out   dbusout,zero      ;open collector drive via ddr
      out   abushi,d          ;reenable I2C bus 
      out   cbus,c
      clc
      bst   yl,2              ;set speed
      rcall i2c_putack
   loop     i2c_drb_loop
   out   dbusddr,zero      ;end writing to memory
   sts   dma_tab+1,zl      ;save next memory address
   sts   dma_tab+2,zh
   ldi   a,0x15            ;last successful command was read
   sts   dma_last_cmd,a
   in    a,timsk           ;undo block single step
   sbr   a,1<<toie0
   out   timsk,a
   cbr   flags,(1<<dma_rpt) ;clear dma interrupt modifier
   rjmp  iw_exit

;DMA I2C write block
dma_i2c_wb:
   lds   a,dma_last_cmd
   lds   b,dma_tab_index
   cpi   a,4               ;current i2c read block (retry)?
   ifeq  i2c_dwb_retry
      lds   xl,dma_tab+5      ;load in progress block count
      lds   xh,dma_tab+6
      rjmp  dma_i2c_wb1
   end   i2c_dwb_retry
   cpi   a,0x14            ;previous i2c read block?
   ifeq_or  i2c_dwb
   cpi   a,1               ;previous set block address table?
   ifeq_and i2c_dwb
   cpi   b,5               ;block address table valid?
   ifeq     i2c_dwb
      lds   xl,dma_tab+3      ;load block count
      lds   xh,dma_tab+4
      rjmp  dma_i2c_wb1
   end      i2c_dwb
dma_i2c_rwfail:
   sts   dma_last_cmd,allon ;mark invalid sequence of command
   rjmp  iw_exit
dma_i2c_wb1:
   sts   dma_last_cmd,operand ;mark command in progress
   lds   zl,dma_tab+1      ;load memory address
   lds   zh,dma_tab+2
   prep_io_select i2c_sel
   mov   e,b               ;save I2C register idle
   mov   d,yl              ;save I2C register active
   mov   c,yh              ;save I2C register enable
   lds   yl,i2c_statreg    ;load I2C status
   mov   a,yl
   andi  a,0b1001          ;not stopped and not read?
   brne  dma_i2c_rwfail
   in    a,timsk           ;block single step on retry
   cbr   a,1<<toie0
   out   timsk,a
   sbr   flags,(1<<dma_rpt) ;set dma interrupt modifier
   do       i2c_dwb_loop
      ;read memory
      out   abuslo,zl
      out   abushi,zh
      .ifdef irq_dis_real
         sbrs  stat2,2           ;all interrupts disable
      .endif
            sei
            nop
            cli
            adiw  z,1
            bst   yl,2              ;set speed
      wait_data_valid 5
      in    a,dbusin
      ;write i2c
      out   cbus,clear
      out   dbusout,allon     ;float all outputs high
      out   dbusddr,allon     ;(discharge low cap)
      out   dbusddr,zero
      out   dbusout,zero      ;open collector drive via ddr
      out   abushi,d          ;reenable I2C bus 
      out   cbus,c
      rcall i2c_write
   ifcc  i2c_dwb_ack
      .if (io_sel_direct > 0)
         out   abushi,e          ;close i2c interface
      .endif
      out   cbus,readmem      ;clear IO select & set read mode
      sbiw  x,1
   loopne   i2c_dwb_loop
      ldi   a,0x14            ;last successful command was read
   else  i2c_dwb_ack
      andi  yl,0b100          ;keep speed, clear ack & the rest
      sbr   yl,0b1            ;set stop
      sts   i2c_statreg,yl
      rcall i2c_stop
      .if (io_sel_direct > 0)
         out   abushi,e          ;close i2c interface
      .endif
      out   cbus,readmem
      mov   a,allon           ;mark dma failed unless...
      sbiw  x,1
      ifeq  i2c_dwb_end_nak   ;...it is at the end of the block
         ldi   a,0x14
      end   i2c_dwb_end_nak
   end   i2c_dwb_ack
   sts   dma_last_cmd,a
   sts   dma_tab+1,zl      ;save next memory address
   sts   dma_tab+2,zh
   in    a,timsk           ;undo block single step
   sbr   a,1<<toie0
   out   timsk,a
   cbr   flags,(1<<dma_rpt) ;clear dma interrupt modifier
   rjmp  iw_exit

;*********************


;
; wait_i2c  @0 = # of cycles, @1 = cyles already used 
;
;     i2c clock delay for 400 kHz (100kHz if T clear)
;     
;
.macro      wait_i2c
      .set cycles = (@0 - 2) - @1       ;400 kHz delay -BRTS -used     
      .if (cycles > 2)
        .set  loop_cycles = (cycles / 3)      
        ldi   b,loop_cycles
        brts  i2c_400kHz
        ldi   b,loop_cycles + @0        ;add 3* delay (100 kHz))
i2c_400kHz:
        dec   b
        brne  pc-1
        .set  cycles = (cycles - (loop_cycles * 3))
      .else
        .set  loop_cycles = @0
        brts  i2c_400kHz
        ldi   b,loop_cycles           ;add 3* delay (100 kHz)
        dec   b
        brne  pc-1
        nop                           ;compensate branch not taken
i2c_400kHz:
      .endif
      .if (cycles > 0)
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
.endmacro
;
; i2c macros - single master
;   allow clock stretching but no collision detection
;
.macro      sda1        ;set sda high, single master
            cbi   i2c_ddr,i2c_sda
.endmacro 
.macro      scl1        ;set scl high, wait scl high
            cbi   i2c_ddr,i2c_scl
            sbis  i2c_in,i2c_scl    ;subtract 2 clocks
            rjmp  pc-1              ;in next wait_ns
.endmacro
.macro      sda0        ;set sda low
            sbi   i2c_ddr,i2c_sda
.endmacro
.macro      scl0        ;set scl low
            sbi   i2c_ddr,i2c_scl
.endmacro

;
; i2c interface reset & check for stuck bus
;
i2c_reset:
   rcall i2c_open
i2c_reset_rs:
   sbi   i2c_ddr,i2c_res   ;optional hw reset
   wait_i2c 190,0          ;allow reset to disengage slaves
   clr   zl                ;clear all
   sbic  i2c_in,i2c_scl    ;SCL stuck low?
   ifs   i2c_stuck         ;set interface not usable status
      sbr   zl,0b100001       ;set stop & stuck
   else  i2c_stuck         ;reset for devices without hw-reset
      clt                     ;set low speed
      ldi   i2c_data,0xff     ;9 clocks with nak
      rcall i2c_write
      sbic  i2c_in,i2c_sda    ;SDA stuck low?
      ifs   i2c_sda_stuck
         sbr   zl,0b100001       ;set stop & stuck
      else  i2c_sda_stuck
         sbr   zl,0b1            ;set stop
      end   i2c_sda_stuck
      rcall i2c_stop
   end   i2c_stuck
   cbi   i2c_ddr,i2c_res   ;end optional hw reset
   .if (io_sel_direct > 0)
      out   abushi,e          ;close i2c interface
   .endif
   out   cbus,readmem      ;clear IO select & set read mode
   ret

;
; i2c start with address
;
;     start -> slave,  i2c_data -> slave address
;     slave ack -> SREG.C     
;
i2c_start:  sbic  i2c_in,i2c_sda    ;clock previous ack
            ifs   start_prev_ack
               scl0
               wait_i2c i2c_clk,2      ;scl1
               scl1
               wait_i2c i2c_clk/2,4    ;scl1-sda0    
            end   start_prev_ack    
            sda0
            wait_i2c i2c_clk/2,4    ;sec-rol-scl0
;
; i2c write: i2c_data -> slave data, slave ack -> SREG,C
;
i2c_write:  sec                     ;mark incomplete
            rol   i2c_data
i_wrtloop:  scl0
            wait_i2c i2c_clk/2,4    ;brcc-(nop)-sdax
            brcc  i_wrt0            ;data bit = 0
            nop                     ;sda set after 4 clocks
            sda1
            rjmp  i_wrt1
i_wrt0:     sda0
            rjmp  i_wrt1            ;nop 2 clocks
i_wrt1:     wait_i2c i2c_clk/2,4    ;rjmp-scl1
            scl1
            wait_i2c i2c_clk,7      ;scl1-lsl-brne-(nop)-scl0
            lsl   i2c_data
            brne  i_wrtloop
            nop

i2c_getack: scl0
            wait_i2c i2c_clk/2,2    ;sda1
            sda1                    ;release sda after write
            wait_i2c i2c_clk/2,2    ;scl1
            scl1
            wait_i2c i2c_clk/2,3    ;scl1-clc
            clc
            sbic  i2c_in,i2c_sda
            sec
            ret
;
; i2c read: slave data -> i2c_data, 
;
i2c_read:   ldi   i2c_data,1        ;precharge register full marker
i_rdloop:   wait_i2c i2c_clk/2,7    ;sbic-(sec)-rol-brcc-scl0
            scl0
            wait_i2c i2c_clk/2,2    ;sda1
            sda1                    ;clear sda out from prev. putack
            wait_i2c i2c_clk/2,2    ;scl1
            scl1
            wait_i2c i2c_clk/2,3    ;scl1-clc    
            clc
            sbic  i2c_in,i2c_sda
            sec
            rol   i2c_data
            brcc  i_rdloop          ;precharged bit hasn't arrived
            ret
;
; i2c putack: SREG,C -> slave ack
;
i2c_putack: scl0
            wait_i2c i2c_clk/2,4    ;brcc-(nop)-sdax
            brcc  i_pack1
            nop                     ;compensate brcc
            sda1
            rjmp  i_pack2
i_pack1:    sda0
            rjmp  i_pack2           ;nop 2 clocks
i_pack2:    wait_i2c i2c_clk/2,4    ;rjmp-scl1
            scl1
            wait_i2c i2c_clk/2,5    ;scl1-ret-rcall-ldi+wait(-7)-scl0
            ret
;
; i2c stop
;
i2c_stop:   wait_i2c i2c_clk/2,6    ;scl1+wait(-5)-ret-rcall-scl0
            scl0
            wait_i2c i2c_clk/2,2    ;sda0
            sda0
            wait_i2c i2c_clk/2,2    ;scl1
            scl1
            wait_i2c i2c_clk/2,4    ;scl1-sda1
            sda1
            ret
.else
   .equ i2c_data_wrt = iw_exit
   .equ i2c_cmd      = iw_exit
   .equ i2c_data_rd  = ir_exit
   .equ i2c_stat     = ir_exit
   .equ i2c_rs       = rs_exit
   .equ dma_i2c_wb   = dma_cmd_inv
   .equ dma_i2c_rb   = dma_cmd_inv
.endif

.ifdef eep_vld
;*****************************************************
;
; external SPI/I2C EEPROM as non volatile program storage
;
;*****************************************************
;
; EEP open - check spi/i2c bus & EEPROM presence
eep_open:
   .ifdef spi_eep_adr
      out   spsr,one          ;speed 8MHz, mode 0, enabled
      ldi   a,0x50
      out   spcr,a
      ldi   a,spi_eep_adr
      out   spi_out,a
      set_io_select spi_sel
      ena_io_select
      mov   e,b               ;deselcted IO (abushi)
      ldi   a,5               ;command: read status register
      out_spi a
      in_spi a
      in    f,spi_out         ;selected slave (abuslo a.k.a. spi_out)
      ldi   b,spi_idle        ;stop slave select
      out   spi_out,b
      cpi   a,0xff            ;MISO open?
      breq  eep_no_device
   .else
      set                     ;speed 400kHz
      rcall i2c_open
      sbis  i2c_in,i2c_scl    ;SCL stuck low?
      rjmp  eep_no_device
      ldi   i2c_data,0xff     ;9 clocks with nak
      rcall i2c_write
      sbis  i2c_in,i2c_sda    ;SDA stuck low?
      rjmp  eep_no_device
      rcall i2c_stop
      ldi   a,eep_adr         ;test device present
      rcall i2c_start
      brcs  eep_no_device
      rcall i2c_stop
   .endif
   .if (io_sel_direct > 0)
      in    d,abushi          ;selected IO exp. (abushi)
   .endif
   in    c,cbus            ;selected IO (cbus)
   ret 
eep_no_device:
   PrintStr err_no_eep     ;" SPI/I2C EEPROM not found"
   jmp   skip_main
;
; EEPROM write 1 Byte, select new slot if necessary
;
eep_write:
   tst   yl            ;need new slot?
   ifeq  eep_new_slot
      lds   yl,prog_num
      do       eep_find_slot
         do    eep_write_wait ;previous write complete?
            sbic  eecr,eewe
         loop  eep_write_wait
         out   eearl,yh       ;read slot to find prog#
         out   eecr,one
         in    b,eedr
         cp    b,yl           ;overwrite same
      exiteq   eep_find_slot
         inc   b              ;new free slot
         ifeq  eep_slot_allocate
            out   eedr,yl        ;write prog#
            in    b,sreg         ;atomic write sequence
            cli
            sbi   eecr,eemwe
            sbi   eecr,eewe
            out   sreg,b
      exit     eep_find_slot
         end   eep_slot_allocate
         inc   yh             ;all slots done?
         breq  eep_full
      loop     eep_find_slot
      ldi   yl,0
   end   eep_new_slot
   cpi   yl,0           ;new slot?
   ifeq_or  eep_write_open
   cpi   yl,128         ;new page_write?
   ifeq     eep_write_open
      push  a
      ;open next eeprom page for write
      .ifdef spi_eep_adr
      ldi   b,spi_idle
         do       eep_write_ready
            out   spi_out,f   ;set slave select
            ldi   a,5         ;command: read status
            out_spi a
            in_spi a          ;get status
            out   spi_out,b   ;end slave select
            andi  a,1         ;write in progress
         loopne   eep_write_ready
         out   spi_out,f   ;set slave select
         ldi   a,6         ;command: write enable
         out_spi a
         out   spi_out,b   ;end slave select
         out   spi_out,f   ;set slave select
         ldi   a,2         ;command: write
         out_spi a
         out_spi yh        ;send write address
         out_spi yl
      .else
         do       eep_write_ready
            ldi   a,eep_adr
            rcall i2c_start   ;may need timeout if dead EEPROM
         loopcs   eep_write_ready
         mov   a,yh        ;send write address
         rcall i2c_write
         mov   a,yl
         rcall i2c_write
      .endif
      pop   a
   end      eep_write_open
   ;write data
   .ifdef spi_eep_adr
      out_spi a
   .else
      rcall i2c_write
   .endif
   cpi   yl,127      ;EEP write buffer filled?
   ifeq_or eep_write_buf
   cpi   yl,255      ;EEP write buffer filled?
   ifeq    eep_write_buf
      ;end of page - internal eep write
      .ifdef spi_eep_adr
         ldi   a,spi_idle  ;stop slave select
         out   spi_out,a
      .else
         rcall i2c_stop    ;write buffer
      .endif
   end     eep_write_buf
   adiw  y,1
   breq  eep_full  
   ret

eep_full:               ;no more slots
   PrintStr err_eep_full   ;" EEPROM full"
;
; EEP error during save
;
eep_write_err:          ;close after errors
   .ifdef spi_eep_adr
      ldi   a,spi_idle     ;stop slave select
      out   spi_out,a
   .else
      rcall i2c_stop
   .endif
   ldi   yh,0           ;free all slots for this prog#
   rcall eep_int_close
   jmp   discard_serial_stream   ;skip remaining input, end of command
;
; EEP write close
;
eep_write_close:
   .ifdef spi_eep_adr
      ldi   a,spi_idle     ;close spi interface
      out   spi_out,a
   .else
      rcall i2c_stop
   .endif
   tst   yl             ;slot already incremented?
   ifne  eep_close_nxt_slot
      inc   yh
   end   eep_close_nxt_slot
   tst   yh             ;was last slot?
   ifne  eep_close_last_slot
      rcall eep_int_close
   end   eep_close_last_slot
   sbrc  flags,deb_act     ;outside debugger
      jmp   discard_serial_stream   ;skip remaining input, end of command

;general close SPI/I2C EEPROM
eep_close:
   ;disconnect SPI/I2C, connect RAM
   .ifdef spi_eep_adr
      out   dbusddr,zero      ;dbus = output
   .endif
   .if (io_sel_direct > 0)
      out   abushi,e
   .endif
   out   cbus,readmem      ;clear IO select & set read mode
   out   eearl,zero        ;set to unused address
   out   eearh,zero
   sts   prog_num,allon    ;load/save ended
   ;restore SPI parameters from last spi_cmd
   .ifdef spi_eep_adr
      lds   b,spi_cmd
      out   spsr,zero
      sbrc  b,4               ;test SPI2X
         out   spsr,one          ;SPI2X is set
      sbr   b,(1<<mstr)       ;always master
      cbr   b,(1<<spie|1<<spe) ;no interrupt enable & no spi enable
      out   spcr,b
   .endif
   ret

;vacate unused slots for this prog#
eep_int_close:
   lds   yl,prog_num
   do       eep_find_slot2
      do    eep_write_wait2
         sbic  eecr,eewe      ;previous write complete?
      loop  eep_write_wait2
      out   eearl,yh       ;read slot to find prog#
      out   eecr,one
      in    d,eedr
      cp    d,yl           ;remaining slot for prog#
      ifeq  eep_free_slot
         out   eedr,allon     ;mark free
         in    d,sreg         ;atomic write sequence
         cli
         sbi   eecr,eemwe
         sbi   eecr,eewe
         out   sreg,d
      end   eep_free_slot
      inc   yh             ;all slots done?
   loopne   eep_find_slot2
   do    eep_write_wait4
      sbic  eecr,eewe      ;last internal write complete?
   loop  eep_write_wait4
   ret

;
; load program from SPI/I2C EEPROM
;
eep_load_prog:
   PrintStr eep_prog       ;10,13,"Program "
   lds   a,prog_num        ;prog#
   rcall PrintHex
   cbr   flags,(1<<emu_run) ;force emulator halted  
   clr   xl                ;check prog exists
   out   eearh,one
   lds   a,prog_num
   do       eep_scan_loop
      out   eearl,xl       ;read slot pointer
      out   eecr,one
      in    d,eedr
      cp    d,a            ;prog# match?
   exiteq   eep_scan_loop
      inc   xl             ;all slots done?
      ifeq  eep_load_bad_prog
         PrintStr eep_notfound   ;" not found"
         ldi   a,1
         rjmp  eep_close
      end   eep_load_bad_prog
   loop     eep_scan_loop
   rcall eep_open
   ldi   yl,0           ;start reading from 1st slot
   ldi   yh,0
   rcall eep_read
   tst   a
   ifeq  eep_file_incompat
      PrintStr bad_format  ;" incompatible format"
      ldi   a,1
      rjmp  eep_close
   end   eep_file_incompat
   mov   xl,a
   do       eep_load_loop
      rcall eep_read       ;read address
      mov   zh,a           ;big endian
      rcall eep_read
      mov   zl,a
      .ifdef spi_eep_adr
         out   dbusddr,allon
      .endif
      do       eep_load_byte
         rcall eep_read
         ;disconnect SPI/I2C, connect RAM
         .if (io_sel_direct > 0)
            out   abushi,e          ;disable IO select
         .endif
         out   cbus,clear        ;disable IO select
         .ifdef spi_eep_adr
            cbi   spcr,spe
         .else
            out   dbusddr,allon     ;dbus = output
         .endif
         out   abuslo,zl 
         out   abushi,zh
         out   cbus,writemem     ;WE, ~OE
         out   dbusout,a         ;dbus write
         adiw  z,1
         out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
         ;connect SPI/I2C
         .if (io_sel_direct > 0)
            out   abushi,d          ;prepare IO select
         .endif
        .ifdef spi_eep_adr
            out   spi_out,f         ;set slave select
            sbi   spcr,spe
         .else
            out   dbusout,allon     ;float all outputs high
            out   dbusddr,zero
            out   dbusout,zero      ;open collector drive via ddr
         .endif
         out   cbus,c            ;activate IO select
         dec   xl                ;record counter expired
      loopne   eep_load_byte
      rcall eep_read       ;read count
      mov   xl,a
      tst   xl
   loopne   eep_load_loop
   rcall eep_read          ;read PCH
   tst   a                 ;is valid?
   ifne  eep_load_pc
      mov   pch,a             ;set PC
      rcall eep_read
      mov   pcl,a
   end   eep_load_pc
   PrintStr eep_loaded     ;" loaded"
   .ifdef spi_eep_adr
      ldi   a,spi_idle     ;close spi slave
      out   spi_out,a
   .else
      sec                  ;nak & stop
      rcall i2c_putack
      rcall i2c_stop
   .endif
   ldi   a,0               ;post returncode OK
   rjmp eep_close

; EEPROM read 1 Byte, select new slot if necessary
eep_read:
   tst   yl                ;need to open next slot?
   ifeq  eep_next_slot
      tst   yh             ;1st slot?
      ifne  eep_read_first
         .ifdef spi_eep_adr
            ldi   a,spi_idle     ;close spi slave
            out   spi_out,a
         .else
            sec                  ;nak & stop
            rcall i2c_putack
            rcall i2c_stop
         .endif
      end   eep_read_first
      lds   a,prog_num
      do       eep_find_prog
         out   eearl,yh       ;read slot to find prog#
         out   eecr,one
         in    b,eedr
         cp    b,a            ;match?
      exiteq   eep_find_prog
         inc   yh             ;all slots done?
         breq  eep_read_overrun
      loop     eep_find_prog
      .ifdef spi_eep_adr
         out   spi_out,f      ;set slave select
         ldi   a,3            ;command: read eep
         out_spi a
         out_spi yh
         out_spi yl
      .else
         do       eep_read_ready
            ldi   a,eep_adr
            rcall i2c_start      ;may need timeout if dead EEPROM
         loopcs   eep_read_ready
         mov   a,yh
         rcall i2c_write
         mov   a,yl
         rcall i2c_write
         ldi   a,eep_adr+1    ;switch to read
         rcall i2c_start
      .endif
      .ifndef spi_eep_adr
   else  eep_next_slot
         clc                  ;ack
         rcall i2c_putack
      .endif
   end   eep_next_slot
   ;read data byte
   .ifdef spi_eep_adr
      in_spi a
   .else
      rcall i2c_read
   .endif
   adiw  y,1         ;next
   ret
; overran the last slot, no ending count 0x00
eep_read_overrun:
   PrintStr err_corrupt    ;" corrupted"
   pop   a                 ;remove return address
   pop   a
   ldi   a,1               ;post returncode fail
   sbrc  flags,deb_act
      rjmp  eep_close
   rjmp  eep_app_read_exit

;dma save program
dma_eep_save:
   lds   a,dma_last_cmd
   lds   b,dma_tab_index
   sts   dma_last_cmd,allon ;mark invalid sequence of command
   cpi   a,0               ;previous set block address table?
   ifne_or  eep_ds_inv
   cpi   b,5               ;block address table valid?
   ifne_or  eep_ds_inv
   lds   a,dma_tab         ;program number valid?
   cpi   a,0xff
   ifeq     eep_ds_inv
      rjmp  iw_exit           ;exit - invalid parameters
   end      eep_ds_inv
   sts   prog_num,a
   in    b,spl             ;save sp - allows sub to exit
   sts   sp_save,b
   in    b,sph
   sts   sp_save+1,b
   rcall eep_open
   PrintStr eep_prog       ;10,13,"Program "
   lds   a,prog_num        ;prog#
   rcall PrintHex
   lds   zl,dma_tab+1      ;load memory address
   lds   zh,dma_tab+2
   lds   xl,dma_tab+3      ;load block count
   lds   xh,dma_tab+4
   ldi   yl,0              ;initial slot allocation
   ldi   yh,0
   out   eearh,one
   ldi   a,0               ;set app save marker
   rcall eep_write
   mov   a,xl              ;calculate count from end address
   sub   a,zl
   rcall eep_write
   mov   a,xh
   sbc   a,zh
   rcall eep_write
   do       eep_app_save_data
      cp    xl,zl
      ifeq  eep_app_save_adr
         cp    xh,zh
   exiteq   eep_app_save_data
      end   eep_app_save_adr
      ;disconnect SPI/I2C, connect RAM
      .if (io_sel_direct > 0)
         out   abushi,e          ;disable IO sel. exp.
      .endif
      out   cbus,readmem      ;disable IO select
      .ifdef spi_eep_adr
         cbi   spcr,spe       ;disable SPI
      .endif
      out   abuslo,zl         ;read from RAM
      out   abushi,zh
            adiw  z,1            ;next address
      wait_data_valid 1
      in    a,dbusin
      ;connect SPI/I2C
      out   cbus,clear
      .if (io_sel_direct > 0)
         out   abushi,d          ;prepare IO select
      .endif
      .ifdef spi_eep_adr
         out   spi_out,f         ;select slave
         sbi   spcr,spe          ;enable SPI
      .else
         out   dbusout,allon     ;float all outputs high
         out   dbusddr,allon     ;(discharge low cap)
         out   dbusddr,zero
         out   dbusout,zero      ;open collector drive via ddr
      .endif
      out   cbus,c            ;set IO select
       ena_io_select
      rcall eep_write         ;store in EEPROM
   loop     eep_app_save_data
   .ifndef spi_eep_adr
      lds      zl,i2c_statreg    ;set ack
      sbr      zl,0b1000000
      sts      i2c_statreg,zl
   .endif
   rcall eep_write_close
   PrintStr eep_saved      ;" saved"
   rcall crlf
   ldi   a,0x16            ;last successful command was read
   sts   dma_last_cmd,a
   rjmp  iw_exit

;dma load program
dma_eep_load:
   lds   a,dma_last_cmd
   lds   b,dma_tab_index
   sts   dma_last_cmd,allon ;mark invalid sequence of command
   cpi   a,0               ;previous set block address table?
   ifne_or  eep_dl_inv
   cpi   b,3               ;block address table valid?
   ifne_or  eep_dl_inv
   lds   a,dma_tab         ;program number valid?
   cpi   a,0xff
   ifeq     eep_dl_inv
      rjmp  iw_exit           ;exit - invalid parameters
   end      eep_dl_inv
   sts   prog_num,a
   PrintStr eep_prog       ;10,13,"Program "
   lds   a,prog_num        ;prog#
   rcall PrintHex
   clr   c        ;check prog exists
   out   eearh,one
   lds   a,prog_num
   do       eep_app_scan_loop
      out   eearl,c        ;read slot pointer
      out   eecr,one
      in    d,eedr
      cp    d,a            ;prog# match?
   exiteq   eep_app_scan_loop
      inc   c              ;all slots done?
      ifeq  eep_app_load_bad_prog
         PrintStr eep_notfound  ;" not found"
         rjmp  eep_app_read_exit
      end   eep_app_load_bad_prog
   loop     eep_app_scan_loop
   rcall eep_open
   ldi   yl,0           ;start reading from 1st slot
   ldi   yh,0
   rcall eep_read
   tst   a
   ifne  eep_app_file_incompat
      PrintStr bad_format  ;" incompatible format"
      rjmp  eep_app_read_exit2
   end   eep_app_file_incompat
   lds   zl,dma_tab+1      ;load memory address
   lds   zh,dma_tab+2
   rcall eep_read             ;read count - set end address
   mov   xl,a
   rcall eep_read
   mov   xh,a
   add   xl,zl
   sts   dma_tab+3,xl
   adc   xh,zh
   sts   dma_tab+4,xh
   .ifdef spi_eep_adr
      out   dbusddr,allon
   .endif
   do       eep_app_load_loop
      cp    xl,zl
      ifeq  eep_app_load_adr
         cp    xh,zh
   exiteq   eep_app_load_loop
      end   eep_app_load_adr
      rcall eep_read
      ;disconnect SPI/I2C, connect RAM
      .if (io_sel_direct > 0)
         out   abushi,e          ;disable IO select
      .endif
      out   cbus,clear        ;disable IO select
      .ifdef spi_eep_adr
         cbi   spcr,spe          ;disable SPI
      .else
         out   dbusddr,allon     ;dbus = output
      .endif
      out   abuslo,zl         ;set write address 
      out   abushi,zh
      out   cbus,writemem     ;WE, ~OE
      out   dbusout,a         ;dbus write
      adiw  z,1               ;next address
      out   cbus,clear        ;~WE / write cycle ends 180ns/16MHz
      ;connect SPI/I2C
      .if (io_sel_direct > 0)
         out   abushi,d          ;prepare IO select
      .endif
      .ifdef spi_eep_adr
         out   spi_out,f         ;set slave select = EEP
         sbi   spcr,spe          ;enable SPI
      .else
         out   dbusout,allon     ;float all outputs high
         out   dbusddr,zero
         out   dbusout,zero      ;open collector drive via ddr
      .endif
      out   cbus,c            ;set IO select
   loop     eep_app_load_loop
   PrintStr eep_loaded     ;" loaded"
   ldi   a,0x17            ;last successful command was read
   sts   dma_last_cmd,a
   .ifndef spi_eep_adr
      lds   a,i2c_statreg
      sbr   a,0b1000000       ;set ack
      sts   i2c_statreg,a
   .endif
eep_app_read_exit2:
   .ifdef spi_eep_adr
      ldi   a,spi_idle     ;close spi slave
      out   spi_out,a
   .else
      sec                  ;nak & stop
      rcall i2c_putack
      rcall i2c_stop
   .endif
eep_app_read_exit:
   rcall crlf
   rcall eep_close
   rjmp  iw_exit

eep_info1:     .db   13,10,"EEPROM autoload ",0,0
eep_info2:     .db   " - ",0
eep_info3:     .db   " slots free",0
eep_info4:     .db   13,10,"prog#/slots  ",0
eep_none:      .db   "none",0,0
eep_notfound:  .db   " not found",0,0
eep_prog:      .db   13,10,"Program ",0,0
eep_loaded:    .db   " loaded",0
eep_saved:     .db   " saved",0,0
err_eep_full:  .db   " EEPROM full",0,0
   .ifdef spi_eep_adr
err_no_eep:    .db   13,10,"SPI EEPROM not found",0,0
   .else
err_no_eep:    .db   13,10,"I2C EEPROM not found",0,0
   .endif
err_corrupt:   .db   " corrupted",0,0
save_wait:     .db   13,10,"Saving, <ESC> to abort",13,10,0,0
save_ok:       .db   13,"Save OK",0,0
save_abort:    .db   " Save aborted",0
bad_format:    .db   " incompatible format",0,0

.else
   .equ  dma_eep_load   = dma_cmd_inv
   .equ  dma_eep_save   = dma_cmd_inv
.endif

