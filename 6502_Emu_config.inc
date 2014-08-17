.if config_part == 1 ;invoked from 6502_Emu.asm

;*********************************************************************
;
; 6502 Emulator Configuration
;
; This file should preserve your personal configuration when upgrading
; the source code.
;
;*********************************************************************
.set  config_version = 831

;ATMega platform - ATMega16 or ATMega32
;  ATMega32 may be required due to flash size for configured features 
.IFNDEF   SIGNATURE_000  ;not already defined per default device in Atmel Studio >4
   .NOLIST
   .INCLUDE "m16def.inc"
   ;.INCLUDE "m32def.inc"
   .LIST
.ENDIF

;AVR clock speed 
.equ  Osc_Hz            = 16000000    ;16 MHz

;external SRAM waittime between applying address and data stable
.set  data_valid_ns     = 180
;  the time is calculated as follows:
;    RAM access time + AVR input sync latch (1/2 clock)
;    + input & output propagation delay (~ 2 * 15ns)
;    for a 62256-80 on an AVR at 16MHz: 80 + 62,5 / 2 + 15 + 15
;    only change if RAM is slower than 120ns
;    in most cases at least 3 waitcycles are used for housekeeping anyway

;Baudrate
.equ BAUD  = 38400      ;maximum standard rate with <1% deviation @ 16MHz
 
;RS232 software flow control watermarks, disabled if undefined
;  the buffer max is 255 bytes
;  RX only to prevent overruning applications input capacity
.equ flowlo = 128       ;low watermark, send XON if less buffer used
.equ flowhi = 192       ;high watermark, send XOFF if reached

;select emulator core CMOS 65C02, NMOS 6502 if undefined
;.equ cmos_core = 1

;interrupt disable mode real, virtual if undefined
;.equ irq_dis_real = 1
;  virtual = background interrupts always allowed = always full control,
;            no atomic timing for disabled instruction sequence.
;  real    = absolutely no interrupts allowed when disabled except single step and NMI,
;            program must not have extended disable periods, atomic timing is enforced,
;            reset button mandatory to recover the monitor/debugger when I bit remains set.

;ROM load & write protect page boundary - RAM at this page and above is write protected
;adds 2 cycles (125ns) to instructions writing to memory (stores & modifying memory)
;.equ rommap    = 0xc0      

;**** IO ****
;internal IO access page (RS232, Timers, Latches) - disabled if undefined
.equ iomap     = 0xbf
;   Can be configured overlapping ROM-space if no operand fetch from the same page
;   is required. Instruction fetch including immediate operands will always be
;   served from RAM (used as peudo-ROM) - writes and reads of nonimmediate operands
;   will be to and from IO-space.
;   setting iomap to page zero is not allowed.

; enable phi2 output on portd7, disabled if undefined
; required to drive 65xx IO chips 
.equ phi2_ena  = 1

; enable NMI input on portd2, disabled if undefined
;.ifndef irq_dis_real
   .equ nmi_ena   = 1
;.endif

;**** IO select expansion registers ****
;define inactive values of IO-select pins on IO-select expansion registers
;defines the opposite value as strobe polarity for each IO select pin

.equ  ios1_default = 0b10000001        ;IO select pin 1 expansion default
;io select bits 10 = -74HC573 input latch
;               11 = 74HC573 output latch
;               12 = HD44780 compatible character LCD
;               13 = 65xx IO
;               14 = I2C connect (74hc4066)
;               15 = XMEM select
;               16 = SPI select
;               17 = -IO reset

.ifndef  phi2_ena       ;not available if phi2 uses cbus pin 7
   ;.equ  ios2_default = 0b00000000    ;IO select pin 2 expansion default
   ;io select bits 20 =
   ;               21 = 
   ;               22 = 
   ;               23 = 
   ;               24 = 
   ;               25 = 
   ;               26 = 
   ;               27 = 
.endif

.ifndef  nmi_ena        ;not available if NMI uses cbus pin 2
   ;.equ  ios3_default = 0b00000000    ;IO select pin 3 expansion default
   ;io select bits 30 =
   ;               31 = 
   ;               32 = 
   ;               33 = 
   ;               34 = 
   ;               35 = 
   ;               36 = 
   ;               37 = 
.endif

;**** IO select for builtin modules ****
; enable SPI module and define IO select signal for SPI, disabled if undefined
.equ spi_sel   = 16     ;IO select register 1, pin 6

; enable I2C module and define IO select signal for I2C, disabled if undefined
.equ i2c_sel   = 14     ;IO select register 1, pin 4

; enable LCD module and define IO select signal for LCD, disabled if undefined
.equ lcd_sel   = 12     ;IO select register 1, pin 2

; IO selects of the addon IO modules in part 2 of the configuration

;**** SPI software address translation ****
;translate IO-address to slave select 
;reset state, must be defined
.equ spi_idle  = 0b11111   ;all slaves inactive
;.equ spi_idle  = 0xf       ;unused address for hardware translation
;SS bit 0      = -SD card
;SS bit 1      = -74HC165 parallel input shifter
;SS bit 2      = -74HC595 parallel output shifter
;SS bit 3      = -25LC512 SPI EEPROM 64kB
;SS bit 4      = -unused

;enable spi virtual address translation, disabled if undefined
.equ spi_vat = 1           

;oplow BCD to SS pin table
.macro   spi_virt_ss
;  x0 = SD card on bit0 (-cs) & bit4 (+oe)
;  x1 = 74HC165 on bit1 (-sh/+ld -oe)
   .db 0b00001^spi_idle,0b00010^spi_idle
;  x2 = 74HC595 on bit2 (+rck)
;  x3 = 25LC512 EEPROM on bit 3 (-cs)
;   .db 0b00100^spi_idle,0b01000^spi_idle
   .db 0b00100^spi_idle,spi_idle ;EEPROM masked
;  x4 unused
;  x5 inactive
   .db 0b10000^spi_idle,spi_idle
;  x6 inactive
;  x7 inactive
   .db spi_idle,spi_idle
.endmacro

;**** IO resets ****
;a reset pin can be defined and must reside on an IO-select expansion register
;this pin is strobed low during reset if defined
.equ  io_reset_pin = 17

;definition for io modules with a reset vector
.macro   io_modules_reset 
   rcall ltch_rs        ;reset 74HC573
.endmacro

;**** miscelaneous ****
;64kB serial EEPROM reserved for non volatile program storage
;eep_adr >= 8 = I2C slave adr.; eep_adr < 8 = SPI device adr.
;none if undefined
;.equ  eep_adr = 0xa0       ;I2C EEPROM
.equ  eep_adr = 3          ;SPI EEPROM

; self diag register enable, allow force IRQ, NMI, RESET
; should stay undefined to prevent you from shooting yourself in the foot
.equ ena_diag  = 1

.endif
.if config_part == 2 ;invoked from 6502_Emu_IO.inc

;**************************************************************
;
; I/O address decode tables
; assign internal or external IO modules
;
;**************************************************************

   align 4           ;keep each table on same page

; I/O address table, reserve 0x10 addresses at a time
;     larger chunks can be defined by pointing sequential addresses
;     to the same entry point
; read address table
ior_tab:
   rjmp  ir_exit     ;0x
   rjmp  ir_exit     ;1x
   rjmp  ir_exit     ;2x
   rjmp  ir_exit     ;3x
   rjmp  ir_exit     ;4x
   rjmp  ir_exit     ;5x
   rjmp  ir_exit     ;6x
   rjmp  ir_exit     ;7x
   rjmp  ir_exit     ;8x
   rjmp  spr_data    ;9x   SPI data register
   rjmp  io65xx_rd   ;Ax   65xx IO chip RAM
   rjmp  io65xx_rd   ;Bx   65xx IO chip
   rjmp  ltch_rd     ;Cx   74HC573 latch(es)
   rjmp  lcd_rd      ;Dx   character LCD
   rjmp  iorE0       ;Ex - subdecoded 0xe0 - 0xef
   rjmp  iordreg     ;Fx - internal emulated registers

; write address table
iow_tab:
   rjmp  iw_exit     ;0x
   rjmp  iw_exit     ;1x
   rjmp  iw_exit     ;2x
   rjmp  iw_exit     ;3x
   rjmp  iw_exit     ;4x
   rjmp  iw_exit     ;5x
   rjmp  iw_exit     ;6x
   rjmp  iw_exit     ;7x
   rjmp  iw_exit     ;8x
   rjmp  spw_data    ;9x   SPI data register
   rjmp  io65xx_wrt  ;Ax   65xx IO chip (RAM)
   rjmp  io65xx_wrt  ;Bx   65xx IO chip
   rjmp  ltch_wrt    ;Cx   74HC573 latch(es)
   rjmp  lcd_wrt     ;Dx   character LCD
   rjmp  iowE0       ;Ex - subdecoded 0xe0 - 0xef
   rjmp  iowtreg     ;Fx - internal emulated registers

; sub address decode tables
; - select 1 from a range of 0x10 addresses

; subdecode 0xe0 - 0xef
; read address decode table iorE0 (0xe0 - 0xef)
iorE0_tab:
   rjmp  ir_exit     ;0
   rjmp  ir_exit     ;1
   rjmp  ir_exit     ;2
   rjmp  ir_exit     ;3
   rjmp  ir_exit     ;4
   rjmp  ir_exit     ;5
   rjmp  ir_exit     ;6
   rjmp  ir_exit     ;7
   rjmp  ir_exit     ;8
   rjmp  ir_exit     ;9
   rjmp  ir_exit     ;A
   rjmp  ir_exit     ;B
   rjmp  ir_exit     ;C
   rjmp  ir_exit     ;D
   rjmp  ir_exit     ;E
   rjmp  ir_exit     ;F


; write address decode table iowE0 (0xe0 - 0xef)
iowE0_tab:
   rjmp  iw_exit     ;0
   rjmp  iw_exit     ;1
   rjmp  iw_exit     ;2
   rjmp  iw_exit     ;3
   rjmp  iw_exit     ;4
   rjmp  iw_exit     ;5
   rjmp  iw_exit     ;6
   rjmp  iw_exit     ;7
   rjmp  iw_exit     ;8
   rjmp  iw_exit     ;9
   rjmp  iw_exit     ;A
   rjmp  iw_exit     ;B
   rjmp  iw_exit     ;C
   rjmp  iw_exit     ;D
   rjmp  iw_exit     ;E
   rjmp  iw_exit     ;F

; internal I/O table, subgroup of range defined by 
; iordreg / iowtreg
; read internal register address table
ior_int_tab:
   rjmp  acia_rx     ;0 - ACIA receive data register
   rjmp  tcdn_r      ;1 - 10ms tick countdown timer
   rjmp  i2c_stat    ;2 - i2c read status
   rjmp  i2c_data_rd ;3 - i2c read data register
   rjmp  t1r_adr     ;4 - timer 1 register address
   rjmp  t1r_data    ;5 - timer 1 data
   rjmp  spr_dat_lst ;6 - spi data after last write
   rjmp  dmr_stat    ;7 - dma status
   rjmp  dmr_data    ;8 - dma parameter data
   rjmp  ir_exit     ;9
   rjmp  ir_exit     ;A
   rjmp  ir_exit     ;B
   rjmp  ir_diag     ;C - diag selftest register
   rjmp  ir_vector   ;D - global internal IRQ vector
   rjmp  ir_mask     ;E - global internal IRQ mask
   rjmp  ir_flag     ;F - global internal IRQ flags

; write internal register address
iow_int_tab:
   rjmp  acia_tx     ;0 - ACIA transmit data register
   rjmp  tcdn_w      ;1 - 10ms tick countdown timer
   rjmp  i2c_cmd     ;2 - i2c command register 
   rjmp  i2c_data_wrt ;3 - i2c write data register
   rjmp  t1w_adr     ;4 - timer 1 register address
   rjmp  t1w_data    ;5 - timer 1 data
   rjmp  spw_cmd     ;6 - spi command register
   rjmp  dmw_cmd     ;7 - dma command register
   rjmp  dmw_data    ;8 - dma parameter data
   rjmp  iw_exit     ;9
   rjmp  iw_exit     ;A
   rjmp  iw_exit     ;B
   rjmp  iw_diag     ;C - diag selftest register
   rjmp  iw_exit     ;D - read only IRQ vector
   rjmp  iw_mask     ;E - global internal IRQ mask
   rjmp  iw_flag     ;F - global internal IRQ flags


;check address decode table alignment for missing or extra entry
.if (pc & 0xf)
   .error "Address decode tables are misaligned!"
.endif

;SPI address to slave select translation
.ifdef spi_vat
spi_vss:
   spi_virt_ss
.endif

;**************************************************************
;
; IO address decoder
;
;**************************************************************

;decoding bits 0-3 of an IO address

iorE0:                  ;read decode for 0xe0 - 0xef
   io_adr_dec  iorE0_tab

iowE0:                  ;write decode for 0xe0 - 0xef
   io_adr_dec  iowE0_tab

;**************************************************************
;
; parallel IO modules
;
; examples: 74HC573 latch IO, Phi2 synced IO
;
;**************************************************************

;IO to 74HC573 latch
;
;  multiple latches require individual IO-selects. these can be set by
;  software address decode or hardware decoder 74HC237 (up to 8 latches,
;  up to 4 each mixed r/w)

ltch_rd:
   set_io_select 10     ;read with strobe low io select extension reg 1, pin 0
   par_rd 100           ;min 60ns to data valid (3state output enable time OE - Q)
                        ;+ 1/2 clock (31.3ns) avr input latch delay
                   
ltch_wrt:
   set_io_select 11     ;write with strobe high io select extension reg 1, pin 1
   par_wrt 60           ;min 60ns enable strobe (propagation delay time LE - Q)
   rjmp  iw_exit        ;next instruction

ltch_rs:
   clr   operand        ;clear 74hc573 output latch
   set_io_reset 11      ;select latch strobe
   par_wrt 60           ;write operand with strobe
   ret                        

;*****************************************************************
;
;IO to 65xx chip using phi2
;
io65xx_rd:
   set_io_select 13     ;read with strobe on io select extension reg 1, pin 3
   phi2_rd              ;timed read synchronized to phi2 (OC2)
                   
io65xx_wrt:
   set_io_select 13     ;write with strobe on io select extension reg 1, pin 3
   phi2_wrt             ;timed write synchronized to phi2 (OC2)

;no software reset - hardware reset driven by reset pin


;.org pc + 0x400         ;test sufficient space for custom modules

.endif
