;
;
;   I O   s u b p r o c e s s o r
;
; An ATMega16 allows access to its IO resources through a 4-bit bus
;
; Copyright (C) 2016-2018  Klaus Dormann
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
#define  version "0.70 "      ;makes a printable version number
;
; version history:
;        12-jul-15   start of development
;  0.70  21-feb-18   initial release
;
; ATMEGA16 fuse settings:
;   16 MHz crystal
;   JTAGEN unprogrammed
;   BOD enabled - 4,0V
;   Preserve EEPROM
;
; description of hardware:
;   An upper (bits 4:7) or lower port (bits 0:3) for the bus
;   A port pin as chip select input
;   A port pin as acknowledge (gated by a 74HC126)
;   A port pin as interrupt request output, open drain
;   Freely configurable, see definition of hardware below 

   .NOLIST
   .include "sam.inc"
   .IFNDEF   SIGNATURE_000  ;not already defined per default device in Atmel Studio >4
      .INCLUDE "m16def.inc"
;      .INCLUDE "m32def.inc"
   .ENDIF
   .LIST
 
;
; reserved registers
;     (V) = vector mode during IRQ 
;
; 0x00 non immediate
.DEF  unused_r0   =r0         ; unused
.DEF  unused_r1   =r1         ; unused
.DEF  spipat      =r2         ; pattern to clock SPI read
.DEF  d           =r3         ; save a for debug trace
; 0x04
.DEF  altregadr   =r4         ; alternate IO register address & funtion
.DEF  altregradr  =r5         ; alternate IO register read address
.DEF  regadr      =r6         ; current IO register address
.DEF  dregadr     =r7         ; data register address & function
; 0x08
.DEF  unused_r8   =r8         ; unused
.DEF  unused_r9   =r9         ; unused
.DEF  unused_r10  =r10        ; unused
.DEF  unused_r11  =r11        ; unused
; 0x0c
.DEF  valtregadr  =r12        ; (V) alternate IO register address & function
.DEF  valtregradr =r13        ; (V) alternate IO register read address
.DEF  vregadr     =r14        ; (V) current IO register address
.DEF  vdregadr    =r15        ; (V) data register address & function
; 0x10 immediate
.DEF  a           =r16        ; immediate GPR1
.DEF  b           =r17        ; immediate GPR2
.DEF  c           =r18        ; immediate GPR3
.DEF  irqvect     =r19        ; current irq request vector
; 0x14
.DEF  sbx         =r20        ; index to sbus function (vectored)
.DEF  unused_r21  =r21        ; unused
.DEF  unused_r22  =r22        ; unused
.DEF  unused_r23  =r23        ; unused
; 0x18 immediate word
.DEF  unused_r24  =r24        ; unused
.DEF  unused_r25  =r25        ; unused
;     x                       ; debug trace input index
; 0x1c
;     y                       ; index to IO register, IO register mask & maps
;     z                       ; index to function table

;
; Definition of Hardware
;
; 4 bit slave bus
.equ  sbus_bits   = 1         ;0 = bits 0:3, 1 = bits 4:7
.equ  sbus_port   = portc     ;ASSR AS2 = 0, JTAGEN fuse unprogrammed (1)!
.equ  sbus_ddr    = ddrc
.equ  sbus_in     = pinc
; chip select input - high active
.equ  cs_pin      = 3
.equ  cs_in       = pinc
; acknowledge output - high active
.equ  ack_pin     = 4         ;SPCR MSTR = 1!
.equ  ack_port    = portb
.equ  ack_ddr     = ddrb
; interrupt request (open drain, low active)
.equ  irq_pin     = 2  
.equ  irq_out     = ddrc      ;set to one to force int req low

; required init for non default protected IO register bits
.macro      ioreg_init
   ldi   a,(1<<mstr)          ;prohibit SPI slave mode
   out   spcr,a
.endmacro

; protected IO register bitmap, 1 = protected, 0 = writable
;
; adjust according to above IO ports usage and don't forget to
; disable the secondary functions of the IO pins used
.macro      protio
   .db 0,0,0,0                               ;0x00 TWBR TWSR TWAR TWDR
   .db 0,0,0,0                               ;0x04 ADCL ADCH ADCSRA ADMUX
   .db 0,0,0,0                               ;0x08 ACSR UBRRL UCSRB UCSRA
   .db 0,0x10,0,0                            ;0x0C UDR SPCR SPSR SPDR
   .db 0,0,0,0xfc                            ;0x10 PIND DDRD PORTD PINC
   .db 0xfc,0xfc,0x10,0x10                   ;0x14 DDRC PORTC PINB DDRB
   .db 0x10,0,0,0                            ;0x18 PORTB PINA DDRA PORTA
   .db 0,0,0,0                               ;0x1C EECR EEDR EEARL EEARH
   .db 0,0xff,0x8,0                          ;0x20 UBRRH/UCSRC WDTCR ASSR OCR2
   .db 0,0,0,0                               ;0x24 TCNT2 TCCR2 ICR1L ICR1H
   .db 0,0,0,0                               ;0x28 OCR1BL OCR1BH OCR1AL OCR1AH
   .db 0,0,0,0                               ;0x2C TCNT1L TCNT1H TCCR1B TCCR1A
   .db 0,0,0,0                               ;0x30 SFIOR OSCCAL TCNT0 TCCR0
   .db 0,0xf0,0,0xff                         ;0x34 MCUCSR MCUCR TWCR SPMCR
   .db 0,0,0,3                               ;0x38 TIFR TIMSK GIFR GICR
   .db 0,0xff,0xff,0xff                      ;0x3C OCR0 SPL SPH SREG
.endmacro

/* fresh copy without any protected IO pins
   no writes to stackpointer & status
   no self destruct
.macro      protio
   .db 0,0,0,0                               ;0x00 TWBR TWSR TWAR TWDR
   .db 0,0,0,0                               ;0x04 ADCL ADCH ADCSRA ADMUX
   .db 0,0,0,0                               ;0x08 ACSR UBRRL UCSRB UCSRA
   .db 0,0,0,0                               ;0x0C UDR SPCR SPSR SPDR
   .db 0,0,0,0                               ;0x10 PIND DDRD PORTD PINC
   .db 0,0,0,0                               ;0x14 DDRC PORTC PINB DDRB
   .db 0,0,0,0                               ;0x18 PORTB PINA DDRA PORTA
   .db 0,0,0,0                               ;0x1C EECR EEDR EEARL EEARH
   .db 0,0xff,0,0                            ;0x20 UBRRH/UCSRC WDTCR ASSR OCR2
   .db 0,0,0,0                               ;0x24 TCNT2 TCCR2 ICR1L ICR1H
   .db 0,0,0,0                               ;0x28 OCR1BL OCR1BH OCR1AL OCR1AH
   .db 0,0,0,0                               ;0x2C TCNT1L TCNT1H TCCR1B TCCR1A
   .db 0,0,0,0                               ;0x30 SFIOR OSCCAL TCNT0 TCCR0
   .db 0,0xf0,0,0xff                         ;0x34 MCUCSR MCUCR TWCR SPMCR
   .db 0,0,0,3                               ;0x38 TIFR TIMSK GIFR GICR
   .db 0,0xff,0xff,0xff                      ;0x3C OCR0 SPL SPH SREG
.endmacro
*/

; gapless stuffing of byte groups
.set        odd_byte = -1
.macro      db1      ;define 1 byte, end on even byte
   .if   odd_byte < 0
      .set  odd_byte = @0 
   .else
      .db   odd_byte,@0
      .set  odd_byte = -1 
   .endif
.endmacro
.macro      db3      ;define 3 bytes, end on even byte
   .if   odd_byte < 0
      .db   @0,@1
      .set  odd_byte = @2 
   .else
      .db   odd_byte,@0,@1,@2
      .set  odd_byte = -1 
   .endif
.endmacro

; define word registers
   .equ  tcnt1 = tcnt1l+0x40 
   .equ  icr1 = icr1l+0x40 
   .equ  ocr1a = ocr1al+0x40 
   .equ  ocr1b = ocr1bl+0x40 
   .equ  adc2 = adcl+0x40 

; IRQ default register address map
;
; IRQ vector to unit register map
.macro      vregmap
   db1 unitmap+3*3            ;  2 Int0
   db1 unitmap+3*3            ;  4 Int1
   db1 unitmap+3*10           ;  6 T2 compare match
   db1 unitmap+3*10           ;  8 T2 overflow
   db1 unitmap+3*9            ; 10 T1 capture event
   db1 unitmap+3*7            ; 12 T1 compare match A
   db1 unitmap+3*8            ; 14 T1 compare match B 
   db1 unitmap+3*7            ; 16 T1 overflow
   db1 unitmap+3*6            ; 18 T0 overflow
   db1 unitmap+3*0            ; 20 SPI transfer complete
   db1 unitmap+3*2            ; 22 USART RX complete
   db1 unitmap+3*2            ; 24 USART data register empty
   db1 unitmap+3*2            ; 26 USART TX complete
   db1 unitmap+3*4            ; 28 ADC conversion complete
   db1 unitmap+3*11           ; 30 EEPROM ready
   db1 unitmap+3*5            ; 32 Analog comparator
   db1 unitmap+3*1            ; 34 TWI/I2C
   db1 unitmap+3*3            ; 36 INT2
   db1 unitmap+3*6            ; 38 T0 compare match
   .if   odd_byte >= 0
      .db   odd_byte,-1
      .set  odd_byte = -1
   .endif
.endmacro

; Unit connect map
;
; connect unit by command register (7)
; regadr, altregadr, dregadr
.macro      ucmap
   db3 spcr,spcr,spdr+0xc0    ;  0 SPI
   db3 twbr,twcr,twdr+0x80    ;  1 TWI/I2C
   db3 ubrrl,ucsrc,udr+0x80   ;  2 USART
   db3 mcucsr,gifr,gicr       ;  3 Ext. Interrupt
   db3 adch,sfior,adc2        ;  4 ADC
   db3 adcsra,sfior,icr1      ;  5 Analog comparator
   db3 tcnt0,tifr,ocr0        ;  6 Timer 0
   db3 tccr1b,tcnt1,ocr1a     ;  7 Timer 1 (ocr1a)
   db3 tccr1b,tcnt1,ocr1b     ;  8 Timer 1 (ocr1b)
   db3 tccr1b,tcnt1,icr1      ;  9 Timer 1 (icr1)
   db3 ocr2,tifr,timsk        ; 10 Timer 2
   db3 eedr,eecr,eedr+0xc0    ; 11 EEPROM data
   db3 eedr,eecr,eearh+0xc0   ; 12 EEPROM address
   .if   odd_byte >= 0
      .db   odd_byte,-1
      .set  odd_byte = -1
   .endif
.endmacro

; bit set / clear map
.macro      regbit            ; @0 = register, @1 = bit
   .db   @0+regprot,(1<<@1)
.endmacro
.macro   regbitmap
                              ; ## + 0x60 to enable, + 0x40 to disable
   regbit   adcsra,adie       ; 00: analog to digital converter interrupt enable
   regbit   acsr,acie         ; 01: analog comparator interrupt enable
   regbit   ucsrb,rxcie       ; 02: USART receive complete interrupt enable
   regbit   ucsrb,txcie       ; 03: USART transmit complete interrupt enable
   regbit   ucsrb,udrie       ; 04: USART data register empty interrupt enable
   regbit   spcr,spie         ; 05: SPI interrupt enable
   regbit   eecr,eerie        ; 06: EEPROM ready interrupt enable
   regbit   twcr,twie         ; 07: TWI/I2C interrupt enable
   regbit   timsk,ocie2       ; 08: timer2 output compare match interrupt enable
   regbit   timsk,toie2       ; 09: timer2 overflow interrupt enable
   regbit   timsk,ticie1      ; 10: timer1 input capture interrupt enable
   regbit   timsk,ocie1a      ; 11: timer1 output compare match a interrupt enable
   regbit   timsk,ocie1b      ; 12: timer1 output compare match b interrupt enable
   regbit   timsk,toie1       ; 13: timer1 overflow interrupt enable
   regbit   timsk,ocie2       ; 14: timer0 output compare match interrupt enable
   regbit   timsk,toie2       ; 15: timer0 overflow interrupt enable
   regbit   gicr,int1         ; 16: external interrupt request 1 enable
   regbit   gicr,int0         ; 17: external interrupt request 0 enable
   regbit   gicr,int2         ; 18: external interrupt request 2 enable
   .dw   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 ; fill 19-31
.endm

;enabling the debug trace feature below causes degraded performance.
;it should be left disabled in normal operation
;.set      deb_en = 1          ; debug trace enabled if defined

.macro      addi
            subi  @0,-@1      ; subtract the negative of an immediate value
.endmacro

.macro      align             ;align to (1<<@0)
alignfromhere:
   .if (alignfromhere & ((1<<@0)-1))   ;if not already aligned
      .org  (alignfromhere & (0xffff<<@0)) + (1<<@0)
   .endif
.endmacro

.macro      align4more        ;align to 2^4 and check
   align 4
   .if pc != (aligncheck + 0x10)
      .error "The subprocessor function table is misaligned!"
   .endif
   .set aligncheck = pc
.endmacro


.DSEG
   .org  0x60
regprot:    .byte 0x42        ;protected register bits
unitmap:    .byte 13*3        ;13 connectable units
vecmap:     .byte 1           ;even byte stuffed / vector 0
            .byte 19          ;vector to unit map
   .org  0x100
csbitmap:   .byte 0x40        ;map of clearable/settable register bits
;deb_reg:    .byte 6           ;DEBUG save of registers
eep_adr:    .byte 2           ;shadow EEPROM address
diag_adr:   .byte 2           ;current diag read/write address
ver_msgx:   .byte 2           ;index to version message after reset
   .ifdef deb_en
      .org  0x200
tracebuf:
      .byte 0x200
tracebufend:
   .endif

;*****************************************************************
;
; reset and interrupt vectors according to µc type
;
;*****************************************************************
.CSEG
#ifdef _M16DEF_INC_
   jmp   reset             ; Reset handler
   ldi   irqvect,2         ; INT0 handler
   rjmp  setirq
   ldi   irqvect,4         ; INT1 handler
   rjmp  setirq
   ldi   irqvect,6         ; Timer2 compare match
   rjmp  setirq
   ldi   irqvect,8         ; Timer2 overflow
   rjmp  setirq
   ldi   irqvect,10        ; Timer1 capture event
   rjmp  setirq
   ldi   irqvect,12        ; Timer1 compare match A 
   rjmp  setirq
   ldi   irqvect,14        ; Timer1 compare match B
   rjmp  setirq
   ldi   irqvect,16        ; Timer1 overflow handler
   rjmp  setirq
   ldi   irqvect,18        ; Timer0 overflow handler
   rjmp  setirq
   ldi   irqvect,20        ; SPI - Serial transfer complete
   rjmp  setirq
   ldi   irqvect,22        ; USART - RX complete
   rjmp  setirq
   ldi   irqvect,24        ; USART - data register empty
   rjmp  setirq
   ldi   irqvect,26        ; USART - TX complete
   rjmp  setirq
   ldi   irqvect,28        ; ADC conversion complete
   rjmp  setirq
   ldi   irqvect,30        ; EEPROM ready
   rjmp  setirq
   ldi   irqvect,32        ; Analog comparator
   rjmp  setirq
   ldi   irqvect,34        ; TWI serial interface
   rjmp  setirq
   ldi   irqvect,36        ; INT2 handler
   rjmp  setirq
   ldi   irqvect,38        ; Timer0 compare match
   rjmp  setirq
   ldi   irqvect,40        ; SPM ready
#endif
#ifdef _M32DEF_INC_
; ATMega32 reordered vectors to match the ATMega16 vector numbers
   jmp   reset             ; Reset handler
   ldi   irqvect,2         ; INT0 handler
   rjmp  setirq
   ldi   irqvect,4         ; INT1 handler
   rjmp  setirq
   ldi   irqvect,36        ; INT2 handler
   rjmp  setirq
   ldi   irqvect,6         ; Timer2 compare match
   rjmp  setirq
   ldi   irqvect,8         ; Timer2 overflow
   rjmp  setirq
   ldi   irqvect,10        ; Timer1 capture event
   rjmp  setirq
   ldi   irqvect,12        ; Timer1 compare match A 
   rjmp  setirq
   ldi   irqvect,14        ; Timer1 compare match B
   rjmp  setirq
   ldi   irqvect,16        ; Timer1 overflow handler
   rjmp  setirq
   ldi   irqvect,38        ; Timer0 compare match
   rjmp  setirq
   ldi   irqvect,18        ; Timer0 overflow handler
   rjmp  setirq
   ldi   irqvect,20        ; SPI - Serial transfer complete
   rjmp  setirq
   ldi   irqvect,22        ; USART - RX complete
   rjmp  setirq
   ldi   irqvect,24        ; USART - data register empty
   rjmp  setirq
   ldi   irqvect,26        ; USART - TX complete
   rjmp  setirq
   ldi   irqvect,28        ; ADC conversion complete
   rjmp  setirq
   ldi   irqvect,30        ; EEPROM ready
   rjmp  setirq
   ldi   irqvect,32        ; Analog comparator
   rjmp  setirq
   ldi   irqvect,34        ; TWI serial interface
   rjmp  setirq
   ldi   irqvect,40        ; SPM ready
#endif
; queue interrupt vector codes
setirq:
   sbi   irq_out,irq_pin   ;set interrupt request
   mov   yl,irqvect        ;load IO register default addresses
   lsr   yl                
   addi  yl,vecmap
   ld    yl,y              ;load unit index
   ld    vregadr,y+        ;load register address
   ld    valtregadr,y      ;load alternate register address
   ld    valtregradr,y+    ;load alternate register read address
   ld    vdregadr,y        ;load data register address
   .ifdef deb_en
      ldi   a,0x01            ;mark interrupt vector set
      st    x+,a
      st    x+,irqvect
      cpi   xh,high(tracebufend)
      ifeq  irq_db_end
         ldi   xh,high(tracebuf)
      end   irq_db_end
   .endif
   ret                     ;leave interrupts disabled until the
                           ;vector is pulled and acknowledged

;
; *********** HARDWARE RESET VECTOR ****************
;
reset:
   ldi   a,high(ramend)    ;set Stack-Pointer
   out   sph,a
   ldi   a,low(ramend)
   out   spl,a

   in    a,mcucsr          ;save reset flags
   clr   c                 ;clear the watchdog
   ldi   b,(1<<wdtoe|1<<wde)
   out   wdtcr,b
   out   wdtcr,c

   sbi   ack_ddr,ack_pin   ;set ack low

   .ifdef deb_en
; clear trace buffer and pointer on power on or brown out
      andi  a,(1<<borf)|(1<<porf)
      ifne  clear_trace
         clr   xl
         ldi   xh,high(tracebuf)
         do       deb_clr
            st    x+,c
            cpi   xh,high(tracebufend)
          loopne   deb_clr
         ldi   xh,high(tracebuf)
      end   clear_trace
   .endif
   out   mcucsr,c          ;clear reset flags

   clr   irqvect           ;no irq pending
   ldi   b,spl             ;invalid (points to write protected reg)
   mov   regadr,b
   mov   altregadr,b
   mov   altregradr,b
   mov   vregadr,b
   mov   valtregadr,b
   mov   valtregradr,b
   mov   vdregadr,b
   ldi   a,wdtcr+0xc0      ;version message
   mov   dregadr,a
   ldi   a,low(ver_msg<<1)
   sts   ver_msgx,a
   ldi   a,high(ver_msg<<1)
   sts   ver_msgx+1,a
   ldi   zh,high(bitmap<<1)   ;move register protection and
   ldi   zl,low(bitmap<<1)    ;default register address to RAM
   ldi   yh,high(regprot)
   ldi   yl,low(regprot)
   do       flash2ram
      lpm   a,z+
      st    y+,a
      cpi   zl,low(bitmapend<<1)
   loopne   flash2ram
   lds   a,regprot+spcr    ;spi enabled?
   sbrc  a,spe
      sts   unitmap+2,b       ;disable spdr to drx
   lds   a,regprot+twcr    ;twi enabled?
   sbrc  a,twen
      sts   unitmap+5,b       ;disable twdr to drx
   lds   a,regprot+ucsrb   ;usart enabled?
   andi  a,(1<<rxen|1<<txen)
   cpi   a,(1<<rxen|1<<txen)
   ifeq  rs_usart_disable
      sts   unitmap+8,b       ;disable udr to drx
   end   rs_usart_disable
   clr   spipat            ;preset spi pattern to 0
;   clr   spiport           ;none selected
   ldi   sbx,high(sbus_func)
   mov   zh,sbx
   ldi   yh,0              ;load index to IO register

   ioreg_init

   rjmp  main

; table with protected IO register bits and
; default vector mode register assignment
bitmap:
   protio   ;as configured earlier
   .dw   -1             ;overflow (reg+2) protection
   ucmap
   vregmap
   .org  bitmap+0x50
   regbitmap
bitmapend:

; version message (read from data register after reset)
ver_msg:   .db   version," built ",__DATE__," ",__TIME__,0,0

; common code for read sbus functions
sbus_read:
   .ifdef deb_en
      in    zl,sbus_in        ;trace address later
      mov   d,a               ;trace a later
   .endif
   mov   b,a               ;prepare high nibble
   andi  b,0xf0            ;mask nibble
   .if sbus_bits == 0
                              ;xx  clocks after ACK raised
      sbi   ack_port,ack_pin  ; 0  set ack
      in    c,sbus_port       ; 1  preserve remaining port state
      swap  b                 ; 2  move to lower
      andi  c,0xf0            ; 3  keep upper port
      or    b,c               ; 4  merge with low nibble
      out   sbus_port,b       ; 5  prepare for output
      in    b,sbus_ddr        ; 6  prepare ddr to write
      ori   b,0xf             ; 7  ddr to write low nibble
      out   sbus_ddr,b        ; 8! output = data high
      andi  a,0xf             ; 9  mask low nibble
      or    a,c               ;10  merge with low nibble
      cbi   ack_port,ack_pin  ;12  clear ack
      out   sbus_port,a       ;13! output = data low
      andi  b,0xf0            ;14  prepare ddr to read
   .else
      sbi   ack_port,ack_pin  ; 0  set ack
      in    c,sbus_port       ; 1  preserve remaining port state
      swap  a                 ; 2  move lower to upper
      andi  c,0xf             ; 3  keep lower port
      or    b,c               ; 4  merge with high nibble
      out   sbus_port,b       ; 5  prepare for output
      in    b,sbus_ddr        ; 6  preserve low nibble ddr
      ori   b,0xf0            ; 7  ddr to write high nibble
      out   sbus_ddr,b        ; 8! output = data high
      andi  a,0xf0            ; 9  mask low nibble
      or    a,c               ;10  merge with low nibble
      cbi   ack_port,ack_pin  ;12  clear ack
      out   sbus_port,a       ;13! output = data low
      andi  b,0xf             ;14  prepare ddr to read
   .endif
   nop                        ;15
   nop                        ;16
   nop                        ;17
   out   sbus_ddr,b           ;18! return sbus to read mode
   out   sbus_port,c          ;clear nibble
   .ifdef deb_en
      .if sbus_bits == 0
         swap  zl                ;low to high
      .endif
      andi  zl,0x70           ;remove read and unused bits
      cpi   zl,0x60           ;skip trace if diag read
      ifeq  rd_db_dr
         cpi   sbx,high(sbus_vfunc)
         ifeq  rd_db_vec
            mov   b,vdregadr
         else  rd_db_vec
            mov   b,dregadr
         end   rd_db_vec
         cpi   b,0xfd
         brsh  rd_db_diag
      end   rd_db_dr
      or    zl,sbx            ;mark read normal or vectored
      swap  zl
      st    x+,zl             ;trace address
      st    x+,d              ;trace data
      cpi   xh,high(tracebufend)
      ifeq  rd_db_end
         ldi   xh,high(tracebuf)
      end   rd_db_end
rd_db_diag:
   .endif
   tst   irqvect
   brne  vmain                ;pending IRQ vector
;
; MAIN LOOP - waiting for master requests
;
main:
   sbis  irq_out,irq_pin   ;IRQ pending?
   sei                     ;end atomic mode
vmain:
wait_cs:
   sbis  cs_in,cs_pin      ;wait for chip select
   rjmp  wait_cs
   cli                     ;atomic mode
   .if sbus_bits == 0
                              ; x  clocks after ACK raised
      sbis  sbus_in,3         ;-1! no ack for read / address -2.5 
      sbi   ack_port,ack_pin  ; 0  set ack on write
      in    zl,sbus_in        ; 1  read address -0.5
      swap  zl                ; 2  move nibble
      andi  zl,0xf0           ; 3  mask for function table 
      sbrc  zl,7              ; 5  continue write function
      ijmp                    ; -  execute read function for address
      inc   zh                ; 6  adjust offset to write function
      ori   zl,0x80           ; 7
      nop                     ; 8
      nop                     ; 9
      in    a,sbus_in         ;10! receive data high nibble 8.5
      cbi   ack_port,ack_pin  ;12  clear ack
      swap  a                 ;13  move to high nibble
      andi  a,0xf0            ;14  mask data high nibble
      in    b,sbus_in         ;15! receive data low nibble 13.5
   .else
      sbis  sbus_in,7         ;-1! no ack for read / address -2.5 
      sbi   ack_port,ack_pin  ; 0  set ack on write
      in    zl,sbus_in        ; 1  read address -0.5
      andi  zl,0xf0           ; 2  mask for function table 
      sbrc  zl,7              ; 4  continue write function
      ijmp                    ; -  execute read function for address
      nop                     ; 5
      inc   zh                ; 6  adjust offset to write function
      ori   zl,0x80           ; 7
      nop                     ; 8
      nop                     ; 9
      in    a,sbus_in         ;10! receive data high nibble 8.5
      cbi   ack_port,ack_pin  ;12  clear ack
      andi  a,0xf0            ;13  mask data high nibble
      nop                     ;14
      in    b,sbus_in         ;15! receive data low nibble 13.5
      swap  b                 ;move to low nibble
   .endif
   andi  b,0xf             ;mask data low nibble
   or    a,b               ;merge data
   .ifdef deb_en
      cpi   zl,0xe0           ;skip trace if diag write
      ifeq  wr_db_dr
         cpi   sbx,high(sbus_vfunc)
         ifeq  wr_db_vec
            mov   b,vdregadr
         else  wr_db_vec
            mov   b,dregadr
         end   wr_db_vec
         cpi   b,0xfd
         brsh  wr_db_diag
      end   wr_db_dr
      mov   b,zh
      or    b,zl
      andi  b,0x7f
      swap  b
      st    x+,b
      st    x+,a
      cpi   xh,high(tracebufend)
      ifeq  wr_db_end
         ldi   xh,high(tracebuf)
      end   wr_db_end
wr_db_diag:
   .endif
   ijmp                    ;continue write function for address

   align 7
   .if (pc & 0xff) != 0x80
      .org pc+0x80
   .endif
   .set aligncheck = pc
;
; table with subprocessor bus functions
;
; 16 words for each function
;
sbus_func:
;read 0 - read IO register
   mov   yl,regadr         ;load IO register by index
   addi  yl,0x20
   ld    a,y
   rjmp  sbus_read

   align4more
;read 1 - read IO register +1, may address SRAM 0x60!
   mov   yl,regadr         ;load IO register by index +1
   addi  yl,0x21
   ld    a,y
   rjmp  sbus_read

   align4more
;read 2 - read IO register +2, may address SRAM 0x61!
   mov   yl,regadr         ;load IO register by index +2
   addi  yl,0x22
   ld    a,y
   rjmp  sbus_read

   align4more
;read 3 - read interrupt vector & set vector mode
   mov   a,irqvect
   tst   irqvect           ;previous vector pulled?
   ifne     irq_service
      cbi   irq_out,irq_pin
      ldi   sbx,high(sbus_vfunc)
      mov   zh,sbx
   end      irq_service
   rjmp  sbus_read

   align4more
;read 4 - read alternate IO register
   mov   yl,altregradr     ;load index to IO register
   andi  yl,0x3f           ;mask valid address range
   addi  yl,0x20           ;adjust IO to common address space
   ld    a,y               ;load original IO register
   sbrs  altregradr,6      ;decode read double
      rjmp  sbus_read
   ldi   b,1               ;alternate address for low & high byte
   eor   altregradr,b
   rjmp  sbus_read

   align4more
;read 5 - read data register without tags
   clr   a
   mov   yl,dregadr        ;load index to IO register
   andi  yl,0x3f           ;mask valid address range
   addi  yl,0x20           ;adjust IO to common address space
   ld    a,y               ;load original IO register
   sbrs  dregadr,6         ;decode read double
      rjmp  sbus_read      ;end read single
   sbrc  dregadr,7
      rjmp  sbus_read      ;end read single
   ldi   b,1               ;alternate address for low & high byte
   eor   dregadr,b
   rjmp  sbus_read

   align4more
;read 6 - read data register with tags
   mov   yl,dregadr        ;load index to IO register
   cpi   yl,0xc0           ;decode special non IO register
   brsh  read_special
   andi  yl,0x3f           ;mask valid address range
   addi  yl,0x20           ;adjust IO to common address space
   ld    a,y               ;load original IO register
   sbrc  dregadr,7         ;decode read with tags
      rjmp  read_tags
   sbrs  dregadr,6         ;decode read double
      rjmp  sbus_read      ;end read single
   ldi   b,1               ;alternate address for low & high byte
   eor   dregadr,b
   rjmp  sbus_read

   align4more
;read 7 - read data register address or state
   mov   a,dregadr
   rjmp  sbus_read
      
; dregadr = 0b11xxxxxx 
read_special:              ;read with presets
   cpi   yl,spdr+0xc0
   ifeq  read_spi
      in    a,spdr
      in    b,spcr            ;dummy access
      out   spdr,spipat       ;write dummy pattern
      rjmp  sbus_read
   end   read_spi
   cpi   yl,eedr+0xc0
   ifeq  read_eep
      lds   yl,eep_adr        ;preload eep address
      lds   yh,eep_adr+1
      out   eearl,yl          ;read a from eep address
      out   eearh,yh
      sbi   eecr,eere
      in    a,eedr
      adiw  y,1               ;increment address
      sts   eep_adr,yl
      sts   eep_adr+1,yh
      clr   yh                ;restore index to IO registers
      rjmp  sbus_read
   end   read_eep
   cpi   yl,0xff           ;read diag data?
   ifeq  diag_rddata
      lds   yl,diag_adr
      lds   yh,diag_adr+1
      ld    a,y+
      sts   diag_adr+1,yh
      sts   diag_adr,yl
      clr   yh
      rjmp  sbus_read
   end   diag_rddata
   cpi   yl,wdtcr+0xc0     ;version message after reset?
   ifeq  read_ver
      lds   zl,ver_msgx
      lds   zh,ver_msgx+1
      lpm   a,z+
      tst   a
      ifne  read_ver_nxt
         sts   ver_msgx,zl
         sts   ver_msgx+1,zh
      end   read_ver_nxt
      mov   zh,sbx
      rjmp  sbus_read
   end   read_ver
   ldi   a,0               ;pad null byte
   rjmp  sbus_read

; dregadr = 0b10xxxxxx 
read_tags:                 ;read & set tags
   cpi   yl,twdr+0x20
   ifeq  read_twdr
      in    a,twdr
      in    b,twcr            ;clear twint with ack
      andi  b,(1<<twie)       ;keep interrupt enable
      ori   b,(1<<twint|1<<twea|1<<twen)
      out   twcr,b
   end   read_twdr
   rjmp  sbus_read

   align 7
   .if (pc & 0xff) != 0x80
      .org pc+0x80
   .endif
   .set aligncheck = pc
;write 0 - write IO register
   mov   yl,regadr
   addi  yl,0x60           ;adjust to regprot table
   ld    c,y               ;read protected bits
   subi  yl,0x40           ;adjust IO to common address space
   tst   c                 ;any protected bits?
   ifne  wr0_apb
      ld    b,y               ;load original IO register
      and   b,c               ;mask bits to keep
      com   c
      and   a,c               ;mask bits to change
      or    a,b               ;merge
   end   wr0_apb
   st    y,a               ;store modified IO register
   mov   zh,sbx
   rjmp  main

   align4more
;write 1 - write IO register +1
   mov   yl,regadr
   addi  yl,0x61           ;adjust to regprot table +1
   ld    c,y               ;read protected bits
   subi  yl,0x40           ;adjust IO to common address space
   tst   c                 ;any protected bits?
   ifne  wr1_apb
      ld    b,y               ;load original IO register
      and   b,c               ;mask bits to keep
      com   c
      and   a,c               ;mask bits to change
      or    a,b               ;merge
   end   wr1_apb
   st    y,a               ;store modified IO register
   mov   zh,sbx
   rjmp  main

   align4more
;write 2 - write IO register +2
   mov   yl,regadr
   addi  yl,0x62           ;adjust to regprot table +2
   ld    c,y               ;read protected bits
   subi  yl,0x40           ;adjust IO to common address space
   tst   c                 ;any protected bits?
   ifne  wr2_apb
wr4_apb:
      ld    b,y               ;load original IO register
      and   b,c               ;mask bits to keep
      com   c
      and   a,c               ;mask bits to change
      or    a,b               ;merge
   end   wr2_apb
   st    y,a               ;store modified IO register
   mov   zh,sbx
   rjmp  main

   align4more
;write 3 - set IO register address
   andi  a,0x3f            ;mask valid address range
   mov   regadr,a          ;save to register address
   mov   zh,sbx
   rjmp  main

   align4more
;write 4 - write alternate IO register
   sbrc  altregadr,7       ;decode options
      rjmp  setclr_byte
   sbrc  altregadr,6
      rjmp  write_double
   mov   yl,altregadr
   addi  yl,0x60           ;adjust to regprot table
   ld    c,y               ;read protected bits
   subi  yl,0x40           ;adjust IO to common address space
   tst   c                 ;any protected bits?
   brne  wr4_apb
   st    y,a               ;store modified IO register
   mov   zh,sbx
   rjmp  main

   align4more
;write 5 - set alternate IO register address
   mov   zh,sbx
   sbrc  a,7   
      rjmp  port2pin
   sbrs  a,6
      rjmp  port2pin
   andi  a,0xfe            ;force even address for word access
   mov   altregadr,a       ;save to register address
   mov   altregradr,a
   rjmp  main

   align4more
;write 6 - data register DR
   mov   zh,sbx
   sbrs  dregadr,6         ;decode options
      rjmp  write_dr
   sbrc  dregadr,7
      rjmp  write_special
   ldi   b,1               ;pre toggle odd/even
   eor   dregadr,b
   rjmp  write_dr

   align4more
;write 7 - command register
   sbrc  a,7               ;>0x80
      rjmp  setclr_portbit
   sbrc  a,6               ;>0x40
      rjmp  setclr_regbit
   mov   zh,sbx
   cpi   a,13                 ;if < 13 then connect units
   iflo  load_unit
      mov   yl,a              ;address = 3 * a + offset
      lsl   yl  
      add   yl,a
      addi  yl,unitmap
      ld    regadr,y+         ;registers 0-2
      ld    altregadr,y       ;alternate register 4
      ld    altregradr,y+     ;alternate read register 4
      ld    dregadr,y         ;data register 6
      rjmp  main
   end   load_unit
   cpi   a,62
   breq  wr7_exit             ;already normal mode
   ifsh  force_reset          ;63
      mov   a,altregadr
      cpi   a,wdtcr           ;prev. write 5 (wdtcr)
      ifeq  force_watchdog
         ldi   a,(1<<wde)
         out   wdtcr,a
         rjmp  pc
      end   force_watchdog
wr7_exit:
      rjmp  main
   end   force_reset
   mov   b,altregadr          ;diag safety lock?
   cpi   b,sreg
   brne  wr7_exit
   cpi   a,60
   ifeq  diag_data            ;60 - access diag data
      ldi   b,0xff            ;dummy sreg
      mov   dregadr,b
      jmp   main
   end   diag_data
   ifsh  diag_adr             ;61 - store diag address
      ldi   b,0xfd            ;dummy spl
      mov   dregadr,b
      jmp   main
   end   diag_adr
   rjmp  main
     

;extra options - write 4 (alternate IO register)

setclr_byte:               ;set or clear bits in byte (write 4)
   mov   yl,altregadr
   andi  yl,0x3f           ;adjust to regprot table
   addi  yl,0x60
   ld    c,y               ;read protected bits
   subi  yl,0x40           ;adjust IO to common address space
   ld    b,y               ;load original IO register
   sbrs  altregadr,6       ;set or clear?
   ifs   set_byte
      com   c
      and   a,c               ;bits not to set
      or    b,a               ;merge
   else  set_byte
      com   a                 ;build and mask
      or    a,c               ;bits not to clear
      and   b,a               ;apply bit mask
   end   set_byte
   st    y,b               ;store modified IO register
   ldi   a,0x40            ;toggle next op: set <-> clear
   eor   altregadr,a
   mov   zh,sbx
   rjmp  main

write_double:              ;alternate between high and low byte
   ldi   b,1               ;pre toggle odd/even
   eor   altregadr,b
   mov   yl,altregadr
   addi  yl,0x20           ;adjust to regprot table
   ld    c,y               ;read protected bits
   subi  yl,0x40           ;adjust IO to common address space
   tst   c                 ;any protected bits?
   ifne  wdbl_apb
      ld    b,y               ;load original IO register
      and   b,c               ;mask bits to keep
      com   c
      and   a,c               ;mask bits to change
      or    a,b               ;merge
   end   wdbl_apb
   st    y,a               ;store modified IO register
   mov   zh,sbx
   rjmp  main

;convert port or ddr to pin on read - write 5

port2pin:
   mov   altregadr,a       ;save to register address
   andi  a,0x3f
   cpi   a,pind
   ifsh_and portrange
   cpi   a,porta+1
   iflo     portrange
      cpi   a,pinb
      iflo  portlow
         cpi   a,pinc
         iflo  port2pind
            ldi   a,pind
         else  port2pind
            ldi   a,pinc
         end   port2pind
      else  portlow
         cpi   a,pina
         iflo  port2pinb
            ldi   a,pinb
         else  port2pinb
            ldi   a,pina
         end   port2pinb
      end   portlow
   end      portrange
   mov   altregradr,a
   rjmp  main

;extra options - write 6 (data register)

write_special:
   mov   b,dregadr
   cpi   b,spdr+0xc0
   ifeq  wr_spi
      in    b,spcr            ;dummy access
      out   spdr,a
      mov   spipat,a
      rjmp  main
   end   wr_spi
   cpi   b,eedr+0xc0       ;write data and increment shadow address
   ifeq  eep_data
      lds   yl,eep_adr
      lds   yh,eep_adr+1
      out   eearl,yl          ;write a to eep address
      out   eearh,yh
      out   eedr,a
      sbi   eecr,eemwe
      sbi   eecr,eewe
      adiw  y,1               ;increment address
      sts   eep_adr,yl
      sts   eep_adr+1,yh
      clr   yh                ;restore index to IO registers
      rjmp  main
   end   eep_data
   cpi   b,eearh+0xc0         ;set eeprom write address high
   ifeq  eep_adrh
      sts   eep_adr+1,a
      dec   dregadr
      rjmp  main
   end   eep_adrh
   cpi   b,eearl+0xc0         ;set eeprom write address low
   ifeq  eep_adrl
      sts   eep_adr,a
      dec   dregadr
      rjmp  main
   end   eep_adrl
   cpi   b,0xff               ;write diag data
   ifeq  diag_wrdata
      lds   yl,diag_adr
      lds   yh,diag_adr+1
      st    y+,a
      sts   diag_adr+1,yh
      sts   diag_adr,yl
      clr   yh
      rjmp  main
   end   diag_wrdata
   cpi   b,0xfd
   ifeq  diag_adrhi           ;write diag address high
      sts   diag_adr+1,a
      inc   dregadr
      rjmp  main
   end   diag_adrhi
   cpi   b,0xfe               ;write diag address low
   ifeq  diag_adrlo
      sts   diag_adr,a
      inc   dregadr
      rjmp  main
   end   diag_adrlo
   rjmp  main

write_dr:                  ;write data register
   mov   yl,dregadr        ;build index
   andi  yl,0x3f           ;mask options
   addi  yl,0x60           ;adjust to regprot table
   ld    c,y               ;read protected bits
   subi  yl,0x40           ;adjust IO to common address space
   tst   c                 ;any protected bits?
   ifne  wdr_apb
      ld    b,y               ;load original IO register
      and   b,c               ;mask bits to keep
      com   c
      and   a,c               ;mask bits to change
      or    a,b               ;merge
   end  wdr_apb
   st    y,a               ;store modified IO register
   sbrs  dregadr,7         ;with tags?
      rjmp  main           ;no tags
   cpi   yl,twdr+0x20
   ifeq  wdr_twi
      in    b,twcr
      andi  b,(1<<twie)       ;keep interrupt enable
      ori   b,(1<<twint|1<<twen)
      out   twcr,b            ;start TWI op
   end   wdr_twi
   rjmp  main

;bit commands - write 7 (extended register)

;1abbcddd  set or clear a port-bit
;     ddd- pin# 0-7
;    c---- 0=clear, 1=set
;  bb----- port# a-d
; a------- 0=port, 1=ddr
setclr_portbit:
   ldi   b,1               ;generate bit position
   sbrc  a,1
   ldi   b,(1<<2)
   sbrc  a,0
   lsl   b
   sbrc  a,2
   swap  b
   ldi   yl,porta+regprot  ;generate IO register address + protect map
   sbrc  a,6
   dec   yl                ;is ddr
   sbrc  a,5
   subi  yl,6
   sbrc  a,4
   subi  yl,3
   ld    c,y               ;read protected bits
   subi  yl,regprot-0x20   ;adjust IO to common address space
;   sts   deb_reg,a         ;DEBUG command
;   sts   deb_reg+1,b       ;      bits to clear/set
;   sts   deb_reg+2,c       ;      protected bits
;   sts   deb_reg+3,yl      ;      IO-address
   sbrs  a,3               ;set or clear?
   ifs   set_portbyte
      ld    a,y               ;load original IO register
;      sts   deb_reg+4,a       ;DEBUG IO-Register before
      com   c
      and   b,c               ;bits not to set
      or    a,b               ;merge
   else  set_portbyte
      ld    a,y               ;load original IO register
;      sts   deb_reg+4,a       ;DEBUG IO-Register before
      com   b                 ;build and mask
      or    b,c               ;bits not to clear
      and   a,b               ;apply bit mask
   end   set_portbyte
;   sts   deb_reg+5,a       ;DEBUG IO-Register after
   st    y,a               ;store modified IO register
   mov   zh,sbx
   rjmp  main

;10abbbbb  set or clear a bit in register according to regbitmap
;   bbbbb- bitmap position 0-31
;  a------ 0=clear, 1=set
setclr_regbit:
   mov   zl,a              ;refer to regbitmap
   andi  zl,0x1f
   lsl   zl
   ldi   zh,high(csbitmap)
   ld    yl,z+             ;get register address
   ld    b,z               ;get bit
   cpi   yl,0xff           ;is valid address?
   ifne  regbitmap_valid
      ld    c,y               ;read protected bits
      subi  yl,regprot-0x20   ;adjust IO to common address space
      sbrs  a,5               ;set or clear?
      ifs   set_regbyte
         ld    a,y               ;load original IO register
         com   c
         and   b,c               ;bits not to set
         or    a,b               ;merge
      else  set_regbyte
         ld    a,y               ;load original IO register
         com   b                 ;build and mask
         or    b,c               ;bits not to clear
         and   a,b               ;apply bit mask
      end   set_regbyte
      st    y,a               ;store modified IO register
   end   regbitmap_valid
   mov   zh,sbx
   rjmp  main

   align 7
   .if (pc & 0xff) != 0x80
      .org pc+0x80
   .endif
   .set aligncheck = pc
;
; table with subprocessor bus functions in vector mode
;
; 16 words for each function
;
sbus_vfunc:
;read 0 - read IO register
   mov   yl,vregadr        ;load IO register by index
   addi  yl,0x20
   ld    a,y
   rjmp  sbus_read

   align4more
;read 1 - read IO register +1, may address SRAM 0x60!
   mov   yl,vregadr        ;load IO register by index +1
   addi  yl,0x21
   ld    a,y
   rjmp  sbus_read

   align4more
;read 2 - read IO register +2, may address SRAM 0x61!
   mov   yl,vregadr        ;load IO register by index +2
   addi  yl,0x22
   ld    a,y
   rjmp  sbus_read

   align4more
;read 3 - clear vector mode
   clr   irqvect
   clr   a
   ldi   sbx,high(sbus_func)
   mov   zh,sbx
   rjmp  sbus_read

   align4more
;read 4 - read alternate IO register
   mov   yl,valtregradr    ;load index to IO register
   andi  yl,0x3f           ;mask valid address range
   addi  yl,0x20           ;adjust IO to common address space
   ld    a,y               ;load original IO register
   sbrs  valtregradr,6     ;decode read double
      rjmp  sbus_read
   ldi   b,1               ;alternate address for low & high byte
   eor   valtregradr,b
   rjmp  sbus_read

   align4more
;read 5 - read data register without tags
   clr   a
   mov   yl,vdregadr       ;load index to IO register
   andi  yl,0x3f           ;mask valid address range
   addi  yl,0x20           ;adjust IO to common address space
   ld    a,y               ;load original IO register
   sbrs  vdregadr,6        ;decode read double
      rjmp  sbus_read      ;end read single
   sbrc  vdregadr,7
      rjmp  sbus_read      ;end read single
   ldi   b,1               ;alternate address for low & high byte
   eor   vdregadr,b
   rjmp  sbus_read

   align4more
;read 6 - read data register with tags
   mov   yl,vdregadr       ;load index to IO register
   cpi   yl,0xc0           ;decode special non IO register
   brsh  vread_special
   andi  yl,0x3f           ;mask valid address range
   addi  yl,0x20           ;adjust IO to common address space
   ld    a,y               ;load original IO register
   sbrc  vdregadr,7        ;decode read with tags
      rjmp  vread_tags
   sbrs  vdregadr,6        ;decode read double
      rjmp  sbus_read      ;end read single
   ldi   b,1               ;alternate address for low & high byte
   eor   vdregadr,b
   rjmp  sbus_read

   align4more
;read 7 - read data register address or state
   mov   a,vdregadr
   rjmp  sbus_read
      
; vdregadr = 0b11xxxxxx 
vread_special:             ;read with presets
   cpi   yl,spdr+0xc0
   ifeq  vread_spi
      in    a,spdr
      in    b,spcr            ;dummy access
      out   spdr,spipat       ;write dummy pattern
      clr   irqvect           ;end of vector mode
      ldi   sbx,high(sbus_func)
      mov   zh,sbx
      rjmp  sbus_read
   end   vread_spi
   cpi   yl,eedr+0xc0
   ifeq  vread_eep
      lds   yl,eep_adr        ;preload eep address
      lds   yh,eep_adr+1
      out   eearl,yl          ;read a from eep address
      out   eearh,yh
      sbi   eecr,eere
      in    a,eedr
      adiw  y,1               ;increment address
      sts   eep_adr,yl
      sts   eep_adr+1,yh
      clr   yh                ;restore index to IO registers
      clr   irqvect           ;end of vector mode
      ldi   sbx,high(sbus_func)
      mov   zh,sbx
      rjmp  sbus_read
   end   vread_eep
   cpi   yl,0xff           ;read diag data
   ifeq  vdiag_rddata
      lds   yl,diag_adr
      lds   yh,diag_adr+1
      ld    a,y+
      sts   diag_adr+1,yh
      sts   diag_adr,yl
      clr   yh
      rjmp  sbus_read
   end   vdiag_rddata
   ldi   a,0               ;pad null byte
   rjmp  sbus_read

; dregadr = 0b10xxxxxx 
vread_tags:                ;read & set tags
   clr   irqvect           ;end of vector mode
   ldi   sbx,high(sbus_func)
   mov   zh,sbx
   cpi   yl,twdr+0x20
   ifeq  vread_twdr
      in    a,twdr
      in    b,twcr            ;clear twint with ack
      andi  b,(1<<twie)       ;keep interrupt enable
      ori   b,(1<<twint|1<<twea|1<<twen)
      out   twcr,b
   end   vread_twdr
   rjmp  sbus_read

   align 7
   .if (pc & 0xff) != 0x80
      .org pc+0x80
   .endif
   .set aligncheck = pc
;write 0 - write IO register
   mov   yl,vregadr
   addi  yl,0x60           ;adjust to regprot table
   ld    c,y               ;read protected bits
   subi  yl,0x40           ;adjust IO to common address space
   tst   c                 ;any protected bits?
   ifne  vwr0_apb
      ld    b,y               ;load original IO register
      and   b,c               ;mask bits to keep
      com   c
      and   a,c               ;mask bits to change
      or    a,b               ;merge
   end   vwr0_apb
   st    y,a               ;store modified IO register
   mov   zh,sbx
   rjmp  vmain

   align4more
;write 1 - write IO register +1
   mov   yl,vregadr
   addi  yl,0x61           ;adjust to regprot table +1
   ld    c,y               ;read protected bits
   subi  yl,0x40           ;adjust IO to common address space
   tst   c                 ;any protected bits?
   ifne  vwr1_apb
      ld    b,y               ;load original IO register
      and   b,c               ;mask bits to keep
      com   c
      and   a,c               ;mask bits to change
      or    a,b               ;merge
   end   vwr1_apb
   st    y,a               ;store modified IO register
   mov   zh,sbx
   rjmp  vmain

   align4more
;write 2 - write IO register +2
   mov   yl,vregadr
   addi  yl,0x62           ;adjust to regprot table +2
   ld    c,y               ;read protected bits
   subi  yl,0x40           ;adjust IO to common address space
   tst   c                 ;any protected bits?
   ifne  vwr2_apb
vwr4_apb:
      ld    b,y               ;load original IO register
      and   b,c               ;mask bits to keep
      com   c
      and   a,c               ;mask bits to change
      or    a,b               ;merge
   end   vwr2_apb
   st    y,a               ;store modified IO register
   mov   zh,sbx
   rjmp  vmain

   align4more
;write 3 - set IO register address
   andi  a,0x3f            ;mask valid address range
   mov   vregadr,a         ;save to register address
   mov   zh,sbx
   rjmp  vmain

   align4more
;write 4 - write alternate IO register
   sbrc  valtregadr,7      ;decode command
      rjmp  vsetclr_byte
   sbrc  valtregadr,6
      rjmp  vwrite_double
   mov   yl,valtregadr
   addi  yl,0x60           ;adjust to regprot table
   ld    c,y               ;read protected bits
   subi  yl,0x40           ;adjust IO to common address space
   tst   c                 ;any protected bits?
   brne  vwr4_apb
   st    y,a               ;store modified IO register
   mov   zh,sbx
   rjmp  vmain

   align4more
;write 5 - set alternate IO register address
   mov   zh,sbx
   sbrc  a,7   
      rjmp  vport2pin
   sbrs  a,6
      rjmp  vport2pin
   andi  a,0xfe            ;force even address for word access
   mov   valtregadr,a      ;save to register address
   mov   valtregradr,a
   rjmp  vmain

   align4more
;write 6 - data register DR
   sbrs  vdregadr,6        ;decode options
      rjmp  vwrite_dr
   sbrc  vdregadr,7
      rjmp  vwrite_special
   ldi   b,1               ;pre toggle odd/even
   eor   vdregadr,b
   rjmp  vwrite_dr

   align4more
;write 7 - command register
   sbrc  a,7               ;>0x80
      rjmp  vsetclr_portbit
   sbrc  a,6               ;>0x40
      rjmp  vsetclr_regbit
   mov   zh,sbx
   cpi   a,13                 ;if < 13 then connect units
   iflo  vload_unit
      mov   yl,a              ;address = 3 * a + offset
      lsl   yl  
      add   yl,a
      addi  yl,unitmap
      ld    vregadr,y+        ;registers 0-2
      ld    valtregadr,y      ;alternate register 4
      ld    valtregradr,y+    ;alternate read register 4
      ld    vdregadr,y        ;data register 6
      rjmp  vmain
   end   vload_unit
   cpi   a,62
   ifeq  vexit_vectored
      ldi   sbx,high(sbus_func)
      mov   zh,sbx
      clr   irqvect
      rjmp  main
   end   vexit_vectored
   ifsh  vforce_reset
      mov   a,valtregadr
      cpi   a,wdtcr           ;prev. write 5 (wdtcr)
      ifeq  vforce_watchdog
         ldi   a,(1<<wde)
         out   wdtcr,a
         rjmp  pc
      end   vforce_watchdog
vwr7_exit:
      rjmp  vmain
   end   vforce_reset
   mov   b,valtregadr         ;diag safety lock?
   cpi   b,sreg
   brne  vwr7_exit
   cpi   a,60
   ifeq  vdiag_data           ;60 - access diag data
      ldi   b,0xff            ;dummy sreg
      mov   vdregadr,b
      jmp   main
   end   vdiag_data
   ifsh  vdiag_adr            ;61 - store diag address
      ldi   b,0xfd            ;dummy spl
      mov   vdregadr,b
      jmp   main
   end   vdiag_adr
   rjmp  main

;extra options - write 4 (alternate IO register)
vsetclr_byte:              ;set or clear bits in byte (write 4)
   mov   yl,valtregadr
   andi  yl,0x3f           ;adjust to regprot table
   addi  yl,0x60
   ld    c,y               ;read protected bits
   subi  yl,0x40           ;adjust IO to common address space
   ld    b,y               ;load original IO register
   sbrs  valtregadr,6      ;set or clear?
   ifs   vset_byte
      com   c
      and   a,c               ;bits not to set
      or    b,a               ;merge
   else  vset_byte
      com   a                 ;build and mask
      or    a,c               ;bits not to clear
      and   b,a               ;apply bit mask
   end   vset_byte
   st    y,b               ;store modified IO register
   ldi   a,0x40            ;toggle next op: set <-> clear
   eor   valtregadr,a
   mov   zh,sbx
   rjmp  vmain

vwrite_double:             ;alternate between high and low byte
   ldi   b,1               ;pre toggle odd/even
   eor   valtregadr,b
   mov   yl,valtregadr
   addi  yl,0x20           ;adjust to regprot table
   ld    c,y               ;read protected bits
   subi  yl,0x40           ;adjust IO to common address space
   tst   c                 ;any protected bits?
   ifne  vwdbl_apb
      ld    b,y               ;load original IO register
      and   b,c               ;mask bits to keep
      com   c
      and   a,c               ;mask bits to change
      or    a,b               ;merge
   end   vwdbl_apb
   st    y,a               ;store modified IO register
   mov   zh,sbx
   rjmp  vmain

;convert port or ddr to pin on read - write 5

vport2pin:
   mov   valtregadr,a      ;save to register address
   andi  a,0x3f
   cpi   a,pind
   ifsh_and vportrange
   cpi   a,porta+1
   iflo     vportrange
      cpi   a,pinb
      iflo  vportlow
         cpi   a,pinc
         iflo  vport2pind
            ldi   a,pind
         else  vport2pind
            ldi   a,pinc
         end   vport2pind
      else  vportlow
         cpi   a,pina
         iflo  vport2pinb
            ldi   a,pinb
         else  vport2pinb
            ldi   a,pina
         end   vport2pinb
      end   vportlow
   end      vportrange
   mov   valtregradr,a
   rjmp  main

;extra options - write 6 (data register)

vwrite_special:
   mov   b,vdregadr
   cpi   b,spdr+0xc0
   ifeq  vwr_spi
      in    b,spcr            ;dummy access
      out   spdr,a
      mov   spipat,a
      ldi   sbx,high(sbus_func) ;end vectored mode
      mov   zh,sbx
      clr   irqvect
      rjmp  main
   end   vwr_spi
   cpi   b,eedr+0xc0       ;write data and increment shadow address
   ifeq  veep_data
      lds   yl,eep_adr
      lds   yh,eep_adr+1
      out   eearl,yl          ;write a to eep address
      out   eearh,yh
      out   eedr,a
      sbi   eecr,eemwe
      sbi   eecr,eewe
      adiw  y,1               ;increment address
      sts   eep_adr,yl
      sts   eep_adr+1,yh
      clr   yh                ;restore index to IO registers
      ldi   sbx,high(sbus_func) ;end vectored mode
      mov   zh,sbx
      clr   irqvect
      rjmp  main
   end   veep_data
   mov   zh,sbx
   cpi   b,eearh+0xc0         ;set eeprom write address high
   ifeq  veep_adrh
      sts   eep_adr+1,a
      dec   vdregadr
      rjmp  vmain
   end   veep_adrh
   cpi   b,eearl+0xc0         ;set eeprom write address low
   ifeq  veep_adrl
      sts   eep_adr,a
      dec   vdregadr
      rjmp  vmain
   end   veep_adrl
   cpi   b,0xff               ;write diag data
   ifeq  vdiag_wrdata
      lds   yl,diag_adr
      lds   yh,diag_adr+1
      st    y+,a
      sts   diag_adr+1,yh
      sts   diag_adr,yl
      clr   yh
      rjmp  vmain
   end   vdiag_wrdata
   cpi   b,0xfd               ;write diag address high
   ifeq  vdiag_adrhi
      sts   diag_adr+1,a
      inc   vdregadr
      rjmp  vmain
   end   vdiag_adrhi
   cpi   b,0xfe               ;write diag address low
   ifeq  vdiag_adrlo
      sts   diag_adr,a
      inc   vdregadr
   end   vdiag_adrlo
   rjmp  vmain

vwrite_dr:                 ;write data register
   mov   zh,sbx
   mov   yl,vdregadr       ;build index
   andi  yl,0x3f           ;mask options
   addi  yl,0x60           ;adjust to regprot table
   ld    c,y               ;read protected bits
   subi  yl,0x40           ;adjust IO to common address space
   tst   c                 ;any protected bits?
   ifne  vwdr_apb
      ld    b,y               ;load original IO register
      and   b,c               ;mask bits to keep
      com   c
      and   a,c               ;mask bits to change
      or    a,b               ;merge
   end  vwdr_apb
   st    y,a               ;store modified IO register
   sbrs  vdregadr,7        ;with tags?
      rjmp  vmain          ;no tags
   ldi   sbx,high(sbus_func)
   mov   zh,sbx
   clr   irqvect
   cpi   yl,twdr+0x20
   ifeq  vwdr_twi
      in    b,twcr
      andi  b,(1<<twie)       ;keep interrupt enable
      ori   b,(1<<twint|1<<twen)
      out   twcr,b            ;start TWI op
   end   vwdr_twi
   rjmp  main

;bit commands - write 7 (extended register)

;1abbcddd  set or clear a port-bit
;     ddd- pin# 0-7
;    c---- 0=clear, 1=set
;  bb----- port# a-d
; a------- 0=port, 1=ddr
vsetclr_portbit:
   ldi   b,1               ;generate bit position
   sbrc  a,1
   ldi   b,(1<<2)
   sbrc  a,0
   lsl   b
   sbrc  a,2
   swap  b
   ldi   yl,porta+regprot  ;generate IO register address + protect map
   sbrc  a,6
   dec   yl                ;is ddr
   sbrc  a,5
   subi  yl,6
   sbrc  a,4
   subi  yl,3
   ld    c,y               ;read protected bits
   subi  yl,regprot-0x20   ;adjust IO to common address space
   sbrs  a,3               ;set or clear?
   ifs   vset_portbyte
      ld    a,y               ;load original IO register
      com   c
      and   b,c               ;bits not to set
      or    a,b               ;merge
   else  vset_portbyte
      ld    a,y               ;load original IO register
      com   b                 ;build and mask
      or    b,c               ;bits not to clear
      and   a,b               ;apply bit mask
   end   vset_portbyte
   st    y,a               ;store modified IO register
   mov   zh,sbx
   rjmp  vmain

;10abbbbb  set or clear a bit in register according to regbitmap
;   bbbbb- bitmap position 0-31
;  a------ 0=clear, 1=set
vsetclr_regbit:
   mov   zl,a              ;refer to regbitmap
   andi  zl,0x1f
   lsl   zl
   ldi   zh,high(csbitmap)
   ld    yl,z+             ;get register address
   ld    b,z               ;get bit
   cpi   yl,0xff           ;is valid address?
   ifne  vregbitmap_valid
      ld    c,y               ;read protected bits
      subi  yl,regprot-0x20   ;adjust IO to common address space
      sbrs  a,5               ;set or clear?
      ifs   vset_regbyte
         ld    a,y               ;load original IO register
         com   c
         and   b,c               ;bits not to set
         or    a,b               ;merge
      else  vset_regbyte
         ld    a,y               ;load original IO register
         com   b                 ;build and mask
         or    b,c               ;bits not to clear
         and   a,b               ;apply bit mask
      end   vset_regbyte
      st    y,a               ;store modified IO register
   end   vregbitmap_valid
   mov   zh,sbx
   rjmp  vmain
