Modular 6502 SBC with emulated CPU

The minimum system consists of an ATMega16, an SRAM IC and some components to 
provide a clock source and an RS232 interface. A serial EEPROM can be added as
non volatile program storage. Examples in the source code showcase, how 
parallel IO can be added from simple latches to 65xx IO ICs.

The emulation provides:
   some of the ATMega's internal IO capabilities
      RS232
      I²C/TWI
      SPI
      Timers
      Interrupt pins as IRQ and NMI inputs
   proper sequencing of tags for parallel IO devices
      glue logic replaced by emulation IO modules and simple registers
   a debugger & monitor outside of the emulated CPUs code space
      alter and display registers and memory
      program control – start, stop, single step & breakpoint
      trap undefined opcodes or STP instruction
   loading or booting code images to RAM from
      RS232 (monitor console)
      serial I²C or SPI EEPROM

The achievable 6502 equivalent clock speed is approximately 2 MHz at 
16 MHz ATMega clock.

In order to combine the required IO blocks for your own SBC design, some 
AVR assembler knowledge is required. The emulator source code contains extensive
comments about configurable items and the usage of emulated registers in the 
IO page.