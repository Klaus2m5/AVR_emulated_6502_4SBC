
; minimal monitor for EhBASIC 2.22 running under the 2m5.de 6502 emulator
; 25-mar-2014 - save/load to match V0.83 or higher DMA register

; some minor modifications are in the basic program itself

; RAM scan has been removed.
; maximum RAM must be defined here, you can still configure less RAM during at coldstart

; 32k RAM mirrored = 12k EhBASIC code at top + 20k work space at bottom

Ram_base		= $0300	; start of user RAM (set as needed, must be page aligned)
Ram_top		= $5000	; end of user RAM+1 (set as needed, should be page aligned)

; This start can be changed to suit your system

	*=	$D000

	JMP	RES_vec	;startup soft reset

	.INCLUDE "basic.asm"

; IO setup for the 2m5.de 6502 emulator environment

IO_PAGE	= $BFF0		; set I/O area +$f0 (internal registers)

ACIAtx	= IO_PAGE		; ACIA write port
ACIArx	= IO_PAGE		; ACIA read port
EEP_cmd	= IO_PAGE+7		; eep load/save dma command register
EEP_stat	= IO_PAGE+7		; eep load/save dma status (non $ff)
EEP_data	= IO_PAGE+8		; port to set prog#, start & end address
EMU_flag	= IO_PAGE+$f	; bit 0 = 10ms tick
EMU_diag	= IO_PAGE+$c	; bit 7 = force debugger
IR_mask	= IO_PAGE+$e	; interrupt enable mask

; now the code. all this does is set up the vectors and interrupt code
; and wait for the user to select [C]old or [W]arm start.

	*=	$FE00		; pretend this is in a 1/2K ROM

; reset vector points here

RES_vec
	CLD			; clear decimal mode
	LDX	#$FF		; empty stack
	TXS			; set the stack

; set up vectors and interrupt code, copy them to page 2

	LDY	#END_vec-LAB_vec	; set index/count
LAB_stlp
	LDA	LAB_vec-1,Y	; get byte from interrupt code
	STA	VEC_IN-1,Y	; save to RAM
	DEY			; decrement index/count
	BNE	LAB_stlp	; loop if more to do

; now do the signon message, Y = $00 here

LAB_signon
	JSR	prt_msg	; Y=0 - print signon message

LAB_nokey
	JSR	V_INPT	; call scan input device
	BCC	LAB_nokey	; loop if no key

	AND	#$DF		; mask xx0x xxxx, ensure upper case
	CMP	#'W'		; compare with [W]arm start
	BEQ	LAB_dowarm	; branch if [W]arm start

	CMP	#'C'		; compare with [C]old start
	BNE	RES_vec	; loop if not [C]old start

	JMP	LAB_COLD	; do EhBASIC cold start

LAB_dowarm
	JMP	LAB_WARM	; do EhBASIC warm start

; byte out to emulated ACIA

ACIAout
	CMP	#8		; backspace?
	BNE	skip_bs
	STA	ACIAtx	; make erasing backspace
	LDA	#' '
	STA	ACIAtx
	LDA	#8
skip_bs
	STA	ACIAtx	; send byte to ACIA
	RTS

; byte in from emulated ACIA

ACIAin
	LDA	ACIArx	; test data available
	BEQ	LAB_nobyw	; branch if no byte waiting
	CMP 	#127        ; convert delete to backspace
	BNE 	conv_bs2del
	LDA 	#8
conv_bs2del
	CMP 	#27         ; escape?
	BNE 	skip_esc_no
	TXA       		; discard escape sequence
	PHA
	LDX 	#5		; timer loop - 5*10ms
skip_esc_next
	LDA 	#1		; ack last tick
	STA 	EMU_flag
skip_esc_wait  
	LDA 	EMU_flag
	AND 	#1		; next tick
	BEQ 	skip_esc_wait
	DEX
	BNE 	skip_esc_next
skip_esc_discard  
	INX			; any data = X > 1
	LDA 	ACIArx
	BNE 	skip_esc_discard
	CPX 	#1
	BEQ 	skip_esc_esc
	PLA			; was special key - skip
	TAX
	LDA 	#0
	CLC
	RTS
skip_esc_esc		; escape only - send to basic  
	PLA
	TAX
	LDA 	#27

skip_esc_no
	EOR 	#0		; set flags NZ
	SEC			; flag byte received
	RTS

LAB_nobyw
	CLC			; flag no byte received
	RTS
		
; save a program to EEPROM by number

save_eep
	JSR	check_prognum
	LDA	Svarl		; set end address
	STA	EEP_data
	LDA	Svarh
	STA	EEP_data
	LDA	#6		; save command
	STA	EEP_cmd
	LDA	EEP_stat
	CMP	#$16
	BEQ	save_fail
	JMP	LAB_GBYT	; continue 

; load a program from EEPROM by number

load_eep
	JSR	check_prognum
	LDA	#7		; load command
	STA	EEP_cmd
	LDA	EEP_stat	; test for ack
	CMP	#$17		; load complete?
	BNE	load_fail
	LDA	EEP_data	; set end_address
	STA	Svarl
	LDA	EEP_data
	STA	Svarh
	LDA	#<LAB_RMSG	; "READY"
	LDY	#>LAB_RMSG
	JSR	LAB_18C3
	JMP	LAB_1319	; rebuild line pointers	
load_fail
save_fail
	CMP	#$ef		; program not found
	BNE	load_ef
	LDY	#msg_ef-msg_pool
	JSR	prt_msg	
	JMP	LAB_GBYT	; continue
load_ef
	CMP	#$ee		; EEPROM full
	BNE	save_ee
	LDY	#msg_ee-msg_pool
	JSR	prt_msg	
	JMP	LAB_GBYT	; continue
save_ee	
	CMP	#$ed		; EEPROM unaccessible
	BNE	load_ed
	LDY	#msg_ed-msg_pool
	JSR	prt_msg	
	JMP	LAB_GBYT	; continue
load_ed	
	CMP	#$ea		; program incompatible (Intel Hex)
	BNE	load_ea
	LDY	#msg_ea-msg_pool
	JSR	prt_msg	
	JMP	LAB_GBYT	; continue
load_ea
	CMP	#$e9		; program damaged
	BNE	load_e9
	LDY	#msg_e9-msg_pool
	JSR	prt_msg	
	JMP	LAB_1463	; perform NEW
load_e9			; unspecified internal error
	LDY	#msg_ff-msg_pool
	JSR	prt_msg	
	JMP	LAB_GBYT	; continue

; check program number for validity (0-0xfe)
; 0xff drops to debugger to allow EEPROM utility commands

check_prognum
	JSR	LAB_EVNM	; get program number to load
	JSR	LAB_F2FX	; convert to integer
	LDA	Itemph	; filenumber range 0-0xfe
	BEQ	prog_num_ok
	PLA			; discard return address
	PLA
	JMP	LAB_FCER	; msg "function call error"
prog_num_ok	
	LDA	Itempl
	CMP	#$ff		; special number
	BNE	prog_set_start
	LDA	#$80		; drop to debugger
	STA	EMU_diag
	PLA			; discard return address
	PLA
	JMP	LAB_GBYT	; continue
prog_set_start
	LDX	#0		; set eep command
	STX	EEP_cmd
	STA	EEP_data	; set program number
	LDA	Smeml		; set load address
	STA	EEP_data
	LDA	Smemh
	STA	EEP_data
	RTS

; print load/save errormessage

prt_msg_loop
	JSR	V_OUTP
	INY
prt_msg
	LDA	msg_pool,y
	BNE	prt_msg_loop
	RTS
	
; vector tables

LAB_vec
	.word	ACIAin		; byte in from simulated ACIA
	.word	ACIAout		; byte out to simulated ACIA
	.word	load_eep		; load program from EEPROM
	.word	save_eep		; save program to EEPROM
	.word	IRQ_CODE		; vector to handle IRQ 
	.word	NMI_CODE		; vector to handle IRQ 
END_vec

; EhBASIC IRQ support

IRQ_vec
;	JMP	(IRQ_indirect)	; jump through a vector in RAM
					; uncomment if needed
IRQ_CODE
	STA	irq_a_reg		; save A
	LDA	IrqBase		; get the IRQ flag byte
	BNE	IRQ_IS_SET		; Basic ready to handle?
	STA	IR_mask		; no, silence internal interrupts 
IRQ_IS_SET
	ORA	#$20			; 00100000 set IRQ pending
	STA	IrqBase		; save the new IRQ flag byte
	PLA				; pop saved flags
	ORA	#4			; set interrupt disable on stack
	PHA				; save flags again for RTI
	LDA	irq_a_reg		; restore A
	RTI				; return disabled!
	
; EhBASIC NMI support

NMI_vec
;	JMP	(NMI_indirect)	; jump through a vector in RAM
					; uncomment if needed
NMI_CODE
	PHA				; save A
	LDA	NmiBase		; get the NMI flag byte
	ORA	#$20			; 00100000 set NMI pending
	STA	NmiBase		; save the new NMI flag byte
	PLA				; restore A
	RTI

msg_pool
LAB_mess	.byte	$0D,$0A,"6502 EhBASIC [C]old/[W]arm ?",$00
msg_ef	.byte	"program not found",0
msg_ee	.byte	"EEPROM full",0
msg_ed	.byte	"EEPROM unaccessible",0
msg_ea	.byte	"program incompatible",0
msg_e9	.byte	"program damaged",0
msg_ff	.byte	"min_mon error",0

; system vectors

	*=	$FFFA

	.word	NMI_vec		; NMI vector
	.word	RES_vec		; RESET vector
	.word	IRQ_vec		; IRQ vector

