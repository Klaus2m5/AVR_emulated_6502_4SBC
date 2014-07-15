100 REM ****************************************************
100 REM
100 REM  2m5.de 6502 emulator interrupt test under EhBasic
100 REM
100 REM ****************************************************
100 REM IO declarations
100 iopage=$bff0
120 acia=iopage+0
125 tc=iopage+1
130 ta=iopage+4
140 td=iopage+5
150 idiag=iopage+12
160 ivect=iopage+13
180 imask=iopage+14
190 iflag=iopage+15
200 REM clear interrupt conditions
200 POKE idiag,0
220 POKE imask,0
230 POKE iflag,$ff
240 IRQ CLEAR:NMI CLEAR
250 REM asign interrupt vectors
250 ON NMI 10000
270 ON IRQ 11000
280 vm=0

900 ?"Force NMI & IRQ Tests (needs diag register) y/n?"
910 GET x$
920 IF x$="n" GOTO 3000
930 IF x$<>"y" GOTO 910

1000 ? "Test 1 - force NMI"
1010 reg=idiag:bit=1:it=0
1020 BITSET reg,bit
1030 GOSUB 20000
1040 IF it=1 GOTO 1100
1050 ?"NMI could not be forced"
1060 STOP
1100 ? "Test 1.1 - force NMI with NMI OFF"
1110 NMI OFF
1120 reg=idiag:bit=1:it=0
1130 BITSET reg,bit
1140 FOR x = 1 TO 1000:NEXT
1150 ? "Test 1.2 - turn NMI ON"
1160 NMI ON
1170 GOSUB 20000
1180 IF it=1 GOTO 2000
1190 ?"NMI could not be forced"
1200 STOP

2000 ? "Test 2 - force IRQ"
2010 reg=idiag:bit=0:it=0
2020 BITSET reg,bit
2030 GOSUB 20000
2040 IF it=1 GOTO 2100
2050 ?"NMI could not be forced"
2060 STOP
2100 ? "Test 2.1 - force IRQ with IRQ OFF"
2110 IRQ OFF
2120 reg=idiag:bit=0:it=0
2130 BITSET reg,bit
2140 FOR x = 1 TO 1000:NEXT
2150 ? "Test 2.2 - turn IRQ ON"
2160 IRQ ON
2170 GOSUB 20000
2180 IF it=1 GOTO 3000
2190 ?"NMI could not be forced"
2200 STOP

3000 ? "Test 3 - enable 10ms tick timer, 10 ticks countdown"
3010 countdown=10
3020 reg=imask:bit=0
3030 BITSET reg,bit
3040 GOSUB 20000
3100 ? "Test 3.1 - tick countdown timer, 100 ticks (1s), 10s countdown"
3110 countdown=10:POKE tc,100
3120 reg=imask:bit=1
3130 BITSET reg,bit
3140 ti=1000:GOSUB 20020

4000 ? "Test 4 - enable TDRE and send a string"
4010 t$="This is sent one character at a time as long as there is a"
4015 t$=t$+" TDRE interrupt"
4020 tx=1:te=LEN(t$)+1
4030 reg=imask:bit=7
4040 BITSET reg,bit
4050 GOSUB 20000

5000 ? "Test 5 - enable RDRF and wait for input, close with <CR>"
5010 t$=""
5030 reg=imask:bit=6
5040 BITSET reg,bit
5050 DO:LOOP WHILE PEEK(reg) AND (1 << bit)
5060 ?t$

6000 bit=8
6010 ?"Test 6 - Timer 1 Interrupts: ";
6020 ?"V=TOV1, A=OCF1A, B=OCF1B, C=ICF1"
6030 ?"Conditions: OCR1A=$4000, OCR1B=$8000, ICR1=$C000"
6040 ?"            TCCR PSR=5, 10 Interrupts each WGM mode"
6050 ?"            TCNT=0 at the beginning of every test"
6060 FOR w=0 TO 15
6070 ? "Test 6.";HEX$(w,1);" - Timer 1 Test, WGM:";BIN$(w,4)
6080 ti=0
6090 POKE ta,5:POKE td,$40:POKE td,0
6100 POKE ta,3:POKE td,$80:POKE td,0
6110 POKE ta,1:POKE td,$c0:POKE td,0
6120 POKE ta,7:POKE td,0:POKE td,1
6130 POKE if,$3c
6140 IF ((w<8) AND (w AND %11)) THEN POKE im,4 ELSE POKE im,$3c
6150 ts=0:POKE tc,100:POKE if,2
6160 POKE ta,8:POKE td,(w*16+5)
6180 DO
6190 IF (PEEK(if) AND 2) = 0 GOTO 6210
6200 INC ts:POKE if,2
6210 LOOP WHILE PEEK(im)
6220 POKE ta,8:POKE td,0
6230 te=PEEK(tc):tf=PEEK(if)
6240 ?ts+((100-te)/100)+((tf AND 2));" seconds"
6300 NEXT w

7000 IF vm>0 GOTO 9980
7010 ?:?"Switch to vector mode and repeat tests 3-6":?
7020 ON IRQ 16000
7030 INC vm
7040 GOTO 3000

9980 ?:?"All tests completed successfully"
9990 END

10000 REM   NMI handler - diag register only
10000 nx=PEEK(idiag)
10020 IF (nx AND 2) GOTO 10100
10030 ? "Unsolicited NMI!"
10040 STOP
10100 ? "NMI OK"
10110 it=1
10120 BITCLR idiag,1
10130 RETNMI

11000 REM   IRQ flag handler
11000 IF bit = 6 GOTO 14000
11010 ix=PEEK(iflag) AND PEEK(imask)
11020 IF ix=0 GOTO 11100
11030 IF (ix AND (1<<bit))=0 GOTO 11300
11040 IF bit = 7 GOTO 13000
11050 IF bit = 0 GOTO 12000
11060 IF bit = 1 GOTO 12500
11070 ?"Unexpected enabled interrupt ";HEX$(ix,2)
11080 STOP
11090 RETIRQ

11100 REM   no match between IRQ flags & mask
11100 IF (PEEK(idiag) AND 1) GOTO 11200
11110 IF bit = 6 GOTO 14000
11120 ? "Unsolicited IRQ! flag:";
11130 ? HEX$(PEEK(if),2);" mask:";HEX$(PEEK(im),2)
11140 STOP
11150 RETIRQ

11200 REM   IRQ forced by diag register
11200 ? "IRQ OK"
11210 it=1
11220 BITCLR reg,bit
11230 RETIRQ

11300 REM   t1 flag 2 index
11300 IF bit <> 8 GOTO 11070
11310 iy=2
11320 IF (ix AND 4) GOTO 15000
11330 INC iy
11340 IF (ix AND 8) GOTO 15000
11350 INC iy
11360 IF (ix AND $10) GOTO 15000
11370 INC iy
11380 IF (ix AND $20) GOTO 15000
11390 GOTO 11070

12000 REM   10ms tick handler
12000 ?countdown;
12010 DEC countdown
12020 POKE if,(1 << bit)
12030 IF countdown > 0 GOTO 12090
12040 BITCLR reg,bit
12050 ?
12090 RETIRQ

12500 REM   100 ticks (1s) countdown handler
12500 ?countdown;
12510 DEC countdown
12520 POKE if,(1 << bit)
12530 IF countdown > 0 GOTO 12590
12540 BITCLR reg,bit
12550 ?
12590 RETIRQ

13000 REM   TDRE handler
13000 ? MID$(t$,tx,1);
13010 INC tx
13020 IF tx<te GOTO 13090
13030 BITCLR reg,bit
13040 ?
13090 RETIRQ

14000 REM   RDRF handler
14000 GET rs$
14010 IF LEN(rs$) GOTO 14050
14020 ?"Unexpected RDRF"
14030 STOP
14040 RETIRQ
14050 ?rs$;
14060 t$=t$+rs$
14070 IF rs$=CHR$(13) THEN BITCLR reg,bit
14080 RETIRQ

14900 REM   t1 interrupt handler
14900 INC iy
14910 INC iy
14920 INC iy
15000 ?MID$(" VBAC",iy,1);
15005 POKE if,(1<<iy)
15010 INC ti
15020 IF ti < 10 GOTO 15050
15030 POKE im,0
15050 RETIRQ

16000 REM   IRQ vector handler
16000 ix=(PEEK(iv)>>1)
16010 iy=2
16020 ON ix GOTO 12000,12500,15000,14920,14910,14900,14000,13000
16030 GOTO 11110

20000 REM   wait for interrupt source to be cleared with timeout
20000 timout=100
20020 DO
20040 timout=timout-1
20055 IF timout > 0 GOTO 20080
20060 IF reg = idiag THEN reg$="diag" ELSE reg$="mask"
20065 ? "Timeout waiting for ";reg$;" register bit";bit;" to be cleared"
20070 STOP
20080 LOOP WHILE PEEK(reg) AND (1 << bit)
20090 RETURN

