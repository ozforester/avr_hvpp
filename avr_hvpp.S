; Copyright (c) 2021 ozforester. All rights reserved.
; Use of this source code is goverened by a MIT license
; that can be found in the LICENSE file.

; ATmega8 @4MHz internal osc. (H:D9, L:23)
; JHD162A HD44780 via twi extension PCF8574AT
; Four buttons pad voltage divider connected to adc port.
; Two DIP28 sockets. One for programmer and one for target chip
; Schematics and PCB sources: https://github.com/ozforester/avr_hvpp/hardware
; It utilizes AVR HVPP protocol to change fuses or erase flash and eeprom
; lcd twi address autodetected each time this programmer switched on
; once after erasing the programmer keys calibration starts and remember levels
; debug and error state blinks with B6 (bottom led of Q1 base network)

/*

 SIGNAL   PROGARMMER  		TARGET
 OE    -  PC2 // 25  	- 	4
 WR    -  PC1 // 24  	- 	5
 BS1   -  PC0 // 23  	-	6
 XA0   -  PB4 // 18  	- 	11
 XA1   -  PB3 // 17  	- 	12
 PAGEL -  PB2 // 16  	- 	13
 BS2   -  PD1 // 3   	- 	25
 XTAL1 -  PB5 // 19   	- 	9
 RST   -  PD0 // 2 (Q2 base) - 	1
 VCC   -  PB6 // 9 (Q1 base) - 	2
 DATA7 -  PD2 // 4  	- 	24
 DATA6 -  PD3 // 5  	- 	23
 DATA5 -  PB7 // 10 	- 	19
 DATA4 -  PD5 // 11 	- 	18
 DATA3 -  PD6 // 12 	- 	17
 DATA2 -  PD7 // 13 	- 	16
 DATA1 -  PB0 // 14 	- 	15
 DATA0 -  PB1 // 15 	-	14
 RDY   -  PD4 //  6 	-	3 (routed but not used yet)

 pcf8574 <-> HD44780
 P0 - rs
 P1 - rw
 P2 - e (cs)
 P3 - backlight
 P4 - DB4
 P5 - DB5
 P6 - DB6
 P7 - DB7

 hvpp flags
 7 rdy/bsy
 6 oe
 5 wr
 4 bs1
 3 bs2
 2 xa0
 1 xa1
 0 pagel

*/

#include <avr/io.h>
#undef  __SFR_OFFSET
#define __SFR_OFFSET 0
#define  START       0x08
#define  REP_START   0x10
#define  SLAW_ACK    0x18
#define  DATAW_ACK   0x28
#define  SLAR_ACK    0x40
#define  DATAR_ACK   0x50
#define  DATAR_NACK  0x58
#define rs 0
#define rw 1
#define  e 2
#define bl 3
#define p4 4
#define p5 5
#define p6 6
#define p7 7

; registers

accum   = 0	; r0
row	= 4	; cursor row
column	= 5	; cursor pos
hvppc   = 6	; hvpp command byte
hvppah  = 7	; hvpp address high
hvppal  = 8	; hvpp address low
level   = 9	; menu level
oldkey	= 10	; previous button code
item    = 11	; menu item
ent     = 12	; adc value for enter
rgt     = 13	; adc value for right
lft     = 14	; adc value for left
esc     = 15	; adc value for escape
temp    = 16	; good worker (.set)
temp2   = 17	; and its helper
tcntr   = 18	; time counter
char    = 19   	; message
address = 20    ; twi address
data    = 21    ; twi data
len     = 22	; message length
sample  = 23	; adc sample
key	= 24	; button code
flags   = 25	; hvpp flags
; 31:26 	; Z, Y, X

.section .eeprom

label:  .byte 0xFF
enter:  .byte 0xFF
right:  .byte 0xFF
left:   .byte 0xFF
escape: .byte 0xFF

msg0:   .asciz "Hvpp start"
msg1:   .asciz "SIGNATURE >>"
msg2:   .asciz "<< ERASE >>"
msg3:   .asciz "<< FUSES"
msg4:   .asciz "LF  HF  EF  LB"
msg5:   .asciz "="
msg6:   .asciz "                "
msg7:   .asciz "WRITING.."
msg8:   .asciz "ERASING.."
msg9:   .asciz "ozforester"
msg10:	.asciz "Press Ent"
msg11:	.asciz "Press >>"
msg12:	.asciz "Press <<"
msg13:	.asciz "Press Esc"
msg14:	.asciz "Twi address"
msg15:  .asciz "OK"
msg16:	.asciz "LCD"
msg17:	.asciz "SRAM"
msg18:	.asciz "CHEAP HEALER"
msg19:	.asciz "BY CHIP KILLER"
msg20:	.asciz "--------------"
msg21:	.asciz "BY OZFORESTER "

sign1:  .byte 0x1e, 0x93, 0x07
chip1: .asciz "atmega8"
sign2:  .byte 0x1e, 0x93, 0x89
chip2: .asciz "atmega8"
sign3:  .byte 0x1e, 0x95, 0x0f
chip3: .asciz "mega328p"
sign4:  .byte 0x1e, 0x95, 0x14
chip4: .asciz "mega328"

.section .bss

signature: .byte 0,0,0		; chip signature
chipname:  .asciz "-unknown-"	; chip name
low_fuse:  .byte 0		;
high_fuse: .byte 0              ;
ext_fuse:  .byte 0              ;
lock_byte: .byte 0              ;


.section .text

.global main			;
.global TIMER0_OVF_vect		;
.global __vector_default        ;

; -------------------------  M A I N  -------------------------

main:				;
        rcall timer_init        ;
        rcall delay_20ms        ;
        rcall twi_init          ;
        rcall twi_scan          ;
        cpi address, 0          ;
        brne main1              ;
        rcall error             ;
        main1:                  ;
        rcall pcf_high          ;
        rcall lcd_init          ;
        rcall adc_init          ;
	rcall keys_init		;
	rcall hvpp_init		;
	;rcall hvpp_header	;
	rcall hvpp_menu		; primary loop
        rcall error		; j-tag (:
	loop: rjmp loop		;
ret				;

; --------------  H V P P  M E N U  --------------

hvpp_header:			;
        rcall lcd_clr           ;
        ldi len, msg16          ; lcd address
        rcall print_msg         ;
	ldi temp, 1		;
	mov row, temp		;
	clr temp		;
	mov column, temp	;
	rcall delay_1s		;
	rcall lcd_clr		;
        ldi len, msg17          ; free sram
        rcall print_msg         ;
        rcall lcd_clr           ;
        ldi len, msg18          ; cheap healer
        rcall print_msg         ;
        ldi len, msg19          ; chip killer
        rcall print_msg         ;
        ldi len, msg20          ; -------
        rcall print_msg         ;
        ldi len, msg21          ; ozforester
        rcall print_msg         ;
ret				;

hvpp_menu:
        ldi temp, 0xff          ;
        mov oldkey, temp        ;
        ldi temp, 0             ;
        mov level, temp         ; level = 0
        mov item, temp          ; item = 0
        rcall lcd_clr           ;
        ldi len, msg1           ; initial "signature" string
        rcall print_msg         ;
	menu_loop:		;
	rcall delay_100ms	; loop delay
	rcall adc_read		; key sample <-8bit
	rcall key_code		; key code 0=open,1=esc,2=left,3=right,4=enter
	;rcall print_adc	; sample debug
	;rcall print_key_code	; key debug
	;rcall print_pos	; debug
        cp key, oldkey         	; same key
        breq menu_loop         	;
        mov oldkey, key        	; new key
	; --- MENU ACTIONS ---
	cpi key, 1		;
	brne m1			;
	rjmp pressed_esc	;
	m1:			;
	cpi key, 2		;
	brne m2			;
	rjmp pressed_left	;
	m2:			;
        cpi key, 3              ;
	brne m3			;
        rjmp pressed_right      ;
	m3:			;
        cpi key, 4              ;
	brne m4			;
        rjmp pressed_enter      ;
	m4:			;
	rcall menu_loop		; !(1,2,3,4)
	pressed_esc:		;
	ldi temp, 1		;
	cp level, temp		;
	brlo esc1		;
	mov temp, level		;
	dec temp		;
	mov level, temp  	;
	esc1:			;
        rcall menu_action       ;
	pressed_left:		;
	ldi temp, 1		;
	cp item, temp		;
	brlo lf1		;
        mov temp, item          ;
        dec temp                ;
        mov item, temp          ;
	lf1:			;
        rcall menu_action       ;
        pressed_right:		;
        ;ldi temp, 2             ; пока убрал ограничение для всех уровней
        ;cp item, temp           ;
        ;brsh rg1                ;
        mov temp, item          ;
        inc temp                ;
        mov item, temp          ;
	rg1:			;
        rcall menu_action       ;
	pressed_enter:		;
        ldi temp, 2             ;
        cp level, temp          ;
        brsh ent1               ;
        mov temp, level         ;
        inc temp                ;
        mov level, temp         ;
	ent1:			;
        rcall menu_action       ;
	; --- MENU MESSAGES ---
	menu_action:		; messages w/ accordance to states
	cpi key, 0		;
	brne actions		;
	rcall error		; wrong state
	actions:		;
	ldi temp, 0		;
	cpse level, temp	; level 0 (choose command)
	rjmp am1		;
	rjmp level0		;
	am1:			;
	inc temp		;
	cpse level, temp	; level 1 (get signature, erase flash, read fuses)
	rjmp am2		;
	rjmp level1		;
	am2:			;
        inc temp                ;
        cpse level, temp        ; level 2 (choose fuse)
        rjmp am3                ;
        rjmp level2             ;
	am3:			;
       inc temp                ;
        cpse level, temp        ; level 3 (change fuse)
        rjmp am4                ;
        rjmp level3             ;
        am4:                    ;



	rcall error		; got wrong level
	level0:			;
	ldi temp, 0		;
	cp item, temp		;
	breq l0i0		;
	inc temp		;
	cp item, temp		;
	breq l0i1		;
        inc temp                ;
        cp item, temp           ;
        breq l0i2               ;
	rcall error		; impossible
	l0i0:			;
        rcall lcd_clr  	 	;
        ldi len, msg1  	 	; "signature"
        rcall print_msg	 	;
        rjmp menu_loop          ;
	l0i1:			;
        rcall lcd_clr           ;
        ldi len, msg2           ; "erase"
        rcall print_msg         ;
        rjmp menu_loop          ;
	l0i2:
        rcall lcd_clr  	 	;
        ldi len, msg3  	 	; "fuses"
        rcall print_msg	 	;
        rjmp menu_loop 	 	;
	level1:				;
        ldi temp, 0             	;
        cp item, temp           	;
        breq l1i0               	;
        ldi temp, 1                     ;
        cp item, temp                   ;
        breq l1i1                       ;
        ldi temp, 2    			;
        cp item, temp  			;
        breq l1i2	   		;
	rcall error			;
	l1i0:				;
        rcall lcd_clr           	;
        ldi len, msg1           	; "signature"
        rcall print_msg         	;
	rcall hvpp_read_signature	;
	rcall print_signature		;
        rjmp menu_loop          	;
        l1i1:                   ;
	clr temp		;
	mov column, temp	;
	inc temp		;
	mov row, temp		;
        rcall lcd_cur           ; 1,0
        ldi len, msg8           ; "erased"
        rcall print_msg         ;
	rcall delay_500ms	;
	rcall lcd_clr		;
        ldi len, msg2           ;
        rcall print_msg         ;
	mov temp, level		;
	dec temp		;
	mov level, temp		;
        rjmp menu_loop          ;
	l1i2:			;
        rcall lcd_clr   	;
        ldi len, msg4   	; "LF---LB"
        rcall print_msg 	;
	rcall hvpp_read_fuses	;
	rcall print_fuses	;
        rjmp menu_loop  	;
	level2:			;
        ;ldi temp, 2     	;
        ;cp item, temp  	;
        ;breq l2i2	   	;
        ;rjmp menu_loop  	;
	l2i2:			;
        rcall lcd_clr   	;
	clr temp		;
	mov row, temp		; row 0
	mov column, item	; pos = item
	rcall lcd_cur		; 0, pos
        ldi len, msg5		; "="
        rcall print_msg 	;
	rcall print_fuses	;
        rjmp menu_loop  	;

        level3:                 ;
	mov temp, item		;
	cpi temp, 0		;
	breq l3a		;
        cpi temp, 1             ;
        breq l3a                ;
        cpi temp, 4             ;
        breq l3b                ;
        cpi temp, 5             ;
        breq l3b                ;
        cpi temp, 8             ;
        breq l3c                ;
        cpi temp, 9             ;
        breq l3c                ;
        cpi temp, 12            ;
        breq l3d                ;
        cpi temp, 13            ;
        breq l3d                ;
        l3end:                  ;
        ldi temp, 2             ;
        mov item, temp          ;
        mov level, temp         ;
        rjmp menu_loop          ;

	l3a:			;
        rjmp add_lf             ;
        l3b:                    ;
        rjmp add_hf             ;
	l3c:			;
        rjmp add_ef             ;
	l3d:			;
        rjmp add_lb             ;

	add_lf:			;
	ldi temp, 0		;
	breq alf		;
	lds temp, low_fuse	; h-nibble
	add temp, 0x10		;
	sts low_fuse, temp	;
        rjmp l3end		;
	alf:			; l-nibble
	lds temp, low_fuse	;
	add temp, 0x01		;
        sts low_fuse, temp      ;
        rjmp l3end		;

	add_hf:			;
        rjmp l3end	        ;

	add_ef:			;
        rjmp l3end              ;
	add_lb:			;

        rjmp l3end	        ;

ret

; ---------------  H V P P   R O U T I N E S  -------------

hvpp_get_name:
	rcall hvpp_read_signature	; table
	lds char, signature     	;
	cpse char, 0x1e			; atmel
	rjmp hgne			;
        lds char, signature+1   	;
        cpi char, 0x93			; 8k
	breq hgn8 			;
	cpi char, 0x95			;
	breq hgn32			; 32k
	rjmp hgne			;
	hgn8:				;
	lds char, signature+2		;
	cpi char, 0x07			; atmega8
	rjmp hgne			;
        hgn32:                          ;
        lds char, signature+2           ;
        cpi char, 0x0f                  ; atmega328
        rjmp hgne                       ;
	hgne:				;
ret

hvpp_on:			;
	sbi PORTC, PC2		; OE HIGH no actions
	sbi PORTC, PC1		; WR HIGH
	mode1:			; m8 NORMAL (XTAL)
	sbi PORTB, PB6		; VCC - up target chip
        rcall delay_100us               ;
	rcall delay_100us		;
	sbi PORTD, PD0		; RST - 0V
	ldi temp, 7		;
	hvo1:			;
	rcall hvpp_toggle_xtal	; toggle xtal
	dec temp		; 7 times
	brbc 1, hvo1		;
	cbi PORTB, PB2		; PAGEL
	cbi PORTB, PB3		; XA1   prog
	cbi PORTB, PB4		; XA0   enable
	cbi PORTC, PC0		; BS1
	rcall delay_100us	;
	cbi PORTD, PD0		; RST - 12V
	rjmp hvoe		;
	mode2:			; mega8 w RSTDISBL etc
	cbi PORTB, PB2		; PAGEL
	cbi PORTB, PB3		; XA1
	cbi PORTB, PB4		; XA0
	cbi PORTC, PC0		; BS1
	sbi PORTB, PB6		; VCC
	cbi PORTD, PD0		; RST
	rcall delay_100us	;
	rjmp hvoe		;
	mode3:			; m328 NORMAL
	cbi PORTB, PB6		; VCC
	sbi PORTD, PD0		; RST
	cbi PORTB, PB2		; PAGEL
	cbi PORTB, PB3		; XA1
	cbi PORTB, PB4		; XA0
	cbi PORTC, PC0		; BS1
	sbi PORTB, PB6		; VCC
	rcall delay_100us	;
	cbi PORTD, PD0		; RST
	rcall delay_1ms		; > 300 us
	hvoe:			;
ret				;

hvpp_chip_erase:			;
	rcall hvpp_on			;
	ldi data, 0b10000000		; earse chip
	rcall hvpp_load_command		;
	rcall delay_1ms			;
	cbi PORTC, PC1			; WR nagative pulse
	sbi PORTC, PC1			; WR
	rcall delay_20ms		; waits for RDY
	rcall hvpp_off			;
ret					;

hvpp_write_fuses:			; write fuses from sram to chip
	rcall hvpp_on			;
	; low fuse
	lds data, low_fuse		;
	ldi temp, 0			;
	ldi temp2, 0			;
	rcall hvpp_write_fuse		;  fuse, 0, 0 ); // write low fuse
        ; high fuse
        lds data, high_fuse             ;
        ldi temp, 1                     ;
        ldi temp2, 0                    ;
        rcall hvpp_write_fuse           ;  fuse, 1, 0 ); // write high fuse
	; ext fuse
        lds data, ext_fuse              ;
        ldi temp, 0                     ;
        ldi temp2, 1                    ;
        rcall hvpp_write_fuse           ;  fuse, 0, 1 ); // write ext fuse
	; lock byte
	lds data, lock_byte		;
	rcall hvpp_write_lock		;
	rcall hvpp_off			;
ret					;

hvpp_write_fuse:			;
	ldi data, 0b01000000		;
	rcall hvpp_load_command		; write fuse
	ldi data, 0xff			; debug
	rcall hvpp_load_data_low_byte	;  bs1 0,  bs2 0 - sets lf
	nop				;
	cpi temp, 0			; BS1
	breq hwf1			;
        sbi PORTC, PC0                  ;
	rjmp hwf2			;
	hwf1:				;
	cbi PORTC, PC0			;
	hwf2:				;
        cpi temp2, 0                    ; BS2
        breq hwf3                       ;
        sbi PORTD, PD1			;
	rjmp hwf4 	                ;
        hwf3:                           ;
        cbi PORTD, PD1                  ;
	hwf4:				;
	cbi PORTC, PC1			; WR
	nop				;
	nop				;
	sbi PORTC, PC1			; WR
	rcall delay_20ms		; waits RDY
	cbi PORTC, PC0			; BS1
	cbi PORTD, PD1			; BS2
ret					;

hvpp_write_lock:
	ldi data, 0b00100000		; write lock byte
	rcall hvpp_load_command		;
	ldi data, 0xff			; debug
	rcall hvpp_load_data_low_byte	;
	cbi PORTC, PC1			; WR
	sbi PORTC, PC1			; WR
	rcall delay_20ms		; waits RDY
ret					;

hvpp_read_fuses:		; read chip and write to sram
	rcall hvpp_on		;
	ldi data, 0b00000100	; read fuses
	rcall hvpp_load_command	;
	; low fuse		;
	cbi PORTC, PC2		; OE
	cbi PORTD, PD1		; BS2
	cbi PORTC, PC0		; BS1
	rcall hvpp_read_data	;
	sts low_fuse, data	;
	; high fuse		;
	cbi PORTC, PC2		; OE
	sbi PORTD, PD1		; BS2
	sbi PORTC, PC0		; BS1
	rcall hvpp_read_data	;
	sts high_fuse, data	;
	; ext fuse (cond.)	;
	;  fuse byte or 0xff
	cbi PORTC, PC2		; OE
	sbi PORTD, PD1		; BS2
	cbi PORTC, PC0		; BS1
	rcall hvpp_read_data	;
	sts ext_fuse, data	;
	; lock byte		;
	cbi PORTC, PC2		; OE
	cbi PORTD, PD1		; BS2
	sbi PORTC, PC0		; BS1
	rcall hvpp_read_data	;
	sts lock_byte, data	;
	sbi PORTC, PC2		; OE
	rcall hvpp_off		;
ret				;

hvpp_read_signature:
	rcall hvpp_on			;
	ldi temp, 0			;
	mov hvppal, temp		;
	rcall hvpp_read_signature_byte	; // Read byte 0
	sts signature, data		; store to sram
	ldi temp, 1			;
        mov hvppal, temp                ;
        rcall hvpp_read_signature_byte  ; // Read byte 1
        sts signature+1, data           ; store to sram
        ldi temp, 2                     ;
	mov hvppal, temp		;
        rcall hvpp_read_signature_byte  ; // Read byte 2
        sts signature+2, data           ; store to sram
	rcall  hvpp_off			;
ret

hvpp_read_signature_byte:			;
	ldi data, 0b00001000			; command 0x08
	rcall hvpp_load_command			; data ->
	mov data, hvppal			; address 0x00, 0x01, 0x02
	rcall hvpp_load_address_low_byte	; data ->
	cbi PORTC, PC2				; OE low
	cbi PORTC, PC0				; BS1 low
	rcall hvpp_read_data			; data <-
	sbi PORTC, PC2				; OE high
ret						;

hvpp_read_data: 		;
	rcall hvpp_bus_in	;
	in temp,  PIND		;
	in temp2, PINB		;
        clr data                ;
	sbrc  temp, PD2		;
	sbr  data, 1<<7		;
	sbrc  temp, PD3		;
	sbr  data, 1<<6		;
	sbrc temp2, PB7		;
        sbr  data, 1<<5         ;
	sbrc  temp, PD5		;
        sbr  data, 1<<4         ;
	sbrc  temp, PD6		;
        sbr  data, 1<<3         ;
	sbrc  temp, PD7		;
        sbr  data, 1<<2         ;
	sbrc temp2, PB0		;
        sbr  data, 1<<1         ;
	sbrc temp2, PB1		;
        sbr  data, 1<<0         ;
ret

hvpp_toggle_xtal:	;
	rcall delay_1ms	;
	sbi PORTB, PB5	; XTAL1
	rcall delay_1ms	;
	cbi PORTB, PB5	; XTAL1
ret			;

hvpp_toggle_pagel:
	rcall delay_1ms	; >150 ns
	sbi PORTB, PB2	; PAGEL
	rcall delay_1ms	;
	cbi PORTB, PB2	;
ret

hvpp_load_command: 		; set controls
	sbi PORTB, PB3		; XA1 1
	cbi PORTB, PB4		; XA0 0
	cbi PORTC, PC0		; BS1 0
	rcall hvpp_set_data	;
ret				;

hvpp_load_address_low_byte:	;
	cbi PORTB, PB3		; XA1 0
	cbi PORTB, PB4		; XA0 0
	cbi PORTC, PC0		; BS1 0
	rcall hvpp_set_data	;
ret				;

hvpp_load_address_high_byte:	;
	cbi PORTB, PB3		; XA1 0
	cbi PORTB, PB4		; XA0 0
	sbi PORTC, PC0		; BS1 1
	rcall hvpp_set_data	;
ret				;

hvpp_load_data_low_byte:	;
	cbi PORTB, PB3		; XA1 0
	sbi PORTB, PB4		; XA0 1
        cbi PORTC, PC0          ; BS1 0
	rcall hvpp_set_data	;
ret				;

hvpp_load_data_high_byte:	;
	cbi PORTB, PB3		; XA1 0
	sbi PORTB, PB4		; XA0 1
        sbi PORTC, PC0          ; BS1 1
	rcall hvpp_set_data	;
ret				;

hvpp_set_data:			; byte among pins
	rcall hvpp_bus_out	; also clears all bits
	sbrc data, 0		;
	sbi PORTB, PB1		; and set if app
	sbrc data, 1		;
	sbi PORTB, PB0		;
	sbrc data, 2		;
	sbi PORTD, PD7		;
	sbrc data, 3		;
	sbi PORTD, PD6		;
	sbrc data, 4		;
	sbi PORTD, PD5		;
	sbrc data, 5		;
	sbi PORTB, PB7		;
	sbrc data, 6		;
	sbi PORTD, PD3		;
	sbrc data, 7		;
	sbi PORTD, PD2		;
	rcall hvpp_toggle_xtal	; latch
ret				;

hvpp_off:
	rcall delay_1ms		;
	rcall hvpp_bus_in	; data bus off
	rcall hvpp_ctrl		; control pins off
	sbi PORTD, PD0		; RST
	cbi PORTB, PB6		; VCC
ret				;

hvpp_bus_in:		;
	cbi DDRB, PB1	; DATA0
	cbi DDRB, PB0	; DATA1
	cbi DDRD, PD7	; DATA2
	cbi DDRD, PD6	; DATA3
	cbi DDRD, PD5	; DATA4
	cbi DDRB, PB7	; DATA5
	cbi DDRD, PD3	; DATA6
	cbi DDRD, PD2	; DATA7
	cbi PORTB, PB1	;
	cbi PORTB, PB0	;
	cbi PORTD, PD7	;
	cbi PORTD, PD6	;
	cbi PORTD, PD5	;
	cbi PORTB, PB7	;
	cbi PORTD, PD3	;
	cbi PORTD, PD2	;
ret			;

hvpp_bus_out:		;
	sbi DDRB, PB1	; DATA0
	sbi DDRB, PB0	; DATA1
	sbi DDRD, PD7	; DATA2
	sbi DDRD, PD6	; DATA3
	sbi DDRD, PD5	; DATA4
	sbi DDRB, PB7	; DATA5
	sbi DDRD, PD3	; DATA6
	sbi DDRD, PD2	; DATA7
	cbi PORTB, PB1	;
	cbi PORTB, PB0	;
	cbi PORTD, PD7	;
	cbi PORTD, PD6	;
	cbi PORTD, PD5	;
	cbi PORTB, PB7	;
	cbi PORTD, PD3	;
	cbi PORTD, PD2	;
ret			;

hvpp_ctrl:		;
	cbi PORTC, PC2	; OE
	cbi PORTC, PC1	; WR
	cbi PORTC, PC0	; BS1
	cbi PORTB, PB4	; XA0
	cbi PORTB, PB3	; XA1
	cbi PORTB, PB2	; PAGEL
	cbi PORTD, PD1	; BS2
	cbi PORTB, PB5	; XTAL1
	sbi PORTD, PD0	; RST  opens bjt and drops down 12V
	cbi PORTB, PB6	; VCC closes bjt and mosfet (target off)
	cbi PORTD, PD4	; target RDY/BSY 0-busy, 1-ready
ret			;

hvpp_init:			;
	sbi DDRC, PC2		; OE
	sbi DDRC, PC1		; WR
	sbi DDRC, PC0		; BS1
	sbi DDRB, PB4		; XA0
	sbi DDRB, PB3		; XA1
	sbi DDRB, PB2		; PAGEL
	sbi DDRD, PD1		; BS2
	sbi DDRB, PB5		; XTAL1
	sbi DDRD, PD0		; RST
	sbi DDRB, PB6		; VCC
	cbi DDRD, PD4		; RDY reply from target
	rcall hvpp_bus_in	;
	rcall hvpp_ctrl		;
ret				;

; -------------- K E Y S   S U B R O U T I N E S  ---------------

keys_init:			;
	ldi len, label		;
	rcall eeprom_read	;
        ldi temp, 0x55          ;
	cp char, temp		;
	breq kii		; calibrated?
	rcall key_calibrate	;
	rjmp kir		;
	kii:			;
	ldi len, enter		;
	rcall eeprom_read	;
	mov ent, char		;
        ldi len, right          ;
        rcall eeprom_read       ;
        mov rgt, char		;
        ldi len, left           ;
        rcall eeprom_read       ;
        mov lft, char		;
        ldi len, escape         ;
        rcall eeprom_read       ;
        mov esc, char		;
	kir:			;
ret				;

key_calibrate:
	rcall adc_read		; open
        subi sample, 10         ;
        mov key, sample         ;
	rcall lcd_clr		; enter
	ldi len, msg10		;
	rcall print_msg		;
	wait_ent:		;
	rcall adc_read		;
	cp sample, key		;
	brlo kc1		;
        rcall delay_20ms        ;
	rjmp wait_ent		;
	kc1:			;
	rcall print_ok		;
	mov ent, sample		;
	rcall delay_20ms	;
        subi sample, 10         ;
        mov key, sample         ;
        rcall lcd_clr           ; right
        ldi len, msg11          ;
        rcall print_msg         ;
        wait_right:             ;
        rcall adc_read          ;
        cp sample, key          ;
        brlo kc2                ;
        rcall delay_20ms        ;
        rjmp wait_right         ;
        kc2:                    ;
        rcall print_ok          ;
	mov rgt, sample		;
        rcall delay_20ms	;
        subi sample, 10         ;
        mov key, sample         ;
        rcall lcd_clr           ; left
        ldi len, msg12          ;
        rcall print_msg         ;
        wait_left:              ;
        rcall adc_read          ;
        cp sample, key          ;
        brlo kc3                ;
        rcall delay_20ms        ;
        rjmp wait_left          ;
        kc3:                    ;
        rcall print_ok          ;
	mov lft, sample		;
        rcall delay_20ms	;
        subi sample, 10         ;
        mov key, sample         ;
        rcall lcd_clr           ; escape
        ldi len, msg13          ;
        rcall print_msg         ;
        wait_escape:            ;
        rcall adc_read          ;
        cp sample, key          ;
        brlo kc4                ;
        rcall delay_20ms        ;
        rjmp wait_escape        ;
        kc4:                    ;
        rcall print_ok          ;
	mov esc, sample		;
        rcall delay_20ms	;
	nop			;
        cp rgt, ent             ; ent > rgt > lft > esc
        brlo kc6                ;
        rcall error             ;
        kc6:                    ;
        cp lft, rgt             ;
        brlo kc7                ;
        rcall error             ;
        kc7:                    ;
        cp esc, lft             ;
        brlo kc8                ;
        rcall error             ;
        kc8:                    ;
	nop			; len -> eeprom address, char -> eeprom data
        ldi len, enter          ;
        mov char, ent           ;
        rcall eeprom_write      ;
        ldi len, right          ;
        mov char, rgt           ;
        rcall eeprom_write      ;
        ldi len, left           ;
        mov char, lft           ;
        rcall eeprom_write      ;
        ldi len, escape         ;
        mov char, esc           ;
        rcall eeprom_write      ;
        ldi len, label          ;
        ldi char, 0x55          ; calibration completed
        rcall eeprom_write      ;
ret

key_code:			; 4,3,2,1
	clr key			; key=0
	subi sample, 10		;
	cp sample, ent		;
	brsh code_end		; > ent-10
	ldi key, 4		;
	cp sample, rgt		;
	brsh code_end		; > rgt-10
        ldi key, 3              ;
        cp sample, lft		;
        brsh code_end           ; > lft-10
        ldi key, 2              ;
        cp sample, esc		;
        brsh code_end           ; > esc-10
        ldi key, 1              ;
	code_end:		;
ret                    		;

; ---------------- E E P R O M ---------------

eeprom_write:                   ;
        sbic EECR, EEWE         ; wait clear
        rjmp eeprom_write       ;
        clr temp                ;
        out EEARH, temp         ; address byte high
        out EEARL, len          ; address byte low
	out EEDR, char		; data byte
        sbi EECR, EEMWE         ; set write enable
        sbi EECR, EEWE          ; write eeprom
ret                             ;

eeprom_read:                    ;
        sbic EECR, EEWE         ; wait clear
        rjmp eeprom_read        ;
        clr temp                ;
        out EEARH, temp         ; H addr
        out EEARL, len          ; L addr
        sbi EECR, EERE          ; set read enable
        in char, EEDR           ; char <- eeprom
ret                             ;

; ----------------- P R I N T I N G -----------------

print_fuses:                	        ;
        lds data, low_fuse             	;
        ldi temp, 0                     ;
        mov column, temp                ; column 1
        ldi temp, 1                     ;
        mov row, temp                   ; row 1
	rcall lcd_cur			;
        rcall print_hex_byte           	; low fuse

        lds data, high_fuse            	;
        ldi temp, 4                     ;
        mov column, temp                ; column 1
        ldi temp, 1                     ;
        mov row, temp                   ; row 1
        rcall lcd_cur                   ;
        rcall print_hex_byte           	; high fuse

        lds data, ext_fuse             	;
        ldi temp, 8                     ;
        mov column, temp                ; column 1
        ldi temp, 1                     ;
        mov row, temp                   ; row 1
        rcall lcd_cur                   ;
        rcall print_hex_byte           	; ext fuse

        lds data, lock_byte            	;
        ldi temp, 12                    ;
        mov column, temp                ; column 1
        ldi temp, 1                     ;
        mov row, temp                   ; row 1
        rcall lcd_cur                   ;
        rcall print_hex_byte           	; lock byte
ret                                    	;

print_signature:		;
	ldi temp, 1		;
	mov row, temp		; row 1
	ldi temp, 0		;
	mov column, temp	; column 1
	rcall lcd_cur		;
        lds char, signature     ;
	rcall print_hex_byte	;
        lds char, signature+1   ;
        rcall print_hex_byte    ;
        lds char, signature+2   ;
        rcall print_hex_byte    ;
ret				;

print_pos:      	        ;
        rcall lcd_clr           ;
        mov char, level         ;
        rcall print_hex_byte    ;
        mov char, item          ;
        rcall print_hex_byte    ;
ret                             ;

print_key_code:			;
	rcall lcd_clr		;
	mov char, key		;
	rcall print_hex_byte	;
ret				;

print_adc:                      ; sampling and printing
        rcall lcd_clr           ;
        rcall adc_read          ;
	nop			;
        ldi zh, 0	        ; dec
        mov zl, sample          ; dec
        rcall print_ascii	; dec
	nop			;
	;mov char, sample	; hex
	;rcall print_hex_byte	; hex
	nop			;
        rcall delay_20ms        ;
        rjmp print_adc          ; loop
ret                             ;

print_twi_address:		;
	ldi len, msg14		;
	rcall lcd_clr		;
	rcall print_msg		;
	clr temp		;
	mov column, temp	; column 0
	inc temp		;
	mov row, temp		; row 1
        rcall lcd_cur           ;
	;ldi zh, 0x00		;
	;mov zl, address	;
	;rcall print_ascii	;
	mov char, address	;
	rcall print_hex_byte	;
        rcall delay_100ms       ;
ret				;

print_ok:			;
	rcall lcd_clr		;
	ldi len, msg15		;
	rcall print_msg		;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
	rcall delay_100ms	;
ret				;

print_start:                    ;
	ldi len, msg0		; len <- "Hvpp start"
        rcall print_msg         ;
	rcall delay_100ms	;
ret                             ;

print_msg:                      ;
        rcall twi_start         ;
        rcall twi_sla_w         ;
	pm1:			;
	rcall eeprom_read	;
        cpi char, 0             ;
        breq pm2                ;
        rcall lcd_char          ;
        inc len                 ; next eeprom address
        rjmp pm1	        ;
        pm2:                    ;
        rcall twi_stop          ;
        ;rcall delay_100ms       ;
        ;rcall delay_100ms       ;
        ;rcall delay_100ms       ;
ret                             ;

print_eeprom_byte:		;
        rcall twi_start         ;
        rcall twi_sla_w         ;
        rcall eeprom_read       ;
	ldi zh, 0		;
        mov zl, char            ;
        rcall print_ascii       ;
	rcall twi_stop		;
ret				;

print_hex_byte:			;
        rcall twi_start         ;
        rcall twi_sla_w         ;
	; high tetr
	push char		;
	swap char		;
	andi char, 0x0f		;
	subi char, -'0'		;
	cpi char, '9'+1
	brcs phb1
	subi char, -7
	phb1:
	rcall lcd_char
	; low tetr
        pop char	        ;
        andi char, 0x0f         ;
        subi char, -'0'         ;
        cpi char, '9'+1
        brcs phb2
        subi char, -7
        phb2:
        rcall lcd_char
        rcall twi_stop          ;
ret				;

print_ascii:			;
        rcall twi_start         ;
        rcall twi_sla_w         ;

        ; if < 1000 go 100
        cpi zh, 0x03            ;
        brlo pa2                ;
        cpi zl, 0xe8            ;
        brlo pa2                ;

        pa1:                    ;
        ldi char, 0x2f          ; x1000
        clc                     ;
        b1000:                  ;
        inc char                ;
        subi zl, 0xe8           ;
        sbci zh, 0x03           ;
        brcc b1000              ;
        rcall lcd_char          ; send char via twi
        subi zl, 0x18           ;
        subi zh, 0xfc           ; 1000 rev
        pa2:
        ldi char, 0x2f          ; x100
        clc                     ;
        b100:                   ;
        inc char                ;
        subi zl, 0x64           ;
        sbci zh, 0x00           ;
        brcc b100               ;
        rcall lcd_char          ; send char via twi
        subi zl, 0x9c           ; 100 rev
        subi zh, 0xff           ;

        ldi char, 0x2f          ; x10
        clc                     ;
        b10:                    ;
        inc char                ;
        subi zl, 0x0a           ;
        brcc b10                ;
        rcall lcd_char          ; send char
        subi zl, 0xf6           ; 10 rev

        mov char, zl            ; x1
        subi char, 0xd0         ;
        rcall lcd_char          ; send char
        rcall twi_stop          ;
ret				;

; ---------------- L C D   S P E C I F I C  ------------------


lcd_cur:			; uses row and column
	rcall twi_start		;
	rcall twi_sla_w		;
	rcall lcd_wait_bf 	;
        rcall lcd_instr_w       ;

	; next line offset	;
	clr char		;
        ldi temp, 0             ; -xxx xxxx ddram address
        ldi temp2, 0xc0         ; 2 line offset
        cpse row, temp          ; if row!=0 then
        add char, temp2         ; add offset
        add char, column        ; and add column

	; high nibble
	clr data		;
        rcall d7_high           ; set ddram addr is 1aaa aaaa
        sbrc char, 6            ;
        rcall d6_high           ;
        sbrc char, 5            ;
        rcall d5_high           ;
        sbrc char, 4            ;
        rcall d4_high           ;
        rcall twi_data_w        ;
        rcall lcd_e_pulse       ;

        ; low nibble            ;
        clr data                ;
        sbrc char, 3            ;
        rcall d7_high           ;
        sbrc char, 2            ;
        rcall d6_high           ;
        sbrc char, 1            ;
        rcall d5_high           ;
        sbrc char, 0            ;
        rcall d4_high           ;
        rcall twi_data_w        ;
        rcall lcd_e_pulse       ;

	rcall twi_stop		;
ret				;

lcd_line2:			; cursor position line 2
        rcall twi_start         ;
        rcall twi_sla_w         ;
        rcall lcd_wait_bf           ;
        andi data, 0x0F         ; clear data bits
        rcall d7_high           ; 11xx - dram
        rcall d6_high           ;
        rcall lcd_instr_w           ;
        rcall lcd_e_pulse       ; take
        andi data, 0x0F         ; clear data bits
        rcall lcd_e_pulse       ;
        rcall twi_stop          ;
ret				;

lcd_home:			; pos 0,0
        rcall twi_start         ;
        rcall twi_sla_w         ;
        rcall lcd_wait_bf       ;
        andi data, 0x0F         ; clear data bits
        rcall lcd_instr_w       ; 0000 - display
        rcall lcd_e_pulse       ; take
        rcall d5_high           ; 001* - home
        rcall lcd_e_pulse       ;
        rcall twi_stop          ;
ret				;

lcd_clr:                        ; clear screen
        rcall twi_start         ;
        rcall twi_sla_w         ;
        rcall lcd_wait_bf           ;
        andi data, 0x0F         ; clear data bits
        rcall lcd_instr_w        	; set
        rcall lcd_e_pulse       ; take
        rcall d4_high           ; 0001 - clear
        rcall lcd_e_pulse       ;
	rcall twi_stop		;
	rcall lcd_home		;
ret                             ;

lcd_char:			; write one char
	rcall lcd_wait_bf	; delay
        ; high nibble		;
	clr data		; 0000 0000
	rcall lcd_data_w	; ---- --10
	rcall bl_high		; ---- x---
	sbrc char, 7		;
	rcall d7_high		; x--- ----
        sbrc char, 6            ;
        rcall d6_high       	; -x-- ----
        sbrc char, 5            ;
        rcall d5_high       	; --x- ----
        sbrc char, 4            ;
        rcall d4_high       	; ---x ----
	rcall twi_data_w	;
        rcall lcd_e_pulse       ; ---- -x--
	; low nibble		;
        clr data                ; 0000 0000
        rcall lcd_data_w        ; ---- --10
	rcall bl_high		; ---- x---
        sbrc char, 3            ;
        rcall d7_high       	; x--- ----
        sbrc char, 2            ;
        rcall d6_high       	; -x-- ----
        sbrc char, 1            ;
        rcall d5_high       	; --x- ----
        sbrc char, 0            ;
        rcall d4_high       	; ---x ----
	rcall twi_data_w	;
        rcall lcd_e_pulse 	; ---- -x--
ret				;

lcd_init:			; optimize when out of pmem
        rcall delay_20ms        ; >= 40 ms
        rcall delay_20ms        ;
	rcall twi_start		;
	rcall twi_sla_w		;
				; 8-bit mode w/o busy flag check
        rcall bl_low            ; backlight bit
	rcall d4_high		;
	rcall d5_high		; 0011 - init1
	rcall d6_low		;
	rcall d7_low		;
	rcall lcd_instr_w		;
	rcall twi_data_w	;
	rcall lcd_e_pulse	; 1
				;
        rcall delay_1ms         ;
        rcall delay_1ms         ;
        rcall delay_1ms         ;
	rcall twi_data_w	;
        rcall lcd_e_pulse       ; 2
				;
        rcall delay_100us       ;
	rcall twi_data_w	;
	rcall lcd_e_pulse	; 3
				;
        rcall delay_100us       ;
        rcall d4_low            ; 0010 - pre function set
	rcall twi_data_w	;
        rcall lcd_e_pulse       ; 4
				;
        rcall delay_100us       ; flag chech & 4-bit mode next
				;
        rcall d5_high           ; 0010 - function set
	rcall twi_data_w	;
	rcall lcd_e_pulse	; 5
        rcall d7_high           ;
	rcall d5_low		; 1000 - 2 lines, 5x8 font
	rcall twi_data_w	;
        rcall lcd_e_pulse       ; 6
				;
        rcall lcd_wait_bf           ;
	rcall d7_low		; 0000 - display
	rcall twi_data_w	;
        rcall lcd_e_pulse       ; 7
	rcall d7_high		; 1000 - off
	rcall twi_data_w	;
        rcall lcd_e_pulse       ; 8
				;
        rcall lcd_wait_bf           ;
	rcall d7_low		; 0000 - display
	rcall twi_data_w	;
        rcall lcd_e_pulse       ; 9
        rcall d4_high           ; 0001 - clear
	rcall twi_data_w	;
        rcall lcd_e_pulse       ; 10
				;
        rcall lcd_wait_bf           ;
	rcall d4_low		; 0000 - entry
	rcall twi_data_w	;
        rcall lcd_e_pulse       ; 11
        rcall d6_high           ;
        rcall d5_high           ; 0110 - mode
	rcall twi_data_w	;
        rcall lcd_e_pulse       ; 12
				;
        rcall lcd_wait_bf           ;
	rcall d6_low		;
	rcall d5_low		; 0000 - display
	rcall twi_data_w	;
        rcall lcd_e_pulse       ; 13
        rcall d7_high           ;
        rcall d6_high           ; 1100 - on
        rcall d5_high           ; 1110 - cursor
        rcall d4_high           ; 1111 - blink
        rcall bl_high	        ;
	rcall twi_data_w	;
        rcall lcd_e_pulse       ; 14
				;
	rcall twi_stop		;
        rcall delay_20ms        ; voila
ret				;

lcd_wait_bf:           ; wait until busy flag cleared
       rcall delay_1ms ;
       rcall delay_1ms ;
       rcall delay_1ms ;
       rcall delay_1ms ;
ret                    ;

lcd_e_pulse:                    ;
        rcall e_high            ;
        rcall twi_data_w        ;
        nop                     ;
        nop                     ;
        nop                     ; about us (microsecond)
        nop                     ;
        rcall e_low             ;
        rcall twi_data_w        ;
ret                             ;

lcd_backlight_on:		;
	rcall bl_high		;
	rcall twi_start		;
	rcall twi_sla_w		;
	rcall twi_data_w	;
	rcall twi_stop		;
ret				;

lcd_backlight_off:		;
	rcall bl_low		;
        rcall bl_high           ;
        rcall twi_start         ;
        rcall twi_sla_w         ;
        rcall twi_data_w        ;
        rcall twi_stop          ;
ret				;

lcd_instr_w:                ; instrustion write
        rcall rs_low        ;
        rcall rw_low        ;
ret                         ;

lcd_instr_r:                ; busy flag or a-counter read
        rcall rs_low        ;
        rcall rw_high       ;
ret                         ;

lcd_data_w:                 ; data write
        rcall rs_high       ;
        rcall rw_low        ;
ret                         ;

lcd_data_r:                 ; data read
        rcall rs_high       ;
        rcall rw_high       ;
ret                         ;

d4_low:                 ;
        cbr data, 1<<p4 ;
ret	                ;

d4_high:                ;
        sbr data, 1<<p4 ;
ret                     ;

d5_low:                 ;
        cbr data, 1<<p5 ;
ret                     ;

d5_high:                ;
        sbr data, 1<<p5 ;
ret                     ;

d6_low:                 ;
        cbr data, 1<<p6 ;
ret                     ;

d6_high:                ;
        sbr data, 1<<p6 ;
ret                     ;

d7_low:                 ;
        cbr data, 1<<p7 ;
ret                     ;

d7_high:                ;
        sbr data, 1<<p7 ;
ret                     ;

rs_low:		        ;
        cbr data, 1<<rs ;
ret			;

rs_high:	        ;
        sbr data, 1<<rs ;
ret			;

rw_low:                 ;
        cbr data, 1<<rw ;
ret			;

rw_high:                ;
        sbr data, 1<<rw ;
ret			;

e_low:                  ;
        cbr data, 1<<e  ;
ret			;

e_high:                 ;
        sbr data, 1<<e  ;
ret			;

bl_high:		;
	sbr data, 1<<bl	;
ret			;

bl_low:		        ;
        cbr data, 1<<bl ;
ret                     ;

; ------ P C F 8 5 7 4 --------

pcf_high:                       ; (none sink)
        rcall twi_start         ;
        rcall twi_sla_w         ;
        ser data                ; 0xff
        rcall twi_data_w        ;
        rcall twi_stop          ;
ret                             ;

; --------- A D C -----------

adc_read:			;
        sbi ADCSRA, ADSC        ; start conversion
        adc:			;
        sbis ADCSRA, ADIF       ; conversion complete
        rjmp adc                ;
	in sample, ADCH		; 8 enough
ret

adc_init:
        sbi ADMUX, REFS0        ; internal reference
        sbi ADMUX, REFS1        ; 2.56
        sbi ADMUX, MUX1         ; mux
        sbi ADMUX, MUX0         ; PC3
        sbi ADMUX, ADLAR        ; left justified result
        sbi ADCSRA, ADPS2       ; 16
        sbi ADCSRA, ADPS1       ; 64
        sbi ADCSRA, ADEN        ; enable adc
        sbi ADCSRA, ADSC        ; start conversion
ret

; ------------------- T W I ------------------------------

twi_scan:			;
	ldi address, 127	; goes down
	twscan1:		;
	rcall twi_start		; start/rep_start
	rcall twi_sla_w		; slave addres + w
	cpi temp, 0xFF		; touched?
	brne twscan2		; if not cleared
	clr temp		; (temp=0x00)
	subi address, 1		; else next address
	cpi address, 0		; last address
	breq twscan2		; exit with zero address
	rjmp twscan1		;
	twscan2:		;
	rcall twi_stop		;
ret				;

twi_data_r:                                     ;
        ldi temp, ((1<<TWINT)|(1<<TWEN))        ; reset flag +nack
        out TWCR, temp                          ;
        tdr1:                                   ;
        in temp, TWCR                           ; check if
        sbrs temp, TWINT                        ; flag set?
        rjmp tdr1                               ; not yet, repeat
	in data, TWDR				; data<-
ret                                             ;

twi_data_w:					;
        tdw1:                                   ;
        in temp, TWCR                           ;
        sbrs temp, TWINT                        ; ready?
        rjmp tdw1                               ; not yet, repeat
	out TWDR, data				; data->
	ldi temp, ((1<<TWINT)|(1<<TWEN))	;
	out TWCR, temp				; go
	tdw2:					;
	in temp, TWCR				;
	sbrs temp, TWINT			; sent?
	rjmp tdw2				; not yet, repeat
	in temp, TWSR				; check status
	andi temp, 0xF8				;
	cpi temp, DATAW_ACK			; 0x28
	breq tdw3				; ok
        rcall error	                        ; led
	tdw3:					;
ret						;

twi_sla_r:                                      ;
        mov temp, address                       ; slave address (e.g. 0x3F as pcf of mine)
        lsl temp                                ; <<address + 0, R/W = W
	ori temp, 0x01				; +1, R/W = R
        out TWDR, temp                          ;
        ldi temp, ((1<<TWINT)|(1<<TWEN))        ;
        out TWCR, temp                          ; go
        slar1:                                  ;
        in temp, TWCR                           ;
        sbrs temp, TWINT                        ; sent
        rjmp slar1                              ;
        in temp, TWSR                           ; read status
        andi temp, 0xF8                         ;
        cpi temp, SLAR_ACK                      ; 0x40
        breq slar2                              ; acked
        rcall error                             ;
        slar2:                                  ;
ret                                             ;

twi_sla_w:					;
	mov temp, address			; slave address (e.g. 0x3F as pcf of mine)
	lsl temp				; <<address + 0, R/W = W
	out TWDR, temp				;
	ldi temp, ((1<<TWINT)|(1<<TWEN))	;
	out TWCR, temp				; go
	slaw1:					;
        in temp, TWCR                           ;
        sbrs temp, TWINT                        ; sent
        rjmp slaw1                              ;
        in temp, TWSR                           ; read status
        andi temp, 0xF8                         ;
        cpi temp, SLAW_ACK                      ; 0x18
	breq slaw2				; acked (stay temp untouched if scanned)
	;rcall error				; uncomment if used w/o autodetect
	ser temp				; else (temp=0xFF)
	slaw2:					;
ret						;

twi_start:						;
	ldi temp, ((1<<TWINT)|(1<<TWSTA)|(1<<TWEN))	;
	out TWCR, temp					; go
	start1:						;
	in temp, TWCR					;
	sbrs temp, TWINT				; sent
        rjmp start1                                     ;
	in temp, TWSR					; read status
	andi temp, 0xF8					;
	cpi temp, START					; start acked? 0x08
	breq start2					;
        cpi temp, REP_START                             ; 0x10
        breq start2                                     ; maybe rep. acked?
	rcall error					; error led
	start2:						;
ret							; acked

twi_stop:						;
	ldi temp, ((1<<TWINT)|(1<<TWEN)|(1<<TWSTO))	;
        out TWCR, temp                                  ;
ret							;

twi_init:			; 100 kHz @4MHz clock
	ldi temp, 0b00001100	; 4000000 / ( 16 + 2*TWBR * 4^TWPS )
	out TWBR, temp		; 0x0c
        in temp, TWSR	        ;
	ori temp, 0b00000000	; 1
	out TWSR, temp		;
ret				;

; -----------  T I M E R  R E L A T E D  --------------

delay_1s:			;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
ret				;

delay_500ms:			;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
        rcall delay_100ms       ;
ret				;

delay_100ms:                    ;
        rcall delay_20ms        ;
        rcall delay_20ms        ;
        rcall delay_20ms        ;
        rcall delay_20ms        ;
        rcall delay_20ms        ;
ret                             ;

delay_20ms:                     ;
        rcall delay_10ms         ;
        rcall delay_10ms         ;
ret                             ;

delay_10ms:			;
        rcall delay_1ms         ;
        rcall delay_1ms         ;
        rcall delay_1ms         ;
        rcall delay_1ms         ;
        rcall delay_1ms         ;
        rcall delay_1ms         ;
        rcall delay_1ms         ;
        rcall delay_1ms         ;
        rcall delay_1ms         ;
        rcall delay_1ms		;
ret				;

delay_1ms:                      ;
        rcall delay_500us       ;
        rcall delay_500us       ;
ret                             ;

delay_500us:
        rcall delay_100us       ;
        rcall delay_100us       ;
        rcall delay_100us       ;
        rcall delay_100us       ;
	rcall delay_100us	;
ret				;

delay_100us:                    ;
        ldi tcntr, 4            ;
        du1:                    ;
        cpi tcntr, 0            ;
        brne du1                ;
ret                             ;

timer_init:                     ;
        ldi temp, (1<<CS00)     ; prescaler 1, 0.25 us
        out TCCR0, temp         ;
        ldi temp, 155           ; 25 us
        out TCNT0, temp         ;
        ldi temp, (1<<TOIE0)    ;
        out TIMSK, temp         ;
        sei                     ; go
ret                             ;

TIMER0_OVF_vect:                ;
        cli                     ;
        push temp               ; save temp
        in temp, SREG           ; save SREG
        push temp
        ldi temp, 155           ; next 25 us
        out TCNT0, temp         ;
        subi tcntr, 1           ; delay counter
        pop temp
        out SREG, temp          ; restore SREG
        pop temp                ; restore temp
        sei                     ;
reti                            ;


; ------- D E B U G  ---------

error:                          ;
        sbi DDRB, PB6           ; B6 output
	blink:
        cbi PORTB, PB6          ; B6 low
	rcall delay_100ms	;
        sbi PORTB, PB6          ; B6 high
        rcall delay_100ms       ;
        rjmp blink              ;
ret                             ;

; ------ V E C T O R S --------

__vector_default:		;
reti				;

.end
