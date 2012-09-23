#include <avr/io.h>

; Include header
#include <sat/receiver.h>

; Get external vars
.extern length, rellength, speed, dir

; Get external functions
.extern encoder_reset
.extern sys_reset, led_on, led_off

; Declare UART Rx vector in assembler - faster :))

.global USART_RX_vect

USART_RX_vect:
    
    ; Saving R20, R21 and SREG
    push R20
    push R21
    in R21, _SFR_IO_ADDR(SREG)
    push R21
    push ZL
    push ZH

    rcall led_on

    in R20, _SFR_IO_ADDR(UDR)   ; receive the character

    dec R20                     ; decrement - we haven't command 0x00, only 0x01 and higher

    cpi R20, 0x0B               ; check input value for range 0x01 - 0x0E
    brcc uartint_close          ; if higher - do nothing

    lsl R20                     ; shift it left (equals *2)

    clr ZL
    clr ZH

    ldi ZL, lo8(Commands)       ; load commands table address to Z register pair
    ldi ZH, hi8(Commands)

    clr R21                     ; clear R21
    add ZL, R20                 ; add command request to the address
    adc ZH, R21                 ; and add Carry to higher

    lpm R20, Z+
    lpm R21, Z

    movw ZL, R20                ; load the address of the function to Z pair

    icall

    rcall led_off

uartint_close:
    pop ZH
    pop ZL
    pop R21
    out _SFR_IO_ADDR(SREG), R21
    pop R21
    pop R20

    reti

; Here comes the table of functions to call (15)

Commands: .word pm(Enc_getdir), pm(Drv_getdir), pm(Enc_getlen), pm(Enc_getpath), pm(Drv_getspeed), pm(Rq_getspeed), pm(Enc_getspeed), pm(Drv_setdir), pm(Drv_setspeed), pm(Rq_setspeed), pm(encoder_reset)

; Here comes the command handlers

; This command will send the value of encoder direction
; Note that this fucntion call the C subroutine

UART_wait:
    sbis _SFR_IO_ADDR(UCSRA), UDRE
    rjmp UART_wait
    ret

Enc_getdir:
    
    push r24
    .extern encoder_get_dir
    rcall encoder_get_dir

    rcall UART_wait
    out _SFR_IO_ADDR(UDR), R24 ; in R24 return value is stored

    pop r24

    ret

Drv_getdir:
    
    push r24
    .extern driver_get_dir
    rcall driver_get_dir

    rcall UART_wait
    out _SFR_IO_ADDR(UDR), R24
    pop r24

    ret

Int32_send:
    rcall UART_wait
    out _SFR_IO_ADDR(UDR), R25
    rcall UART_wait
    out _SFR_IO_ADDR(UDR), R24
    rcall UART_wait
    out _SFR_IO_ADDR(UDR), R23
    rcall UART_wait
    out _SFR_IO_ADDR(UDR), R22
    ret

Enc_getlen:

    push r22
    push r23
    push r24
    push r25

    .extern encoder_get_length
    rcall encoder_get_length

    rcall Int32_send

    pop r25
    pop r24
    pop r23
    pop r22

    ret

Enc_getpath:
 
    push r22
    push r23
    push r24
    push r25

    .extern encoder_get_rellength
    rcall encoder_get_rellength

    rcall Int32_send

    pop r25
    pop r24
    pop r23
    pop r22

    ret

Drv_getspeed:
    
    push r20

    in r20, _SFR_IO_ADDR(OCR1BH)
    rcall UART_wait
    out _SFR_IO_ADDR(UDR), r20

    in r20, _SFR_IO_ADDR(OCR1BL)
    rcall UART_wait
    out _SFR_IO_ADDR(UDR), r20

    pop r20

    ret

Rq_getspeed:
    ; TODO: Stabilistaion algos
    ret

Enc_getspeed:
    
    push r24
    
    .extern encoder_get_speed
    rcall encoder_get_speed

    rcall UART_wait
    out _SFR_IO_ADDR(UDR), r24

    pop r24

    ret

UART_receive:
    ; Ok, here we go
    ; We need to turn off the Rx interrupt
    ; and stay waiting for incoming byte,
    ; then turn interrupt on

    cbi _SFR_IO_ADDR(UCSRB), RXCIE ; Turn off the interrupt
UART_receive_wait:
    sbis _SFR_IO_ADDR(UCSRA), RXC
    rjmp UART_receive_wait

    in r24, _SFR_IO_ADDR(UDR)

    sbi _SFR_IO_ADDR(UCSRB), RXCIE

    ret


Drv_setdir:

    push r24
    push r25

    rcall UART_receive
    mov r25, r24

    .extern driver_set_dir
    rcall driver_set_dir

    pop r25
    pop r24

    ret

Drv_setspeed:

    push r24
    push r25

    rcall UART_receive
    mov r25, r24

    rcall UART_receive

    .extern driver_set_speed
    rcall driver_set_speed

    pop r25
    pop r24
    
    ret

Rq_setspeed:

    ; TODO: stabilisation algos
    ret
