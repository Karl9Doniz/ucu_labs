.intel_syntax noprefix
.global main
.extern printf
.extern func

.data
    msg1: .asciz "unsorted:\n"
    msg2: .asciz "sorted:\n"
    msg: .asciz "%u\n"
    array_sort: .quad 3,67,76,54,98,21,12,45,96,34
    size: .quad 10

.text
main:
    push rbp
    mov rbp, rsp
    xor r15, r15
    xor r12, r12
    xor r13, r13
    xor r14, r14

sort:
    inc r14
    cmp r14, 1
    je print_unsorted
    cmp r14, 2
    je print_sorted
    jmp end_main

print_unsorted:
    lea r12, [array_sort]
    mov r15, [size]
    dec r15
    lea rdi, [msg1]
    mov rsi, [r12]
    call printf
    mov r13, 0
    jmp print_lst

print_sorted:
    lea rdi, [array_sort]
    mov rsi, [size]
    call func
    lea rdi, [msg2]
    call printf
    mov r13, 0

print_lst:
    cmp r13, r15
    ja sort
    lea rdi, [msg]
    mov rsi, [r12 + r13 * 8]
    call printf
    inc r13
    jmp print_lst

end_main:
    pop r15
    pop r12
    pop r13
    pop r14
    mov rsp, rbp
    pop rbp
    mov rax, 60
    xor rdi, rdi
    syscall
