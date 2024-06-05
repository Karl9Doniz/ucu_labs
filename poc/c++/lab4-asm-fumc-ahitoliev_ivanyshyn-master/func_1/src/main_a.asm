.intel_syntax noprefix
.global main
.extern func
.extern printf

.data
    is_prime_array: .byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    array: .int 2, 4, 77, 1, 97, 88, 12, 13, 5, 0
    size: .quad 10
    msg_true: .asciz "%u true\n"
    msg_false: .asciz "%u false\n"
    msg_list: .asciz "%u\n"

.text
main:
    push rbp
    mov rbp, rsp
    push rbx
    push r12
    xor rbx, rbx
    xor r12, r12

check_primes:
    lea rdi, [array]
    lea rsi, [is_prime_array]
    mov rdx, [size]
    call func
    lea r12, [array]
    mov rbx, [size]

loop_prime:
    cmp rbx, 0
    je end_main
    mov al, [is_prime_array + rbx - 1]
    cmp al, 1
    je print_prime

print_not_prime:
    lea rdi, [msg_false]
    mov rsi, [r12 + (rbx - 1)*4]
    call printf
    dec rbx
    jmp loop_prime

print_prime:
    lea rdi, [msg_true]
    mov rsi, [r12 + (rbx - 1)*4]
    call printf
    dec rbx
    jmp loop_prime

end_main:
    pop r12
    pop rbx
    mov rsp, rbp
    pop rbp
    mov rax, 60
    xor rdi, rdi
    syscall
