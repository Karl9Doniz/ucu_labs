.intel_syntax noprefix
.section .debug_info
.section .debug_line
.global func
.text
func:
    push rbp
    mov rbp, rsp
    xor ecx, ecx
    xor r8, r8
    mov r9, rdx
    xor edx, edx
outer_cycle:
    mov ecx, 1
    cmp r8, r9
    je end_cycle
inner_cycle:
    mov eax, [rdi + r8*4]
    cmp eax, 1
    jbe not_prime
    inc ecx
    cmp ecx, eax
    je is_prime
    mov eax, [rdi + r8*4]
    xor edx, edx
    div ecx
    test edx, edx
    jz not_prime
    jmp inner_cycle
is_prime:
    cmp r8, r9
    jge end_cycle
    lea r10, [rsi + r8]
    mov al, 1
    mov [r10], al
    jmp new_loop
not_prime:
    cmp r8, r9
    jge end_cycle
    lea r10, [rsi + r8]
    xor al, al
    mov [r10], al
    jmp new_loop
new_loop:
    inc r8
    jmp outer_cycle
end_cycle:
    mov rsp, rbp
    pop rbp
    ret
