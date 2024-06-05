.intel_syntax noprefix

.text
.global func

func:
    xor rcx, rcx

outer_loop:
    xor rdx, rdx

inner_loop:
    mov rax, rsi
    sub rax, rcx
    sub rax, 1
    cmp rdx, rax
    jge end_inner_loop
    mov rax, [rdi + rdx * 8]
    mov r8, [rdi + rdx * 8 + 8]

    cmp rax, r8
    jle no_swap

    xchg rax, r8

    mov [rdi + rdx * 8], rax
    mov [rdi + rdx * 8 + 8], r8

no_swap:
    inc rdx
    jmp inner_loop

end_inner_loop:
    inc rcx
    cmp rcx, rsi
    jl outer_loop

end_process:
    mov rax, 60
    xor rdi, rdi
    ret
