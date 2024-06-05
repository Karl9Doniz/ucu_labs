#include <stdint.h>
#include <stdio.h>

extern void func(uint32_t* input_array, uint8_t* is_prime_array, size_t size);

int main() {
    uint32_t input_array_1[] = {2, 8, 5, 7, 12};
    size_t size = sizeof(input_array_1) / sizeof(input_array_1[0]);

    printf("Input Array for Primes: ");
    for (size_t i = 0; i < size; ++i) {
        printf("%u ", input_array_1[i]);
    }
    printf("\n");

    uint8_t is_prime_array[size];

    func(input_array_1, is_prime_array, size);

    printf("Is Prime Array: ");
    for (size_t i = 0; i < size; ++i) {
        printf("%s ", is_prime_array[i] ? "true" : "false");
    }
    printf("\n");

    return 0;
}
