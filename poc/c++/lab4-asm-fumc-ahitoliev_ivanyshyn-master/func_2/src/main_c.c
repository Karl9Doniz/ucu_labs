#include <stdint.h>
#include <stdio.h>

extern void func(uint64_t* input_array, size_t size);

int main() {
    uint64_t input_array_2[] = {11, 2, 3, 18, 6};
    size_t size = sizeof(input_array_2) / sizeof(input_array_2[0]);

    printf("Input Array for Sorting: ");
    for (size_t i = 0; i < size; ++i) {
        printf("%lu ", input_array_2[i]);
    }
    printf("\n");

    func(input_array_2, size);

    printf("Sorted Array: ");
    for (size_t i = 0; i < size; ++i) {
        printf("%lu ", input_array_2[i]);
    }
    printf("\n");

    return 0;
}
