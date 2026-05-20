#include <stdint.h>

static inline uint32_t read_mcycle32(void){
    uint32_t value;
    __asm__ volatile ("csrr %0, mcycle" : "=r"(value));
    return value;
}

volatile float fpu_smoke_sink;
volatile uint32_t fpu_smoke_cycles;

void fpu_smoke_test(void){
    volatile float a = 1.25f;
    volatile float b = 2.50f;
    uint32_t start = read_mcycle32();
    float c = a + b;
    c = c * 3.0f;
    c = c * 0.5f;
    uint32_t stop = read_mcycle32();
    fpu_smoke_sink = c;
    fpu_smoke_cycles = stop - start;
}
