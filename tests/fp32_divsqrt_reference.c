// Native binary32 reference. Compile with rounding and fenv access enabled.
#include <fenv.h>
#include <float.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
               FLT_MAX_EXP == 128 && FLT_MIN_EXP == -125 && FLT_EVAL_METHOD == 0,
               "The reference requires native IEEE binary32 arithmetic");

static float decode(uint32_t word) {
    float value;
    memcpy(&value, &word, sizeof(value));
    return value;
}

int main(void) {
    if (fesetround(FE_TONEAREST) != 0)
        return 1;

    unsigned mode;
    uint32_t a, b;
    int fields;
    while ((fields = scanf("%u %" SCNx32 " %" SCNx32, &mode, &a, &b)) == 3) {
        volatile float left = decode(a);
        volatile float right = decode(b);
        feclearexcept(FE_ALL_EXCEPT);
        volatile float evaluated = mode ? sqrtf(left) : left / right;
        const int raised = fetestexcept(FE_ALL_EXCEPT);
        const float value = evaluated;
        uint32_t word;
        memcpy(&word, &value, sizeof(word));

        if ((word & UINT32_C(0x7fffffff)) > UINT32_C(0x7f800000))
            word = UINT32_C(0x7fc00000);
        // The lane uses after-rounding tininess. A host that detects tininess
        // before rounding can also raise FE_UNDERFLOW on a minimum normal result.
        const unsigned flags = ((raised & FE_INVALID) != 0) << 4 |
                               ((raised & FE_DIVBYZERO) != 0) << 3 |
                               ((raised & FE_OVERFLOW) != 0) << 2 |
                               (((raised & FE_UNDERFLOW) != 0 &&
                                 (word & UINT32_C(0x7f800000)) == 0) << 1) |
                               ((raised & FE_INEXACT) != 0);
        printf("%08" PRIx32 " %02x\n", word, flags);
    }
    return fields == EOF ? 0 : 1;
}
