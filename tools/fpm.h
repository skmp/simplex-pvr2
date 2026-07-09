//
// fpm.h - parametric reduced-precision float, matching the DC/PVR setup datapath
// arithmetic (rtl/isp_min/fp_mul16.sv, fp_add24.sv).
//
//   fpm<N>            a float with an N-bit significand (1 hidden + N-1 stored),
//                     16 <= N <= 64. Interface:
//                       fpm<N>(float)   construct from a host float32
//                       .tof32()        read back as a host float32
//                       -a              unary negate
//                       a + b, a - b    add / sub
//                     Internally it holds a float32 bit-pattern with the low
//                     mantissa bits below N truncated away, so the value is
//                     always exactly N-significand-bit representable.
//   fp_mul<M,N>(a,b)  multiply two fpm<N>, result an fpm<M> (M-bit output
//                     significand). Input precision N, output precision M.
//
// ALL arithmetic follows the datapath's non-IEEE rules:
//   - DaZ    : any operand with biased exponent 0 (zero or subnormal) is 0.
//   - no inf/NaN : exponent 0xFF is treated as a normal number; results never
//                  produce inf/NaN.
//   - truncate : no rounding anywhere - significands are chopped, not rounded.
//   - saturate : exponent overflow -> max finite {exp=0xFE, mant=all-ones
//                truncated to the target precision}; underflow -> signed zero.
//
// fpm<24> is (nearly) plain float32 done with truncation instead of round-to-
// nearest; fpm<16> / fp_mul<16,16> match the fp_mul16 setup path.
//
#ifndef FPM_H
#define FPM_H

#include <cstdint>
#include <cstring>

template <int M, int N> struct fp_mul_impl;   // fwd (friendship for cross-N bits)

template <int N>
struct fpm {
    static_assert(N >= 16 && N <= 64, "fpm<N>: N (significand bits) must be 16..64");

    // ---- construct from a host float32 (DaZ + truncate mantissa to N bits) ----
    explicit fpm(float f) {
        uint32_t raw; std::memcpy(&raw, &f, 4);
        uint32_t exp = (raw >> 23) & 0xFFu;
        if (exp == 0u) { bits_ = 0u; return; }             // DaZ: flush to +0
        uint32_t mant = raw & 0x7FFFFFu;
        if (N < 24) {                                      // chop stored bits below N
            int drop = 24 - N;
            mant &= (0x7FFFFFu >> drop) << drop;
        }
        bits_ = (raw & 0x80000000u) | (exp << 23) | mant;
    }

    // ---- read back as a host float32 ----
    float tof32() const { float f; std::memcpy(&f, &bits_, 4); return f; }

    // ---- unary negate: flip the sign; a DaZ zero stays +0 ----
    fpm operator-() const {
        fpm r;
        if ((bits_ & 0x7F800000u) == 0u) { r.bits_ = 0u; return r; }
        r.bits_ = bits_ ^ 0x80000000u;
        return r;
    }

    // ---- add / sub (fp_add24 rules) ----
    fpm operator+(const fpm& o) const { return add_sub(*this, o, false); }
    fpm operator-(const fpm& o) const { return add_sub(*this, o, true);  }

    template <int M, int NN> friend fpm<M> fp_mul(const fpm<NN>&, const fpm<NN>&);

private:
    uint32_t bits_;                        // float32 layout, mantissa truncated to N
    fpm() : bits_(0) {}                     // uninitialized (internal use only)

    // build from an already-N-truncated raw pattern (internal fast path)
    static fpm raw(uint32_t b) { fpm r; r.bits_ = b; return r; }

    // re-truncate a raw float32 pattern to N significand bits (with DaZ).
    static fpm quant(uint32_t rawbits) {
        uint32_t exp = (rawbits >> 23) & 0xFFu;
        if (exp == 0u) return raw(0u);
        uint32_t mant = rawbits & 0x7FFFFFu;
        if (N < 24) { int drop = 24 - N; mant &= (0x7FFFFFu >> drop) << drop; }
        return raw((rawbits & 0x80000000u) | (exp << 23) | mant);
    }

    // saturated max-finite mantissa for this precision (all-ones truncated to N).
    static uint32_t satmant() {
        return (N < 24) ? (((0x7FFFFFu >> (24 - N)) << (24 - N))) : 0x7FFFFFu;
    }

    // align larger/smaller by exponent diff, add/sub the aligned significands,
    // normalize (leading-1 search), truncate to N, saturate/underflow.
    static fpm add_sub(const fpm& A, const fpm& B, bool sub) {
        uint32_t ab = A.bits_;
        uint32_t bb = sub ? (B.bits_ ^ 0x80000000u) : B.bits_;

        uint32_t sa = ab >> 31,           sbb = bb >> 31;
        uint32_t ea = (ab >> 23) & 0xFFu, eb  = (bb >> 23) & 0xFFu;

        // DaZ: an exponent-0 operand is zero -> result is the other side.
        bool az = (ea == 0u), bz = (eb == 0u);
        if (az && bz) return raw(0u);
        if (az) return quant(bb);
        if (bz) return quant(ab);

        // 24-bit significands (hidden 1 + 23 stored), carried in 64 bits so the
        // alignment shift has head/room.
        uint64_t sig_a = (uint64_t)(0x800000u | (ab & 0x7FFFFFu));
        uint64_t sig_b = (uint64_t)(0x800000u | (bb & 0x7FFFFFu));

        bool a_ge = (ea > eb) || (ea == eb && sig_a >= sig_b);
        uint64_t sig_big = a_ge ? sig_a : sig_b;
        uint64_t sig_sml = a_ge ? sig_b : sig_a;
        uint32_t e_big   = a_ge ? ea : eb;
        uint32_t e_sml   = a_ge ? eb : ea;
        uint32_t s_big   = a_ge ? sa : sbb;
        uint32_t s_sml   = a_ge ? sbb : sa;

        uint32_t shamt  = e_big - e_sml;
        uint64_t sml_sh = (shamt >= 64u) ? 0ull : (sig_sml >> shamt);

        bool same_sign = (s_big == s_sml);
        uint64_t sum = same_sign ? (sig_big + sml_sh) : (sig_big - sml_sh);
        if (sum == 0ull) return raw(0u);

        // normalize: put the leading 1 at bit23.
        int e_norm = (int)e_big;
        uint64_t norm = sum;
        if (norm & (1ull << 24)) { norm >>= 1; e_norm += 1; }         // carry out
        else while (!(norm & (1ull << 23)) && e_norm > 0) { norm <<= 1; e_norm -= 1; }

        uint32_t mant = (uint32_t)(norm & 0x7FFFFFu);
        if (N < 24) { int drop = 24 - N; mant &= (0x7FFFFFu >> drop) << drop; }

        if (e_norm <= 0)   return raw(s_big << 31);                   // underflow -> signed 0
        if (e_norm >= 255) return raw((s_big << 31) | (0xFEu << 23) | satmant());
        return raw((s_big << 31) | ((uint32_t)e_norm << 23) | mant);
    }
};

//
// fp_mul<M, N> - multiply two fpm<N> operands, produce an fpm<M>.
//   M = output significand bits, N = input significand bits.
// N x N product truncated to M output bits. DaZ, no inf/NaN, truncate,
// saturate/underflow - fp_mul16 generalized.
//
template <int M, int N>
fpm<M> fp_mul(const fpm<N>& a, const fpm<N>& b) {
    static_assert(M >= 16 && M <= 64, "fp_mul<M,N>: M must be 16..64");
    uint32_t ab = a.bits_, bb = b.bits_;

    uint32_t sa = ab >> 31, sb = bb >> 31;
    uint32_t ea = (ab >> 23) & 0xFFu, eb = (bb >> 23) & 0xFFu;
    uint32_t res_sign = sa ^ sb;

    // DaZ: a zero operand -> signed zero.
    if (ea == 0u || eb == 0u) return fpm<M>::raw(res_sign << 31);

    // N-bit significands: hidden 1 at bit (N-1), then the top (N-1) float32 stored
    // mantissa bits. float32 has only 23 stored bits; for N>24 the extra low bits
    // are zero (the input carries no more precision than float32).
    uint32_t mant_a = ab & 0x7FFFFFu, mant_b = bb & 0x7FFFFFu;
    int msh = 24 - N;
    uint64_t sig_a, sig_b;
    if (msh >= 0) {
        sig_a = ((uint64_t)1 << (N - 1)) | (uint64_t)(mant_a >> msh);
        sig_b = ((uint64_t)1 << (N - 1)) | (uint64_t)(mant_b >> msh);
    } else {
        sig_a = ((uint64_t)1 << (N - 1)) | ((uint64_t)mant_a << (-msh));
        sig_b = ((uint64_t)1 << (N - 1)) | ((uint64_t)mant_b << (-msh));
    }

    // N x N -> up to 2N-bit product (128-bit accumulator so N up to 64 is safe).
    __uint128_t prod = (__uint128_t)sig_a * (__uint128_t)sig_b;
    int e_sum = (int)ea + (int)eb - 127;

    // leading 1 at bit (2N-2) if product in [1,2), or (2N-1) if in [2,4).
    bool top = ((prod >> (2 * N - 1)) & 1u) != 0u;
    int lead = top ? (2 * N - 1) : (2 * N - 2);
    if (top) e_sum += 1;

    // shift so the leading 1 sits at bit (M-1); low (M-1) bits are the mantissa.
    int sh = lead - (M - 1);
    uint64_t sig_m = (sh >= 0) ? (uint64_t)(prod >> sh) : (uint64_t)(prod << (-sh));
    uint64_t mant_m = sig_m & (((uint64_t)1 << (M - 1)) - 1);

    // map M-bit mantissa high-aligned into the float32 23-bit stored field.
    int up = 24 - M;
    uint32_t stored = (up >= 0) ? (uint32_t)((mant_m << up) & 0x7FFFFFu)
                                : (uint32_t)((mant_m >> (-up)) & 0x7FFFFFu);

    if (e_sum <= 0)   return fpm<M>::raw(res_sign << 31);                        // underflow
    if (e_sum >= 255) return fpm<M>::raw((res_sign << 31) | (0xFEu << 23) | fpm<M>::satmant());
    return fpm<M>::raw((res_sign << 31) | ((uint32_t)e_sum << 23) | stored);
}

#endif // FPM_H
