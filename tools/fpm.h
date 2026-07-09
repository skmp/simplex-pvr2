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
//   fp_mul<M,N>(a,b)  multiply two operands, seen at N-bit significands, result
//                     an fpm<M> (M-bit output significand).
//
// REPRESENTATION: a value is carried as {sign, 8-bit biased exponent, an N-bit
// normalized significand} - NOT as a float32 bit-pattern. This is what lets
// fpm<32>, fpm<48>, fpm<64> hold MORE mantissa precision than float32's 23 stored
// bits. The float32 exponent range/bias is reused (8-bit, bias 127). Only
// .tof32() collapses back to 23 stored bits.
//   sig_ : uint64_t, the N-bit significand with the hidden 1 at bit (N-1), i.e.
//          value = (sig_ / 2^(N-1)) * 2^(exp_-127).  sig_ in [2^(N-1), 2^N).
//   exp_ : uint16_t biased exponent (0 => the whole value is zero, DaZ).
//   sgn_ : sign bit.
//
// ALL arithmetic follows the datapath's non-IEEE rules:
//   - DaZ    : any operand with biased exponent 0 (zero or subnormal) is 0.
//   - no inf/NaN : exponent 0xFF is treated as a normal number; results never
//                  produce inf/NaN.
//   - truncate : no rounding anywhere - significands are chopped, not rounded.
//   - saturate : exponent overflow -> max finite {exp=0xFE, significand all-ones
//                to N bits}; underflow -> signed zero.
//
// fpm<24> is (nearly) plain float32 with truncation instead of round-to-nearest;
// fpm<16> / fp_mul<16,16> match the fp_mul16 setup path.
//
#ifndef FPM_H
#define FPM_H

#include <cstdint>
#include <cstring>

template <int N>
struct fpm {
    static_assert(N >= 16 && N <= 64, "fpm<N>: N (significand bits) must be 16..64");

    // ---- construct from a host float32 (DaZ + widen/keep to an N-bit significand) --
    explicit fpm(float f) {
        uint32_t raw; std::memcpy(&raw, &f, 4);
        uint32_t exp = (raw >> 23) & 0xFFu;
        sgn_ = (raw >> 31) & 1u;
        if (exp == 0u) { exp_ = 0; sig_ = 0; sgn_ = 0; return; }   // DaZ: flush to +0
        exp_ = (uint16_t)exp;
        // float32 significand = hidden 1 + 23 stored (24 bits). Place it as an
        // N-bit significand (leading 1 at bit N-1): shift up for N>24, chop for N<24.
        uint64_t s24 = (uint64_t)(0x800000u | (raw & 0x7FFFFFu));  // 24-bit significand
        sig_ = place_from(s24, 24);
    }

    // ---- read back as a host float32 (truncate the N-bit significand to 23 stored) --
    float tof32() const {
        if (exp_ == 0) { uint32_t z = sgn_ << 31; float f; std::memcpy(&f, &z, 4); return f; }
        // bring the N-bit significand down to 24 bits (leading 1 at bit23).
        uint64_t s24 = (N >= 24) ? (sig_ >> (N - 24)) : (sig_ << (24 - N));
        uint32_t mant = (uint32_t)(s24 & 0x7FFFFFu);
        uint32_t bits = (sgn_ << 31) | ((uint32_t)exp_ << 23) | mant;
        float f; std::memcpy(&f, &bits, 4); return f;
    }

    // ---- unary negate: flip the sign; a DaZ zero stays +0 ----
    fpm operator-() const {
        fpm r = *this;
        if (exp_ == 0) { r.sgn_ = 0; return r; }
        r.sgn_ ^= 1u;
        return r;
    }

    // ---- add / sub (fp_add24 rules, at N-bit significand precision) ----
    fpm operator+(const fpm& o) const { return add_sub(*this, o, false); }
    fpm operator-(const fpm& o) const { return add_sub(*this, o, true);  }

    template <int MM, int NN, int KK> friend fpm<MM> fp_mul(const fpm<KK>&, const fpm<KK>&);
    template <int MM> friend struct fpm;

private:
    uint64_t sig_;    // N-bit significand, leading 1 at bit (N-1); 0 iff value is 0
    uint16_t exp_;    // 8-bit biased exponent (0 => zero)
    uint8_t  sgn_;    // sign

    fpm() : sig_(0), exp_(0), sgn_(0) {}

    // place a `from`-bit significand (leading 1 at bit from-1) as an N-bit one.
    static uint64_t place_from(uint64_t s, int from) {
        if (N >= from) return s << (N - from);
        return s >> (from - N);                       // chop low bits (truncate)
    }

    // build from raw fields (already an N-bit significand).
    static fpm make(uint8_t sgn, int e, uint64_t sig) {
        fpm r;
        if (sig == 0 || e <= 0) { r.exp_ = 0; r.sig_ = 0; r.sgn_ = 0; return r; }  // underflow -> +0? keep sign below
        r.sgn_ = sgn;
        if (e >= 255) { r.exp_ = 0xFE; r.sig_ = satsig(); return r; }              // saturate max finite
        r.exp_ = (uint16_t)e; r.sig_ = sig;
        return r;
    }
    // signed-zero underflow helper (keeps sign, value 0).
    static fpm szero(uint8_t sgn) { fpm r; r.exp_ = 0; r.sig_ = 0; r.sgn_ = sgn ? 1 : 0; return r; }

    // all-ones N-bit significand (max finite).
    static uint64_t satsig() {
        return (N >= 64) ? ~0ull : ((uint64_t)1 << N) - 1;
    }

    // align larger/smaller by exponent diff, add/sub, normalize, truncate to N.
    static fpm add_sub(const fpm& A, const fpm& B, bool sub) {
        uint8_t sa = A.sgn_, sb = B.sgn_ ^ (sub ? 1u : 0u);
        // DaZ: an exponent-0 operand is zero -> result is the other side.
        if (A.exp_ == 0 && B.exp_ == 0) { fpm r; return r; }         // +0
        if (A.exp_ == 0) { fpm r = B; r.sgn_ = (r.exp_==0)?0:sb; return r; }
        if (B.exp_ == 0) return A;

        // carry the significands with 1 guard bit of head-room for the carry-out.
        uint64_t sig_a = A.sig_, sig_b = B.sig_;
        int ea = A.exp_, eb = B.exp_;

        bool a_ge = (ea > eb) || (ea == eb && sig_a >= sig_b);
        uint64_t sig_big = a_ge ? sig_a : sig_b;
        uint64_t sig_sml = a_ge ? sig_b : sig_a;
        int      e_big   = a_ge ? ea : eb;
        int      e_sml   = a_ge ? eb : ea;
        uint8_t  s_big   = a_ge ? sa : sb;
        uint8_t  s_sml   = a_ge ? sb : sa;

        int shamt = e_big - e_sml;
        uint64_t sml_sh = (shamt >= 64) ? 0ull : (sig_sml >> shamt);

        bool same_sign = (s_big == s_sml);
        uint64_t sum = same_sign ? (sig_big + sml_sh) : (sig_big - sml_sh);
        if (sum == 0ull) { fpm r; return r; }                        // exact cancel -> +0

        int e_norm = e_big;
        uint64_t norm = sum;
        // same-sign add can carry into bit N -> shift right, exp+1.
        if (norm & ((uint64_t)1 << N)) { norm >>= 1; e_norm += 1; }
        else {
            // subtract can drop the leading 1 below bit (N-1) -> shift left.
            while (!(norm & ((uint64_t)1 << (N - 1))) && e_norm > 0) { norm <<= 1; e_norm -= 1; }
        }
        norm &= (N >= 64) ? ~0ull : (((uint64_t)1 << N) - 1);        // keep N bits

        if (e_norm <= 0)   return szero(s_big);
        return make(s_big, e_norm, norm);
    }
};

//
// fp_mul<M, N> - multiply two operands, seeing their significands at N bits and
// producing an fpm<M> (M-bit output significand).
//   M = output significand bits, N = INPUT significand bits.
// The operands may be any fpm<K> (K deduced); their significands are (re)quantised
// to N bits, the N x N product formed, and the result truncated to M output bits.
// DaZ, no inf/NaN, truncate, saturate/underflow - fp_mul16 generalized.
// A MAC product in the reduced datapath (fp_mul16) is fp_mul<24,16>(a,b): 16-bit
// inputs, 24-bit output.
//
template <int M, int N, int K>
fpm<M> fp_mul(const fpm<K>& a, const fpm<K>& b) {
    static_assert(M >= 16 && M <= 64, "fp_mul<M,N>: M must be 16..64");
    static_assert(N >= 16 && N <= 64, "fp_mul<M,N>: N must be 16..64");

    uint8_t res_sign = a.sgn_ ^ b.sgn_;
    // DaZ: a zero operand -> signed zero.
    if (a.exp_ == 0 || b.exp_ == 0) return fpm<M>::szero(res_sign);

    // requantise each K-bit significand to N bits (leading 1 at bit N-1).
    uint64_t sig_a = (K >= N) ? (a.sig_ >> (K - N)) : (a.sig_ << (N - K));
    uint64_t sig_b = (K >= N) ? (b.sig_ >> (K - N)) : (b.sig_ << (N - K));

    // N x N -> up to 2N-bit product (128-bit accumulator; N up to 64 safe).
    __uint128_t prod = (__uint128_t)sig_a * (__uint128_t)sig_b;
    int e_sum = (int)a.exp_ + (int)b.exp_ - 127;

    // inputs have their hidden 1 at bit (N-1), so the product's leading 1 is at
    // bit (2N-2) if in [1,2), or (2N-1) if in [2,4).
    bool top = ((prod >> (2 * N - 1)) & 1u) != 0u;
    int lead = top ? (2 * N - 1) : (2 * N - 2);
    if (top) e_sum += 1;

    // extract M significand bits: leading 1 -> bit (M-1), truncate the rest.
    int sh = lead - (M - 1);
    uint64_t sig_m = (sh >= 0) ? (uint64_t)(prod >> sh) : (uint64_t)(prod << (-sh));
    sig_m &= (M >= 64) ? ~0ull : (((uint64_t)1 << M) - 1);

    if (e_sum <= 0)   return fpm<M>::szero(res_sign);
    return fpm<M>::make(res_sign, e_sum, sig_m);
}

#endif // FPM_H
