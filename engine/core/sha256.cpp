// =============================================================================
//  MUMBAI 1:1 EXACT — engine/core/sha256.cpp
//
//  FIPS 180-4 SHA-256, streaming.  See the header for why the engine carries it.
//
//  The constants are the standard's: the first 32 bits of the fractional parts of
//  the cube roots of the first sixty-four primes (K), and of the square roots of
//  the first eight (H0).  They are copied, not derived, because a hash that
//  derives its own constants is a hash nobody can check against a published vector.
// =============================================================================
#include "engine/core/sha256.h"

#include <cstdio>

namespace mumbai {
namespace {

constexpr u32 K[64] = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u,
    0xab1c5ed5u, 0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu,
    0x9bdc06a7u, 0xc19bf174u, 0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu,
    0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau, 0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u,
    0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu,
    0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u, 0xa2bfe8a1u, 0xa81a664bu,
    0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u, 0x19a4c116u,
    0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
    0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u,
    0xc67178f2u,
};

constexpr u32 H0[8] = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
                       0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};

inline u32 rotr(u32 x, u32 n) { return (x >> n) | (x << (32 - n)); }

}  // namespace

void Sha256::reset() {
    for (int i = 0; i < 8; ++i) m_state[i] = H0[i];
    m_totalBits = 0;
    m_used = 0;
    for (u8& byte : m_buffer) byte = 0;
}

void Sha256::compress(const u8* block) {
    u32 w[64];
    for (int i = 0; i < 16; ++i) {
        w[i] = (static_cast<u32>(block[i * 4]) << 24) | (static_cast<u32>(block[i * 4 + 1]) << 16) |
               (static_cast<u32>(block[i * 4 + 2]) << 8) | static_cast<u32>(block[i * 4 + 3]);
    }
    for (int i = 16; i < 64; ++i) {
        const u32 s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
        const u32 s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }

    u32 a = m_state[0], b = m_state[1], c = m_state[2], d = m_state[3];
    u32 e = m_state[4], f = m_state[5], g = m_state[6], h = m_state[7];

    for (int i = 0; i < 64; ++i) {
        const u32 s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
        const u32 ch = (e & f) ^ (~e & g);
        const u32 t1 = h + s1 + ch + K[i] + w[i];
        const u32 s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
        const u32 maj = (a & b) ^ (a & c) ^ (b & c);
        const u32 t2 = s0 + maj;
        h = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }

    m_state[0] += a;
    m_state[1] += b;
    m_state[2] += c;
    m_state[3] += d;
    m_state[4] += e;
    m_state[5] += f;
    m_state[6] += g;
    m_state[7] += h;
}

void Sha256::update(const void* data, size_t bytes) {
    const u8* input = static_cast<const u8*>(data);
    m_totalBits += static_cast<u64>(bytes) * 8ull;

    if (m_used > 0) {
        while (bytes > 0 && m_used < 64) {
            m_buffer[m_used++] = *input++;
            --bytes;
        }
        if (m_used == 64) {
            compress(m_buffer);
            m_used = 0;
        }
    }
    while (bytes >= 64) {
        compress(input);
        input += 64;
        bytes -= 64;
    }
    while (bytes > 0) {
        m_buffer[m_used++] = *input++;
        --bytes;
    }
}

std::array<u8, 32> Sha256::digest() {
    // Padding: a 0x80 byte, zeros, and the message length in bits as a 64-bit
    // big-endian integer, so that the padded length is a multiple of 64.
    const u64 bits = m_totalBits;
    u8 pad[128];
    size_t padLen = 0;
    pad[padLen++] = 0x80;
    while ((m_used + padLen) % 64 != 56) pad[padLen++] = 0x00;
    for (int i = 7; i >= 0; --i) pad[padLen++] = static_cast<u8>((bits >> (i * 8)) & 0xFFu);

    // update() would add to m_totalBits, which no longer matters: the digest is the
    // end of this object's life.
    const u64 savedBits = m_totalBits;
    update(pad, padLen);
    m_totalBits = savedBits;

    std::array<u8, 32> out{};
    for (int i = 0; i < 8; ++i) {
        out[i * 4] = static_cast<u8>(m_state[i] >> 24);
        out[i * 4 + 1] = static_cast<u8>(m_state[i] >> 16);
        out[i * 4 + 2] = static_cast<u8>(m_state[i] >> 8);
        out[i * 4 + 3] = static_cast<u8>(m_state[i]);
    }
    reset();
    return out;
}

std::string Sha256::digestHex() {
    static const char* digits = "0123456789abcdef";
    const std::array<u8, 32> raw = digest();
    std::string hex;
    hex.reserve(kHexBytes);
    for (u8 byte : raw) {
        hex.push_back(digits[byte >> 4]);
        hex.push_back(digits[byte & 0x0Fu]);
    }
    return hex;
}

std::string sha256Hex(const void* data, size_t bytes) {
    Sha256 hash;
    hash.update(data, bytes);
    return hash.digestHex();
}

bool sha256HexFile(const std::string& path, std::string* outHex) {
    if (outHex) outHex->clear();
    std::FILE* file = std::fopen(path.c_str(), "rb");
    if (!file) return false;
    Sha256 hash;
    std::vector<u8> buffer(1u << 16);
    bool ok = true;
    for (;;) {
        const size_t got = std::fread(buffer.data(), 1, buffer.size(), file);
        if (got > 0) hash.update(buffer.data(), got);
        if (got < buffer.size()) {
            ok = std::feof(file) != 0;
            break;
        }
    }
    std::fclose(file);
    if (!ok) return false;
    if (outHex) *outHex = hash.digestHex();
    return true;
}

}  // namespace mumbai
