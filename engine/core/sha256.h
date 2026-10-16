// =============================================================================
//  MUMBAI 1:1 EXACT — engine/core/sha256.h
//
//  WHY A HASH IMPLEMENTATION IS PART OF THE ENGINE AND NOT OF THE BUILD.
//
//  The city does not fit in the APK: it is a download, and a download that is not
//  verified is a download that can quietly become a different city — a truncated
//  chunk reads as terrain that ends in a cliff, a corrupted instance table reads
//  as a building standing in the sea.  Both are "artifacts" by the specification's
//  own definition, and neither is visible in a log.
//
//  So the client checks every byte it wrote against the hashes the build
//  published, which means hashing while the bytes stream past rather than after
//  they are on disk.  That is this class.
//
//  It is FIPS 180-4 SHA-256 and nothing more: no HMAC, no signing, no crypto
//  library.  Verification against a hash that came from the same place as the data
//  is a corruption check, not a security boundary, and it is described that way
//  here so nobody mistakes it for one.
// =============================================================================
#pragma once
#include "engine/core/core.h"

#include <array>
#include <string>

namespace mumbai {

class Sha256 {
  public:
    Sha256() { reset(); }

    void reset();
    void update(const void* data, size_t bytes);

    // Finishes the hash.  The digest is returned; the object is left reset, so a
    // caller that wants to keep hashing starts again by itself.
    std::array<u8, 32> digest();
    std::string digestHex();

    static constexpr size_t kDigestBytes = 32;
    static constexpr size_t kHexBytes = 64;

  private:
    void compress(const u8* block);

    u32 m_state[8]{};
    u64 m_totalBits = 0;
    u8 m_buffer[64]{};
    size_t m_used = 0;
};

// One-shot helpers.  `sha256HexFile` returns false and an empty string when the
// file cannot be read: a missing file is a state, not an exception in this project.
std::string sha256Hex(const void* data, size_t bytes);
bool sha256HexFile(const std::string& path, std::string* outHex);

}  // namespace mumbai
