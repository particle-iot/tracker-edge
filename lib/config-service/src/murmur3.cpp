/*
 * Copyright (c) 2020 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// adapted from public domain implementation of murmur3 at https://github.com/aappleby/smhasher

#include "murmur3.h"

#include <stdint.h>
#include <string.h>

#include <algorithm>

#include "Particle.h"

static constexpr uint32_t c1 = 0x239b961b;
static constexpr uint32_t c2 = 0xab0e9789;
static constexpr uint32_t c3 = 0x38b34ae5;
static constexpr uint32_t c4 = 0xa1e38b93;

static uint32_t rotl32(uint32_t x, int8_t r)
{
    return (x << r) | (x >> (32 - r));
}

static uint32_t fmix32(uint32_t h)
{
    h ^= h >> 16;
    h *= 0x85ebca6b;
    h ^= h >> 13;
    h *= 0xc2b2ae35;
    h ^= h >> 16;

    return h;
}

static void murmur3_hash_step(murmur3_hash_t &hash, const uint8_t *block)
{
    // k1 through k4 should not need to be declared volatile
    // however, the compiler appears to generate an illegal instruction
    // sequence under optimization that causes the operation to hardfault
    // and declaring as volatile functions to prevent that
    volatile uint32_t k1 = ((uint32_t *) block)[0];
    volatile uint32_t k2 = ((uint32_t *) block)[1];
    volatile uint32_t k3 = ((uint32_t *) block)[2];
    volatile uint32_t k4 = ((uint32_t *) block)[3];

    k1 *= c1;
    k1  = rotl32(k1,15);
    k1 *= c2;
    hash.h[0] ^= k1;

    hash.h[0] = rotl32(hash.h[0],19);
    hash.h[0] += hash.h[1];
    hash.h[0] = hash.h[0]*5+0x561ccd1b;

    k2 *= c2;
    k2  = rotl32(k2,16);
    k2 *= c3; hash.h[1] ^= k2;

    hash.h[1] = rotl32(hash.h[1],17);
    hash.h[1] += hash.h[2];
    hash.h[1] = hash.h[1]*5+0x0bcaa747;

    k3 *= c3;
    k3  = rotl32(k3,17);
    k3 *= c4;
    hash.h[2] ^= k3;

    hash.h[2] = rotl32(hash.h[2],15);
    hash.h[2] += hash.h[3];
    hash.h[2] = hash.h[2]*5+0x96cd1c35;

    k4 *= c4;
    k4  = rotl32(k4,18);
    k4 *= c1;
    hash.h[3] ^= k4;

    hash.h[3] = rotl32(hash.h[3],13);
    hash.h[3] += hash.h[0];
    hash.h[3] = hash.h[3]*5+0x32ac3b17;
}

void murmur3_hash_start(murmur3_hash_t &hash, uint32_t seed)
{
    hash.h[0] = seed;
    hash.h[1] = seed;
    hash.h[2] = seed;
    hash.h[3] = seed;

    hash.accum_len = 0;
}

void murmur3_hash_update(murmur3_hash_t &hash, const void * data, unsigned int len)
{
    const uint8_t * bytes = (const uint8_t *) data;

    while(len > 0)
    {
        // handle partial block updates
        if(hash.accum_len || len < sizeof(hash.accum))
        {
            int to_copy = std::min(sizeof(hash.accum) - hash.accum_len, len);
            memcpy(hash.accum + hash.accum_len, bytes, to_copy);
            hash.accum_len += to_copy;
            bytes += to_copy;
            len -= to_copy;

            if(hash.accum_len == sizeof(hash.accum))
            {
                murmur3_hash_step(hash, hash.accum);
                hash.accum_len = 0;
            }
        }
        // handle full block updates
        else
        {
            while(len >= sizeof(hash.accum))
            {
                murmur3_hash_step(hash, bytes);
                bytes += sizeof(hash.accum);
                len -= sizeof(hash.accum);
            }
        }
    }
}

void murmur3_hash_finalize(murmur3_hash_t &hash)
{
    const uint8_t * tail = hash.accum;

    uint32_t k1 = 0;
    uint32_t k2 = 0;
    uint32_t k3 = 0;
    uint32_t k4 = 0;

    switch(hash.accum_len & 15)
    {
    case 15: k4 ^= tail[14] << 16;
    case 14: k4 ^= tail[13] << 8;
    case 13: k4 ^= tail[12] << 0;
            k4 *= c4; k4  = rotl32(k4,18); k4 *= c1; hash.h[3] ^= k4;

    case 12: k3 ^= tail[11] << 24;
    case 11: k3 ^= tail[10] << 16;
    case 10: k3 ^= tail[ 9] << 8;
    case  9: k3 ^= tail[ 8] << 0;
            k3 *= c3; k3  = rotl32(k3,17); k3 *= c4; hash.h[2] ^= k3;

    case  8: k2 ^= tail[ 7] << 24;
    case  7: k2 ^= tail[ 6] << 16;
    case  6: k2 ^= tail[ 5] << 8;
    case  5: k2 ^= tail[ 4] << 0;
            k2 *= c2; k2  = rotl32(k2,16); k2 *= c3; hash.h[1] ^= k2;

    case  4: k1 ^= tail[ 3] << 24;
    case  3: k1 ^= tail[ 2] << 16;
    case  2: k1 ^= tail[ 1] << 8;
    case  1: k1 ^= tail[ 0] << 0;
            k1 *= c1; k1  = rotl32(k1,15); k1 *= c2; hash.h[0] ^= k1;
    };

    hash.h[0] ^= hash.accum_len; hash.h[1] ^= hash.accum_len; hash.h[2] ^= hash.accum_len; hash.h[3] ^= hash.accum_len;

    hash.h[0] += hash.h[1]; hash.h[0] += hash.h[2]; hash.h[0] += hash.h[3];
    hash.h[1] += hash.h[0]; hash.h[2] += hash.h[0]; hash.h[3] += hash.h[0];

    hash.h[0] = fmix32(hash.h[0]);
    hash.h[1] = fmix32(hash.h[1]);
    hash.h[2] = fmix32(hash.h[2]);
    hash.h[3] = fmix32(hash.h[3]);

    hash.h[0] += hash.h[1]; hash.h[0] += hash.h[2]; hash.h[0] += hash.h[3];
    hash.h[1] += hash.h[0]; hash.h[2] += hash.h[0]; hash.h[3] += hash.h[0];

    hash.accum_len = 0;
}
