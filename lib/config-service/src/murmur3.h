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

#pragma once

#include <stdint.h>
#include <string.h>

struct murmur3_hash_t {
    uint32_t h[4];
    uint8_t accum[16];
    int accum_len;

    murmur3_hash_t() : h{0}, accum_len(0) {}

    bool operator==(const murmur3_hash_t& other) const
    {
        return !memcmp(h, other.h, sizeof(h)) && (accum_len == other.accum_len);
    }

    bool operator!=(const murmur3_hash_t& other) const
    {
    return !(other == *this);
    }
};

void murmur3_hash_start(murmur3_hash_t &hash, uint32_t seed);
void murmur3_hash_update(murmur3_hash_t &hash, const void * data, unsigned int len);
void murmur3_hash_finalize(murmur3_hash_t &hash);
