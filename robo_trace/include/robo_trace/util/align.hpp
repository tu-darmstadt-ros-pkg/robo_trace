#pragma once

// Alignment magic
#include <new>
#ifdef __cpp_lib_hardware_interference_size
using std::hardware_constructive_interference_size;
using std::hardware_destructive_interference_size;
#else
#include <cstddef>
//TODO this probably needs to be adjusted depending on the architecture / CPU
// 64 bytes on x86-64 │ L1_CACHE_BYTES │ L1_CACHE_SHIFT │ __cacheline_aligned │ ...
static constexpr std::size_t hardware_constructive_interference_size = 2 * sizeof(std::max_align_t);
static constexpr std::size_t hardware_destructive_interference_size = 2 * sizeof(std::max_align_t);
#endif
