/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */
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
