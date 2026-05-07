// Copyright 2023 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_SRC_THREAD_THREAD_POOL_H_
#define MUJOCO_SRC_THREAD_THREAD_POOL_H_

#include <stddef.h>

#include <mujoco/mjexport.h>
#include <mujoco/mjthread.h>

#ifdef __cplusplus
namespace mujoco {
extern "C" {
#endif

// Create a thread pool with the specified number of threads running.
MJAPI mjThreadPool* mju_threadPoolCreate(size_t number_of_threads);

// Enqueue a task in a thread pool.
MJAPI void mju_threadPoolEnqueue(mjThreadPool* thread_pool, mjTask* task);

// Destroy a thread pool.
MJAPI void mju_threadPoolDestroy(mjThreadPool* thread_pool);

#ifdef __cplusplus
}  // extern "C"
}  // namespace mujoco
#endif  // __cplusplus

#endif  // MUJOCO_SRC_THREAD_THREAD_POOL_H_
