/*****************************************************************************
 * Copyright (C) 2013 x265 project
 *
 * Authors: Steve Borho <steve@borho.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *
 * This program is also available under a commercial proprietary license.
 * For more information, contact us at license @ x265.com
 *****************************************************************************/

#ifndef X265_THREADING_H
#define X265_THREADING_H

#include "common.h"
#include "x265.h"

#ifdef _WIN32
#include <windows.h>
#include "winxp.h"  // XP workarounds for CONDITION_VARIABLE and ATOMIC_OR
#else
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include <fcntl.h>
#endif

#if MACOS
#include <sys/param.h>
#include <sys/sysctl.h>
#endif

#if NO_ATOMICS

#include <sys/time.h>
#include <unistd.h>

namespace X265_NS {
// x265 private namespace
int no_atomic_or(int* ptr, int mask);
int no_atomic_and(int* ptr, int mask);
int no_atomic_inc(int* ptr);
int no_atomic_dec(int* ptr);
int no_atomic_add(int* ptr, int val);
}

#define CLZ(id, x)            id = (unsigned long)__builtin_clz(x) ^ 31
#define CTZ(id, x)            id = (unsigned long)__builtin_ctz(x)
#define ATOMIC_OR(ptr, mask)  no_atomic_or((int*)ptr, mask)
#define ATOMIC_AND(ptr, mask) no_atomic_and((int*)ptr, mask)
#define ATOMIC_INC(ptr)       no_atomic_inc((int*)ptr)
#define ATOMIC_DEC(ptr)       no_atomic_dec((int*)ptr)
#define ATOMIC_ADD(ptr, val)  no_atomic_add((int*)ptr, val)
#define GIVE_UP_TIME()        usleep(0)

#elif __GNUC__               /* GCCs builtin atomics */

#include <sys/time.h>
#include <unistd.h>

#define CLZ(id, x)            id = (unsigned long)__builtin_clz(x) ^ 31
#define CTZ(id, x)            id = (unsigned long)__builtin_ctz(x)
#define ATOMIC_OR(ptr, mask)  __sync_fetch_and_or(ptr, mask)
#define ATOMIC_AND(ptr, mask) __sync_fetch_and_and(ptr, mask)
#define ATOMIC_INC(ptr)       __sync_add_and_fetch((volatile int32_t*)ptr, 1)
#define ATOMIC_DEC(ptr)       __sync_add_and_fetch((volatile int32_t*)ptr, -1)
#define ATOMIC_ADD(ptr, val)  __sync_fetch_and_add((volatile int32_t*)ptr, val)
#define GIVE_UP_TIME()        usleep(0)

#elif defined(_MSC_VER)       /* Windows atomic intrinsics */

#include <intrin.h>

#define CLZ(id, x)            _BitScanReverse(&id, x)
#define CTZ(id, x)            _BitScanForward(&id, x)
#define ATOMIC_INC(ptr)       InterlockedIncrement((volatile LONG*)ptr)
#define ATOMIC_DEC(ptr)       InterlockedDecrement((volatile LONG*)ptr)
#define ATOMIC_ADD(ptr, val)  InterlockedExchangeAdd((volatile LONG*)ptr, val)
#define ATOMIC_OR(ptr, mask)  _InterlockedOr((volatile LONG*)ptr, (LONG)mask)
#define ATOMIC_AND(ptr, mask) _InterlockedAnd((volatile LONG*)ptr, (LONG)mask)
#define GIVE_UP_TIME()        Sleep(0)

#endif // ifdef __GNUC__

#endif