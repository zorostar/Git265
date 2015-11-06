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

#include "common.h"
#include "threadpool.h"
#include "threading.h"

#include <new>

#if X86_64

#ifdef __GNUC__

#define SLEEPBITMAP_CTZ(id, x)     id = (unsigned long)__builtin_ctzll(x)
#define SLEEPBITMAP_OR(ptr, mask)  __sync_fetch_and_or(ptr, mask)
#define SLEEPBITMAP_AND(ptr, mask) __sync_fetch_and_and(ptr, mask)

#elif defined(_MSC_VER)

#define SLEEPBITMAP_CTZ(id, x)     _BitScanForward64(&id, x)
#define SLEEPBITMAP_OR(ptr, mask)  InterlockedOr64((volatile LONG64*)ptr, (LONG)mask)
#define SLEEPBITMAP_AND(ptr, mask) InterlockedAnd64((volatile LONG64*)ptr, (LONG)mask)

#endif // ifdef __GNUC__

#else

/* use 32-bit primitives defined in threading.h */
#define SLEEPBITMAP_CTZ CTZ
#define SLEEPBITMAP_OR  ATOMIC_OR
#define SLEEPBITMAP_AND ATOMIC_AND

#endif

#if MACOS
#include <sys/param.h>
#include <sys/sysctl.h>
#endif
#if HAVE_LIBNUMA
#include <numa.h>
#endif

